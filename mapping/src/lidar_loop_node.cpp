#include "mapping/lidar_loop_node.h"
#include <filesystem>
#include <fstream>
#include <sstream>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#ifdef ROS_HUMBLE
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#else
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#endif
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

using namespace gtsam;

LidarLoopNode::LidarLoopNode() : Node("lidar_loop_node") {
    perf_ = std::make_unique<runtime_diag::Reporter>(this, "mapping",
        std::initializer_list<const char*>{"odom", "cloud", "pose_graph", "loop_icp"});
    initParameters();
    
    // 填充 MapPavingEngine 配置并实例化
    MapPavingEngine::Config paving_config;
    paving_config.save_directory = saveDirectory_;
    paving_config.T_body_to_baselink = T_body_to_baselink_;
    paving_config.pcd2pgm_map_resolution = pcd2pgm_map_resolution_;
    paving_config.pcd2pgm_thre_z_min = pcd2pgm_thre_z_min_;
    paving_config.pcd2pgm_thre_z_max = pcd2pgm_thre_z_max_;
    paving_config.pcd2pgm_thre_radius = pcd2pgm_thre_radius_;
    paving_config.pcd2pgm_thres_point_count = pcd2pgm_thres_point_count_;

    factorGraphManager_ = std::make_unique<FactorGraphManager>();
    loopDetector_ = std::make_unique<LoopDetector>(this);
    mapPavingEngine_ = std::make_unique<MapPavingEngine>(paving_config, this->get_logger());

    // 初始化噪声参数并传参
    factorGraphManager_->initNoises(
        PRIOR_NOISE_VAR_, ODOM_NOISE_VAR_TRANS_, ODOM_NOISE_VAR_ROT_,
        loopNoiseScore_, GPS_XY_NOISE_VAR_, GPS_Z_NOISE_VAR_
    );

    downSizeFilterScancontext_.setLeafSize(HISTORY_CLOUD_DOWNSAMPLE_, HISTORY_CLOUD_DOWNSAMPLE_,
                                           HISTORY_CLOUD_DOWNSAMPLE_);

    initFileSave();
    if (isIncrementalMode_ &&
        (!loopDetector_->loadHistoricalDatabase(saveDirectory_ + "sc_database.txt", pgScansDirectory_) ||
         loopDetector_->getHistoricalKeyframeCount() != keyframeIdOffset_ ||
         histPosesInMapFrame_.size() != static_cast<size_t>(keyframeIdOffset_))) {
        throw std::runtime_error("Cumulative mapping requires consistent historical poses, scans and ScanContext database");
    }
    initPublishers();
    initSubscribers();

    // 启动多线程
    posegraphThread_ = std::thread(&LidarLoopNode::process_pg, this);
    lcDetectionThread_ = std::thread(&LidarLoopNode::process_lcd, this);
    icpThread_ = std::thread(&LidarLoopNode::process_icp, this);
    isamThread_ = std::thread(&LidarLoopNode::process_isam, this);

    if (isIncrementalMode_) {
        crossSessionThread_ = std::thread(&LidarLoopNode::process_cross_session, this);
    }

    RCLCPP_INFO(this->get_logger(), "Node initialized successfully.");
}

LidarLoopNode::~LidarLoopNode() {
    stopThreads_ = true;

    if (posegraphThread_.joinable()) posegraphThread_.join();
    if (lcDetectionThread_.joinable()) lcDetectionThread_.join();
    if (icpThread_.joinable()) icpThread_.join();
    if (isamThread_.joinable()) isamThread_.join();
    if (crossSessionThread_.joinable()) crossSessionThread_.join();
    if (historicalMapThread_.joinable()) historicalMapThread_.join();

    // 确保最近更新的关键帧索引覆盖所有已添加的帧，以防优化线程没来得及完成
    recentIdxUpdated_ = std::max(recentIdxUpdated_, int(loopDetector_->getKeyframePosesUpdated().size()) - 1);

    // 未对齐或没有新增帧时不覆盖历史地图，界面保留本次临时目录供排查。
    bool can_merge = isIncrementalMode_ && hasAnchored_;
    if (isIncrementalMode_ && (!can_merge || recentIdxUpdated_ < 0)) {
        RCLCPP_WARN(this->get_logger(), "Cumulative mapping not saved: missing anchor or new keyframes.");
        return;
    }

    // 🟠 全局累积轨迹立即持久化（放在析构函数最前部，耗时 <2ms，防止后续生成大地图耗时过长被 launch SIGKILL 终止！）：
    if (can_merge && std::filesystem::exists(saveDirectory_ + "optimized_poses_hist.txt")) {
        std::string hist_file = saveDirectory_ + "optimized_poses_hist.txt";
        std::string full_poses_file = saveDirectory_ + "optimized_poses.txt";
        std::string cum_poses_file = saveDirectory_ + "optimized_poses_cumulative.txt";

        std::filesystem::copy_file(hist_file, full_poses_file, std::filesystem::copy_options::overwrite_existing);

        std::ofstream ofs_full(full_poses_file, std::ios::out | std::ios::app);
        if (ofs_full.is_open()) {
            ofs_full << std::fixed << std::setprecision(6);
            auto& kfPoses = loopDetector_->getKeyframePosesUpdated();
            Eigen::Matrix4f T_m_init_f = T_map_to_init_.matrix().cast<float>();

            for (int i = 0; i <= recentIdxUpdated_ && i < int(kfPoses.size()); ++i) {
                const auto& p = kfPoses[i];
                Eigen::Affine3f aff = pcl::getTransformation(p.x, p.y, p.z, p.roll, p.pitch, p.yaw);
                Eigen::Matrix4f T_world = T_m_init_f * aff.matrix();

                // KITTI 3x4 row-major 格式
                ofs_full << T_world(0,0) << " " << T_world(0,1) << " " << T_world(0,2) << " " << T_world(0,3) << " "
                         << T_world(1,0) << " " << T_world(1,1) << " " << T_world(1,2) << " " << T_world(1,3) << " "
                         << T_world(2,0) << " " << T_world(2,1) << " " << T_world(2,2) << " " << T_world(2,3) << "\n";
            }
            ofs_full.close();
            std::filesystem::copy_file(full_poses_file, cum_poses_file, std::filesystem::copy_options::overwrite_existing);
            RCLCPP_INFO(this->get_logger(),
                        "[LidarLoopNode] 🟠 Full multi-session trajectory saved to optimized_poses.txt and cumulative backup (Cumulative keyframes: %d).",
                        keyframeIdOffset_ + recentIdxUpdated_ + 1);
        }
    }

    Eigen::Matrix4f T_m_init = T_map_to_init_.matrix().cast<float>();

    // 保存 ScanContext 特征库 (支持增量合并与 O_A 世界坐标对齐)
    loopDetector_->saveSCDatabase(saveDirectory_ + "sc_database.txt", isIncrementalMode_, hasAnchored_, T_m_init);

    // 保存全量因子图拓扑
    factorGraphManager_->saveFactorsToFile(saveDirectory_ + "factors.txt", isIncrementalMode_, keyframeIdOffset_);

    // 调用 MapPavingEngine 进行拼接、调平、过滤与文件保存
    bool map_saved = mapPavingEngine_->buildAndSaveMaps(
        loopDetector_->getKeyframeLaserClouds(),
        loopDetector_->getKeyframePosesUpdated(),
        recentIdxUpdated_,
        can_merge,
        T_m_init
    );

    // 终极优化完成后，一次性计算各 ArUco 地标最优世界位姿并落盘
    saveArucoLandmarks();

    if (pgTimeSaveStream_.is_open()) {
        pgTimeSaveStream_.close();
    }
    // 界面还会复核文件更新时间和关键帧数量，再允许用户确认保存。
    if (can_merge && map_saved) {
        std::ofstream(saveDirectory_ + ".cumulative_complete") << "complete\n";
    }
}

void LidarLoopNode::RGB2XYZI(const pcl::PointCloud<PointTypeRGB>::Ptr& cloudIn, pcl::PointCloud<PointType>::Ptr& cloudOut) {
    cloudOut->points.resize(cloudIn->points.size());
    cloudOut->header = cloudIn->header;
    cloudOut->width = cloudIn->width;
    cloudOut->height = cloudIn->height;
    for (size_t i = 0; i < cloudIn->points.size(); ++i) {
        cloudOut->points[i].x = cloudIn->points[i].x;
        cloudOut->points[i].y = cloudIn->points[i].y;
        cloudOut->points[i].z = cloudIn->points[i].z;
        cloudOut->points[i].intensity = 0.0f;
    }
}

void LidarLoopNode::initParameters() {
    this->declare_parameter<std::string>("save_directory", "./src/DeepRobotics_ws/data/");
    this->declare_parameter<double>("keyframe_meter_gap", 2.0);
    this->declare_parameter<double>("keyframe_deg_gap", 10.0);
    this->declare_parameter<double>("sc_dist_thres", 0.15);
    this->declare_parameter<double>("sc_max_radius", 80.0);
    this->declare_parameter<double>("historyKeyframeSearchRadius", 10.0);
    this->declare_parameter<double>("historyKeyframeSearchTimeDiff", 30.0);
    this->declare_parameter<int>("historyKeyframeSearchNum", 25);
    this->declare_parameter<double>("loopNoiseScore", 0.5);
    this->declare_parameter<int>("graphUpdateTimes", 2);
    this->declare_parameter<double>("loopFitnessScoreThreshold", 0.20);
    this->declare_parameter<double>("speedFactor", 1.0);
    this->declare_parameter<double>("loopClosureFrequency", 2.0);
    this->declare_parameter<double>("graphUpdateFrequency", 1.0);
    this->declare_parameter<double>("vizmapFrequency", 0.5);
    this->declare_parameter<double>("vizPathFrequency", 10.0);
    this->declare_parameter<double>("mapviz_filter_size", 0.05);
    this->declare_parameter<bool>("is_incremental_mode", false);

    // pcd2pgm 栅格化参数
    this->declare_parameter<double>("pcd2pgm_map_resolution", 0.05);
    this->declare_parameter<double>("pcd2pgm_thre_z_min", -0.4);
    this->declare_parameter<double>("pcd2pgm_thre_z_max", 0.2);
    this->declare_parameter<double>("pcd2pgm_thre_radius", 0.8);
    this->declare_parameter<int>("pcd2pgm_thres_point_count", 10);

    saveDirectory_ = this->get_parameter("save_directory").as_string();
    if (!saveDirectory_.empty() && saveDirectory_.back() != '/') saveDirectory_ += '/';
    keyframeMeterGap_ = this->get_parameter("keyframe_meter_gap").as_double();
    keyframeDegGap_ = this->get_parameter("keyframe_deg_gap").as_double();
    keyframeRadGap_ = deg2rad(keyframeDegGap_);
    scDistThres_ = this->get_parameter("sc_dist_thres").as_double();
    scMaximumRadius_ = this->get_parameter("sc_max_radius").as_double();
    historyKeyframeSearchRadius_ = this->get_parameter("historyKeyframeSearchRadius").as_double();
    historyKeyframeSearchTimeDiff_ = this->get_parameter("historyKeyframeSearchTimeDiff").as_double();
    historyKeyframeSearchNum_ = this->get_parameter("historyKeyframeSearchNum").as_int();

    pcd2pgm_map_resolution_ = this->get_parameter("pcd2pgm_map_resolution").as_double();
    pcd2pgm_thre_z_min_ = this->get_parameter("pcd2pgm_thre_z_min").as_double();
    pcd2pgm_thre_z_max_ = this->get_parameter("pcd2pgm_thre_z_max").as_double();
    pcd2pgm_thre_radius_ = this->get_parameter("pcd2pgm_thre_radius").as_double();
    pcd2pgm_thres_point_count_ = this->get_parameter("pcd2pgm_thres_point_count").as_int();
    loopNoiseScore_ = this->get_parameter("loopNoiseScore").as_double();
    graphUpdateTimes_ = this->get_parameter("graphUpdateTimes").as_int();
    loopFitnessScoreThreshold_ = this->get_parameter("loopFitnessScoreThreshold").as_double();
    speedFactor_ = this->get_parameter("speedFactor").as_double();
    loopClosureFrequency_ = this->get_parameter("loopClosureFrequency").as_double() * speedFactor_;
    graphUpdateFrequency_ = this->get_parameter("graphUpdateFrequency").as_double() * speedFactor_;
    vizPathFrequency_ = this->get_parameter("vizPathFrequency").as_double() * speedFactor_;
    mapVizFilterSize_ = this->get_parameter("mapviz_filter_size").as_double();
    isIncrementalMode_ = this->get_parameter("is_incremental_mode").as_bool();

    // 载入激光雷达和机体外参
    std::vector<double> ext_matrix = std::vector<double>(16, 0.0);
    if (!this->has_parameter("T_lidar_base")) {
        this->declare_parameter<std::vector<double>>("T_lidar_base", std::vector<double>(16, 0.0));
    }
    this->get_parameter("T_lidar_base", ext_matrix);
    if (ext_matrix.size() == 16 && ext_matrix[0] != 0.0) {
        T_body_to_baselink_ << ext_matrix[0], ext_matrix[1], ext_matrix[2], ext_matrix[3],
                               ext_matrix[4], ext_matrix[5], ext_matrix[6], ext_matrix[7],
                               ext_matrix[8], ext_matrix[9], ext_matrix[10], ext_matrix[11],
                               ext_matrix[12], ext_matrix[13], ext_matrix[14], ext_matrix[15];
    } else {
        T_body_to_baselink_ = Eigen::Matrix4f::Identity();
    }
    RCLCPP_INFO(this->get_logger(), "Parameters loaded successfully.");
    RCLCPP_INFO(this->get_logger(), "  save_directory: %s", saveDirectory_.c_str());
    RCLCPP_INFO(this->get_logger(), "  is_incremental_mode: %s", isIncrementalMode_ ? "TRUE" : "FALSE");
    RCLCPP_INFO(this->get_logger(), "  keyframe_meter_gap: %.2f", keyframeMeterGap_);
    RCLCPP_INFO(this->get_logger(), "  loopClosureFrequency: %.2f", loopClosureFrequency_);
}

void LidarLoopNode::initFileSave() {
    std::filesystem::remove(saveDirectory_ + ".cumulative_complete");
    std::filesystem::remove(saveDirectory_ + ".cumulative_anchored");
    pgKITTIformat_ = saveDirectory_ + "optimized_poses.txt";
    odomKITTIformat_ = saveDirectory_ + "odom_poses.txt";
    pgScansDirectory_ = saveDirectory_ + "Scans/";
    pgImageDirectory_ = saveDirectory_ + "Image/";

    std::string cmd1 = "mkdir -p " + saveDirectory_;
    auto ret1 = system(cmd1.c_str());
    (void)ret1;

    keyframeIdOffset_ = 0;
    if (isIncrementalMode_) {
        // 累积建图模式：不清空历史 Scans 与地图文件，扫描已有最大索引
        std::string cmd_mkdir = "mkdir -p " + pgScansDirectory_ + " " + pgImageDirectory_;
        auto ret_mkdir = system(cmd_mkdir.c_str());
        (void)ret_mkdir;

        namespace fs = std::filesystem;
        if (fs::exists(pgScansDirectory_)) {
            for (const auto& entry : fs::directory_iterator(pgScansDirectory_)) {
                if (entry.is_regular_file() && entry.path().extension() == ".pcd") {
                    std::string stem = entry.path().stem().string();
                    try {
                        int id = std::stoi(stem);
                        if (id >= keyframeIdOffset_) {
                            keyframeIdOffset_ = id + 1;
                        }
                    } catch (...) {}
                }
            }
        }
        RCLCPP_INFO(this->get_logger(),
                    "[LidarLoopNode] Incremental mapping mode activated. Historical keyframe offset: %d.",
                    keyframeIdOffset_);
        pgTimeSaveStream_.open(saveDirectory_ + "times.txt", std::fstream::out | std::fstream::app);

        // 使用已验证的当前地图轨迹，不能按行数猜测并复活过期备份。
        std::string hist_src = saveDirectory_ + "optimized_poses.txt";
        std::string hist_dst = saveDirectory_ + "optimized_poses_hist.txt";
        std::string hist_cum = saveDirectory_ + "optimized_poses_cumulative.txt";
        std::filesystem::copy_file(hist_src, hist_dst, std::filesystem::copy_options::overwrite_existing);
        std::filesystem::copy_file(hist_src, hist_cum, std::filesystem::copy_options::overwrite_existing);

        // Preload historical poses into histPosesInMapFrame_ right here in the main thread (thread-safe!)
        loadHistoricalPosesInMapFrame(hist_dst);
    } else {
        // 单次建图模式：彻底清理旧文件
        std::string cmd2 = "rm -rf " + pgScansDirectory_ + " " + pgImageDirectory_;
        std::string cmd_clean_old = "rm -f " + saveDirectory_ + "*.pcd " + saveDirectory_ + "*.pgm " +
                                    saveDirectory_ + "*.yaml " + saveDirectory_ + "*.txt " + saveDirectory_ + "*.bin";
        std::string cmd3 = "mkdir -p " + pgScansDirectory_ + " " + pgImageDirectory_;

        auto ret2 = system(cmd2.c_str());
        auto ret_clean = system(cmd_clean_old.c_str());
        auto ret3 = system(cmd3.c_str());
        (void)ret2;
        (void)ret_clean;
        (void)ret3;

        pgTimeSaveStream_.open(saveDirectory_ + "times.txt", std::fstream::out);
    }

    pgTimeSaveStream_.precision(std::numeric_limits<double>::max_digits10);
}

void LidarLoopNode::initPublishers() {
    pubOdomAftPGO_ = this->create_publisher<nav_msgs::msg::Odometry>("/aft_pgo_odom", 100);
    pubPathAftPGO_ = this->create_publisher<nav_msgs::msg::Path>("/aft_pgo_path", 100);

    // 自测三轨迹对比发布器
    pubPathOdom_ = this->create_publisher<nav_msgs::msg::Path>("/path_odom", 100);
    pubPathPGOArUco_ = this->create_publisher<nav_msgs::msg::Path>("/path_pgo_aruco", 100);

    pubLoopScanLocal_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/loop_scan_local", 100);
    pubLoopSubmapLocal_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/loop_submap_local", 100);
    pubLoopScanLocalRegisted_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("/loop_scan_local_registed", 100);

    pubMapHistorical_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("/map_historical", rclcpp::QoS(1).transient_local());
    pubPathHistorical_ =
        this->create_publisher<nav_msgs::msg::Path>("/path_historical", rclcpp::QoS(1).transient_local());
    pubKeyFramesId_ = this->create_publisher<std_msgs::msg::Header>("/key_frames_ids", 10);

    tfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tfStaticBroadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);

    // 默认发送初始 map -> camera_init (Identity)
    geometry_msgs::msg::TransformStamped initMapTf;
    initMapTf.header.stamp = this->now();
    initMapTf.header.frame_id = "map";
    initMapTf.child_frame_id = "camera_init";
    initMapTf.transform.rotation.w = 1.0;
    tfStaticBroadcaster_->sendTransform(initMapTf);

    publishHistoricalMap();
}

void LidarLoopNode::initSubscribers() {
    subLaserCloudFullRes_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud_registered", 100, std::bind(&LidarLoopNode::laserCloudFullResHandler, this, std::placeholders::_1));

    subLaserOdometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/aft_mapped_to_init", 100, std::bind(&LidarLoopNode::laserOdometryHandler, this, std::placeholders::_1));

    subGPS_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps/fix", 100, std::bind(&LidarLoopNode::gpsHandler, this, std::placeholders::_1));

    subVisualLoop_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
        "/visual_loop_candidate", 100, std::bind(&LidarLoopNode::visualLoopHandler, this, std::placeholders::_1));

    // ArUco landmark 观测订阅 (latched QoS 匹配 visual_loop_node 侧的发布者)
    subArucoLandmarkObs_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/aruco_landmark_obs", rclcpp::QoS(10).transient_local(),
        std::bind(&LidarLoopNode::arucoLandmarkObsHandler, this, std::placeholders::_1));

    subVisualPGO_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/visual_pgo_constraint", 100,
        std::bind(&LidarLoopNode::visualPGOConstraintHandler, this, std::placeholders::_1));

    subRelocPose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10,
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            this->relocPoseCallback(msg);
        });

    subMapToOdom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/map_to_odom", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            this->mapToOdomCallback(msg);
        });
}

void LidarLoopNode::laserOdometryHandler(const nav_msgs::msg::Odometry::SharedPtr msg) {
    mBuf_.lock();
    odometryBuf_.push(msg);
    perf_->at(0).receive();
    perf_->at(0).queued = odometryBuf_.size();

    // 自测 1: 缓存原始前端 Odom 轨迹
    geometry_msgs::msg::PoseStamped pose_stamp;
    pose_stamp.header = msg->header;
    pose_stamp.pose = msg->pose.pose;
    pathOdomMsg_.header = msg->header;
    pathOdomMsg_.header.frame_id = "camera_init";
    pathOdomMsg_.poses.push_back(pose_stamp);
    pubPathOdom_->publish(pathOdomMsg_);

    // 实时 10Hz 广播当前位姿 TF (camera_init -> aft_mapped) 确保 RViz 满速流畅渲染
    geometry_msgs::msg::TransformStamped tf_curr;
    tf_curr.header = msg->header;
    tf_curr.child_frame_id = "aft_mapped";
    tf_curr.transform.translation.x = msg->pose.pose.position.x;
    tf_curr.transform.translation.y = msg->pose.pose.position.y;
    tf_curr.transform.translation.z = msg->pose.pose.position.z;
    tf_curr.transform.rotation = msg->pose.pose.orientation;
    tfBroadcaster_->sendTransform(tf_curr);

    mBuf_.unlock();
}

void LidarLoopNode::laserCloudFullResHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    mBuf_.lock();
    fullResBuf_.push(msg);
    perf_->at(1).receive();
    perf_->at(1).queued = fullResBuf_.size();
    mBuf_.unlock();
}

void LidarLoopNode::gpsHandler(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    if (useGPS_) {
        mBuf_.lock();
        gpsBuf_.push(msg);
        mBuf_.unlock();
    }
}

void LidarLoopNode::visualLoopHandler(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    if (msg->data.size() < 2) return;
    int prev_node_idx = msg->data[0];
    int curr_node_idx = msg->data[1];
    loopDetector_->addVisualLoopCandidate(prev_node_idx, curr_node_idx);
}

void LidarLoopNode::visualPGOConstraintHandler(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() < 8) return;
    std::lock_guard<std::mutex> lock(mVisualPGO_);
    visualPGOQueue_.push(*msg);
}

std::string LidarLoopNode::padZeros(int val, int num_digits) {
    std::ostringstream out;
    out << std::internal << std::setfill('0') << std::setw(num_digits) << val;
    return out.str();
}

gtsam::Pose3 LidarLoopNode::Pose6DtoGTSAMPose3(const Pose6D& p) {
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(p.roll, p.pitch, p.yaw), gtsam::Point3(p.x, p.y, p.z));
}

Pose6D LidarLoopNode::getOdom(const nav_msgs::msg::Odometry::SharedPtr odom) {
    auto tx = odom->pose.pose.position.x;
    auto ty = odom->pose.pose.position.y;
    auto tz = odom->pose.pose.position.z;

    tf2::Quaternion q(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z,
                       odom->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    Pose6D pose;
    pose.x = tx;
    pose.y = ty;
    pose.z = tz;
    pose.roll = roll;
    pose.pitch = pitch;
    pose.yaw = yaw;
    pose.seq = 0;

    return pose;
}

Pose6D LidarLoopNode::diffTransformation(const Pose6D& p1, const Pose6D& p2) {
    Eigen::Affine3f SE3_p1 = pcl::getTransformation(p1.x, p1.y, p1.z, p1.roll, p1.pitch, p1.yaw);
    Eigen::Affine3f SE3_p2 = pcl::getTransformation(p2.x, p2.y, p2.z, p2.roll, p2.pitch, p2.yaw);
    Eigen::Matrix4f SE3_delta0 = SE3_p1.matrix().inverse() * SE3_p2.matrix();
    Eigen::Affine3f SE3_delta;
    SE3_delta.matrix() = SE3_delta0;
    float dx, dy, dz, droll, dpitch, dyaw;
    pcl::getTranslationAndEulerAngles(SE3_delta, dx, dy, dz, droll, dpitch, dyaw);

    Pose6D dtf;
    dtf.x = double(std::abs(dx));
    dtf.y = double(std::abs(dy));
    dtf.z = double(std::abs(dz));
    dtf.roll = double(std::abs(droll));
    dtf.pitch = double(std::abs(dpitch));
    dtf.yaw = double(std::abs(dyaw));
    dtf.seq = 0;

    return dtf;
}

Eigen::Affine3f LidarLoopNode::Pose6dToAffine3f(const Pose6D& pose) {
    return pcl::getTransformation(pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw);
}

void LidarLoopNode::saveOdometryVerticesKITTIformat(const std::string& filename) {
    std::fstream stream(filename.c_str(), std::fstream::out);
    auto& keyframePoses = loopDetector_->getKeyframePoses();
    loopDetector_->getKFMutex().lock();
    for (const auto& pose6d : keyframePoses) {
        gtsam::Pose3 pose = Pose6DtoGTSAMPose3(pose6d);
        Point3 t = pose.translation();
        Rot3 R = pose.rotation();
        auto col1 = R.column(1);
        auto col2 = R.column(2);
        auto col3 = R.column(3);

        stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x() << " " << col1.y() << " "
               << col2.y() << " " << col3.y() << " " << t.y() << " " << col1.z() << " " << col2.z() << " "
               << col3.z() << " " << t.z() << std::endl;
    }
    loopDetector_->getKFMutex().unlock();
}

void LidarLoopNode::saveOptimizedVerticesKITTIformat(const gtsam::Values& estimates, const std::string& filename) {
    if (isIncrementalMode_ && !hasAnchored_) {
        // 未锚定状态：当前 session 处于局部未对齐坐标系，保存至 curr 临时文件，严禁污染或截断 master 历史轨迹文件！
        std::string curr_file = saveDirectory_ + "optimized_poses_curr.txt";
        std::ofstream stream(curr_file.c_str(), std::ios::out);
        if (!stream.is_open()) return;
        for (const auto& key_value : estimates) {
            auto p = dynamic_cast<const GenericValue<Pose3>*>(&key_value.value);
            if (!p) continue;

            const Pose3& pose = p->value();
            Point3 t = pose.translation();
            Rot3 R = pose.rotation();
            auto col1 = R.column(1);
            auto col2 = R.column(2);
            auto col3 = R.column(3);

            stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x() << " " << col1.y() << " "
                   << col2.y() << " " << col3.y() << " " << t.y() << " " << col1.z() << " " << col2.z() << " "
                   << col3.z() << " " << t.z() << "\n";
        }
        return;
    }

    std::ofstream stream(filename.c_str(), std::ios::out);
    if (!stream.is_open()) return;

    // 1. 增量建图已锚定：先写入所有历史位姿（已处于统一 map 坐标系）
    if (isIncrementalMode_ && hasAnchored_) {
        for (const auto& hpose : histPosesInMapFrame_) {
            Point3 t = hpose.translation();
            Rot3 R = hpose.rotation();
            auto col1 = R.column(1);
            auto col2 = R.column(2);
            auto col3 = R.column(3);

            stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x() << " " << col1.y() << " "
                   << col2.y() << " " << col3.y() << " " << t.y() << " " << col1.z() << " " << col2.z() << " "
                   << col3.z() << " " << t.z() << "\n";
        }
    }

    // 2. 写入当前会话所有位姿（增量建图已锚定时，转换至统一 map 坐标系）
    for (const auto& key_value : estimates) {
        auto p = dynamic_cast<const GenericValue<Pose3>*>(&key_value.value);
        if (!p) continue;

        Pose3 pose = p->value();
        if (isIncrementalMode_ && hasAnchored_) {
            pose = T_map_to_init_ * pose;
        }

        Point3 t = pose.translation();
        Rot3 R = pose.rotation();
        auto col1 = R.column(1);
        auto col2 = R.column(2);
        auto col3 = R.column(3);

        stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x() << " " << col1.y() << " "
               << col2.y() << " " << col3.y() << " " << t.y() << " " << col1.z() << " " << col2.z() << " "
               << col3.z() << " " << t.z() << "\n";
    }
}

// 接收 visual_loop_node 发来的 ArUco landmark 观测数据
void LidarLoopNode::arucoLandmarkObsHandler(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    // 格式: [n_markers, (marker_id, kf_id, marker_size, T_lidar_marker[16]) x n]
    if (msg->data.size() < 1) return;
    int n = static_cast<int>(msg->data[0]);
    size_t idx = 1;
    std::lock_guard<std::mutex> lock(arucoLandmarkMutex_);
    for (int i = 0; i < n; ++i) {
        if (idx + 19 > msg->data.size()) break;
        int marker_id = static_cast<int>(msg->data[idx++]);
        ArucoLandmarkObs obs;
        obs.kf_id       = static_cast<int>(msg->data[idx++]);
        obs.marker_size  = msg->data[idx++];
        for (int j = 0; j < 16; ++j) obs.T_lidar_marker[j] = msg->data[idx++];
        arucoLandmarkObs_[marker_id] = obs;
    }
    RCLCPP_DEBUG(this->get_logger(), "ArUco: Received %d landmark observation(s).", n);
}

// 在 GTSAM 优化完成后，将 marker 的优化后世界坐标系位姿写出到 aruco_landmarks.yaml
// 在 GTSAM 优化完成后，将 marker 的优化后世界坐标系位姿智能合并写出到 aruco_landmarks.yaml
void LidarLoopNode::saveArucoLandmarks() {
    std::lock_guard<std::mutex> lock(arucoLandmarkMutex_);
    if (arucoLandmarkObs_.empty() && !isIncrementalMode_) return;

    auto& kfPoses = loopDetector_->getKeyframePosesUpdated();
    std::string filepath = saveDirectory_ + "aruco_landmarks.yaml";

    struct LandmarkRecord {
        int id;
        double marker_size;
        Eigen::Matrix4d pose; // Row-major 4x4 in O_A frame
        double obs_dist = 999.0;
    };
    std::map<int, LandmarkRecord> merged_landmarks;

    // 1. 如果是累积建图且已成功锚定，先读取并解析历史已有的 aruco_landmarks.yaml
    if (isIncrementalMode_ && hasAnchored_ && std::filesystem::exists(filepath)) {
        std::ifstream ifs(filepath);
        if (ifs.is_open()) {
            std::string line;
            int curr_id = -1;
            double curr_size = 0.2;
            while (std::getline(ifs, line)) {
                if (line.find("id:") != std::string::npos) {
                    size_t pos = line.find("id:");
                    try { curr_id = std::stoi(line.substr(pos + 3)); } catch (...) {}
                } else if (line.find("marker_size:") != std::string::npos) {
                    size_t pos = line.find("marker_size:");
                    try { curr_size = std::stod(line.substr(pos + 12)); } catch (...) {}
                } else if (line.find("[") != std::string::npos && curr_id >= 0) {
                    size_t start = line.find("[") + 1;
                    size_t end = line.find("]");
                    if (end > start) {
                        std::string nums_str = line.substr(start, end - start);
                        std::stringstream ss(nums_str);
                        std::string val_str;
                        Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
                        int idx = 0;
                        while (std::getline(ss, val_str, ',') && idx < 16) {
                            try { mat(idx / 4, idx % 4) = std::stod(val_str); } catch (...) {}
                            idx++;
                        }
                        if (idx == 16) {
                            LandmarkRecord rec;
                            rec.id = curr_id;
                            rec.marker_size = curr_size;
                            rec.pose = mat;
                            rec.obs_dist = 2.5; // 历史基线置信视距
                            merged_landmarks[curr_id] = rec;
                        }
                    }
                    curr_id = -1;
                }
            }
            ifs.close();
        }
    }

    // 2. 遍历当前会话 B 观测到的 markers，并与历史记录智能合并
    Eigen::Matrix4d T_map_to_init_d = T_map_to_init_.matrix().cast<double>();
    for (auto& [id, obs] : arucoLandmarkObs_) {
        int kf_id = obs.kf_id;
        if (kf_id < 0 || kf_id >= static_cast<int>(kfPoses.size())) continue;

        const Pose6D& p = kfPoses[kf_id];
        Eigen::Matrix4d T_init_lidar = Eigen::Matrix4d::Identity();
        Eigen::AngleAxisd roll_angle( p.roll,  Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitch_angle(p.pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yaw_angle(  p.yaw,   Eigen::Vector3d::UnitZ());
        T_init_lidar.block<3,3>(0,0) = (yaw_angle * pitch_angle * roll_angle).toRotationMatrix();
        T_init_lidar(0,3) = p.x;
        T_init_lidar(1,3) = p.y;
        T_init_lidar(2,3) = p.z;

        Eigen::Matrix4d T_lidar_marker;
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c)
                T_lidar_marker(r,c) = obs.T_lidar_marker[r*4+c];

        double obs_dist = std::sqrt(T_lidar_marker(0,3)*T_lidar_marker(0,3) +
                                    T_lidar_marker(1,3)*T_lidar_marker(1,3) +
                                    T_lidar_marker(2,3)*T_lidar_marker(2,3));

        Eigen::Matrix4d T_world_marker;
        if (isIncrementalMode_ && hasAnchored_) {
            T_world_marker = T_map_to_init_d * T_init_lidar * T_lidar_marker;
        } else {
            T_world_marker = T_init_lidar * T_lidar_marker;
        }

        if (merged_landmarks.find(id) != merged_landmarks.end()) {
            // 同一个二维码再次观测到：每个 ID 保证唯一记录！
            Eigen::Vector3d p_hist = merged_landmarks[id].pose.block<3,1>(0,3);
            Eigen::Vector3d p_curr = T_world_marker.block<3,1>(0,3);
            double drift = (p_hist - p_curr).norm();

            // 若本次观测视距更近（成像更清晰）或空间位置高度一致，优选/更新为本次位姿
            if (obs_dist < merged_landmarks[id].obs_dist || drift < 0.3) {
                merged_landmarks[id].pose = T_world_marker;
                merged_landmarks[id].obs_dist = obs_dist;
                RCLCPP_DEBUG(this->get_logger(),
                            "[ArUco] Marker ID=%d updated with higher-confidence observation (drift: %.3fm, obs_dist: %.2fm).",
                            id, drift, obs_dist);
            }
        } else {
            // 新发现的独立二维码，追加进库
            LandmarkRecord rec;
            rec.id = id;
            rec.marker_size = obs.marker_size;
            rec.pose = T_world_marker;
            rec.obs_dist = obs_dist;
            merged_landmarks[id] = rec;
            RCLCPP_DEBUG(this->get_logger(),
                        "[ArUco] New marker ID=%d registered and added to landmarks library.", id);
        }
    }

    if (merged_landmarks.empty()) return;

    // 3. 统一格式化写出到 aruco_landmarks.yaml
    std::ofstream ofs(filepath);
    if (!ofs.is_open()) {
        RCLCPP_WARN(this->get_logger(), "ArUco: Cannot open %s for writing.", filepath.c_str());
        return;
    }

    ofs << "landmarks:" << std::endl;
    for (auto& [id, rec] : merged_landmarks) {
        ofs << "  - id: " << id << std::endl;
        ofs << "    marker_size: " << std::fixed << std::setprecision(3) << rec.marker_size << std::endl;
        ofs << "    pose:  # T_world_marker Row-Major 4x4" << std::endl;
        ofs << "      [";
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                if (r == 0 && c == 0) ofs << std::fixed << std::setprecision(6) << rec.pose(r,c);
                else ofs << ", " << std::fixed << std::setprecision(6) << rec.pose(r,c);
            }
        }
        ofs << "]" << std::endl;
    }
    ofs.close();
    RCLCPP_INFO(this->get_logger(),
                "ArUco: aruco_landmarks.yaml saved successfully (Total unique landmarks: %zu).",
                merged_landmarks.size());
}

void LidarLoopNode::pubPath() {
    nav_msgs::msg::Odometry odomAftPGO;
    nav_msgs::msg::Path pathAftPGO;
    pathAftPGO.header.frame_id = "camera_init";

    auto& keyframePosesUpdated = loopDetector_->getKeyframePosesUpdated();
    auto& keyframeTimes = loopDetector_->getKeyframeTimes();

    loopDetector_->getKFMutex().lock();
    int num_poses = keyframePosesUpdated.size();
    for (int node_idx = 0; node_idx < recentIdxUpdated_ && node_idx < num_poses; node_idx++) {
        const Pose6D& pose_est = keyframePosesUpdated.at(node_idx);

        nav_msgs::msg::Odometry odomAftPGOthis;
        odomAftPGOthis.header.frame_id = "camera_init";
        odomAftPGOthis.child_frame_id = "aft_pgo";
        odomAftPGOthis.header.stamp = rclcpp::Time(static_cast<int64_t>(keyframeTimes.at(node_idx) * 1e9));
        odomAftPGOthis.pose.pose.position.x = pose_est.x;
        odomAftPGOthis.pose.pose.position.y = pose_est.y;
        odomAftPGOthis.pose.pose.position.z = pose_est.z;

        tf2::Quaternion q;
        q.setRPY(pose_est.roll, pose_est.pitch, pose_est.yaw);
        odomAftPGOthis.pose.pose.orientation.x = q.x();
        odomAftPGOthis.pose.pose.orientation.y = q.y();
        odomAftPGOthis.pose.pose.orientation.z = q.z();
        odomAftPGOthis.pose.pose.orientation.w = q.w();

        odomAftPGO = odomAftPGOthis;

        geometry_msgs::msg::PoseStamped poseStampAftPGO;
        poseStampAftPGO.header = odomAftPGOthis.header;
        poseStampAftPGO.pose = odomAftPGOthis.pose.pose;

        pathAftPGO.header.stamp = odomAftPGOthis.header.stamp;
        pathAftPGO.header.frame_id = "camera_init";
        pathAftPGO.poses.push_back(poseStampAftPGO);
    }
    loopDetector_->getKFMutex().unlock();

    pubOdomAftPGO_->publish(odomAftPGO);
    pubPathAftPGO_->publish(pathAftPGO);

    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = odomAftPGO.header.stamp;
    transformStamped.header.frame_id = "camera_init";
    transformStamped.child_frame_id = "aft_pgo";
    transformStamped.transform.translation.x = odomAftPGO.pose.pose.position.x;
    transformStamped.transform.translation.y = odomAftPGO.pose.pose.position.y;
    transformStamped.transform.translation.z = odomAftPGO.pose.pose.position.z;
    transformStamped.transform.rotation = odomAftPGO.pose.pose.orientation;
    tfBroadcaster_->sendTransform(transformStamped);

    if (hasAnchored_) {
        geometry_msgs::msg::TransformStamped mapTf;
        mapTf.header.stamp = odomAftPGO.header.stamp;
        mapTf.header.frame_id = "map";
        mapTf.child_frame_id = "camera_init";
        auto q = T_map_to_init_.rotation().toQuaternion();
        mapTf.transform.translation.x = T_map_to_init_.translation().x();
        mapTf.transform.translation.y = T_map_to_init_.translation().y();
        mapTf.transform.translation.z = T_map_to_init_.translation().z();
        mapTf.transform.rotation.x = q.x();
        mapTf.transform.rotation.y = q.y();
        mapTf.transform.rotation.z = q.z();
        mapTf.transform.rotation.w = q.w();
        tfBroadcaster_->sendTransform(mapTf);
    }
}


void LidarLoopNode::updatePoses(const gtsam::Values& isamCurrentEstimate) {
    auto& keyframePosesUpdated = loopDetector_->getKeyframePosesUpdated();
    loopDetector_->getKFMutex().lock();
    int num_poses = keyframePosesUpdated.size();
    for (int node_idx = 0; node_idx < int(isamCurrentEstimate.size()) && node_idx < num_poses; node_idx++) {
        Pose6D& p = keyframePosesUpdated[node_idx];
        p.x = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().x();
        p.y = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().y();
        p.z = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().z();
        p.roll = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().roll();
        p.pitch = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().pitch();
        p.yaw = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().yaw();
    }
    loopDetector_->getKFMutex().unlock();

    mtxRecentPose_.lock();
    if (!isamCurrentEstimate.empty()) {
        const gtsam::Pose3& lastOptimizedPose =
            isamCurrentEstimate.at<gtsam::Pose3>(int(isamCurrentEstimate.size()) - 1);
        recentOptimizedX_ = lastOptimizedPose.translation().x();
        recentOptimizedY_ = lastOptimizedPose.translation().y();
    }
    recentIdxUpdated_ = int(keyframePosesUpdated.size()) - 1;
    mtxRecentPose_.unlock();
}

void LidarLoopNode::process_pg() {
    while (rclcpp::ok() && !stopThreads_) {
        while (!odometryBuf_.empty() && !fullResBuf_.empty()) {
            mBuf_.lock();
            while (!odometryBuf_.empty() && rclcpp::Time(odometryBuf_.front()->header.stamp).seconds() <
                                                 rclcpp::Time(fullResBuf_.front()->header.stamp).seconds()) {
                odometryBuf_.pop();
                ++perf_->at(0).dropped;
            }
            perf_->at(0).queued = odometryBuf_.size();

            if (odometryBuf_.empty()) {
                mBuf_.unlock();
                break;
            }

            timeLaserOdometry_ = rclcpp::Time(odometryBuf_.front()->header.stamp).seconds();
            perf_->at(2).receive();
            runtime_diag::Scope scope(perf_->at(2));
            timeLaser_ = rclcpp::Time(fullResBuf_.front()->header.stamp).seconds();

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::fromROSMsg(*fullResBuf_.front(), *tempCloudRGB);

            Pose6D pose_curr = getOdom(odometryBuf_.front());
            odometryBuf_.pop();
            perf_->at(0).finish();
            perf_->at(0).queued = odometryBuf_.size();

            Eigen::Affine3f t_body_to_world = Pose6dToAffine3f(pose_curr);
            Eigen::Affine3f t_world_to_body = t_body_to_world.inverse();
            pcl::transformPointCloud(*tempCloudRGB, *tempCloudRGB, t_world_to_body);

            pcl::PointCloud<PointType>::Ptr thisKeyFrame(new pcl::PointCloud<PointType>());
            RGB2XYZI(tempCloudRGB, thisKeyFrame);

            fullResBuf_.pop();
            perf_->at(1).finish();
            perf_->at(1).queued = fullResBuf_.size();

            double eps = 0.1;
            while (!gpsBuf_.empty()) {
                auto thisGPS = gpsBuf_.front();
                auto thisGPSTime = rclcpp::Time(thisGPS->header.stamp).seconds();
                if (std::abs(thisGPSTime - timeLaserOdometry_) < eps) {
                    currGPS_ = thisGPS;
                    hasGPSforThisKF_ = true;
                    break;
                } else {
                    hasGPSforThisKF_ = false;
                }
                gpsBuf_.pop();
            }
            mBuf_.unlock();

            if (thisKeyFrame->empty()) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "Skipping synchronized empty /cloud_registered frame at %.9f.", timeLaser_);
                continue;
            }

            odom_pose_prev_ = odom_pose_curr_;
            odom_pose_curr_ = pose_curr;
            Pose6D dtf = diffTransformation(odom_pose_prev_, odom_pose_curr_);

            double delta_translation = sqrt(dtf.x * dtf.x + dtf.y * dtf.y + dtf.z * dtf.z);
            translationAccumulated_ += delta_translation;
            rotaionAccumulated_ += (dtf.roll + dtf.pitch + dtf.yaw);

            if (translationAccumulated_ > keyframeMeterGap_ || rotaionAccumulated_ > keyframeRadGap_) {
                isNowKeyFrame_ = true;
                translationAccumulated_ = 0.0;
                rotaionAccumulated_ = 0.0;
            } else {
                isNowKeyFrame_ = false;
            }

            if (!isNowKeyFrame_) continue;

            if (!gpsOffsetInitialized_) {
                if (hasGPSforThisKF_) {
                    gpsAltitudeInitOffset_ = currGPS_->altitude;
                    gpsOffsetInitialized_ = true;
                }
            }

            pcl::PointCloud<PointType>::Ptr thisKeyFrameDS(new pcl::PointCloud<PointType>());
            downSizeFilterScancontext_.setInputCloud(thisKeyFrame);
            downSizeFilterScancontext_.filter(*thisKeyFrameDS);

            pcl::PointCloud<PointTypeRGB>::Ptr thisKeyFrameMapDSRGB(new pcl::PointCloud<PointTypeRGB>());
            pcl::VoxelGrid<PointTypeRGB> filterForMapRGB;
            filterForMapRGB.setLeafSize(mapVizFilterSize_, mapVizFilterSize_, mapVizFilterSize_);
            filterForMapRGB.setInputCloud(tempCloudRGB);
            filterForMapRGB.filter(*thisKeyFrameMapDSRGB);

            // 录入关键帧
            loopDetector_->addKeyframe(thisKeyFrameMapDSRGB, pose_curr, timeLaserOdometry_, *thisKeyFrameDS);

            std_msgs::msg::Header keyFrameHeader;
            keyFrameHeader.stamp = this->now();
            pubKeyFramesId_->publish(keyFrameHeader);

            auto& keyframePoses = loopDetector_->getKeyframePoses();
            const int prev_node_idx = keyframePoses.size() - 2;
            const int curr_node_idx = keyframePoses.size() - 1;

            if (!factorGraphManager_->isGraphMade()) {
                const int init_node_idx = 0;
                gtsam::Pose3 poseOrigin = Pose6DtoGTSAMPose3(keyframePoses.at(init_node_idx));
                factorGraphManager_->addPriorFactor(init_node_idx, poseOrigin);
                nodesAddedToGraph_.store(init_node_idx + 1);
                RCLCPP_DEBUG(this->get_logger(), "Prior factor added to pose graph at node %d.",
                            init_node_idx);
            } else {
                gtsam::Pose3 poseFrom = Pose6DtoGTSAMPose3(keyframePoses.at(prev_node_idx));
                gtsam::Pose3 poseTo = Pose6DtoGTSAMPose3(keyframePoses.at(curr_node_idx));

                factorGraphManager_->addOdomFactor(prev_node_idx, curr_node_idx, poseFrom.between(poseTo), poseTo);

                if (hasGPSforThisKF_) {
                    double curr_altitude_offseted = currGPS_->altitude - gpsAltitudeInitOffset_;
                    mtxRecentPose_.lock();
                    gtsam::Point3 gpsConstraint(recentOptimizedX_, recentOptimizedY_, curr_altitude_offseted);
                    mtxRecentPose_.unlock();
                    factorGraphManager_->addGPSFactor(curr_node_idx, gpsConstraint);
                    RCLCPP_DEBUG(this->get_logger(),
                                "GPS prior factor added to pose graph at node %d.",
                                curr_node_idx);
                }

                if (curr_node_idx % 100 == 0) {
                    RCLCPP_DEBUG(this->get_logger(),
                                "Keyframe odometry factors successfully inserted (Last node: %d).",
                                curr_node_idx);
                }
                nodesAddedToGraph_.store(curr_node_idx + 1);
            }

            std::string curr_node_idx_str = padZeros(curr_node_idx + keyframeIdOffset_);
            pcl::io::savePCDFileBinary(pgScansDirectory_ + curr_node_idx_str + ".pcd", *thisKeyFrame);
            pgTimeSaveStream_ << timeLaser_ << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

void LidarLoopNode::process_lcd() {
    auto rate_duration = std::chrono::duration<double>(1.0 / loopClosureFrequency_);
    while (rclcpp::ok() && !stopThreads_) {
        std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::milliseconds>(rate_duration));
        loopDetector_->performSCLoopClosure();
    }
}

void LidarLoopNode::process_icp() {
    while (rclcpp::ok() && !stopThreads_) {
        while (true) {
            std::pair<int, int> loop_idx_pair;
            bool has_data = false;
            
            loopDetector_->getBufMutex().lock();
            auto& queue = loopDetector_->getScLoopICPBuf();
            perf_->at(3).queued = queue.size();
            if (!queue.empty()) {
                if (queue.size() > MAX_ICP_QUEUE_SIZE_) {
                    RCLCPP_WARN(this->get_logger(),
                                "Heavy backlog in loop closure ICP queue (Queue size: %lu). Consider tuning loop closure parameters.",
                                queue.size());
                }
                loop_idx_pair = queue.front();
                
                // If visual loop candidate arrived before LiDAR keyframe is fully inserted into GTSAM, wait.
                int nodes_in_graph = nodesAddedToGraph_.load();
                if (loop_idx_pair.first >= nodes_in_graph || loop_idx_pair.second >= nodes_in_graph) {
                    loopDetector_->getBufMutex().unlock();
                    break;
                }
                
                queue.pop();
                perf_->at(3).queued = queue.size();
                has_data = true;
            }
            loopDetector_->getBufMutex().unlock();

            if (!has_data) break;

            const int prev_node_idx = loop_idx_pair.first;
            perf_->at(3).receive();
            runtime_diag::Scope scope(perf_->at(3));
            const int curr_node_idx = loop_idx_pair.second;
            auto relative_pose = loopDetector_->doICPVirtualRelative(
                prev_node_idx, curr_node_idx,
                pubLoopScanLocal_, pubLoopSubmapLocal_, pubLoopScanLocalRegisted_);

            if (!relative_pose.equals(gtsam::Pose3::identity())) {
                try {
                    factorGraphManager_->addLoopFactor(curr_node_idx, prev_node_idx, relative_pose);
                    mVisualPGO_.lock();
                    closedLoopsByLidar_.insert(std::make_pair(prev_node_idx, curr_node_idx));
                    closedLoopsByLidar_.insert(std::make_pair(curr_node_idx, prev_node_idx));
                    mVisualPGO_.unlock();
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to add loop factor: %s", e.what());
                }
            }
        }

        // 处理纯视觉兜底约束
        mVisualPGO_.lock();
        int nodes_in_graph_v = nodesAddedToGraph_.load();
        while (!visualPGOQueue_.empty()) {
            auto msg = visualPGOQueue_.front();
            if (msg.data.size() < 8) {
                visualPGOQueue_.pop();
                continue;
            }
            int prev_idx = (int)msg.data[0];
            int curr_idx = (int)msg.data[1];
            if (curr_idx >= nodes_in_graph_v || prev_idx >= nodes_in_graph_v) {
                break; // Wait until node is inserted
            }
            visualPGOQueue_.pop();

            // 如果激光雷达已成功匹配闭环该帧对，则跳过纯视觉兜底，避免与高精度雷达位姿冲突
            if (closedLoopsByLidar_.count(std::make_pair(prev_idx, curr_idx)) ||
                closedLoopsByLidar_.count(std::make_pair(curr_idx, prev_idx))) {
                RCLCPP_INFO(this->get_logger(),
                            "Visual PGO skipped between %d and %d: already closed by LiDAR ICP.",
                            prev_idx, curr_idx);
                continue;
            }

            // 提取前端里程计中的两帧相对平移距离，作为单目尺度标尺 (加锁与边界校验)
            Pose6D p_prev, p_curr;
            {
                std::lock_guard<std::mutex> lock_kf(loopDetector_->getKFMutex());
                const auto& keyframePoses = loopDetector_->getKeyframePoses();
                if (prev_idx < 0 || prev_idx >= (int)keyframePoses.size() ||
                    curr_idx < 0 || curr_idx >= (int)keyframePoses.size()) {
                    RCLCPP_WARN(this->get_logger(), "Visual PGO rejected: keyframe index out of bounds (%d, %d, size: %zu)",
                                prev_idx, curr_idx, keyframePoses.size());
                    continue;
                }
                p_prev = keyframePoses[prev_idx];
                p_curr = keyframePoses[curr_idx];
            }

            double dx = p_curr.x - p_prev.x;
            double dy = p_curr.y - p_prev.y;
            double dz = p_curr.z - p_prev.z;
            double scale = std::sqrt(dx*dx + dy*dy + dz*dz);
            if (!std::isfinite(scale) || scale < 0.5 || scale > historyKeyframeSearchRadius_) {
                RCLCPP_WARN(this->get_logger(),
                            "Visual PGO rejected between %d and %d: odometry translation out of bounds (%.2fm, valid: 0.5m ~ %.2fm)",
                            prev_idx, curr_idx, scale, historyKeyframeSearchRadius_);
                continue;
            }

            // 构造相对位姿并校验数值有效性（msg.data 中的向量已由 visual_loop_node 转至雷达坐标系）
            double t_x = msg.data[2] * scale;
            double t_y = msg.data[3] * scale;
            double t_z = msg.data[4] * scale;
            // 地面机器人平地保护：限制相对 Z 轴位移不超过 0.15m，防止垂直拉扯
            if (std::abs(t_z) > 0.15) {
                t_z = (t_z > 0) ? 0.15 : -0.15;
            }
            double r_x = msg.data[5];
            double r_y = msg.data[6];
            double r_z = msg.data[7];

            if (!std::isfinite(t_x) || !std::isfinite(t_y) || !std::isfinite(t_z) ||
                !std::isfinite(r_x) || !std::isfinite(r_y) || !std::isfinite(r_z)) {
                RCLCPP_WARN(this->get_logger(), "Visual PGO rejected: invalid non-finite pose components");
                continue;
            }

            try {
                gtsam::Pose3 visual_pose(gtsam::Rot3::Rodrigues(r_x, r_y, r_z), gtsam::Point3(t_x, t_y, t_z));
                factorGraphManager_->addVisualLoopFactor(curr_idx, prev_idx, visual_pose);
                RCLCPP_INFO(this->get_logger(), "Visual PGO constraint added between %d and %d. Scale: %.2fm", prev_idx, curr_idx, scale);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to add visual loop factor: %s", e.what());
            }
        }
        mVisualPGO_.unlock();

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

void LidarLoopNode::process_isam() {
    auto rate_duration = std::chrono::duration<double>(1.0 / graphUpdateFrequency_);
    while (rclcpp::ok() && !stopThreads_) {
        std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::milliseconds>(rate_duration));
        if (factorGraphManager_->isGraphMade()) {
            // RCLCPP_INFO(this->get_logger(),
            //             "ISAM2 optimization triggered. Updating global trajectory.");
            
            gtsam::Values currentEstimate;
            factorGraphManager_->runOptimization(graphUpdateTimes_, currentEstimate);
            
            updatePoses(currentEstimate);
            pubPath();

            saveOptimizedVerticesKITTIformat(currentEstimate, pgKITTIformat_);
            saveOdometryVerticesKITTIformat(odomKITTIformat_);
        }
    }
}

void LidarLoopNode::process_cross_session() {
    auto rate_duration = std::chrono::milliseconds(1000); // 1Hz 检查跨会话回环
    int last_checked_idx = -1;

    while (rclcpp::ok() && !stopThreads_) {
        std::this_thread::sleep_for(rate_duration);
        if (!isIncrementalMode_) continue;

        int curr_kf_idx = -1;
        {
            std::lock_guard<std::mutex> lock(loopDetector_->getKFMutex());
            if (!loopDetector_->getKeyframePoses().empty()) {
                curr_kf_idx = int(loopDetector_->getKeyframePoses().size()) - 1;
            }
        }
        if (curr_kf_idx < 0 || curr_kf_idx == last_checked_idx) continue;
        last_checked_idx = curr_kf_idx;

        int hist_idx = -1;
        float rel_yaw = 0.0f;
        if (loopDetector_->detectCrossSessionLoop(curr_kf_idx, &hist_idx, &rel_yaw)) {
            RCLCPP_INFO(this->get_logger(),
                        "[CrossSession] Cross-session loop candidate proposed: Current KF %d <-> History KF %d (Estimated yaw diff: %.2f deg). Running ICP...",
                        curr_kf_idx + keyframeIdOffset_, hist_idx, rel_yaw * 180.0 / M_PI);

            float icp_score = 999.0f;
            gtsam::Pose3 pose_curr_in_map = loopDetector_->doCrossSessionICP(
                hist_idx, curr_kf_idx, icp_score,
                pubLoopScanLocal_, pubLoopSubmapLocal_, pubLoopScanLocalRegisted_);

            if (!pose_curr_in_map.equals(gtsam::Pose3::identity())) {
                std::lock_guard<std::mutex> lock(mAnchor_);
                Pose6D curr_local_pose;
                {
                    std::lock_guard<std::mutex> lock_kf(loopDetector_->getKFMutex());
                    curr_local_pose = loopDetector_->getKeyframePosesUpdated()[curr_kf_idx];
                }
                gtsam::Pose3 pose_init_to_curr = Pose6DtoGTSAMPose3(curr_local_pose);

                if (!hasAnchored_) {
                    gtsam::Pose3 candidate_T_init = pose_curr_in_map * pose_init_to_curr.inverse();

                    if (relocCandidates_.empty()) {
                        relocCandidates_.push_back(candidate_T_init);
                        RCLCPP_INFO(this->get_logger(),
                                    "[CrossSession] Reloc verification stage 1/3: Candidate T_init=[%.2f, %.2f, %.2f] (ICP Score: %.4f)",
                                    candidate_T_init.translation().x(), candidate_T_init.translation().y(), candidate_T_init.translation().z(), icp_score);
                    } else {
                        gtsam::Pose3 prev_guess = relocCandidates_.back();
                        double trans_diff = (candidate_T_init.translation() - prev_guess.translation()).norm();
                        double rot_diff = prev_guess.rotation().between(candidate_T_init.rotation()).axisAngle().second;

                        if (trans_diff < 0.35 && rot_diff < (10.0 * M_PI / 180.0)) {
                            relocCandidates_.push_back(candidate_T_init);
                            RCLCPP_INFO(this->get_logger(),
                                        "[CrossSession] Reloc verification stage %zu/3: Consecutive consistency validated (delta_trans: %.3fm, delta_rot: %.2f deg)",
                                        relocCandidates_.size(), trans_diff, rot_diff * 180.0 / M_PI);
                        } else {
                            RCLCPP_WARN(this->get_logger(),
                                        "[CrossSession] Reloc verification failed: Spatial inconsistency (delta_trans: %.3fm, delta_rot: %.2f deg). Resetting verification window.",
                                        trans_diff, rot_diff * 180.0 / M_PI);
                            relocCandidates_.clear();
                            relocCandidates_.push_back(candidate_T_init);
                        }
                    }

                    // 达到连续 3 帧严格时空一致性，正式锁定并锚定！
                    if (relocCandidates_.size() >= 3) {
                        T_map_to_init_ = relocCandidates_.back();
                        hasAnchored_ = true;

                        // 记录全局锚定因子至因子拓扑表
                        factorGraphManager_->addAnchorFactor(keyframeIdOffset_, T_map_to_init_);

                        // 广播全局静态 TF: map -> camera_init
                        geometry_msgs::msg::TransformStamped mapTf;
                        mapTf.header.stamp = this->now();
                        mapTf.header.frame_id = "map";
                        mapTf.child_frame_id = "camera_init";
                        auto q = T_map_to_init_.rotation().toQuaternion();
                        mapTf.transform.translation.x = T_map_to_init_.translation().x();
                        mapTf.transform.translation.y = T_map_to_init_.translation().y();
                        mapTf.transform.translation.z = T_map_to_init_.translation().z();
                        mapTf.transform.rotation.x = q.x();
                        mapTf.transform.rotation.y = q.y();
                        mapTf.transform.rotation.z = q.z();
                        mapTf.transform.rotation.w = q.w();
                        tfStaticBroadcaster_->sendTransform(mapTf);

                        RCLCPP_INFO(this->get_logger(),
                                    "==================================================================");
                        RCLCPP_INFO(this->get_logger(),
                                    "[CrossSession] 3-Frame consistency passed! Relocalized and locked T_map_to_init=[%.3f, %.3f, %.3f] (ICP Score: %.4f)",
                                    T_map_to_init_.translation().x(),
                                    T_map_to_init_.translation().y(),
                                    T_map_to_init_.translation().z(),
                                    icp_score);
                        RCLCPP_INFO(this->get_logger(),
                                    "[CrossSession] Coordinate anchor established. Switching to standard unified mapping pipeline.");
                        RCLCPP_INFO(this->get_logger(),
                                    "==================================================================");

                        // 🟠 Trigger orange historical path loading (non-blocking)
                        loadAndPublishHistoricalPath();

                        RCLCPP_INFO(this->get_logger(),
                                    "[CrossSession] Initial anchor complete! Thread continues monitoring for subsequent cross-session loops to eliminate drift.");
                    }
                } else {
                    // 已锚定状态：机器人再次经过历史路线时，触发跨 Session 闭环拉正！
                    // pose_curr_in_map: 历史点云 ICP 求解出的当前帧在 map 坐标系下的期望绝对位姿
                    // 转换到当前会话的 camera_init 坐标系: T_target_in_init = T_init_to_map * pose_curr_in_map
                    gtsam::Pose3 T_init_to_map = T_map_to_init_.inverse();
                    gtsam::Pose3 target_in_init = T_init_to_map * pose_curr_in_map;

                    double delta_pos = (target_in_init.translation() - pose_init_to_curr.translation()).norm();
                    factorGraphManager_->addCrossSessionAnchorFactor(curr_kf_idx, target_in_init, icp_score);
                    factorGraphManager_->addCrossSessionLoopFactor(
                        curr_kf_idx + keyframeIdOffset_, hist_idx, pose_curr_in_map, icp_score);
                    RCLCPP_INFO(this->get_logger(),
                                "[CrossSession] 🔗 Cross-session loop closure accepted! Curr KF %d pulled by Hist KF %d (Delta: %.3fm, ICP score: %.4f).",
                                curr_kf_idx + keyframeIdOffset_, hist_idx, delta_pos, icp_score);
                }
            }
        }
    }
}

void LidarLoopNode::publishHistoricalMap() {
    if (!isIncrementalMode_) return;
    std::string map_pcd_path = saveDirectory_ + "map_3d.pcd";
    if (!std::filesystem::exists(map_pcd_path)) return;

    historicalMapThread_ = std::thread([this, map_pcd_path]() {
        pcl::PointCloud<PointTypeRGB>::Ptr historical_cloud(new pcl::PointCloud<PointTypeRGB>());
        if (pcl::io::loadPCDFile(map_pcd_path, *historical_cloud) != 0 || historical_cloud->empty()) return;

        pcl::VoxelGrid<PointTypeRGB> ds;
        ds.setLeafSize(0.2f, 0.2f, 0.2f);
        pcl::PointCloud<PointTypeRGB>::Ptr ds_cloud(new pcl::PointCloud<PointTypeRGB>());
        ds.setInputCloud(historical_cloud);
        ds.filter(*ds_cloud);

        // 等待定位成功锁定
        while (rclcpp::ok() && !stopThreads_ && !hasAnchored_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        if (!hasAnchored_ || stopThreads_) return;

        // 获取 T_init_to_map = (T_map_to_init)^-1，将历史点云直接变换到当前会话 B 的 camera_init 坐标系
        gtsam::Pose3 T_init_to_map = T_map_to_init_.inverse();
        Eigen::Matrix4f trans_mat = T_init_to_map.matrix().cast<float>();

        pcl::PointCloud<PointTypeRGB>::Ptr transformed_hist_cloud(new pcl::PointCloud<PointTypeRGB>());
        pcl::transformPointCloud(*ds_cloud, *transformed_hist_cloud, trans_mat);

        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*transformed_hist_cloud, msg);
        msg.header.frame_id = "camera_init";

        RCLCPP_INFO(this->get_logger(),
                    "[LidarLoopNode] Historical RGB base map transformed to current session frame 'camera_init' and streaming to /map_historical.");

        while (rclcpp::ok() && !stopThreads_) {
            msg.header.stamp = this->now();
            pubMapHistorical_->publish(msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    });
}

void LidarLoopNode::loadHistoricalPosesInMapFrame(const std::string& poses_path) {
    histPosesInMapFrame_.clear();
    if (!std::filesystem::exists(poses_path)) {
        RCLCPP_WARN(this->get_logger(), "[HistPoses] Historical poses file not found at %s", poses_path.c_str());
        return;
    }

    std::ifstream fin(poses_path);
    if (!fin.is_open()) return;

    std::string line;
    while (std::getline(fin, line)) {
        std::istringstream ss(line);
        float r00, r01, r02, tx, r10, r11, r12, ty, r20, r21, r22, tz;
        if (!(ss >> r00 >> r01 >> r02 >> tx >> r10 >> r11 >> r12 >> ty >> r20 >> r21 >> r22 >> tz)) continue;

        gtsam::Pose3 pose_in_map_gtsam(gtsam::Rot3(r00, r01, r02, r10, r11, r12, r20, r21, r22),
                                      gtsam::Point3(tx, ty, tz));
        histPosesInMapFrame_.push_back(pose_in_map_gtsam);
    }
    fin.close();

    RCLCPP_INFO(this->get_logger(),
                "[HistPoses] 🟠 Preloaded %zu historical poses in map frame from %s.",
                histPosesInMapFrame_.size(), poses_path.c_str());
}

void LidarLoopNode::loadAndPublishHistoricalPath() {
    if (histPosesInMapFrame_.empty()) {
        std::string poses_path = saveDirectory_ + "optimized_poses_hist.txt";
        loadHistoricalPosesInMapFrame(poses_path);
    }
    if (histPosesInMapFrame_.empty()) {
        RCLCPP_WARN(this->get_logger(), "[HistPath] No historical poses available to publish.");
        return;
    }

    // T_init_to_map: transforms a point from map frame -> camera_init frame
    gtsam::Pose3 T_init_to_map_gtsam = T_map_to_init_.inverse();
    Eigen::Matrix4f T_init_to_map = T_init_to_map_gtsam.matrix().cast<float>();

    nav_msgs::msg::Path orange_path;
    orange_path.header.frame_id = "camera_init";
    orange_path.header.stamp = this->now();
    histPosesInInitFrame_.clear();

    for (const auto& pose_in_map_gtsam : histPosesInMapFrame_) {
        Eigen::Matrix4f T_pose_in_map = pose_in_map_gtsam.matrix().cast<float>();
        Eigen::Matrix4f T_in_init = T_init_to_map * T_pose_in_map;

        Eigen::Affine3f aff(T_in_init);
        float px, py, pz, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(aff, px, py, pz, roll, pitch, yaw);

        Pose6D hp;
        hp.x = px; hp.y = py; hp.z = pz;
        hp.roll = roll; hp.pitch = pitch; hp.yaw = yaw;
        histPosesInInitFrame_.push_back(hp);

        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id = "camera_init";
        ps.header.stamp = this->now();
        ps.pose.position.x = px;
        ps.pose.position.y = py;
        ps.pose.position.z = pz;
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        ps.pose.orientation.x = q.x();
        ps.pose.orientation.y = q.y();
        ps.pose.orientation.z = q.z();
        ps.pose.orientation.w = q.w();
        orange_path.poses.push_back(ps);
    }

    historicalPathLoaded_ = true;
    std::ofstream(saveDirectory_ + ".cumulative_anchored") << "anchored\n";

    // Publish once via Transient Local so RViz receives it regardless of when it subscribes
    pubPathHistorical_->publish(orange_path);
    RCLCPP_INFO(this->get_logger(),
                "[HistPath] 🟠 Historical trajectory published: %zu poses -> /path_historical (orange).",
                histPosesInInitFrame_.size());
}


void LidarLoopNode::relocPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    if (!isIncrementalMode_ || hasAnchored_) return;

    std::lock_guard<std::mutex> lock(mAnchor_);
    if (hasAnchored_) return;

    auto p = msg->pose.pose.position;
    auto q = msg->pose.pose.orientation;
    gtsam::Pose3 pose_curr_in_map(gtsam::Rot3(q.w, q.x, q.y, q.z), gtsam::Point3(p.x, p.y, p.z));

    Pose6D curr_local_pose;
    {
        std::lock_guard<std::mutex> lock_kf(loopDetector_->getKFMutex());
        if (loopDetector_->getKeyframePosesUpdated().empty()) return;
        curr_local_pose = loopDetector_->getKeyframePosesUpdated().back();
    }
    gtsam::Pose3 pose_init_to_curr = Pose6DtoGTSAMPose3(curr_local_pose);

    T_map_to_init_ = pose_curr_in_map * pose_init_to_curr.inverse();
    hasAnchored_ = true;

    factorGraphManager_->addAnchorFactor(keyframeIdOffset_, T_map_to_init_);

    // 广播全局静态 TF: map -> camera_init
    geometry_msgs::msg::TransformStamped mapTf;
    mapTf.header.stamp = this->now();
    mapTf.header.frame_id = "map";
    mapTf.child_frame_id = "camera_init";
    auto rot_q = T_map_to_init_.rotation().toQuaternion();
    mapTf.transform.translation.x = T_map_to_init_.translation().x();
    mapTf.transform.translation.y = T_map_to_init_.translation().y();
    mapTf.transform.translation.z = T_map_to_init_.translation().z();
    mapTf.transform.rotation.x = rot_q.x();
    mapTf.transform.rotation.y = rot_q.y();
    mapTf.transform.rotation.z = rot_q.z();
    mapTf.transform.rotation.w = rot_q.w();
    tfStaticBroadcaster_->sendTransform(mapTf);

    RCLCPP_INFO(this->get_logger(),
                "==================================================================");
    RCLCPP_INFO(this->get_logger(),
                "[CrossSession] Received external relocation pose! Anchored T_map_to_init=[%.3f, %.3f, %.3f]",
                T_map_to_init_.translation().x(),
                T_map_to_init_.translation().y(),
                T_map_to_init_.translation().z());
    RCLCPP_INFO(this->get_logger(),
                "==================================================================");

    loadAndPublishHistoricalPath();
}

void LidarLoopNode::mapToOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!isIncrementalMode_ || hasAnchored_) return;

    std::lock_guard<std::mutex> lock(mAnchor_);
    if (hasAnchored_) return;

    auto p = msg->pose.pose.position;
    auto q = msg->pose.pose.orientation;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return;

    T_map_to_init_ = gtsam::Pose3(gtsam::Rot3(q.w, q.x, q.y, q.z), gtsam::Point3(p.x, p.y, p.z));
    hasAnchored_ = true;

    factorGraphManager_->addAnchorFactor(keyframeIdOffset_, T_map_to_init_);

    // 广播全局静态 TF: map -> camera_init
    geometry_msgs::msg::TransformStamped mapTf;
    mapTf.header.stamp = this->now();
    mapTf.header.frame_id = "map";
    mapTf.child_frame_id = "camera_init";
    mapTf.transform.translation.x = p.x;
    mapTf.transform.translation.y = p.y;
    mapTf.transform.translation.z = p.z;
    mapTf.transform.rotation = q;
    tfStaticBroadcaster_->sendTransform(mapTf);

    RCLCPP_INFO(this->get_logger(),
                "==================================================================");
    RCLCPP_INFO(this->get_logger(),
                "[Localization] Native localization locked pose! Anchored T_map_to_init=[%.3f, %.3f, %.3f]",
                p.x, p.y, p.z);
    RCLCPP_INFO(this->get_logger(),
                "[Localization] Coordinate anchor established. Switching to standard unified mapping pipeline.");
    RCLCPP_INFO(this->get_logger(),
                "==================================================================");

    loadAndPublishHistoricalPath();
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarLoopNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
