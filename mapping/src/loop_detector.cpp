#include "mapping/loop_detector.h"
#include <fstream>
#include <iomanip>
#include <sstream>
#include <fast_gicp/gicp/fast_vgicp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>

namespace {
std::string padZeros(int val, int num_digits = 6) {
    std::stringstream ss;
    ss << std::setfill('0') << std::setw(num_digits) << val;
    return ss.str();
}
}

LoopDetector::LoopDetector(rclcpp::Node* node) : node_(node), kdtreeHistoryKeyPoses_(new pcl::KdTreeFLANN<pcl::PointXYZ>()) {
    double scDistThres = node_->get_parameter("sc_dist_thres").as_double();
    double scMaximumRadius = node_->get_parameter("sc_max_radius").as_double();
    scManager_.setSCdistThres(scDistThres);
    scManager_.setMaximumRadius(scMaximumRadius);

    historyKeyframeSearchRadius_ = node_->get_parameter("historyKeyframeSearchRadius").as_double();
    historyKeyframeSearchTimeDiff_ = node_->get_parameter("historyKeyframeSearchTimeDiff").as_double();
    historyKeyframeSearchNum_ = node_->get_parameter("historyKeyframeSearchNum").as_int();
    loopFitnessScoreThreshold_ = node_->get_parameter("loopFitnessScoreThreshold").as_double();

    double historyCloudDownsample = 0.4;
    downSizeFilterICP_.setLeafSize(historyCloudDownsample, historyCloudDownsample, historyCloudDownsample);
}

void LoopDetector::addKeyframe(const pcl::PointCloud<PointTypeRGB>::Ptr& cloud, const Pose6D& odom_pose, double time, pcl::PointCloud<PointType>& cloud_ds) {
    std::lock_guard<std::mutex> lock(mKF_);
    keyframeLaserClouds_.push_back(cloud);
    keyframePoses_.push_back(odom_pose);
    keyframePosesUpdated_.push_back(odom_pose);
    keyframeTimes_.push_back(time);
    scManager_.makeAndSaveScancontextAndKeys(cloud_ds);
}

void LoopDetector::addVisualLoopCandidate(int prev_node_idx, int curr_node_idx) {
    std::lock_guard<std::mutex> lock(mBuf_);
    auto it = loopIndexContainer_.find(curr_node_idx);
    if (it == loopIndexContainer_.end()) {
        RCLCPP_DEBUG(node_->get_logger(),
                    "Visual loop candidate received in backend. History ID: %d, Current ID: %d.",
                    prev_node_idx, curr_node_idx);
        scLoopICPBuf_.push(std::make_pair(prev_node_idx, curr_node_idx));
    }
}

void LoopDetector::saveSCDatabase(const std::string& filename,
                                  bool is_incremental,
                                  bool has_anchored,
                                  const Eigen::Matrix4f& T_map_to_init) {
    std::ofstream ofs(filename);
    if (!ofs.is_open()) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Failed to open ScanContext database file for saving.");
        return;
    }

    std::lock_guard<std::mutex> lock(mKF_);
    int num_frames_curr = keyframePosesUpdated_.size();
    int sc_size_curr = scManager_.polarcontexts_.size();
    int save_num_curr = std::min(num_frames_curr, sc_size_curr);

    bool can_merge = is_incremental && has_anchored && !historyKeyframePoses_.empty();

    int hist_num = can_merge ? std::min(int(historyKeyframePoses_.size()), int(historyScManager_.polarcontexts_.size())) : 0;
    int total_frames = hist_num + save_num_curr;

    ofs << total_frames << "\n";

    // 1. 如果支持增量合并，先输出历史 A 的全部帧 (已在 O_A 世界参考系下)
    if (can_merge) {
        for (int i = 0; i < hist_num; ++i) {
            Pose6D p = historyKeyframePoses_[i];
            ofs << i << " " << p.x << " " << p.y << " " << p.z << " " << p.roll << " " << p.pitch << " " << p.yaw << "\n";

            Eigen::MatrixXd sc = historyScManager_.polarcontexts_[i];
            ofs << sc.rows() << " " << sc.cols() << "\n";
            for (int r = 0; r < sc.rows(); ++r) {
                for (int c = 0; c < sc.cols(); ++c) {
                    ofs << sc(r, c) << " ";
                }
                ofs << "\n";
            }
        }
        RCLCPP_INFO(node_->get_logger(),
                    "[LoopDetector] ScanContext database: merged %d historical frames into library.", hist_num);
    }

    // 2. 输出当前会话 B 的全部帧 (ID 顺延，位姿通过 T_map_to_init 统一转换至 O_A 世界系)
    for (int i = 0; i < save_num_curr; ++i) {
        int global_id = hist_num + i;
        Pose6D p_global = keyframePosesUpdated_[i];

        if (can_merge) {
            Eigen::Affine3f trans_local = pcl::getTransformation(
                p_global.x, p_global.y, p_global.z,
                p_global.roll, p_global.pitch, p_global.yaw
            );
            Eigen::Matrix4f trans_global = T_map_to_init * trans_local.matrix();
            Eigen::Affine3f aff(trans_global);
            float x, y, z, roll, pitch, yaw;
            pcl::getTranslationAndEulerAngles(aff, x, y, z, roll, pitch, yaw);
            p_global.x = x; p_global.y = y; p_global.z = z;
            p_global.roll = roll; p_global.pitch = pitch; p_global.yaw = yaw;
        }

        ofs << global_id << " " << p_global.x << " " << p_global.y << " " << p_global.z << " "
            << p_global.roll << " " << p_global.pitch << " " << p_global.yaw << "\n";

        Eigen::MatrixXd sc = scManager_.polarcontexts_[i];
        ofs << sc.rows() << " " << sc.cols() << "\n";
        for (int r = 0; r < sc.rows(); ++r) {
            for (int c = 0; c < sc.cols(); ++c) {
                ofs << sc(r, c) << " ";
            }
            ofs << "\n";
        }
    }
    ofs.close();

    if (can_merge) {
        RCLCPP_INFO(node_->get_logger(),
                    "ScanContext database saved successfully (Total: %d frames, Historical: %d, Added: %d).",
                    total_frames, hist_num, save_num_curr);
    } else {
        RCLCPP_INFO(node_->get_logger(),
                    "ScanContext database saved successfully (Frames count: %d).",
                    total_frames);
    }
}

void LoopDetector::RGB2XYZI(const pcl::PointCloud<PointTypeRGB>::Ptr& cloudIn, pcl::PointCloud<PointType>::Ptr& cloudOut) {
    if (!cloudIn || !cloudOut) return;
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

pcl::PointCloud<PointType>::Ptr LoopDetector::local2global(const pcl::PointCloud<PointType>::Ptr& cloudIn, const Pose6D& tf) {
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(tf.x, tf.y, tf.z, tf.roll, tf.pitch, tf.yaw);

#pragma omp parallel for num_threads(16)
    for (int i = 0; i < cloudSize; ++i) {
        const auto& pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y +
                                transCur(0, 2) * pointFrom.z + transCur(0, 3);
        cloudOut->points[i].y = transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y +
                                transCur(1, 2) * pointFrom.z + transCur(1, 3);
        cloudOut->points[i].z = transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y +
                                transCur(2, 2) * pointFrom.z + transCur(2, 3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }
    return cloudOut;
}

pcl::PointCloud<PointTypeRGB>::Ptr LoopDetector::local2globalRGB(const pcl::PointCloud<PointTypeRGB>::Ptr& cloudIn, const Pose6D& tf) {
    pcl::PointCloud<PointTypeRGB>::Ptr cloudOut(new pcl::PointCloud<PointTypeRGB>());
    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);
    cloudOut->header = cloudIn->header;

    Eigen::Affine3f transCur = pcl::getTransformation(tf.x, tf.y, tf.z, tf.roll, tf.pitch, tf.yaw);

#pragma omp parallel for num_threads(16)
    for (int i = 0; i < cloudSize; ++i) {
        const auto& pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y +
                                transCur(0, 2) * pointFrom.z + transCur(0, 3);
        cloudOut->points[i].y = transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y +
                                transCur(1, 2) * pointFrom.z + transCur(1, 3);
        cloudOut->points[i].z = transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y +
                                transCur(2, 2) * pointFrom.z + transCur(2, 3);
        cloudOut->points[i].r = pointFrom.r;
        cloudOut->points[i].g = pointFrom.g;
        cloudOut->points[i].b = pointFrom.b;
    }
    return cloudOut;
}

void LoopDetector::loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum) {
    if (!nearKeyframes) return;
    nearKeyframes->clear();

    std::vector<std::pair<pcl::PointCloud<PointTypeRGB>::Ptr, Pose6D>> kf_samples;
    {
        std::lock_guard<std::mutex> lock(mKF_);
        int max_idx = std::min(keyframeLaserClouds_.size(), keyframePosesUpdated_.size());
        for (int i = -searchNum; i <= searchNum; ++i) {
            int keyNear = key + i;
            if (keyNear >= 0 && keyNear < max_idx && keyframeLaserClouds_[keyNear]) {
                kf_samples.emplace_back(keyframeLaserClouds_[keyNear], keyframePosesUpdated_[keyNear]);
            }
        }
    }

    for (const auto& sample : kf_samples) {
        pcl::PointCloud<PointType>::Ptr cloudTempXYZI(new pcl::PointCloud<PointType>());
        RGB2XYZI(sample.first, cloudTempXYZI);
        *nearKeyframes += *local2global(cloudTempXYZI, sample.second);
    }

    if (nearKeyframes->empty()) return;

    pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    downSizeFilterICP_.setInputCloud(nearKeyframes);
    downSizeFilterICP_.filter(*cloud_temp);
    *nearKeyframes = *cloud_temp;
}

gtsam::Pose3 LoopDetector::doICPVirtualRelative(int loop_kf_idx, int curr_kf_idx,
                                                const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pubLoopScanLocal,
                                                const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pubLoopSubmapLocal,
                                                const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pubLoopScanLocalRegisted) {
    std::lock_guard<std::mutex> lock_icp(mICP_);

    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr targetKeyframeCloud(new pcl::PointCloud<PointType>());

    loopFindNearKeyframes(cureKeyframeCloud, curr_kf_idx, 0);
    loopFindNearKeyframes(targetKeyframeCloud, loop_kf_idx, historyKeyframeSearchNum_);

    pcl::PointCloud<PointType>::Ptr cleanCur(new pcl::PointCloud<PointType>());
    cleanCur->reserve(cureKeyframeCloud->size());
    for (const auto& pt : cureKeyframeCloud->points) {
        if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
            PointType p = pt;
            if (!std::isfinite(p.intensity)) p.intensity = 0.0f;
            cleanCur->points.push_back(p);
        }
    }
    cleanCur->width = cleanCur->points.size();
    cleanCur->height = 1;
    cleanCur->is_dense = true;

    pcl::PointCloud<PointType>::Ptr cleanTar(new pcl::PointCloud<PointType>());
    cleanTar->reserve(targetKeyframeCloud->size());
    for (const auto& pt : targetKeyframeCloud->points) {
        if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
            PointType p = pt;
            if (!std::isfinite(p.intensity)) p.intensity = 0.0f;
            cleanTar->points.push_back(p);
        }
    }
    cleanTar->width = cleanTar->points.size();
    cleanTar->height = 1;
    cleanTar->is_dense = true;

    cureKeyframeCloud = cleanCur;
    targetKeyframeCloud = cleanTar;

    if (cureKeyframeCloud->size() < 30 || targetKeyframeCloud->size() < 30) {
        RCLCPP_WARN(node_->get_logger(), "ICP rejected: Target (%zu pts) or Current (%zu pts) keyframe cloud has too few points (< 30).",
                    targetKeyframeCloud->size(), cureKeyframeCloud->size());
        return gtsam::Pose3::identity();
    }

    sensor_msgs::msg::PointCloud2 cureKeyframeCloudMsg;
    pcl::toROSMsg(*cureKeyframeCloud, cureKeyframeCloudMsg);
    cureKeyframeCloudMsg.header.frame_id = "camera_init";
    cureKeyframeCloudMsg.header.stamp = node_->now();
    pubLoopScanLocal->publish(cureKeyframeCloudMsg);

    sensor_msgs::msg::PointCloud2 targetKeyframeCloudMsg;
    pcl::toROSMsg(*targetKeyframeCloud, targetKeyframeCloudMsg);
    targetKeyframeCloudMsg.header.frame_id = "camera_init";
    targetKeyframeCloudMsg.header.stamp = node_->now();
    pubLoopSubmapLocal->publish(targetKeyframeCloudMsg);

    fast_gicp::FastVGICP<PointType, PointType> icp;
    icp.setResolution(1.0);
    icp.setNumThreads(4);
    icp.setMaxCorrespondenceDistance(150.0);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    icp.setInputSource(cureKeyframeCloud);
    icp.setInputTarget(targetKeyframeCloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);

    sensor_msgs::msg::PointCloud2 cureKeyframeCloudRegMsg;
    pcl::toROSMsg(*unused_result, cureKeyframeCloudRegMsg);
    cureKeyframeCloudRegMsg.header.frame_id = "camera_init";
    cureKeyframeCloudRegMsg.header.stamp = node_->now();
    pubLoopScanLocalRegisted->publish(cureKeyframeCloudRegMsg);

    if (icp.hasConverged() == false || icp.getFitnessScore() > loopFitnessScoreThreshold_) {
        RCLCPP_DEBUG(node_->get_logger(),
                     "Backend ICP verification failed. Current: %.4f, Threshold: %.4f.",
                     icp.getFitnessScore(), loopFitnessScoreThreshold_);
        return gtsam::Pose3::identity();
    } else {
        RCLCPP_INFO(node_->get_logger(),
                    "LiDAR PGO constraint added between %d and %d. ICP Score: %.4f",
                    loop_kf_idx, curr_kf_idx, icp.getFitnessScore());
    }

    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();

    Pose6D curr_pose, loop_pose;
    {
        std::lock_guard<std::mutex> lock(mKF_);
        if (curr_kf_idx < 0 || curr_kf_idx >= (int)keyframePosesUpdated_.size() ||
            loop_kf_idx < 0 || loop_kf_idx >= (int)keyframePosesUpdated_.size()) {
            RCLCPP_WARN(node_->get_logger(), "ICP failed: Keyframe index out of bounds (curr: %d, loop: %d, size: %zu)",
                        curr_kf_idx, loop_kf_idx, keyframePosesUpdated_.size());
            return gtsam::Pose3::identity();
        }
        curr_pose = keyframePosesUpdated_[curr_kf_idx];
        loop_pose = keyframePosesUpdated_[loop_kf_idx];
    }

    Eigen::Affine3f tWrong = pcl::getTransformation(curr_pose.x, curr_pose.y, curr_pose.z, curr_pose.roll, curr_pose.pitch, curr_pose.yaw);
    Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;
    pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
    gtsam::Pose3 poseFrom = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
    gtsam::Pose3 poseTo = gtsam::Pose3(gtsam::Rot3::RzRyRx(double(loop_pose.roll), double(loop_pose.pitch), double(loop_pose.yaw)),
                                       gtsam::Point3(double(loop_pose.x), double(loop_pose.y), double(loop_pose.z)));

    return poseFrom.between(poseTo);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LoopDetector::vector2pc(const std::vector<Pose6D>& vectorPose6d) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr res(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& p : vectorPose6d) {
        res->points.emplace_back(p.x, p.y, p.z);
    }
    return res;
}

bool LoopDetector::detectLoopClosureDistance(int* loopKeyCur, int* loopKeyPre) {
    std::lock_guard<std::mutex> lock(mBuf_);
    auto it = loopIndexContainer_.find(*loopKeyCur);
    if (it != loopIndexContainer_.end()) return false;

    std::vector<Pose6D> poses_copy;
    std::vector<double> times_copy;
    {
        std::lock_guard<std::mutex> lock_kf(mKF_);
        poses_copy = keyframePoses_;
        times_copy = keyframeTimes_;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr copy_cloudKeyPoses3D = vector2pc(poses_copy);
    std::vector<int> pointSearchIndLoop;
    std::vector<float> pointSearchSqDisLoop;
    kdtreeHistoryKeyPoses_->setInputCloud(copy_cloudKeyPoses3D);
    kdtreeHistoryKeyPoses_->radiusSearch(copy_cloudKeyPoses3D->back(), historyKeyframeSearchRadius_,
                                         pointSearchIndLoop, pointSearchSqDisLoop, 0);

    for (size_t i = 0; i < pointSearchIndLoop.size(); ++i) {
        int id = pointSearchIndLoop[i];
        if (std::abs(times_copy[id] - times_copy[*loopKeyCur]) > historyKeyframeSearchTimeDiff_) {
            *loopKeyPre = id;
            break;
        }
    }

    if (*loopKeyPre == -1 || *loopKeyCur == *loopKeyPre) return false;

    return true;
}

void LoopDetector::performSCLoopClosure() {
    int curr_node_idx;
    double curr_time;
    {
        std::lock_guard<std::mutex> lock(mKF_);
        if (int(keyframePoses_.size()) < scManager_.NUM_EXCLUDE_RECENT) return;
        curr_node_idx = keyframePoses_.size() - 1;
        curr_time = keyframeTimes_[curr_node_idx];
    }

    auto detectResult = scManager_.detectLoopClosureID();
    int SCclosestHistoryFrameID = detectResult.first;

    if (SCclosestHistoryFrameID != -1) {
        const int prev_node_idx = SCclosestHistoryFrameID;
        double prev_time;
        {
            std::lock_guard<std::mutex> lock(mKF_);
            prev_time = keyframeTimes_[prev_node_idx];
        }

        double time_diff = std::abs(curr_time - prev_time);
        if (time_diff < historyKeyframeSearchTimeDiff_) {
            return;
        }

        RCLCPP_INFO(node_->get_logger(),
                    "LiDAR ScanContext loop candidate proposed. History ID: %d, Current ID: %d. Running ICP verification...",
                    prev_node_idx, curr_node_idx);

        std::lock_guard<std::mutex> lock_buf(mBuf_);
        scLoopICPBuf_.push(std::make_pair(prev_node_idx, curr_node_idx));
    }
}

void LoopDetector::performRSLoopClosure() {
    int loopKeyCur;
    {
        std::lock_guard<std::mutex> lock(mKF_);
        if (keyframePoses_.empty()) return;
        loopKeyCur = keyframePoses_.size() - 1;
    }
    int loopKeyPre = -1;

    if (detectLoopClosureDistance(&loopKeyCur, &loopKeyPre)) {
        RCLCPP_INFO(node_->get_logger(),
                    "LiDAR spatial distance loop candidate proposed. History ID: %d, Current ID: %d. Running ICP verification...",
                    loopKeyPre, loopKeyCur);
        std::lock_guard<std::mutex> lock_buf(mBuf_);
        scLoopICPBuf_.push(std::make_pair(loopKeyPre, loopKeyCur));
        loopIndexContainer_[loopKeyCur] = loopKeyPre;
    }
}

bool LoopDetector::loadHistoricalDatabase(const std::string& sc_file, const std::string& scans_dir) {
    historyScansDir_ = scans_dir;
    historyKeyframePoses_.clear();
    historyCloudsCache_.clear();

    std::ifstream ifs(sc_file);
    if (!ifs.is_open()) {
        RCLCPP_WARN(node_->get_logger(), "[LoopDetector] Failed to open historical ScanContext database: %s", sc_file.c_str());
        return false;
    }

    int num_frames = 0;
    ifs >> num_frames;
    if (num_frames <= 0) {
        RCLCPP_WARN(node_->get_logger(), "[LoopDetector] Historical ScanContext database is empty.");
        return false;
    }

    historyScManager_.setSCdistThres(scManager_.SC_DIST_THRES);
    historyScManager_.setMaximumRadius(scManager_.PC_MAX_RADIUS);

    for (int i = 0; i < num_frames; ++i) {
        int idx;
        Pose6D p;
        if (!(ifs >> idx >> p.x >> p.y >> p.z >> p.roll >> p.pitch >> p.yaw) || idx != i ||
            !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
            !std::isfinite(p.roll) || !std::isfinite(p.pitch) || !std::isfinite(p.yaw)) return false;
        historyKeyframePoses_.push_back(p);

        int rows, cols;
        if (!(ifs >> rows >> cols) || rows <= 0 || cols <= 0 || rows > 1000 || cols > 1000) return false;
        Eigen::MatrixXd sc(rows, cols);
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
                if (!(ifs >> sc(r, c)) || !std::isfinite(sc(r, c))) return false;
            }
        }
        historyScManager_.saveScancontextAndKeys(sc);
    }
    ifs.close();

    RCLCPP_INFO(node_->get_logger(),
                "[LoopDetector] Successfully loaded historical ScanContext database (%d frames, Path: %s).",
                num_frames, sc_file.c_str());
    return true;
}

bool LoopDetector::detectCrossSessionLoop(int curr_kf_idx, int* best_history_kf_idx, float* relative_yaw) {
    if (historyKeyframePoses_.empty()) return false;
    if (curr_kf_idx < 0) return false;

    std::vector<float> curr_key;
    Eigen::MatrixXd curr_desc;
    {
        std::lock_guard<std::mutex> lock(mKF_);
        if (int(scManager_.polarcontext_invkeys_mat_.size()) <= curr_kf_idx) return false;
        curr_key = scManager_.polarcontext_invkeys_mat_[curr_kf_idx];
        curr_desc = scManager_.polarcontexts_[curr_kf_idx];
    }

    auto detectResult = historyScManager_.detectLoopClosureIDBetweenSession(curr_key, curr_desc);
    int match_id = detectResult.first;
    float yaw_diff = detectResult.second;

    if (match_id != -1 && match_id < int(historyKeyframePoses_.size())) {
        *best_history_kf_idx = match_id;
        *relative_yaw = yaw_diff;
        return true;
    }
    return false;
}

gtsam::Pose3 LoopDetector::doCrossSessionICP(int history_kf_idx, int curr_kf_idx, float& icp_score,
                                             const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pubLoopScanLocal,
                                             const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pubLoopSubmapLocal,
                                             const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pubLoopScanLocalRegisted) {
    std::lock_guard<std::mutex> lock_icp(mICP_);
    icp_score = 999.0f;
    if (history_kf_idx < 0 || history_kf_idx >= int(historyKeyframePoses_.size())) return gtsam::Pose3::identity();

    // 1. 构建当前会话局部帧点云 (从内存获取并剔除无效点)
    pcl::PointCloud<PointType>::Ptr currCloudRaw(new pcl::PointCloud<PointType>());
    {
        std::lock_guard<std::mutex> lock(mKF_);
        if (curr_kf_idx >= int(keyframeLaserClouds_.size())) return gtsam::Pose3::identity();
        RGB2XYZI(keyframeLaserClouds_[curr_kf_idx], currCloudRaw);
    }
    if (currCloudRaw->empty()) return gtsam::Pose3::identity();

    pcl::PointCloud<PointType>::Ptr currCloud(new pcl::PointCloud<PointType>());
    std::vector<int> indices_curr;
    pcl::removeNaNFromPointCloud(*currCloudRaw, *currCloud, indices_curr);
    if (currCloud->empty()) return gtsam::Pose3::identity();

    // 2. 构建历史会话 A 局部子图 (从 Scans/ 目录加载点云并拼接)
    pcl::PointCloud<PointType>::Ptr targetSubmap(new pcl::PointCloud<PointType>());
    int search_num = std::min(historyKeyframeSearchNum_, 10);
    int hist_total = historyKeyframePoses_.size();

    for (int i = -search_num; i <= search_num; ++i) {
        int near_id = history_kf_idx + i;
        if (near_id < 0 || near_id >= hist_total) continue;

        pcl::PointCloud<PointType>::Ptr nearCloud;
        auto it = historyCloudsCache_.find(near_id);
        if (it != historyCloudsCache_.end()) {
            nearCloud = it->second;
        } else {
            std::string pcd_path = historyScansDir_ + padZeros(near_id) + ".pcd";
            pcl::PointCloud<PointType>::Ptr rawPcd(new pcl::PointCloud<PointType>());
            if (pcl::io::loadPCDFile(pcd_path, *rawPcd) == 0 && !rawPcd->empty()) {
                nearCloud.reset(new pcl::PointCloud<PointType>());
                std::vector<int> indices_pcd;
                pcl::removeNaNFromPointCloud(*rawPcd, *nearCloud, indices_pcd);
                if (historyCloudsCache_.size() < 200) {
                    historyCloudsCache_[near_id] = nearCloud;
                }
            } else {
                continue;
            }
        }
        if (nearCloud && !nearCloud->empty()) {
            *targetSubmap += *local2global(nearCloud, historyKeyframePoses_[near_id]);
        }
    }

    if (targetSubmap->empty()) {
        RCLCPP_WARN(node_->get_logger(), "[LoopDetector] Cross-session ICP failed: historical submap cloud is empty (history ID: %d).", history_kf_idx);
        return gtsam::Pose3::identity();
    }

    // 剔除 targetSubmap 中的 NaN/Inf
    pcl::PointCloud<PointType>::Ptr targetSubmapClean(new pcl::PointCloud<PointType>());
    std::vector<int> ind_submap;
    pcl::removeNaNFromPointCloud(*targetSubmap, *targetSubmapClean, ind_submap);
    if (targetSubmapClean->empty()) return gtsam::Pose3::identity();

    pcl::PointCloud<PointType>::Ptr submap_ds(new pcl::PointCloud<PointType>());
    downSizeFilterICP_.setInputCloud(targetSubmapClean);
    downSizeFilterICP_.filter(*submap_ds);

    // 再次确认 submap_ds 无 NaN
    pcl::PointCloud<PointType>::Ptr submap_ds_clean(new pcl::PointCloud<PointType>());
    std::vector<int> ind_ds;
    pcl::removeNaNFromPointCloud(*submap_ds, *submap_ds_clean, ind_ds);
    submap_ds = submap_ds_clean;

    if (submap_ds->empty()) return gtsam::Pose3::identity();

    // 3. 粗配准初值：历史帧绝对位姿
    Pose6D hist_p = historyKeyframePoses_[history_kf_idx];
    Eigen::Affine3f init_guess = pcl::getTransformation(hist_p.x, hist_p.y, hist_p.z, hist_p.roll, hist_p.pitch, hist_p.yaw);

    pcl::PointCloud<PointType>::Ptr curr_transformed(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*currCloud, *curr_transformed, init_guess);

    // 彻底清洗 submap_ds 和 curr_transformed，确保无任何 NaN/Inf 且 is_dense = true
    pcl::PointCloud<PointType>::Ptr submap_ds_dense(new pcl::PointCloud<PointType>());
    submap_ds_dense->reserve(submap_ds->size());
    for (const auto& pt : submap_ds->points) {
        if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
            PointType p = pt;
            if (!std::isfinite(p.intensity)) p.intensity = 0.0f;
            submap_ds_dense->points.push_back(p);
        }
    }
    submap_ds_dense->width = submap_ds_dense->points.size();
    submap_ds_dense->height = 1;
    submap_ds_dense->is_dense = true;
    if (submap_ds_dense->size() < 30) return gtsam::Pose3::identity();

    pcl::PointCloud<PointType>::Ptr curr_transformed_dense(new pcl::PointCloud<PointType>());
    curr_transformed_dense->reserve(curr_transformed->size());
    for (const auto& pt : curr_transformed->points) {
        if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
            PointType p = pt;
            if (!std::isfinite(p.intensity)) p.intensity = 0.0f;
            curr_transformed_dense->points.push_back(p);
        }
    }
    curr_transformed_dense->width = curr_transformed_dense->points.size();
    curr_transformed_dense->height = 1;
    curr_transformed_dense->is_dense = true;
    if (curr_transformed_dense->size() < 30) return gtsam::Pose3::identity();

    // 发布 RViz 调试话题
    if (pubLoopScanLocal) {
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*curr_transformed_dense, msg);
        msg.header.frame_id = "camera_init";
        msg.header.stamp = node_->now();
        pubLoopScanLocal->publish(msg);
    }
    if (pubLoopSubmapLocal) {
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*submap_ds_dense, msg);
        msg.header.frame_id = "camera_init";
        msg.header.stamp = node_->now();
        pubLoopSubmapLocal->publish(msg);
    }

    // 4. FastVGICP 求解
    fast_gicp::FastVGICP<PointType, PointType> icp;
    icp.setResolution(1.0);
    icp.setNumThreads(4);
    icp.setMaxCorrespondenceDistance(150.0);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);

    icp.setInputSource(curr_transformed_dense);
    icp.setInputTarget(submap_ds_dense);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);

    icp_score = icp.getFitnessScore();
    if (!icp.hasConverged() || icp_score > loopFitnessScoreThreshold_) {
        RCLCPP_DEBUG(node_->get_logger(), "[LoopDetector] Cross-session ICP score threshold not met (Score: %.4f, Thres: %.4f)",
                     icp_score, loopFitnessScoreThreshold_);
        return gtsam::Pose3::identity();
    }

    if (pubLoopScanLocalRegisted) {
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*unused_result, msg);
        msg.header.frame_id = "camera_init";
        msg.header.stamp = node_->now();
        pubLoopScanLocalRegisted->publish(msg);
    }

    RCLCPP_INFO(node_->get_logger(),
                "[LoopDetector] Cross-session LiDAR ICP registration succeeded between history KF %d and current KF %d. ICP Score: %.4f",
                history_kf_idx, curr_kf_idx, icp_score);

    // 求解精确的在地图坐标系下的位姿
    Eigen::Matrix4f final_mat = icp.getFinalTransformation() * init_guess.matrix();
    Eigen::Affine3f final_tf(final_mat);
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(final_tf, x, y, z, roll, pitch, yaw);

    gtsam::Pose3 pose_curr_in_map(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
    return pose_curr_in_map;
}

std::mutex& LoopDetector::getKFMutex() { return mKF_; }
std::mutex& LoopDetector::getBufMutex() { return mBuf_; }

std::vector<pcl::PointCloud<PointTypeRGB>::Ptr>& LoopDetector::getKeyframeLaserClouds() { return keyframeLaserClouds_; }
std::vector<Pose6D>& LoopDetector::getKeyframePoses() { return keyframePoses_; }
std::vector<Pose6D>& LoopDetector::getKeyframePosesUpdated() { return keyframePosesUpdated_; }
std::vector<double>& LoopDetector::getKeyframeTimes() { return keyframeTimes_; }
std::map<int, int>& LoopDetector::getLoopIndexContainer() { return loopIndexContainer_; }
std::queue<std::pair<int, int>>& LoopDetector::getScLoopICPBuf() { return scLoopICPBuf_; }
