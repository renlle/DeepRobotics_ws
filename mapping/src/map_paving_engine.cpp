#include "mapping/map_paving_engine.h"
#include <filesystem>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <limits>
#include <algorithm>
#include <map>

MapPavingEngine::MapPavingEngine(const Config& config, const rclcpp::Logger& logger)
    : config_(config), logger_(logger) {}

MapPavingEngine::~MapPavingEngine() {}

pcl::PointCloud<PointTypeRGB>::Ptr MapPavingEngine::local2globalRGB(const pcl::PointCloud<PointTypeRGB>::Ptr& cloudIn,
                                                                   const Pose6D& tf) {
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

bool MapPavingEngine::buildAndSaveMaps(const std::vector<pcl::PointCloud<PointTypeRGB>::Ptr>& keyframeLaserClouds,
                                      const std::vector<Pose6D>& keyframePosesUpdated,
                                      int recentIdxUpdated,
                                      bool is_incremental,
                                      const Eigen::Matrix4f& T_map_to_init) {
    pcl::PointCloud<PointTypeRGB>::Ptr full_map(new pcl::PointCloud<PointTypeRGB>());

    // 1. 如果是累积建图模式且历史 3D 地图存在，先加载历史底图 A
    if (is_incremental) {
        std::string hist_map_path = config_.save_directory + "map_3d.pcd";
        if (std::filesystem::exists(hist_map_path)) {
            pcl::PointCloud<PointTypeRGB>::Ptr hist_map(new pcl::PointCloud<PointTypeRGB>());
            if (pcl::io::loadPCDFile(hist_map_path, *hist_map) == 0 && !hist_map->empty()) {
                *full_map += *hist_map;
                RCLCPP_INFO(logger_, "Loaded historical 3D map with %lu points for incremental stitching.", hist_map->size());
            }
        }
    }

    // 2. 将当前会话 B 的关键帧点云投影并融合
    pcl::PointCloud<PointTypeRGB>::Ptr curr_session_cloud(new pcl::PointCloud<PointTypeRGB>());
    int num_keyframes = keyframeLaserClouds.size();
    for (int i = 0; i <= recentIdxUpdated && i < num_keyframes; i++) {
        Pose6D global_pose = keyframePosesUpdated[i];
        if (is_incremental) {
            Eigen::Affine3f trans_local = pcl::getTransformation(
                global_pose.x, global_pose.y, global_pose.z,
                global_pose.roll, global_pose.pitch, global_pose.yaw
            );
            Eigen::Matrix4f trans_global = T_map_to_init * trans_local.matrix();
            Eigen::Affine3f aff(trans_global);
            float x, y, z, roll, pitch, yaw;
            pcl::getTranslationAndEulerAngles(aff, x, y, z, roll, pitch, yaw);
            global_pose.x = x; global_pose.y = y; global_pose.z = z;
            global_pose.roll = roll; global_pose.pitch = pitch; global_pose.yaw = yaw;
        }
        *curr_session_cloud += *local2globalRGB(keyframeLaserClouds[i], global_pose);
    }

    if (!curr_session_cloud->empty()) {
        pcl::PointCloud<PointTypeRGB>::Ptr curr_session_ds(new pcl::PointCloud<PointTypeRGB>());
        safeVoxelFilter<PointTypeRGB>(curr_session_cloud, curr_session_ds, 0.05f);
        *full_map += *curr_session_ds;
    }

    if (!full_map->empty()) {
        // 下采样滤波保证融合后点云密度适中 (去重叠加点，保证全图高质量均匀分布)
        pcl::PointCloud<PointTypeRGB>::Ptr filtered_full_map(new pcl::PointCloud<PointTypeRGB>());
        safeVoxelFilter<PointTypeRGB>(full_map, filtered_full_map, 0.05f);

        if (pcl::io::savePCDFileBinary(config_.save_directory + "map_3d.pcd", *filtered_full_map) != 0) return false;
        RCLCPP_INFO(logger_,
                    "Full global map saved to file (Directory: %s, Total points: %lu).",
                    config_.save_directory.c_str(), filtered_full_map->size());

        // 过滤出真彩色点（相机可见点），过滤掉灰度退化点（r==g==b 为 intensity 填充的无色点）
        pcl::PointCloud<PointTypeRGB>::Ptr colored_map(new pcl::PointCloud<PointTypeRGB>());
        colored_map->reserve(filtered_full_map->size());
        for (const auto& pt : filtered_full_map->points) {
            if (pt.r != pt.g || pt.r != pt.b) {
                colored_map->push_back(pt);
            }
        }
        if (!colored_map->empty()) {
            pcl::io::savePCDFileBinary(config_.save_directory + "map_3d_colored.pcd", *colored_map);
            RCLCPP_INFO(logger_,
                        "Colored point cloud map saved (total: %zu pts -> colored: %zu pts).",
                        filtered_full_map->size(), colored_map->size());
        } else {
            RCLCPP_WARN(logger_, "No colored points found; map_3d_colored.pcd not saved.");
        }

        return save2DMap(keyframeLaserClouds, keyframePosesUpdated, recentIdxUpdated, is_incremental, T_map_to_init);
    } else {
        RCLCPP_WARN(logger_, "Cannot build map: global point cloud is empty.");
    }
    return false;
}


bool MapPavingEngine::save2DMap(const std::vector<pcl::PointCloud<PointTypeRGB>::Ptr>& keyframeLaserClouds,
                               const std::vector<Pose6D>& keyframePosesUpdated,
                               int recentIdxUpdated,
                               bool is_incremental,
                               const Eigen::Matrix4f& T_map_to_init) {
    if (keyframeLaserClouds.empty() && !is_incremental) return false;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass_through(new pcl::PointCloud<pcl::PointXYZ>());

    // 1. 如果是累积建图模式，多线程并行加载历史关键帧点云并沿着历史运动轨迹进行动态躯干切片
    if (is_incremental) {
        std::string hist_poses_path = config_.save_directory + "optimized_poses_hist.txt";
        std::string scans_dir = config_.save_directory + "Scans/";
        if (std::filesystem::exists(hist_poses_path) && std::filesystem::exists(scans_dir)) {
            std::ifstream fin(hist_poses_path);
            std::string line;
            std::vector<Eigen::Matrix4f> hist_poses;
            while (std::getline(fin, line)) {
                std::istringstream ss(line);
                float r00, r01, r02, tx, r10, r11, r12, ty, r20, r21, r22, tz;
                if (!(ss >> r00 >> r01 >> r02 >> tx >> r10 >> r11 >> r12 >> ty >> r20 >> r21 >> r22 >> tz)) continue;

                Eigen::Matrix4f T_world_lidar = Eigen::Matrix4f::Identity();
                T_world_lidar(0,0)=r00; T_world_lidar(0,1)=r01; T_world_lidar(0,2)=r02; T_world_lidar(0,3)=tx;
                T_world_lidar(1,0)=r10; T_world_lidar(1,1)=r11; T_world_lidar(1,2)=r12; T_world_lidar(1,3)=ty;
                T_world_lidar(2,0)=r20; T_world_lidar(2,1)=r21; T_world_lidar(2,2)=r22; T_world_lidar(2,3)=tz;
                hist_poses.push_back(T_world_lidar);
            }
            fin.close();

            int total_hist = hist_poses.size();
            std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> sliced_clouds(total_hist);
            Eigen::Matrix4f T_body_to_base = config_.T_body_to_baselink.inverse();

            #pragma omp parallel for num_threads(16) schedule(dynamic, 16)
            for (int hist_idx = 0; hist_idx < total_hist; ++hist_idx) {
                std::stringstream ss_pcd;
                ss_pcd << scans_dir << std::setfill('0') << std::setw(6) << hist_idx << ".pcd";
                std::string pcd_file = ss_pcd.str();
                if (!std::filesystem::exists(pcd_file)) continue;

                pcl::PointCloud<PointTypeRGB>::Ptr hist_cloud(new pcl::PointCloud<PointTypeRGB>());
                if (pcl::io::loadPCDFile(pcd_file, *hist_cloud) != 0 || hist_cloud->empty()) continue;

                pcl::PointCloud<pcl::PointXYZ>::Ptr local_sliced(new pcl::PointCloud<pcl::PointXYZ>());
                Eigen::Matrix4f T_base_to_world = hist_poses[hist_idx] * config_.T_body_to_baselink;

                for (const auto& pt : hist_cloud->points) {
                    float bx = T_body_to_base(0,0)*pt.x + T_body_to_base(0,1)*pt.y + T_body_to_base(0,2)*pt.z + T_body_to_base(0,3);
                    float by = T_body_to_base(1,0)*pt.x + T_body_to_base(1,1)*pt.y + T_body_to_base(1,2)*pt.z + T_body_to_base(1,3);
                    float bz = T_body_to_base(2,0)*pt.x + T_body_to_base(2,1)*pt.y + T_body_to_base(2,2)*pt.z + T_body_to_base(2,3);
                    if (bz >= config_.pcd2pgm_thre_z_min && bz <= config_.pcd2pgm_thre_z_max) {
                        pcl::PointXYZ p;
                        p.x = T_base_to_world(0,0)*bx + T_base_to_world(0,1)*by + T_base_to_world(0,2)*bz + T_base_to_world(0,3);
                        p.y = T_base_to_world(1,0)*bx + T_base_to_world(1,1)*by + T_base_to_world(1,2)*bz + T_base_to_world(1,3);
                        p.z = T_base_to_world(2,0)*bx + T_base_to_world(2,1)*by + T_base_to_world(2,2)*bz + T_base_to_world(2,3);
                        local_sliced->points.push_back(p);
                    }
                }
                sliced_clouds[hist_idx] = local_sliced;
            }

            int hist_sliced_pts = 0;
            for (int i = 0; i < total_hist; ++i) {
                if (sliced_clouds[i] && !sliced_clouds[i]->empty()) {
                    *cloud_pass_through += *sliced_clouds[i];
                    hist_sliced_pts += sliced_clouds[i]->size();
                }
            }
            RCLCPP_INFO(logger_,
                        "2D Map: processed %d historical frames in parallel (extracted %d obstacle points along historical trajectory).",
                        total_hist, hist_sliced_pts);
        }
    }

    // 2. 逐帧投影并沿着当前会话 B 的运动轨迹进行动态躯干切片 (OpenMP 多线程并行)
    int num_keyframes = keyframeLaserClouds.size();
    int curr_total = std::min(recentIdxUpdated + 1, num_keyframes);
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> curr_sliced_clouds(curr_total);
    Eigen::Matrix4f T_body_to_base = config_.T_body_to_baselink.inverse();

    #pragma omp parallel for num_threads(16) schedule(dynamic, 8)
    for (int i = 0; i < curr_total; i++) {
        const auto& pose = keyframePosesUpdated[i];
        Eigen::Affine3f transCur = pcl::getTransformation(pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw);
        Eigen::Matrix4f T_base_to_global_map = transCur.matrix() * config_.T_body_to_baselink;
        if (is_incremental) {
            T_base_to_global_map = T_map_to_init * T_base_to_global_map;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr local_sliced(new pcl::PointCloud<pcl::PointXYZ>());
        for (const auto& pt : keyframeLaserClouds[i]->points) {
            float bx = T_body_to_base(0,0)*pt.x + T_body_to_base(0,1)*pt.y + T_body_to_base(0,2)*pt.z + T_body_to_base(0,3);
            float by = T_body_to_base(1,0)*pt.x + T_body_to_base(1,1)*pt.y + T_body_to_base(1,2)*pt.z + T_body_to_base(1,3);
            float bz = T_body_to_base(2,0)*pt.x + T_body_to_base(2,1)*pt.y + T_body_to_base(2,2)*pt.z + T_body_to_base(2,3);
            if (bz >= config_.pcd2pgm_thre_z_min && bz <= config_.pcd2pgm_thre_z_max) {
                pcl::PointXYZ p;
                p.x = T_base_to_global_map(0,0)*bx + T_base_to_global_map(0,1)*by + T_base_to_global_map(0,2)*bz + T_base_to_global_map(0,3);
                p.y = T_base_to_global_map(1,0)*bx + T_base_to_global_map(1,1)*by + T_base_to_global_map(1,2)*bz + T_base_to_global_map(1,3);
                p.z = T_base_to_global_map(2,0)*bx + T_base_to_global_map(2,1)*by + T_base_to_global_map(2,2)*bz + T_base_to_global_map(2,3);
                local_sliced->points.push_back(p);
            }
        }
        curr_sliced_clouds[i] = local_sliced;
    }

    for (int i = 0; i < curr_total; i++) {
        if (curr_sliced_clouds[i] && !curr_sliced_clouds[i]->empty()) {
            *cloud_pass_through += *curr_sliced_clouds[i];
        }
    }

    cloud_pass_through->width = cloud_pass_through->points.size();
    cloud_pass_through->height = 1;
    cloud_pass_through->is_dense = true;

    RCLCPP_INFO(logger_, "2D Map: after dynamic height filter, %lu points remain.", cloud_pass_through->points.size());

    if (cloud_pass_through->empty()) {
        RCLCPP_WARN(logger_, "No points remain after dynamic Z height filtering!");
        return false;
    }

    // 2.5 体素滤波下采样 (Voxel Grid Filter)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>());
    safeVoxelFilter<pcl::PointXYZ>(cloud_pass_through, cloud_downsampled, config_.pcd2pgm_map_resolution);

    RCLCPP_INFO(logger_, "2D Map: after voxel downsampling, %lu points remain.", cloud_downsampled->points.size());

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_radius = cloud_downsampled;

    // 如果 VoxelGrid 因为边界过大发生溢出（点数没变），或者体素滤波后点数依然极为庞大（>100万），
    // 此时强行跑 RadiusOutlierRemoval 会导致进程卡死，因此必须跳过去噪步骤。
    if (cloud_downsampled->points.size() < 1000000) {
        // 3. 半径噪点滤波移除
        cloud_after_radius.reset(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_outlier;
        radius_outlier.setInputCloud(cloud_downsampled);
        radius_outlier.setRadiusSearch(config_.pcd2pgm_thre_radius);
        radius_outlier.setMinNeighborsInRadius(config_.pcd2pgm_thres_point_count);
        radius_outlier.filter(*cloud_after_radius);
        
        RCLCPP_INFO(logger_, "2D Map: after radius outlier filter, %lu points remain.", cloud_after_radius->points.size());
    } else {
        RCLCPP_WARN(logger_, "2D Map: Point cloud too large (%lu points after voxel), skipping RadiusOutlierRemoval to prevent freezing.", cloud_downsampled->points.size());
    }

    if (cloud_after_radius->empty()) {
        RCLCPP_WARN(logger_, "No points remain after filtering!");
        return false;
    }

    // 4. 边界范围计算
    double x_min = std::numeric_limits<double>::max();
    double x_max = std::numeric_limits<double>::lowest();
    double y_min = std::numeric_limits<double>::max();
    double y_max = std::numeric_limits<double>::lowest();

    for (const auto & point : cloud_after_radius->points) {
        x_min = std::min(x_min, static_cast<double>(point.x));
        x_max = std::max(x_max, static_cast<double>(point.x));
        y_min = std::min(y_min, static_cast<double>(point.y));
        y_max = std::max(y_max, static_cast<double>(point.y));
    }

    int width = std::ceil((x_max - x_min) / config_.pcd2pgm_map_resolution);
    int height = std::ceil((y_max - y_min) / config_.pcd2pgm_map_resolution);

    if (width <= 0 || height <= 0) {
        RCLCPP_WARN(logger_, "Invalid 2D map size: %d x %d", width, height);
        return false;
    }

    // 5. 投影与 2D 连通域二次去噪 (Two-Stage 2D Connected Component Denoising)
    cv::Mat map_img(height, width, CV_8UC1, cv::Scalar(254));

    for (const auto & point : cloud_after_radius->points) {
        int i = std::floor((point.x - x_min) / config_.pcd2pgm_map_resolution);
        int j = std::floor((point.y - y_min) / config_.pcd2pgm_map_resolution);

        if (i >= 0 && i < width && j >= 0 && j < height) {
            int img_j = height - 1 - j;
            map_img.at<uchar>(img_j, i) = 0;
        }
    }

    // 第二阶段去噪：使用 8-连通域剔除面积小 < 10 像素的悬空/乱飞离群噪点块
    cv::Mat binary_mask;
    cv::threshold(map_img, binary_mask, 50, 255, cv::THRESH_BINARY_INV);
    cv::Mat labels, stats, centroids;
    int num_labels = cv::connectedComponentsWithStats(binary_mask, labels, stats, centroids, 8);

    int removed_blobs = 0;
    int removed_pixels = 0;
    const int MIN_BLOB_PIXELS = 10; // 连通小黑点剔除阈值 (< 10 px)

    for (int label = 1; label < num_labels; ++label) {
        int area = stats.at<int>(label, cv::CC_STAT_AREA);
        if (area < MIN_BLOB_PIXELS) {
            removed_blobs++;
            removed_pixels += area;

            int left = stats.at<int>(label, cv::CC_STAT_LEFT);
            int top = stats.at<int>(label, cv::CC_STAT_TOP);
            int w = stats.at<int>(label, cv::CC_STAT_WIDTH);
            int h = stats.at<int>(label, cv::CC_STAT_HEIGHT);

            for (int r = top; r < top + h; ++r) {
                for (int c = left; c < left + w; ++c) {
                    if (labels.at<int>(r, c) == label) {
                        map_img.at<uchar>(r, c) = 254;
                    }
                }
            }
        }
    }

    RCLCPP_INFO(logger_,
                "2D Map 2nd Stage: Cleared %d isolated noise blobs (< %d px, total %d pixels).",
                removed_blobs, MIN_BLOB_PIXELS, removed_pixels);

    // 写入 PGM 文件
    std::string pgm_file = config_.save_directory + "map_2d.pgm";
    std::ofstream pgm(pgm_file, std::ios::binary);
    if (pgm.is_open()) {
        pgm << "P5\n" << width << " " << height << "\n255\n";
        pgm.write(reinterpret_cast<const char*>(map_img.data), map_img.total());
        pgm.close();
        RCLCPP_INFO(logger_, "Successfully saved 2D PGM map to: %s", pgm_file.c_str());
    } else {
        RCLCPP_ERROR(logger_, "Failed to open PGM file for writing: %s", pgm_file.c_str());
    }

    // 6. 写入 YAML 配置文件
    std::string yaml_file = config_.save_directory + "map_2d.yaml";
    std::ofstream yaml(yaml_file);
    if (yaml.is_open()) {
        yaml << "image: map_2d.pgm\n";
        yaml << "resolution: " << config_.pcd2pgm_map_resolution << "\n";
        yaml << "origin: [" << x_min << ", " << y_min << ", 0.0]\n";
        yaml << "negate: 0\n";
        yaml << "occupied_thresh: 0.65\n";
        yaml << "free_thresh: 0.196\n";
        yaml.close();
        RCLCPP_INFO(logger_, "Successfully saved 2D YAML config to: %s", yaml_file.c_str());
    } else {
        RCLCPP_ERROR(logger_, "Failed to open YAML file for writing: %s", yaml_file.c_str());
    }
    return pgm.good() && yaml.good();
}

template <typename PointT>
void MapPavingEngine::safeVoxelFilter(const typename pcl::PointCloud<PointT>::Ptr& cloudIn,
                                     typename pcl::PointCloud<PointT>::Ptr& cloudOut,
                                     float leaf_size,
                                     float chunk_size) {
    if (!cloudIn || cloudIn->empty()) {
        cloudOut->clear();
        return;
    }

    // 1. 先探测点云包围盒尺度，如果体素索引安全乘积在 32 位整型安全余量以内，直接跑原生单次 VoxelGrid
    Eigen::Vector4f min_pt, max_pt;
    pcl::getMinMax3D(*cloudIn, min_pt, max_pt);
    int64_t dx = std::ceil((max_pt[0] - min_pt[0]) / leaf_size) + 1;
    int64_t dy = std::ceil((max_pt[1] - min_pt[1]) / leaf_size) + 1;
    int64_t dz = std::ceil((max_pt[2] - min_pt[2]) / leaf_size) + 1;

    // 预留充分的 32 位安全上限 (小于 1.0e9)，避免 PCL 内部 (dx*dy*dz > INT32_MAX) 溢出报错跳过滤波
    if (dx * dy * dz < 1000000000LL && dx * dy * dz > 0) {
        pcl::VoxelGrid<PointT> ds;
        ds.setInputCloud(cloudIn);
        ds.setLeafSize(leaf_size, leaf_size, leaf_size);
        ds.filter(*cloudOut);
        return;
    }

    RCLCPP_INFO(logger_,
                "Large-scale point cloud detected (bounding box: %.1fm x %.1fm x %.1fm, total theoretical voxels: %ld > 10^9). "
                "Enabling chunked parallel VoxelGrid filtering...",
                max_pt[0] - min_pt[0], max_pt[1] - min_pt[1], max_pt[2] - min_pt[2], dx * dy * dz);

    // 2. 超大尺度或广域多会话场景：采用空间网格分块 (Spatial Chunking) 并行体素滤波
    // chunk_size 严格设定为 50m (为 0.05m 的 1000 倍)，确保所有分块边缘与全局体素网格完全对齐无接缝缝隙
    std::map<std::pair<int64_t, int64_t>, typename pcl::PointCloud<PointT>::Ptr> chunk_map;
    for (const auto& pt : cloudIn->points) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
        int64_t cx = static_cast<int64_t>(std::floor(pt.x / chunk_size));
        int64_t cy = static_cast<int64_t>(std::floor(pt.y / chunk_size));
        // 负坐标不使用有符号左移，避免分块索引未定义行为。
        auto key = std::make_pair(cx, cy);
        auto& chunk_cloud = chunk_map[key];
        if (!chunk_cloud) {
            chunk_cloud.reset(new pcl::PointCloud<PointT>());
        }
        chunk_cloud->push_back(pt);
    }

    std::vector<typename pcl::PointCloud<PointT>::Ptr> chunks;
    chunks.reserve(chunk_map.size());
    for (auto& kv : chunk_map) {
        chunks.push_back(std::move(kv.second));
    }

    int num_chunks = chunks.size();
    std::vector<typename pcl::PointCloud<PointT>::Ptr> filtered_chunks(num_chunks);

    #pragma omp parallel for num_threads(16) schedule(dynamic)
    for (int i = 0; i < num_chunks; ++i) {
        if (!chunks[i] || chunks[i]->empty()) continue;
        pcl::VoxelGrid<PointT> ds;
        ds.setInputCloud(chunks[i]);
        ds.setLeafSize(leaf_size, leaf_size, leaf_size);
        filtered_chunks[i].reset(new pcl::PointCloud<PointT>());
        ds.filter(*filtered_chunks[i]);
    }

    cloudOut->clear();
    size_t total_pts = 0;
    for (int i = 0; i < num_chunks; ++i) {
        if (filtered_chunks[i]) total_pts += filtered_chunks[i]->size();
    }
    cloudOut->reserve(total_pts);
    for (int i = 0; i < num_chunks; ++i) {
        if (filtered_chunks[i]) {
            *cloudOut += *filtered_chunks[i];
        }
    }

    RCLCPP_INFO(logger_,
                "Chunked VoxelGrid filtering complete (chunks: %d, input points: %zu -> filtered points: %zu).",
                num_chunks, cloudIn->size(), cloudOut->size());
}

// 显式模板实例化，确保链接器正确导出符号
template void MapPavingEngine::safeVoxelFilter<PointTypeRGB>(const pcl::PointCloud<PointTypeRGB>::Ptr&,
                                                            pcl::PointCloud<PointTypeRGB>::Ptr&,
                                                            float, float);
template void MapPavingEngine::safeVoxelFilter<pcl::PointXYZ>(const pcl::PointCloud<pcl::PointXYZ>::Ptr&,
                                                             pcl::PointCloud<pcl::PointXYZ>::Ptr&,
                                                             float, float);
