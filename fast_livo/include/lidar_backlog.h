#ifndef FAST_LIVO_LIDAR_BACKLOG_H
#define FAST_LIVO_LIDAR_BACKLOG_H

#include <cstddef>

namespace fast_livo {

// 去畸变中的 IMU 位姿以旧积分终点计时；扫描内部时间需要加上被跳过的间隔。
template <typename Cloud>
void offset_scan_time(Cloud& cloud, double gap_seconds) {
    for (auto& point : cloud.points) point.curvature += gap_seconds * 1000.0;
}

// 只跳过已有完整 IMU 覆盖的旧扫描；IMU 队列仍完整积分到新扫描末端。
template <typename CloudQueue, typename TimeQueue>
std::size_t trim_lidar_backlog(CloudQueue& clouds, TimeQueue& times,
                             double latest_imu_time) {
    std::size_t selected = 0;
    for (std::size_t i = 1; i < clouds.size(); ++i) {
        if (!clouds[i] || clouds[i]->points.size() <= 1) continue;
        const double end = times[i] + clouds[i]->points.back().curvature / 1000.0;
        if (end <= latest_imu_time) selected = i;
    }
    for (std::size_t i = 0; i < selected; ++i) {
        clouds.pop_front();
        times.pop_front();
    }
    return selected;
}

}  // namespace fast_livo

#endif
