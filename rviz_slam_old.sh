#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
export ROS_DOMAIN_ID="42"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"

export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/deeprobotics_rviz_logs}"
mkdir -p "${ROS_LOG_DIR}"

source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
    source "${WORKSPACE_DIR}/install/setup.bash"
fi

CONFIG_FILE="./tmp/deeprobotics_slam_${USER}.rviz"
cat > "${CONFIG_FILE}" <<'RVIZ_CONFIG'
Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: 地面网格
      Enabled: true
      Cell Size: 1
      Alpha: 0.35
      Reference Frame: <Fixed Frame>
    - Class: rviz_default_plugins/TF
      Name: TF 坐标系
      Enabled: true
      Frame Timeout: 15
      Marker Scale: 0.5
      Show Arrows: true
      Show Axes: true
      Show Names: true
    - Class: rviz_default_plugins/PointCloud2
      Name: 前端实时配准点云
      Enabled: true
      Topic:
        Value: /cloud_registered
        Reliability Policy: Best Effort
        Durability Policy: Volatile
      Style: Points
      Size (Pixels): 2
      Alpha: 0.8
      Color Transformer: AxisColor
      Axis: Z
      Decay Time: 3
    - Class: rviz_default_plugins/PointCloud2
      Name: 后端全局激光地图
      Enabled: true
      Topic:
        Value: /Laser_map
        Reliability Policy: Best Effort
        Durability Policy: Volatile
      Style: Points
      Size (Pixels): 1
      Alpha: 0.35
      Color Transformer: AxisColor
      Axis: Z
      Decay Time: 0
    - Class: rviz_default_plugins/PointCloud2
      Name: 有效特征点
      Enabled: true
      Topic:
        Value: /cloud_effected
        Reliability Policy: Best Effort
        Durability Policy: Volatile
      Style: Points
      Size (Pixels): 3
      Alpha: 1
      Color Transformer: Intensity
      Decay Time: 1
    - Class: rviz_default_plugins/PointCloud2
      Name: 定位当前扫描
      Enabled: true
      Topic:
        Value: /global_localization/cur_scan
        Reliability Policy: Best Effort
        Durability Policy: Volatile
      Style: Points
      Size (Pixels): 4
      Alpha: 1
      Color Transformer: AxisColor
      Axis: Z
      Decay Time: 1
    - Class: rviz_default_plugins/PointCloud2
      Name: 定位搜索子地图
      Enabled: true
      Topic:
        Value: /global_localization/submap_highlight
        Reliability Policy: Reliable
        Durability Policy: Volatile
      Style: Points
      Size (Pixels): 3
      Alpha: 0.7
      Color Transformer: FlatColor
      Color: 255; 80; 80
      Decay Time: 0
    - Class: rviz_default_plugins/Path
      Name: 前端原始轨迹
      Enabled: true
      Topic:
        Value: /path
        Reliability Policy: Reliable
        Durability Policy: Volatile
      Line Style: Billboards
      Line Width: 0.08
      Color: 0; 255; 0
    - Class: rviz_default_plugins/Path
      Name: 后端优化轨迹
      Enabled: true
      Topic:
        Value: /aft_pgo_path
        Reliability Policy: Reliable
        Durability Policy: Volatile
      Line Style: Billboards
      Line Width: 0.12
      Color: 0; 170; 255
    - Class: rviz_default_plugins/Path
      Name: 定位轨迹
      Enabled: true
      Topic:
        Value: /global_localization/path
        Reliability Policy: Reliable
        Durability Policy: Volatile
      Line Style: Billboards
      Line Width: 0.12
      Color: 255; 170; 0
    - Class: rviz_default_plugins/MarkerArray
      Name: 回环约束
      Enabled: true
      Topic:
        Value: /loop_closure_constraints
        Reliability Policy: Reliable
        Durability Policy: Volatile
    - Class: rviz_default_plugins/MarkerArray
      Name: 可视化标记
      Enabled: true
      Topic:
        Value: /visualization_marker
        Reliability Policy: Reliable
        Durability Policy: Volatile
    - Class: rviz_default_plugins/Image
      Name: 相机图像
      Enabled: true
      Topic:
        Value: /rgb_img
        Reliability Policy: Best Effort
        Durability Policy: Volatile
      Normalize Range: true
    - Class: rviz_default_plugins/Odometry
      Name: 前端里程计
      Enabled: true
      Topic:
        Value: /aft_mapped_to_init
        Reliability Policy: Reliable
        Durability Policy: Volatile
      Shape:
        Axes Length: 0.5
        Axes Radius: 0.05
  Enabled: true
  Global Options:
    Background Color: 25; 25; 25
    Fixed Frame: camera_init
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
    - Class: rviz_default_plugins/SetInitialPose
      Topic:
        Value: /initialpose
    - Class: rviz_default_plugins/PublishPoint
      Topic:
        Value: /clicked_point
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Name: SLAM 3D
      Distance: 30
      Pitch: 0.65
      Yaw: 0.78
Window Geometry:
  Height: 900
  Width: 1500
RVIZ_CONFIG

echo "启动 RViz2：ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "固定坐标系：camera_init（定位模式可在 Global Options 中改为 map）"
exec rviz2 -d "${CONFIG_FILE}"
