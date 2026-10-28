# FAST-LIVO IMU 与外参数据约定核查

本文根据当前活动 YAML、GUI 启动链路、Livox 驱动发布代码、FAST-LIVO 消费代码和实机 Bag 整理。以下内容是当前算法端的实际配置和代码约定，不是仿真侧假设。

## 一、结论摘要

| 项目 | 当前实际约定 |
|---|---|
| 算法域 | `ROS_DOMAIN_ID=42` |
| IMU 话题 | `/livox/imu` |
| IMU 消息类型 | `sensor_msgs/msg/Imu` |
| IMU `frame_id` | `livox_frame` |
| IMU 频率 | 约 `200 Hz` |
| 角速度单位 | `rad/s` |
| 线加速度单位 | `m/s²` |
| 重力约定 | `linear_acceleration` 包含静止重力比力，不能预先去重力 |
| 当前活动 YAML | `/home/ubuntu/Desktop/ROS2_szs/DeepRobotics_ws/config/device_parameters.yaml` |
| `T_imu_lidar` 代码方向 | LiDAR → IMU，`p_imu = T_imu_lidar × p_lidar` |
| `T_lidar_base` 代码方向 | `p_lidar = T_lidar_base × p_base` |

## 二、FAST-LIVO 实际 YAML 路径

当前 GUI 启动 FAST-LIVO 时显式传入：

```text
/home/ubuntu/Desktop/ROS2_szs/DeepRobotics_ws/config/device_parameters.yaml
```

对应启动形式：

```bash
ros2 launch fast_livo frontend.launch.py \
  params_file:=/home/ubuntu/Desktop/ROS2_szs/DeepRobotics_ws/config/device_parameters.yaml
```

代码位置：

- GUI 参数传递：`robot_console/robot_console/main_window.py:977-1001`
- Launch 加载参数：`fast_livo/launch/frontend.launch.py:43-60`

手工启动时应显式传入绝对 `params_file`。`frontend.launch.py` 内部推导的默认路径仍包含旧的 `src/` 目录结构，在当前工作空间布局下不应作为可靠路径使用。

当前活动 YAML 使用通配节点参数：

```yaml
/**:
  ros__parameters:
    ros_domain_id: 42
    common:
      img_topic: "/left_camera/image"
      lid_topic: "/livox/lidar"
      imu_topic: "/livox/imu"
```

配置位置：`config/device_parameters.yaml:3-5,62-69`。

## 三、`/livox/imu` 的 frame_id

当前预期值为：

```text
livox_frame
```

消息形式：

```yaml
header:
  frame_id: livox_frame
```

Livox 驱动中直接写入：

```cpp
imu_msg.header.frame_id = "livox_frame";
```

代码位置：`livox_ros_driver2/src/lddc.cpp:602-618`。

注意事项：

- 驱动把 SDK 提供的三轴陀螺仪和加速度值直接写入 ROS 消息，没有在发布处进行轴交换、旋转或符号翻转。
- FAST-LIVO 的 IMU 回调只处理时间戳和缓存，不检查 `header.frame_id`。
- FAST-LIVO 不会通过 TF 自动把 IMU 数据旋转到标定坐标系。
- 仿真端应发布 `frame_id: livox_frame`，并确保三轴数值使用与外参标定对应的 IMU 坐标轴。

## 四、IMU 发布频率

项目约定频率为：

```text
200 Hz
```

依据：

1. GUI 将 `/livox/imu` 的目标监控频率配置为 `200 Hz`，位置为 `robot_console/robot_console/main_window.py:385-390`。
2. 实机 Bag `data/bags/bag_20260607_173905/metadata.yaml` 中：

```text
持续时间：181.010769142 s
IMU 数量：36204
```

平均频率为：

```text
36204 / 181.010769142 ≈ 200.009 Hz
```

仿真发布建议：

```text
频率：200 Hz
周期：约 0.005 s
```

FAST-LIVO 没有硬编码要求输入必须恰好为 200 Hz，也不对输入重采样，但 IMU 积分依赖消息时间戳。仿真端应保证：

- 时间戳严格递增。
- 不重复时间戳。
- 采样周期尽量稳定。
- 时间戳表示仿真采样时刻，而不是 ROS 消息到达时刻。
- 使用 `use_sim_time=true` 时，IMU 时间戳与 `/clock` 属于同一时间域。

MID360 Launch 中的 `publish_freq: 10.0` 是点云发布频率，不是 IMU 发布频率。

## 五、IMU 加速度与重力约定

FAST-LIVO 要求 `sensor_msgs/msg/Imu.linear_acceleration` 包含加速度计测得的静止重力比力，不能在发布前去除重力。

单位约定：

```text
angular_velocity：rad/s
linear_acceleration：m/s²
```

FAST-LIVO 初始化时直接使用 `linear_acceleration` 均值估计重力：

```cpp
state_inout.gravity =
    -mean_acc / mean_acc.norm() * G_m_s2;
```

代码位置：`fast_livo/src/IMU_Processing.cpp:97-137`。

系统重力常数：

```text
G_m_s2 = 9.81 m/s²
```

如果 IMU 静止且自身 `+Z` 朝上，典型输入应为：

```yaml
angular_velocity:
  x: 0.0
  y: 0.0
  z: 0.0

linear_acceleration:
  x: 0.0
  y: 0.0
  z: 9.81
```

不能发布为：

```yaml
linear_acceleration:
  x: 0.0
  y: 0.0
  z: 0.0
```

在上述静止姿态下，FAST-LIVO 初始化得到的世界重力约为：

```text
[0, 0, -9.81] m/s²
```

积分时的计算关系为：

```text
世界系运动加速度 = R_world_imu × IMU比力 + 世界重力
```

对应代码：`fast_livo/src/IMU_Processing.cpp:391-401`。

当前配置还启用了：

```yaml
uav:
  gravity_align_en: true
```

配置位置：`config/device_parameters.yaml:118-120`。

## 六、`T_imu_lidar`

当前活动配置值：

```text
T_imu_lidar =
[ 1.000000  0.000000  0.000000  -0.011000
  0.000000  1.000000  0.000000  -0.023290
  0.000000  0.000000  1.000000   0.044120
  0.000000  0.000000  0.000000   1.000000 ]
```

来源：`config/device_parameters.yaml:17-24`。

### 实际数学方向

YAML 注释目前写的是“IMU 到激光雷达（imu -> lidar）”，但 FAST-LIVO 代码实际按以下公式使用：

```text
p_imu = T_imu_lidar × p_lidar
```

因此严格含义为：

```text
T_imu_lidar = IMU_T_LiDAR
方向：LiDAR → IMU
```

FAST-LIVO 对 LiDAR 点执行：

```cpp
extR * p_lidar + extT
```

代码证据：

- 参数读取：`fast_livo/src/LIVMapper.cpp:177-188`
- 点坐标转换：`fast_livo/src/LIVMapper.cpp:775-795`
- IMU 去畸变外参：`fast_livo/src/IMU_Processing.cpp:70-83,495-503`

当前平移列表示 LiDAR 原点在 IMU 坐标系中的位置：

```text
x = -0.011000 m
y = -0.023290 m
z =  0.044120 m
```

仿真端应以代码公式为准，不要仅根据 YAML 注释把矩阵取逆。

## 七、`T_lidar_base`

当前活动配置值：

```text
T_lidar_base =
[  0.881054   0.016898  -0.472712  -0.140800
  -0.020738   0.999781  -0.002914  -0.006440
   0.472560   0.012370   0.881212  -0.430040
   0.000000   0.000000   0.000000   1.000000 ]
```

来源：`config/device_parameters.yaml:6-12`。

FAST-LIVO 和定位模块按以下顺序右乘：

```text
T_world_base = T_world_lidar × T_lidar_base
```

代码证据：

- FAST-LIVO：`fast_livo/src/LIVMapper.cpp:1504-1509`
- 2D 定位桥：`localization/src/odom_bridge.cpp:134-144`

严格的点坐标变换含义为：

```text
p_lidar = T_lidar_base × p_base
T_lidar_base = LiDAR_T_base
```

平移列表示 `base_link` 原点在 LiDAR/body 坐标系中的位置：

```text
x = -0.140800 m
y = -0.006440 m
z = -0.430040 m
```

把 LiDAR 点转换到 `base_link` 时应使用：

```text
p_base = inverse(T_lidar_base) × p_lidar
```

不能使用：

```text
p_base = T_lidar_base × p_lidar
```

配置中的“lidar -> base_link”可理解为 TF 父子边描述，不能直接等同于点坐标转换方向。

## 八、仿真端接口摘要

ISSC-AAC 仿真 IMU 建议严格遵循：

```yaml
topic: /livox/imu
type: sensor_msgs/msg/Imu
frame_id: livox_frame
frequency: 200.0
angular_velocity_unit: rad/s
linear_acceleration_unit: m/s^2
linear_acceleration_contains_gravity: true
```

外参公式：

```text
p_imu = T_imu_lidar × p_lidar
T_world_base = T_world_lidar × T_lidar_base
```

这两个矩阵是当前实机配置值。如果仿真传感器安装位置与实机不同，应采用以下一种方式：

1. 让仿真模型严格复现当前实机的传感器相对位姿。
2. 为仿真建立独立 YAML，并填入仿真的真实外参。

不要为适配仿真直接修改生产实机的 `config/device_parameters.yaml`。

## 九、运行核查命令

在算法域检查：

```bash
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
source /home/ubuntu/Desktop/ROS2_szs/DeepRobotics_ws/install/setup.bash

ros2 topic info -v /livox/imu
ros2 topic hz /livox/imu
ros2 topic echo /livox/imu --once
```

重点确认：

```text
类型：sensor_msgs/msg/Imu
frame_id：livox_frame
频率：约 200 Hz
静止时加速度模长：约 9.81 m/s²
时间戳：持续递增并与 /clock 同域
```

可计算静止加速度模长：

```text
sqrt(ax² + ay² + az²) ≈ 9.81 m/s²
```

若模长接近 `0`，通常表示仿真端提前去除了重力，不符合当前 FAST-LIVO 输入约定。
