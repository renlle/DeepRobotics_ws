# Domain 0 仿真机器人接入 Domain 42 算法

## 一、结论

**问：在仿真 ISSC-AAC 中启动一个 `ROS_DOMAIN_ID=0` 的机器人，可以和当前项目的 Domain 42 算法连接吗？**

答：可以，但当前项目只现成实现了一个方向：

```text
Domain 42：SLAM / 定位 / Nav2 / RViz
    │
    │ /NAV_CMD
    ▼
nav_cmd_domain_bridge
    │
    ▼
Domain 0：ISSC-AAC 仿真机器人
```

导航启动后，`nav_cmd_domain_bridge` 在 Domain 42 订阅 `drdds/msg/NavCmd` 类型的 `/NAV_CMD`，再在 Domain 0 发布同类型 `/NAV_CMD`。因此 Domain 0 的蓝色 Box 只要订阅该话题并把速度指令转换为仿真运动，就能接收现有算法命令。

当前桥不转发 Domain 0 的机器人状态、仿真位姿、TF、里程计或传感器到 Domain 42。因此“能收到速度命令”不等于“已经组成闭环导航”。

---

## 二、现有跨域能力

**问：当前桥具体转发哪些内容？**

答：只转发 `/NAV_CMD` 中以下字段：

```text
header.stamp.sec
header.stamp.nanosec
header.frame_id
data.x_vel
data.y_vel
data.yaw_vel
```

实现位置：

```text
navigation/move/move/nav_cmd_domain_bridge.py
```

桥的两个子进程分别运行在：

```text
订阅端：ROS_DOMAIN_ID=42
发布端：ROS_DOMAIN_ID=0
```

**问：当前没有桥接什么？**

答：没有现成的 Domain 0 → Domain 42 反馈桥，至少包括：

```text
/ODOM
/REAL_STEER
/MOTION_STATE
/MOTION_STATUS
/FAULT_STATUS
/IMU
/JOINTS_DATA_10HZ
/BATTERY_DATA
/tf
/tf_static
```

所以 Domain 42 目前无法确认蓝色 Box 是否收到命令、是否真的移动、实际速度是多少，也不知道仿真机器人是否发生故障。

---

## 三、蓝色 Box 的最小实现

**问：Domain 0 的蓝色 Box 最少需要实现什么？**

答：第一阶段只验证跨域控制时，至少实现：

1. 运行环境设置 `ROS_DOMAIN_ID=0`。
2. 安装并加载与本项目一致的 `drdds/msg/NavCmd` 消息定义。
3. 订阅 `/NAV_CMD`。
4. 使用 `x_vel`、`y_vel`、`yaw_vel` 更新 Box 的平面位置和朝向。
5. 设置速度命令 watchdog；超过指定时间未收到新命令，立即将速度归零。
6. 在仿真退出、暂停、故障或急停时强制归零。

推荐运动学关系：

```text
世界系速度 X = cos(yaw) * x_vel - sin(yaw) * y_vel
世界系速度 Y = sin(yaw) * x_vel + cos(yaw) * y_vel
yaw_dot       = yaw_vel
```

其中 `x_vel` 和 `y_vel` 是机器人机体系速度。M20 支持全向运动时可以保留横向速度；如果蓝色 Box 暂时按差速模型实现，应明确忽略或限制 `y_vel`，否则仿真行为与 M20 不一致。

**问：watchdog 怎么设置比较合适？**

答：建议 Domain 0 仿真控制器设置 0.2～0.5 秒命令超时。导航适配器正常约 15 Hz 发布 `/NAV_CMD`；一旦桥或算法停止，Box 应在超时后自动停止，不能保持最后一条速度持续运动。

---

## 四、建议的闭环仿真层级

**问：只让蓝色 Box 订阅 `/NAV_CMD`，可以验证什么？**

答：只能验证：

- Domain 42 到 Domain 0 的发现和通信。
- 自定义 `NavCmd` 消息是否兼容。
- 速度方向、符号、单位和限幅是否正确。
- Domain 0 控制器能否根据命令移动 Box。

它不能验证当前 SLAM/Nav2 是否根据 Box 的真实运动持续更新，因为算法域没有收到 Box 运动后的传感器数据。

**问：怎样实现真正闭环？**

答：有两种方案。

### 方案 A：传感器仿真直接放在 Domain 42

推荐拓扑：

```text
ISSC-AAC 物理机器人 / Box：Domain 0
    │
    ├─ 接收 /NAV_CMD
    └─ 仿真运动

ISSC-AAC 传感器 ROS 发布节点：Domain 42
    │
    ├─ /livox/lidar
    ├─ /livox/imu
    ├─ /left_camera/image
    └─ /clock
        │
        ▼
FAST-LIVO / 定位 / Nav2 / RViz
```

优点是保持现有项目边界：算法传感器一直在 Domain 42，底盘控制一直在 Domain 0，不需要桥接大带宽点云和图像。

### 方案 B：所有仿真话题先发布在 Domain 0，再做白名单桥接

需要把以下话题从 Domain 0 桥到 Domain 42：

```text
/clock
/livox/lidar
/livox/imu
/left_camera/image
```

可选反馈：

```text
/ODOM
/REAL_STEER
/MOTION_STATE
/FAULT_STATUS
```

不建议直接桥接 Domain 0 的整棵 `/tf` 和 `/tf_static`。本项目在 Domain 42 自己生成定位 TF；把 Domain 0 的同名 TF 全量桥入容易形成多父节点、重复发布者或时间戳冲突。若确实需要仿真 TF，应使用独立 frame 名，例如 `sim_world`、`sim_base_link`，并明确唯一变换发布者。

**问：更推荐哪个方案？**

答：推荐方案 A。Domain 0 保持机器狗底盘接口，Domain 42 直接接收仿真传感器，最符合当前项目设计，也避免跨域传输相机和点云的大带宽开销。

---

## 五、算法需要哪些仿真输入？

**问：要让 FAST-LIVO 和定位真正跟随蓝色 Box 运动，至少需要哪些传感器？**

答：当前配置启用视觉融合时需要：

```text
/livox/lidar          Livox 点云
/livox/imu            IMU
/left_camera/image    相机图像
/clock                use_sim_time=true 时必需
```

还需要正确的：

- 雷达、IMU、摄像头时间戳。
- 相机内参和畸变参数。
- `T_camera_lidar`。
- `T_imu_lidar`。
- `T_lidar_base`。
- 与配置一致的话题类型和 frame 约定。

如果 ISSC-AAC 暂时只能模拟雷达和 IMU，可在仿真专用设备参数中将 `img_en` 设为 `0`，先验证雷达惯性定位。不要修改生产实机参数文件；应使用独立仿真 profile。

**问：仅发布 `/ODOM` 能替代传感器吗？**

答：不能直接替代当前 FAST-LIVO 输入。项目的 2D `/odom` 是由 FAST-LIVO 的 `/aft_mapped_to_init` 再投影生成，而不是直接消费 Domain 0 `/ODOM`。如果要让 Nav2 直接使用仿真真值 Odom，需要新增一套仿真专用启动路径，绕过 FAST-LIVO，并统一 `map → odom → base_footprint` TF；当前代码没有现成实现。

---

## 六、RViz 能否正常打开？

**问：Domain 0 启动蓝色 Box 后，`rviz_slam.sh` 可以正常打开吗？**

答：可以打开。脚本默认运行在算法域：

```bash
export ROS_DOMAIN_ID=42
```

它不需要直接加入 Domain 0，也不应改成 Domain 0。RViz 观察的是 Domain 42 中的地图、定位、路径、代价地图和航点；Domain 0 只负责接收 `/NAV_CMD` 并驱动仿真机器人。

启动：

```bash
cd /home/ubuntu/Desktop/ROS2_szs/DeepRobotics_ws
./rviz_slam.sh
```

运行时配置保存到：

```text
./tmp/deeprobotics_slam_${USER}.rviz
```

**问：RViz 打开后为什么可能仍然只有空网格？**

答：RViz 进程打开和可视化数据正常是两件事。当前脚本 Fixed Frame 为 `map`，正常显示至少需要：

- Domain 42 中存在 `/map` 或 `/map_2d`。
- 存在连通且时间戳有效的 `map → camera_init_footprint → base_footprint` TF。
- 定位后端和 Map Server 已启动。
- 使用仿真时间时，Domain 42 中持续发布 `/clock`。
- 显示点云时，相关点云 frame 能转换到 `map`。

只在 Domain 0 移动一个 Box，而不向 Domain 42 提供传感器或位姿反馈，RViz 不会自动知道 Box 的新位置。此时 RViz 可以打开，但地图中的机器人位置不会随 Box 形成真实闭环更新。

**问：能在 RViz 中显示蓝色 Box 吗？**

答：当前 `rviz_slam.sh` 没有订阅 Domain 0，也没有 M20/Box 的 URDF，因此不会直接显示 Domain 0 的蓝色 Box。建议采用以下一种方式：

1. 在 Domain 42 发布 `visualization_msgs/msg/Marker`，用独立话题 `/sim_robot_marker` 显示 Box。
2. 在 Domain 42 发布仿真专用 TF 和 `robot_description`，由 `robot_state_publisher` + RobotModel 显示。
3. 由 ISSC-AAC 同时提供 Domain 42 的可视化镜像节点，但只转发仿真显示所需位姿，不桥接整棵 TF。

最简单的是 Marker。Marker 的 frame 应选 `map` 或独立 `sim_world`；如果使用 `sim_world`，还必须提供 `map ↔ sim_world` 的明确变换。

---

## 七、推荐的 Domain 0 话题

**问：蓝色 Box 建议发布哪些话题？**

答：按阶段建议如下。

### 最小控制验证

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/NAV_CMD` | `drdds/msg/NavCmd` | 接收算法速度命令 |
| 发布 | `/REAL_STEER` | 建议与真机保持相同类型 | 回报实际采用的 x/y/yaw 速度 |
| 发布 | `/MOTION_STATE` | 建议与真机保持相同类型 | 站立、运动、停止、急停状态 |
| 发布 | `/FAULT_STATUS` | 建议与真机保持相同类型 | 故障和安全联锁 |

### 闭环与诊断增强

| 话题 | 说明 |
|---|---|
| `/ODOM` | Domain 0 仿真底盘真值或估计里程计，用于对比算法 `/odom` |
| `/IMU` | 仿真机身 IMU 反馈，用于诊断，不直接替代 `/livox/imu` |
| `/JOINTS_DATA_10HZ` | 后续替换蓝色 Box 为 M20 模型时驱动关节显示 |
| `/BATTERY_DATA` | 仿真电量与低电量策略 |
| `/MOTION_STATUS` | 运动控制器健康状态 |

这些反馈若要让 Domain 42 使用，需要新增白名单反馈桥。第一阶段可以分别在 Domain 0 终端查看，不必立刻全部桥入算法域。

**问：蓝色 Box 的 frame 建议怎么命名？**

答：避免与算法域已有 frame 冲突，Domain 0 建议使用：

```text
sim_world
sim_odom
sim_base_link
```

不要在尚未设计桥接关系时直接发布 `map`、`camera_init`、`base_link`、`base_footprint` 等算法域同名 TF。后续决定把仿真真值用于导航时，再通过单一桥或转换节点建立明确关系。

---

## 八、建议的安全机制

**问：Domain 0 控制器有哪些必须做的安全处理？**

答：建议至少实现：

1. `/NAV_CMD` 超时自动零速。
2. 接收到 NaN、Inf 或超限速度时拒绝并零速。
3. 对 `x_vel`、`y_vel`、`yaw_vel` 二次限幅。
4. 仿真暂停、重置、失焦、急停或碰撞时清零。
5. 记录最后命令时间和最后执行速度。
6. 桥或订阅者重连后，必须等待新命令，不得恢复旧速度。
7. 不使用无限队列执行历史速度命令，只采用最新命令。
8. 提供明确的 enabled/armed 状态，默认未使能。

当前桥使用无界进程队列，并且桥退出时没有保证向 Domain 0 发送最后一条零速。因此 Domain 0 仿真控制器自己的 watchdog 是必需项，而不是可选优化。

---

## 九、启动与验证

**问：怎样验证 Domain 42 → Domain 0 已经打通？**

答：先启动算法、2D 定位和导航。导航 Launch 会自动启动 `nav_cmd_domain_bridge`。

Domain 42 终端：

```bash
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
source /home/ubuntu/Desktop/ROS2_szs/DeepRobotics_ws/install/setup.bash

ros2 topic info -v /NAV_CMD
ros2 topic hz /NAV_CMD
```

应看到算法发布者以及 `nav_cmd_bridge_sub` 订阅者。

Domain 0 终端：

```bash
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
source /home/ubuntu/Desktop/ROS2_szs/DeepRobotics_ws/install/setup.bash

ros2 topic info -v /NAV_CMD
ros2 topic echo /NAV_CMD
```

应看到 `nav_cmd_bridge_pub` 发布者，以及蓝色 Box 控制节点订阅者。

**问：怎样做首次低风险测试？**

答：建议：

1. Box 默认禁用运动，只打印 `/NAV_CMD`。
2. 确认 x/y/yaw 字段和单位正确。
3. 启用运动，但将速度限幅降到 `0.1 m/s`、角速度降到 `0.2 rad/s`。
4. 在 RViz 只下发近距离航点。
5. 确认 Domain 0 `/REAL_STEER` 与 `/NAV_CMD` 一致。
6. 停止导航，确认 Box 在 watchdog 时间内停下。
7. 直接终止桥，确认 Box 同样自动停下。
8. 通过后再逐步增加速度和传感器闭环。

**问：怎样判断完整导航闭环成立？**

答：至少同时满足：

```text
Domain 42 /NAV_CMD 持续生成
Domain 0 蓝色 Box 接收并执行 /NAV_CMD
Box 运动改变仿真雷达、IMU、相机数据
仿真传感器进入 Domain 42
FAST-LIVO /odom 和 TF 随 Box 真实运动更新
定位在地图中稳定
Nav2 根据更新后的机器人位置推进航点
停止或故障时 Domain 0 watchdog 自动零速
```

如果只有 `/NAV_CMD` 能使 Box 移动，而 Box 运动没有反过来改变 Domain 42 的传感器和定位，那么仍然只是单向控制演示，不是闭环导航仿真。

---

## 十、推荐实施顺序

**问：从当前蓝色 Box 开始，建议怎么迭代？**

答：

1. **跨域通路**：Box 订阅 Domain 0 `/NAV_CMD`，实现 watchdog 和低速运动。
2. **反馈可观测**：发布 `/REAL_STEER`、`/MOTION_STATE`、`/FAULT_STATUS`。
3. **RViz 镜像**：在 Domain 42 发布 `/sim_robot_marker`，观察 Box 真值位置。
4. **传感器闭环**：从 ISSC-AAC 生成雷达和 IMU，直接发布到 Domain 42。
5. **视觉闭环**：增加相机图像并校准内外参，启用视觉融合。
6. **定位闭环**：验证 FAST-LIVO、全局重定位、TF 和 `/localization_status`。
7. **导航闭环**：验证全局路径、局部避障、航点推进和停止逻辑。
8. **M20 模型化**：用 M20 尺寸、碰撞体、足端或轮廓替换简单 Box。
9. **安全联锁**：增加定位状态、故障状态、命令超时和急停门禁。

最优先不是把蓝色 Box 做得像机器狗，而是先保证坐标系、速度方向、时间、跨域通信、watchdog 和传感器闭环正确。
