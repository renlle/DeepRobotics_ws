[机器狗侧topic] + ai注释
orin@ubuntu:~/Desktop$ ros2 topic list -t

/BATTERY_CHARGE_ENABLE [drdds/msg/StdMsgInt32]        # 电池充电使能开关（0/1）
/BATTERY_DATA [drdds/msg/BatteryData]                 # 电池状态：电压、电流、电量、温度等
/CHARGE_CMD [drdds/msg/Steer]                         # 充电对接运动指令（速度+角速度）
/CHARGE_STATUS [drdds/msg/StdStatus]                  # 充电对接状态反馈
/CPU_103 [drdds/msg/CpuData]                          # Orin CPU 核心/温度 zone 103 监控
/CPU_104 [drdds/msg/CpuData]                          # Orin CPU zone 104 监控
/CPU_106 [drdds/msg/CpuData]                          # Orin CPU zone 106 监控
/FAULT_STATUS [drdds/msg/FaultStatus]                 # 整机故障码汇总（各模块告警）
/FIBOCOM/net_rtk/gngga [fibocom_msgs/msg/FibocomRtkgngga]   # 广和通 5G-RTK 的 GNGGA 差分数据
/FIBOCOM/net_rtk/heading [fibocom_msgs/msg/FibocomRtkheading] # 广和通 RTK 航向（双天线解算）
/GAIT [drdds/msg/Gait]                               # 当前步态模式（walk/trot/run/stand 等）
/GLOBAL_PLANNER_STATUS [drdds/msg/PlannerStatus]      # 全局规划器状态
/GPS [drdds/msg/NavSat]                               # GPS 定位数据（经纬度、海拔、状态）
/GPS_SYS_MODE [drdds/msg/StdMsgInt32]                 # GPS 工作模式（单点/RTK/DGNSS）
/GRIDS_ID [drdds/msg/GridsID]                         # 局部栅格地图分块 ID（用于多机/增量更新）
/GRID_MAP [nav_msgs/msg/OccupancyGrid]                # 局部可通行栅格地图（避障用）
/HEIGHT_IMAGE [sensor_msgs/msg/Image]                 # 高程图影像（地形感知输出的深度/高度图）
/HEIGHT_MAP_STATUS [drdds/msg/StdStatus]              # 高程图模块状态
/HES_STATUS [drdds/msg/StdMsgInt32]                   # Height Estimation Status，机身离地高度估计状态
/IMU [sensor_msgs/msg/Imu]                            # 机身主体 IMU（滤波后输出）
/JOINTS_CMD [drdds/msg/JointsDataCmd]                 # 关节控制指令（12 个关节的 pos/vel/effort）
/JOINTS_DATA [drdds/msg/JointsData]                   # 关节反馈数据（实际 pos/vel/trq，全频）
/JOINTS_DATA_10HZ [drdds/msg/JointsData]              # 关节数据降采样 10Hz 版本（省带宽）
/LIDAR/IMU201 [sensor_msgs/msg/Imu]                    # 雷达 1 内置 IMU（201 为雷达编号）
/LIDAR/IMU202 [sensor_msgs/msg/Imu]                    # 雷达 2 内置 IMU（202 为雷达编号）
/LIDAR/STATUS [drdds/msg/AiryLidarStatus]             # 激光雷达状态（Airy 是雷达型号名）
/LOCATION_STATUS [drdds/msg/LocationStatus]           # 定位模块状态（LIO/视觉/GPS 融合）
/LOCATION_STATUS/MATCHING_ERROR [std_msgs/msg/Float64MultiArray]  # 定位匹配残差（扫描匹配误差）
/MOTION_INFO [drdds/msg/MotionInfo]                   # 运动层信息（上层下发的速度/模式）
/MOTION_STATE [drdds/msg/MotionState]                 # 运动状态机（站立/行走/倒地/恢复等）
/MOTION_STATUS [drdds/msg/MotionStatus]               # 运动模块健康状态
/NAV_CMD [drdds/msg/NavCmd]                           # 导航指令（目标点 + 行为）
/ODOM [nav_msgs/msg/Odometry]                          # 本体里程计（滤波后位姿 + 线/角速度）
/OOA_STATUS [drdds/msg/StdStatus]                     # 厂商自定义，疑似 Out-of-Area / 避障相关状态
/PLANNER_STATUS [drdds/msg/PlannerStatus]             # 局部规划器状态
/REAL_STEER [drdds/msg/Steer]                         # 实际执行的机身速度指令（反馈用，区别于下发 STEER）
/STEER [drdds/msg/Steer]                              # 机身速度指令（vx/vy/wz，上层下发）
/TERRAIN_CLASSIFIER_STATUS [drdds/msg/StdStatus]       # 地形分类器状态（草地/楼梯/碎石等识别）
/UWB_ODOM_LOST [drdds/msg/StdMsgInt32]                # UWB 里程计丢失标志
/UWB_ONLINE [drdds/msg/StdMsgInt32]                   # UWB 基站在线状态
/accumulate_cloud/status_code [std_msgs/msg/Int32]     # 累积点云模块状态码
/fibo_fusion_pose [nav_msgs/msg/Odometry]              # 广和通 RTK+IMU 融合位姿输出
/fibo_fusion_state [fibocom_msgs/msg/FiboFusionState]  # 广和通融合定位状态（RTK fix/float 等）
/initialpose [geometry_msgs/msg/PoseWithCovarianceStamped]  # 人工给定初始位姿（重定位用）
/parameter_events [rcl_interfaces/msg/ParameterEvent]   # ROS2 参数变更事件（标准）
/passable_status_code [std_msgs/msg/Int32]             # 可通行性判断状态码
/rosout [rcl_interfaces/msg/Log]                       # ROS2 日志输出（标准）
/tf [tf2_msgs/msg/TFMessage]                           # TF 坐标变换树（标准）
---
判断是否停止:

方法 A：读取 /ODOM的速度分量
if abs(msg.twist.twist.linear.x) < 0.01 and \
   abs(msg.twist.twist.linear.y) < 0.01 and \
   abs(msg.twist.twist.angular.z) < 0.01:
    is_stopped = True
阈值可根据实际场景调整（0.01 m/s, 0.01 rad/s）。

方法 B：结合 /MOTION_STATE
def motion_state_callback(msg):
    # 假设 MotionState 中有 state 字段（需查看接口定义）
    # 常见值：0=STAND, 1=WALK, 2=TROT, 3=RUN, 4=RECOVER...
    if msg.state == 0:   # STAND
        is_stopped = True
注：drdds/msg/MotionState的具体字段需要通过 ros2 interface show drdds/msg/MotionState确认。一般包含 state枚举。




---
几个值得提的点：
drdds​ 是云深处（DeepRobotics）私有消息包，StdMsgInt32 / StdStatus这种是通用包装，里面就一个 int32 + 时间戳，用来省事地发状态量。
Steer​ 不是汽车转向，是厂商把"机身速度指令 (vx, vy, wz)"起名叫 Steer 了，STEER是下发、REAL_STEER是执行反馈，可以用来比对有没有被运动层限幅。
CPU_103/104/106​ 对应 Orin 上三个温度 zone（Denver+A78 簇的划分），不是三个物理 CPU。
FIBOCOM​ 是广和通 5G+RTK 模组，走 net_rtk 给 GNGGA 和双天线 heading，给 /fibo_fusion_pose用。
OOA_STATUS​ 没在公开文档里找到释义，M20 代码里跟 UWB/定位放一起，倾向是 Out-of-Area（UWB 基站覆盖范围外）或避障子模块状态，不确定就先按厂商自定义标着。
如果你想挑其中某几个 topic 看字段（ros2 interface show drdds/msg/JointsData那种）或者想对照 M20 的 SDK 源码里哪个节点发哪个，告诉我。
