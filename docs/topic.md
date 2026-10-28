[算法导航侧topic]

orin@ubuntu:~/workspace$ ros2 topic list -t

### 1. 传感器输入层 (Sensor Inputs) ###
/left_camera/image [sensor_msgs/msg/Image]  # 左侧相机的原始图像数据
/livox/imu [sensor_msgs/msg/Imu]  # Livox 激光雷达内置 IMU 数据
/livox/lidar [sensor_msgs/msg/PointCloud2]  # Livox 激光雷达原始点云数据
/points [sensor_msgs/msg/PointCloud]  # 标准点云格式数据
/rgb_img [sensor_msgs/msg/Image]  # 彩色相机图像数据
/scan [sensor_msgs/msg/LaserScan]  # 由 3D 点云截取的 2D 伪激光雷达扫描数据

### 2. 前端里程计与地图层 (Frontend Odometry & Mapping) ###
/LIVO2/imu_propagate [nav_msgs/msg/Odometry]  # Fast-LIVO2 里程计高频 IMU 预测位姿
/Laser_map [sensor_msgs/msg/PointCloud2]  # SLAM 系统构建或加载的三维点云全局地图
/aft_mapped_to_init [nav_msgs/msg/Odometry]  # Fast-LIVO2 输出的低频、高精度里程计位姿 (Map to Odom)
/cloud_effected [sensor_msgs/msg/PointCloud2]  # Fast-LIVO2 处理后参与优化的有效特征点云
/cloud_registered [sensor_msgs/msg/PointCloud2]  # Fast-LIVO2 经过运动畸变校正并在世界坐标系下的当前帧点云
/map [sensor_msgs/msg/PointCloud2]  # 用于导航或重定位的全局点云地图
/odom [nav_msgs/msg/Odometry]  # 底盘轮式里程计输出的相对位姿

### 3. 后端定位与跟踪层 (Backend Localization) ###
/cloud_visual_sub_map_before [sensor_msgs/msg/PointCloud2]  # 视觉模块用于配准前的点云局部子图 (调试用)
/cur_scan_in_map [sensor_msgs/msg/PointCloud2]  # 重定位模块中与全局地图对齐后的当前帧雷达点云
/global_localization/adaptive_search_radius [std_msgs/msg/Float64]  # 重定位模块自适应子图裁剪搜索半径
/initialpose_lidar [geometry_msgs/msg/PoseArray]  # 雷达(ScanContext)全局检索模块下发的三维重定位提议
/initialpose_visual [geometry_msgs/msg/PoseArray]  # 视觉(ORB/BoW)全局检索模块下发的三维重定位提议
/localization [nav_msgs/msg/Odometry]  # 系统的当前高精度融合定位位姿
/localization/trajectory [nav_msgs/msg/Path]  # 系统记录的定位历史轨迹
/localization_3d_confidence [std_msgs/msg/Float32]  # 3D 纯雷达/视觉重定位系统的置信度评估值
/localization_status [std_msgs/msg/Bool]  # 重定位是否成功并锁定的状态信号 (True/False)
/map_to_odom [nav_msgs/msg/Odometry]  # 重定位模块输出的 map 到 odom 坐标系的 TF 修正位姿
/mavros/vision_pose/pose [geometry_msgs/msg/PoseStamped]  # 发送给飞控(Mavros)的视觉/激光定位位姿

### 4. 环境感知与路径规划层 (Perception & Path Planning) ###
/behavior_tree_log [nav2_msgs/msg/BehaviorTreeLog]  # Nav2 行为树(BT)运行时的日志输出
/bt_goal_pose [geometry_msgs/msg/PoseStamped]  # Nav2 行为树当前执行的目标点位姿
/dyn_obj [sensor_msgs/msg/PointCloud2]          # 动态障碍物检测模块输出的动态物体点云
/dyn_obj_removed [sensor_msgs/msg/PointCloud2]  # 剔除动态障碍物后的静态环境点云
/evaluation [dwb_msgs/msg/LocalPlanEvaluation]  # DWB/MPPI 局部规划器对多条候选轨迹的打分评估
/final_goal [geometry_msgs/msg/PoseStamped]  # 机器人导航的最终目的地
/global_costmap/costmap [nav_msgs/msg/OccupancyGrid]  # Nav2 全局代价地图
/global_costmap/costmap_raw [nav2_msgs/msg/Costmap]  # 未完全处理的全局代价地图原始数据
/global_costmap/costmap_updates [map_msgs/msg/OccupancyGridUpdate]  # 全局代价地图的增量更新区域
/global_costmap/footprint [geometry_msgs/msg/Polygon]  # 全局代价地图中配置的机器人轮廓多边形
/global_costmap/published_footprint [geometry_msgs/msg/PolygonStamped]  # 实时发布的机器人在全局代价地图中的足迹
/global_localization/adaptive_submap_highlight [sensor_msgs/msg/PointCloud2]  # 重定位模块实时提取的局部点云子图 (RViz高亮)
/global_path [nav_msgs/msg/Path]  # Nav2 全局规划器生成的全局路径
/local_costmap/costmap [nav_msgs/msg/OccupancyGrid]  # Nav2 局部代价地图
/local_costmap/costmap_raw [nav2_msgs/msg/Costmap]  # 未完全处理的局部代价地图原始数据
/local_costmap/costmap_updates [map_msgs/msg/OccupancyGridUpdate]  # 局部代价地图的增量更新区域
/local_costmap/footprint [geometry_msgs/msg/Polygon]  # 局部代价地图中配置的机器人轮廓多边形
/local_costmap/published_footprint [geometry_msgs/msg/PolygonStamped]  # 实时发布的机器人在局部代价地图中的足迹
/local_path [nav_msgs/msg/Path]  # 局部规划器规划的前方短距离路径
/local_path_rl_debug [nav_msgs/msg/Path]  # 强化学习(RL)局部规划器输出的调试路径
/local_plan [nav_msgs/msg/Path]  # 局部规划器最终决定的执行轨迹
/map_2d [nav_msgs/msg/OccupancyGrid]  # 将 3D 点云压平后生成的 2D 栅格地图(占据栅格)
/map_2d_updates [map_msgs/msg/OccupancyGridUpdate]  # 2D 栅格地图的增量更新
/mppi_path [nav_msgs/msg/Path]  # MPPI 局部规划器生成的最优预测路径
/path [nav_msgs/msg/Path]  # 通用路径话题
/plan [nav_msgs/msg/Path]  # 全局或局部规划器输出的计划路线
/plan_smoothed [nav_msgs/msg/Path]  # 经过平滑算法处理后的优化路径
/received_global_plan [nav_msgs/msg/Path]  # 控制器收到的来自全局规划器的最新路径
/rl_goal_pose [geometry_msgs/msg/PoseStamped]  # 强化学习(RL)模块当前的子目标点
/subgoal [geometry_msgs/msg/PoseStamped]  # 导航过程中的临时中间子目标点
/transformed_global_plan [nav_msgs/msg/Path]  # 转换到局部代价地图坐标系下的全局路径段

### 5. 导航执行与底盘控制层 (Navigation Execution & Control) ###
/NAV_CMD [drdds/msg/NavCmd]  # 导航模块输出给机器狗底层的最终运动控制指令 (发往Domain 0)
/cmd_vel [geometry_msgs/msg/Twist]  # 最终发送给底盘的合成控制速度指令
/cmd_vel_nav [geometry_msgs/msg/Twist]  # Nav2 导航输出的控制速度指令
/cmd_vel_teleop [geometry_msgs/msg/Twist]  # 手柄或键盘遥控下发的控制速度指令
/preempt_teleop [std_msgs/msg/Empty]  # 中断当前自动驾驶，抢占控制权(遥控)信号
/speed_limit [nav2_msgs/msg/SpeedLimit]  # 地图或应用层下发的限速区指令

### 6. ROS2系统与状态机层 (ROS2 System & Lifecycle Events) ###
/behavior_server/transition_event [lifecycle_msgs/msg/TransitionEvent]  # Nav2 行为服务器状态机切换事件
/bond [bond/msg/Status]  # ROS2 节点存活状态心跳检测 (Bond)
/bt_navigator/transition_event [lifecycle_msgs/msg/TransitionEvent]  # Nav2 行为树导航器状态机切换事件
/controller_server/transition_event [lifecycle_msgs/msg/TransitionEvent]  # Nav2 控制器(局部规划器)状态机切换事件
/diagnostics [diagnostic_msgs/msg/DiagnosticArray]  # 机器人硬件和系统的诊断信息
/global_costmap/global_costmap/transition_event [lifecycle_msgs/msg/TransitionEvent]  # Nav2 全局代价地图状态机切换事件
/local_costmap/local_costmap/transition_event [lifecycle_msgs/msg/TransitionEvent]  # Nav2 局部代价地图状态机切换事件
/map_server/transition_event [lifecycle_msgs/msg/TransitionEvent]  # Nav2 地图服务器状态机切换事件
/parameter_events [rcl_interfaces/msg/ParameterEvent]  # ROS2 系统参数动态修改事件监听
/planner_server/transition_event [lifecycle_msgs/msg/TransitionEvent]  # Nav2 全局规划器状态机切换事件
/rosout [rcl_interfaces/msg/Log]  # ROS2 系统所有节点的标准日志输出
/smoother_server/transition_event [lifecycle_msgs/msg/TransitionEvent]  # Nav2 路径平滑服务器状态机切换事件
/tf [tf2_msgs/msg/TFMessage]  # ROS2 动态坐标系变换树 (Transform Tree)
/tf_static [tf2_msgs/msg/TFMessage]  # ROS2 静态坐标系变换树
/velocity_smoother/transition_event [lifecycle_msgs/msg/TransitionEvent]  # Nav2 速度平滑器状态机切换事件
/waypoint_follower/transition_event [lifecycle_msgs/msg/TransitionEvent]  # Nav2 航点跟随器状态机切换事件

### 7. RViz纯可视化与状态反馈 (RViz Visualization & Feedback) ###
/cost_cloud [sensor_msgs/msg/PointCloud2]  # 转换为点云形式的代价地图数据 (调试用)
/dyn_obj_dbg_hist [sensor_msgs/msg/PointCloud2]  # 动态障碍物检测历史轨迹 (调试用)
/global_localization/rviz_cur_scan [sensor_msgs/msg/PointCloud2]  # 重定位验证时展示的当前帧对齐点云
/global_localization/rviz_path [nav_msgs/msg/Path]  # 重定位成功后的机器人运动轨迹展示
/global_localization/rviz_submap_highlight [sensor_msgs/msg/PointCloud2]  # 重定位时 RViz 中高亮的候选区域子图
/global_localization/ui_horizontal_pose [geometry_msgs/msg/Pose]  # UI 界面上展示的水平面 2D 位姿
/marker [visualization_msgs/msg/MarkerArray]  # RViz 各种可视化标记 (箭头、形状等)
/pedestrians/markers [visualization_msgs/msg/MarkerArray]  # 检测到的行人位置可视化标记
/planes [visualization_msgs/msg/MarkerArray]  # 环境中提取出的平面(地面/墙面)可视化标记
/planner_normal [visualization_msgs/msg/Marker]  # 规划器法向量或相关可视化
/rviz_cloud_registered [sensor_msgs/msg/PointCloud2]  # 专用于 RViz 显示的降采样当前帧对齐点云
/rviz_path [nav_msgs/msg/Path]  # 专用于 RViz 显示的历史运动路径
/visualization_marker [visualization_msgs/msg/MarkerArray]  # RViz 通用可视化标记
/voxels [visualization_msgs/msg/MarkerArray]  # 体素化后的 3D 障碍物网格可视化
/waypoint_editor/feedback [visualization_msgs/msg/InteractiveMarkerFeedback]  # RViz 航点编辑器用户交互反馈 (拖拽、点击)
/waypoint_editor/update [visualization_msgs/msg/InteractiveMarkerUpdate]  # RViz 航点编辑器标记更新
/waypoint_sequence/status [std_msgs/msg/String]  # 航点跟随任务当前状态;done 
/waypoints [visualization_msgs/msg/MarkerArray]  # 规划的所有航点可视化标记
/waypoints_pose_array [geometry_msgs/msg/PoseArray]  # 以 PoseArray 格式发布的航点集合

### 8. 对外交互与指令控制接口 (UI & External Control Interfaces) ###
/clicked_point [geometry_msgs/msg/PointStamped]  # RViz 中使用 Publish Point 工具点击的坐标点 (1点)
/global_localization/ui_clear_trigger [std_msgs/msg/Bool]  # UI 触发清理重定位状态的信号
/goal_pose [geometry_msgs/msg/PoseStamped]  # RViz 中设置的 2D Nav Goal (导航目标点)
/initialpose [geometry_msgs/msg/PoseWithCovarianceStamped]  # RViz 中手动打点的 2D 初始位姿估计 (Pose Estimate)

/waypoint_sequence/clear [std_msgs/msg/Empty]  # 清空当前航点序列  （清空航点waypoints, 置空历史轨迹的Path）
/waypoint_sequence/delete_nearest [geometry_msgs/msg/PointStamped]  # 删除离机器人最近的航点
/waypoint_sequence/pause [std_msgs/msg/Empty]  # 暂停航点跟随任务  (置空历史轨迹的Path， paused = True)
/waypoint_sequence/replace_nearest [geometry_msgs/msg/PointStamped]  # 替换最近航点的位置
/waypoint_sequence/resume [std_msgs/msg/Empty]  # 恢复航点跟随任务
/waypoint_sequence/undo [std_msgs/msg/Empty]  # 撤销上一步航点操作


### 9. 新增 topic ###
/navigation/progress_percent [std_msgs/msg/Float32]  # 导航实时进度百分比，独立统计脚本按 10 Hz 发布；计算公式为“已走实际路程 / (已走实际路程 + 当前全局路线剩余路程) × 100”，取值范围 0～100

发布节点：`/navigation_progress_monitor`。

数据来源：

- `/global_path [nav_msgs/msg/Path]`：绿色全局规划路线，重规划后用于重新计算剩余路程。
- `/global_localization/ui_horizontal_pose [geometry_msgs/msg/Pose]`：累计机器人UI pose。
- `/global_localization/path [nav_msgs/msg/Path]`：红色实际行走轨迹，用于图片绘制。 （其他节点向 /global_localization/path 发布一次空 Path，RViz 会短暂清空）
- `/local_path [nav_msgs/msg/Path]`：蓝色局部执行路线，用于图片绘制；蓝线表示 RL/PRIEST 生成并交给 Nav2 DWB 执行的短距离局部路径。

运行方式：执行 `bash debug_menu.sh`，选择“启动导航进度统计与路线图片绘制”。脚本同时按 5 Hz 持续覆盖 `tmp/navigation_progress.jpg`，其中绿色为全局规划路线、红色为实际行走轨迹、蓝色为局部执行路线，并标注进度百分比、已走距离和剩余距离。
