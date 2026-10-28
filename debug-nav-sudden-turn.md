# 导航直行突发大转弯调试记录

- session_id: nav-sudden-turn
- project_root: /home/ubuntu/Desktop/ROS2_szs/DeepRobotics_ws
- evidence_dir: /tmp/trae-debug-DeepRobotics_ws-nav-sudden-turn
- log_path: /tmp/trae-debug-DeepRobotics_ws-nav-sudden-turn/trae-debug-log-nav-sudden-turn.ndjson
- env_path: /tmp/trae-debug-DeepRobotics_ws-nav-sudden-turn/nav-sudden-turn.env
- progress_status: existing_rosbag_analyzed

## 现象

- 机器人向点1直行途中真实发生大幅转弯，红色实际轨迹同步弯折。
- 添加点2后，绿色全局路径看起来从初始位置开始规划，而不是从机器人当前位置开始。
- 随后机器人接近并碰撞障碍，用户执行急停。

## 假设

| 编号 | 假设 | 状态 | 证据要求 |
|---|---|---|---|
| H1 | DWB 或下游控制链主动输出大角速度 | CONFIRMED | cmd_vel_nav 与 cmd_vel 从 29.2 s 起持续输出大角速度，峰值 -0.65 rad/s |
| H2 | localization、odom 或 TF 先跳变并触发控制纠偏 | REJECTED | 首次明显控制转向早于 29.675 s 的 6.24° 单帧变化，且 localization、odom、TF 同步反映真实转动 |
| H3 | scan 或 local costmap 瞬时产生假障碍 | REJECTED | 全包前向 ±35° 最近距离未低于 0.65 m；转弯开始阶段约为 0.83-1.02 m |
| H4 | 全局路径没有锚定机器人当前位置，局部路径方向发生大幅跳变 | CONFIRMED | global_path 起点距机器人 1.607 m，73 次发布几何端点不变；local_path 多次从前向跳到后向 |
| H5 | 多发布者或跨域命令链异常 | INCONCLUSIVE | bag 没有记录发布者图，NAV_CMD 消息数为 0，无法验证 Domain 42 到 Domain 0 |

## 证据源

- record_bag.sh
- log/nav_turn_issue ROS 2 bag
- 用户提供的 RViz 截图

## 当前结论

- 突拐是算法域真实下发的控制动作，不是 RViz 红线或定位单独画错。
- bag 第一个 global_path 在 26.580 s 发布，起点为 (-0.379, -0.164)，同期机器人 map 位姿约为 (0.956, 1.590)，最近路径点距机器人 1.607 m。
- 26.994 s 的 local_path 主体位于机器人后方约 -173°，而控制器仍给出 vx=0.8 m/s；29.2 s 后角速度长期超过 0.3 rad/s，峰值达到配置上限 0.65 rad/s。
- global_path 在 26.580-98.708 s 共发布 73 次，均为 112 个点，起终点和几何保持一致，仅时间戳更新。这解释了添加点2后绿色路径仍从旧起点开始的现象。
- 29.675 s 的 localization、odom 和相关 TF 同时变化约 6.24°，发生在控制器开始大角速度输出之后，属于真实转弯反馈，不支持定位先跳变这一因果关系。
- 当前高置信根因位于 global_path 到 local_path 的输入一致性：全局路径与当前机器人位姿脱节，RL/PRIEST 局部路径持续指向机器人后方并反复换侧，DWB 再被高角速度参数放大。
- NAV_CMD 未录到消息，因此底盘实际接收命令及跨域桥行为仍无法由本 bag 闭环验证。
