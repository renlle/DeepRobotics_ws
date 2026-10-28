# 导航短距离后停止调试记录

- session_id: navigation-stops
- project_root: /home/ubuntu/Desktop/ROS2_szs/DeepRobotics_ws
- evidence_dir: /tmp/trae-debug-DeepRobotics_ws-navigation-stops
- log_path: /tmp/trae-debug-DeepRobotics_ws-navigation-stops/trae-debug-log-navigation-stops.ndjson
- env_path: /tmp/trae-debug-DeepRobotics_ws-navigation-stops/navigation-stops.env
- progress_status: runtime_evidence_collected_instrumentation_added

## 现象

- 新地图环境中定位和导航启动后，全局绿色路径约 4.6 m。
- 机器人实际红色轨迹只前进很短距离，约两步后停止。
- record_bag.sh 多次启动和退出时终端出现红色报错。

## 假设

| 编号 | 假设 | 状态 | 证据要求 |
|---|---|---|---|
| H1 | local_path 超时导致 Adapter 停车 | REJECTED | 164221 中 local_path 从 2.194 s 持续到 10.054 s，约 3 Hz，无超过 1.2 s 的断流 |
| H2 | waypoint 路径经 mux 转发中断 | INCONCLUSIVE | 164221 记录到 10 条 waypoint_global_path，但未发现 global_path topic；163815 两者均为 4 条，需 rosout/节点图确认是否 mux 实例变化 |
| H3 | 1.5 m 到点容差导致提前完成 | REJECTED | 164221 目标约 (7.145,1.337)，机器人约 (2.642,0.625)，距离约 4.56 m，且 waypoint 路径持续非空 |
| H4 | DWB 无合法轨迹或控制器生命周期/目标状态异常 | CONFIRMED | 164221 的 local_path 持续有效且指向前方，但 cmd_vel_nav 与 cmd_vel 全程为零 |
| H5 | 录包脚本退出实现导致 metadata 缺失 | CONFIRMED | 四次算法域 db3 均有 775-1339 条消息但无 metadata.yaml，底盘域均正常生成 metadata；原脚本后台进程收到信号后未可靠等待 ros2 bag 子进程 |

## 证据源

- record_bag.sh 生成的最新双域 bag
- 用户提供的 RViz 截图
- 现有导航参数和迁移后的路径链路

## 当前结论

- 164221 bag 中绿色 waypoint 路径和蓝色 local_path 都持续更新；local_path 为 base_footprint 前方约 4 m，几何正常。
- 同一 bag 中 cmd_vel_nav 79 条、cmd_vel 133 条全部为零，机器人停止由 Nav2 controller 侧未产生运动命令直接导致，不是 local_path 超时，也不是到点提前完成。
- 163815 是另一种状态：cmd_vel_nav/cmd_vel 持续为 vx=0、w=0.65，说明当时 controller 工作但只要求原地旋转，定位基本不动。
- 164221 没有 /global_path topic，但 local_path 仍持续存在，说明至少存在旧缓存/另一链路输入；需要记录 /rosout 和启动时节点图才能确定 mux 是否未启动、退出或当前运行 install 不一致。
- Domain 0 四次均为 0 条 NAV_CMD；算法域 /NAV_CMD 也为 0 条。这与 Adapter 内部应周期发布 NAV_CMD 的设计不一致，强烈提示 Adapter 实例异常、启动版本不一致或录制发现阶段未连上该发布者。
- 已仅增加诊断输出：mux 首帧/每100帧转发日志、Adapter 首条 local_path 和首个 FollowPath 接受日志，并将 /rosout 纳入双域 bag。
- 已修正录包收尾：移除读取正在写入 SQLite 的统计子进程，保留 rosbag2 自带每100条缓存提示，捕获 stdout/stderr，并在退出后检查两个 metadata.yaml。
