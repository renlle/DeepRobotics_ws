# Debug Session: livox-pointcloud-stops

- progress_status: evidence-collection
- PROJECT_ROOT: /home/ubuntu/Desktop/ROS2_szs/DeepRobotics_ws
- DEBUG_EVIDENCE_DIR: /tmp/trae-debug-DeepRobotics_ws-livox-pointcloud-stops
- Evidence log: /tmp/trae-debug-DeepRobotics_ws-livox-pointcloud-stops/trae-debug-log-livox-pointcloud-stops.ndjson
- Environment file: /tmp/trae-debug-DeepRobotics_ws-livox-pointcloud-stops/livox-pointcloud-stops.env

## Symptom

Livox MID360 单驱动测试约 146 秒后，点云从约 10 Hz 降至 0 Hz，IMU 仍稳定约 200 Hz，驱动 CPU 从约 14% 降至约 5%。

## Hypotheses

| ID | Hypothesis | Status | Evidence |
|---|---|---|---|
| H1 | 点云 UDP 数据停止到达主机，IMU UDP 仍正常 | CONFIRMED | 故障后已有 SDK 回调插桩只持续收到 IMU；此前同症状原始抓包也无 56300 点云包 |
| H2 | SDK 收到点云，但驱动内部点云队列或发布线程停止 | REJECTED | 故障后不存在任何非 IMU SDK packet-callback，故障发生在组包和 ROS 发布之前 |
| H3 | 测试订阅器造成 DDS 背压 | REJECTED | 测试进程结束、订阅数为 0 后，外部 ros2 topic hz 仍无点云 |
| H4 | MID360 点云通道自身异常，控制和 IMU 通道仍正常 | CONFIRMED | IMU 200 Hz、ping 正常、驱动存活，但点云 SDK 回调完全停止；此前同症状抓包仅有状态/广播包，无点云包 |

## Evidence

- 用户采样：0-141 秒点云约 10 Hz、IMU 约 200 Hz、CPU 约 14%；146 秒点云 1.59 Hz；151 秒起点云 0 Hz、IMU 约 200 Hz、CPU 约 5%。
- 独立 `ros2 topic hz /livox/lidar` 复核无消息，排除仅统计显示错误。
- `/livox_lidar_publisher` 存活，`/livox/lidar` 和 `/livox/imu` 发布端均存在。
- 进程仍绑定 `192.168.1.50:56301` 和 `192.168.1.50:56401`。
- `ping 192.168.1.100` 3/3 成功，平均 1.282 ms。
- 驱动日志无断线、重连或异常输出。
- 故障期间结构化证据第 1–26 行：IMU 回调计数从 100000 持续增长至 110000，约每 2 秒增加 400；没有任何点云 packet-callback。
- 既有同症状实机抓包：排除 IMU 后仅有状态包和广播包，没有源端口 56300 的点云 UDP 数据。

## Conclusion

- `progress_status: diagnosed`
- 这是实际点云流中断，不是频率脚本误报，也不是 DDS 订阅背压或 ROS 点云发布线程卡住。
- 中断点位于 MID360/SDK 收包链路上游：设备仍发送 IMU 和状态数据，但停止发送点云 UDP 数据。
- 当前证据不能进一步区分设备固件内部停流、点云发送开关被关闭或设备点云模块异常；设备状态此前显示 Normal、IMU enabled、diag status 0。

## Next Evidence

- 对 SDK 点云回调、点云入队和点云发布增加低频结构化计数。
- 测试脚本在点云连续为 0 时记录进程、线程、UDP socket 和网卡计数快照。
- 完成重启前后对照，确认故障是否随驱动重启恢复。
