# Domain 0 → 42 底盘状态转发

移植来源：`orin-v0.3-dog_patrol` 提交 `9da6c37`。启动脚本保留10-28分支的系统Python及总控进程管理方式，仅增加独立nohup状态桥。

转发Topic（保持所有原始字段与时间戳）：

- `/BATTERY_CHARGE_ENABLE`：`drdds/msg/StdMsgInt32`
- `/BATTERY_DATA`：`drdds/msg/BatteryData`
- `/ODOM`：`nav_msgs/msg/Odometry`（大写，不是算法小写`/odom`）

启动`bash start_app.sh`会自动启动桥；日志为`log/domain0to42_bridge.log`。单独启动：`bash scripts/start_domain0to42_bridge.sh`。同工作区、同域对重复启动会替换旧实例；不同工作区的桥不会相互回收，切换工作区前应停止原桥，避免重复转发。

本次同时合入来源分支的`BatteryDataValue.msg`字段宽度修复：balanced_low/high/protected_state为uint16，software_version为uint8，battery_serialnum为char[32]。否则底盘电池CDR可能无法解码或数据错位。必须在10-28工作区重新编译drdds及其依赖，并重启使用该消息的节点；不能沿用旧install中的drdds类型。

```bash
source /opt/ros/humble/setup.bash
bash build.sh
source install/setup.bash
bash start_app.sh
```

桥使用系统`/usr/bin/python3`、rclpy、nav_msgs和本工作区drdds；默认使用现有`config/fastdds_udp_only.xml`。需要Linux支持pidfd接口以安全替换旧进程。脚本默认单向0→42，不转发控制命令。

本次只做静态检查、消息包编译及离线CDR样本校验，不启动桥、不向Domain0/42发布数据，未验证真实底盘连通性。完整隔离域回归保留在`test_battery_domain_bridge.py`，需要具备ROS运行环境后另行执行。
