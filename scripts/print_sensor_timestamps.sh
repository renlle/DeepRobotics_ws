#!/usr/bin/env bash
# 持续显示三个传感器消息的最新 header 时间戳；不启动或停止驱动。
set -e
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(dirname -- "$SCRIPT_DIR")"
source /opt/ros/humble/setup.bash
if [[ -f "$WORKSPACE/install/setup.bash" ]]; then
    source "$WORKSPACE/install/setup.bash"
fi
export ROS_DOMAIN_ID=42
# 避免向系统 Python 目录写入字节码缓存。
export PYTHONDONTWRITEBYTECODE=1
exec python3 -u - <<'PY'
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, PointCloud2

rclpy.init()
node = rclpy.create_node('sensor_timestamp_monitor')
latest = {}
subscriptions = []
sensors = [
    ('相机', '/left_camera/image', Image),
    ('雷达点云', '/livox/lidar', PointCloud2),
    ('雷达 IMU', '/livox/imu', Imu),
]


def receive(label, message):
    stamp = message.header.stamp
    # 使用整数纳秒保存，避免大数浮点转换造成精度损失。
    latest[label] = (stamp.sec * 1_000_000_000 + stamp.nanosec, time.monotonic())


for label, topic, message_type in sensors:
    subscriptions.append(node.create_subscription(
        message_type, topic,
        lambda message, label=label: receive(label, message),
        qos_profile_sensor_data))


def display():
    now = time.monotonic()
    print('\n数据          时间戳（秒）         距最近接收（秒）')
    for label, _, _ in sensors:
        if label not in latest:
            print(f'{label:<8} 等待数据')
            continue
        stamp_ns, received = latest[label]
        seconds, nanoseconds = divmod(stamp_ns, 1_000_000_000)
        print(f'{label:<8} {seconds}.{nanoseconds:09d}    {now - received:.3f}')


# 每秒打印最新值；接收超时可从“距最近接收”识别。
node.create_timer(1.0, display)
print('Domain 42：每秒打印最新消息时间戳，Ctrl+C 退出。')
print('注意：最新值并非配对帧，时间戳接近不等于曝光硬同步。')
try:
    rclpy.spin(node)
except (KeyboardInterrupt, ExternalShutdownException):
    pass
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    print('\n已停止时间戳监视，传感器驱动继续运行。')
PY

# bash /home/ubuntu/Desktop/ROS2_szs/DeepRobotics_ws/scripts/print_sensor_timestamps.sh