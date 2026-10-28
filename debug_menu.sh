#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
# export ROS_DOMAIN_ID=0
export ROS_DOMAIN_ID=42

source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
    source "${WORKSPACE_DIR}/install/setup.bash"
fi

mkdir -p "${WORKSPACE_DIR}/log" "${WORKSPACE_DIR}/tmp"

capture_rgb_image() {
    WORKSPACE_DIR="${WORKSPACE_DIR}" python3 - <<'PY'
import os
import sys
from datetime import datetime

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

rclpy.init()
node = rclpy.create_node("debug_left_camera_capture")
bridge = CvBridge()
result = {"path": None, "error": None}

def callback(message):
    try:
        rgb = bridge.imgmsg_to_cv2(message, desired_encoding="rgb8")
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        filename = f"left_camera_rgb_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}.png"
        path = os.path.join(os.environ["WORKSPACE_DIR"], "log", filename)
        if not cv2.imwrite(path, bgr):
            raise RuntimeError("OpenCV 写入 PNG 失败")
        result["path"] = path
    except Exception as exc:
        result["error"] = str(exc)

subscription = node.create_subscription(
    Image,
    "/left_camera/image",
    callback,
    qos_profile_sensor_data,
)

print("等待 Domain 42 的 /left_camera/image，最长 15 秒...")
started = node.get_clock().now()
while result["path"] is None and result["error"] is None:
    rclpy.spin_once(node, timeout_sec=0.1)
    if (node.get_clock().now() - started).nanoseconds >= 15_000_000_000:
        result["error"] = "15 秒内未收到图像，请检查 Domain 42、话题发布者和 /clock"

node.destroy_subscription(subscription)
node.destroy_node()
rclpy.shutdown()

if result["error"] is not None:
    print(f"保存失败：{result['error']}", file=sys.stderr)
    sys.exit(1)

print(f"RGB 图像已保存：{result['path']}")
PY
}

read_xy() {
    read -r -p "请输入 map 坐标 x: " POINT_X
    read -r -p "请输入 map 坐标 y: " POINT_Y
    if ! [[ "${POINT_X}" =~ ^[-+]?[0-9]*\.?[0-9]+$ && "${POINT_Y}" =~ ^[-+]?[0-9]*\.?[0-9]+$ ]]; then
        echo "坐标格式错误，请输入数字。" >&2
        return 1
    fi
}

publish_point() {
    local topic="$1"
    local description="$2"
    read_xy || return
    ros2 topic pub --once "${topic}" geometry_msgs/msg/PointStamped \
        "{header: {frame_id: 'map'}, point: {x: ${POINT_X}, y: ${POINT_Y}, z: 0.0}}"
    echo "已${description}: (${POINT_X}, ${POINT_Y})"
}

publish_empty() {
    local topic="$1"
    local description="$2"
    ros2 topic pub --once "${topic}" std_msgs/msg/Empty "{}"
    echo "已${description}。"
}

publish_initial_pose() {
    read_xy || return
    read -r -p "请输入朝向 yaw（度，默认 0）: " yaw_degrees
    yaw_degrees="${yaw_degrees:-0}"
    if ! [[ "${yaw_degrees}" =~ ^[-+]?[0-9]*\.?[0-9]+$ ]]; then
        echo "yaw 格式错误，请输入数字。" >&2
        return 1
    fi
    read -r quaternion_z quaternion_w < <(python3 -c \
        "import math; yaw=math.radians(float('${yaw_degrees}')); print(math.sin(yaw/2), math.cos(yaw/2))")
    ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
        "{header: {frame_id: 'map'}, pose: {pose: {position: {x: ${POINT_X}, y: ${POINT_Y}, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: ${quaternion_z}, w: ${quaternion_w}}}}}"
    echo "已发布人工初始位姿: (${POINT_X}, ${POINT_Y}), yaw=${yaw_degrees}°"
}

navigation_control_menu() {
    while true; do
        echo
        echo "---------- 导航交互与指令控制 ----------"
        echo "1) /clicked_point：追加导航航点"
        echo "2) /global_localization/ui_clear_trigger：清除 UI 可视化"
        echo "3) /goal_pose：查看当前目标点（输出 Topic）"
        echo "4) /initialpose：设置人工初始位姿"
        echo "5) /waypoint_sequence/clear：清空全部航点"
        echo "6) /waypoint_sequence/delete_nearest：删除坐标附近航点"
        echo "7) /waypoint_sequence/pause：暂停航点任务"
        echo "8) /waypoint_sequence/replace_nearest：移动坐标附近航点"
        echo "9) /waypoint_sequence/resume：恢复航点任务"
        echo "10) /waypoint_sequence/undo：撤销最后一个航点"
        echo "11) 查询当前位置并打印一次（map → base_link）"
        echo "b) 返回主菜单"
        read -r -p "请选择: " nav_choice

        case "${nav_choice}" in
            1)
                publish_point /clicked_point "追加导航航点"
                ;;
            2)
                read -r -p "仅清除 Robot Console/RViz 可视化，不重置定位，确认发送？[y/N]: " confirm
                if [[ "${confirm}" =~ ^[Yy]$ ]]; then
                    ros2 topic pub --once /global_localization/ui_clear_trigger std_msgs/msg/Bool "{data: true}"
                fi
                ;;
            3)
                echo "等待 /goal_pose 当前目标点，按 Ctrl+C 可取消..."
                ros2 topic echo --once /goal_pose
                ;;
            4)
                publish_initial_pose
                ;;
            5)
                publish_empty /waypoint_sequence/clear "清空全部航点"
                ;;
            6)
                publish_point /waypoint_sequence/delete_nearest "请求删除坐标附近航点"
                ;;
            7)
                publish_empty /waypoint_sequence/pause "暂停航点任务"
                ;;
            8)
                publish_point /waypoint_sequence/replace_nearest "请求移动坐标附近航点"
                ;;
            9)
                publish_empty /waypoint_sequence/resume "恢复航点任务"
                ;;
            10)
                publish_empty /waypoint_sequence/undo "撤销最后一个航点"
                ;;
            11)
                # 查询最新真实 TF，成功后只打印一次；无定位时限时返回菜单。
                python3 - <<'PY'
import math
import time
import rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException

rclpy.init()
node = rclpy.create_node("debug_current_position")
buffer = Buffer()
listener = TransformListener(buffer, node)
print("等待 map → base_link，最长 5 秒...")
deadline = time.monotonic() + 5.0
try:
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            transform = buffer.lookup_transform("map", "base_link", Time())
        except TransformException:
            continue
        p = transform.transform.translation
        q = transform.transform.rotation
        yaw = math.degrees(math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        stamp = transform.header.stamp
        print(f"当前位置（map，单位：米）：x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}")
        print(f"朝向 yaw={yaw:.2f}°，TF 时间戳={stamp.sec}.{stamp.nanosec:09d}")
        break
    else:
        print("未获取到 map → base_link，请检查定位和 transform_fusion 是否运行。")
except KeyboardInterrupt:
    print("已取消查询。")
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
PY
                ;;
            b|B)
                return
                ;;
            *)
                echo "无效选项：${nav_choice}" >&2
                ;;
        esac
    done
}

run_navigation_progress_monitor() {
    echo "启动导航进度统计，按 Ctrl+C 返回菜单。"
    echo "Topic: /navigation/progress_percent (10 Hz)"
    echo "图片: ${WORKSPACE_DIR}/tmp/navigation_progress.jpg (5 Hz 覆盖)"
    python3 -u "${WORKSPACE_DIR}/scripts/navigation_progress_monitor.py"
}

while true; do
    echo
    echo "========== Domain 42 调试菜单 =========="
    echo "1) 查看话题列表"
    echo "2) 保存一张 /left_camera/image RGB 图像"
    echo "3) 导航交互与指令控制菜单"
    echo "4) 启动导航进度统计与路线图片绘制"
    echo "q) 退出"
    read -r -p "请选择: " choice

    case "${choice}" in
        1)
            ros2 topic list
            ;;
        2)
            capture_rgb_image
            ;;
        3)
            navigation_control_menu
            ;;
        4)
            run_navigation_progress_monitor
            ;;
        q)
            exit 0
            ;;
        *)
            echo "无效选项：${choice}" >&2
            ;;
    esac
done
