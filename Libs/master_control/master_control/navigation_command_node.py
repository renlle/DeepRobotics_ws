#!/usr/bin/env python3

import math
import re
import time
import uuid
from typing import List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import PointStamped
from master_control_interfaces.msg import (
    DeviceInfo,
    NavigationTelemetry,
    WaypointReachStatus,
    WaypointReachStatusArray,
    NavigationTaskCommand,
    NavigationTaskStatus,
)
from master_control.path_sampling import sample_telemetry_path
from master_control_interfaces.srv import NavigateToPoint, NavigateToPoints
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, Empty, Float32, String
from std_srvs.srv import Trigger


class NavigationCommandNode(Node):
    """接收 Agent 的同步请求，并转发现有导航话题。"""

    STATE_IDLE = "IDLE"
    STATE_RUNNING = "RUNNING"
    STATE_PAUSED = "PAUSED"

    def __init__(
        self,
        *,
        parameter_overrides: Optional[Sequence[Parameter]] = None,
    ) -> None:
        super().__init__(
            "navigation_command_node",
            parameter_overrides=parameter_overrides,
        )

        self.declare_parameter(
            "single_point_service", "/master_control/navigation/single_point"
        )
        self.declare_parameter(
            "multi_point_service", "/master_control/navigation/multi_point"
        )
        self.declare_parameter(
            "pause_service", "/master_control/navigation/pause"
        )
        self.declare_parameter(
            "resume_service", "/master_control/navigation/resume"
        )
        self.declare_parameter(
            "abort_service", "/master_control/navigation/abort"
        )
        self.declare_parameter(
            "return_home_service", "/master_control/navigation/return_home"
        )
        self.declare_parameter(
            "clear_service", "/master_control/navigation/clear"
        )
        self.declare_parameter("clicked_point_topic", "/clicked_point")
        self.declare_parameter("pause_topic", "/waypoint_sequence/pause")
        self.declare_parameter("resume_topic", "/waypoint_sequence/resume")
        self.declare_parameter("clear_topic", "/waypoint_sequence/clear")
        self.declare_parameter("global_path_topic", "/global_path")
        self.declare_parameter(
            "actual_path_topic", "/global_localization/path"
        )
        self.declare_parameter("local_path_topic", "/local_path")
        self.declare_parameter(
            "progress_topic", "/navigation/progress_percent"
        )
        self.declare_parameter(
            "waypoint_status_topic", "/waypoint_sequence/status"
        )
        self.declare_parameter(
            "telemetry_topic", "/master_control/navigation/telemetry"
        )
        self.declare_parameter(
            "waypoint_reach_status_topic",
            "/master_control/navigation/waypoint_reach_status",
        )
        self.declare_parameter("status_publish_rate", 2.0)
        self.declare_parameter("localization_topic", "/localization")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter(
            "localization_status_topic", "/localization_status"
        )
        self.declare_parameter(
            "device_info_topic", "/master_control/device/info"
        )
        self.declare_parameter("device_info_publish_rate", 1.0)
        self.declare_parameter("heartbeat_timeout", 2.5)
        self.declare_parameter("require_navigation_subscriber", True)

        self._single_point_service = str(
            self.get_parameter("single_point_service").value
        )
        self._multi_point_service = str(
            self.get_parameter("multi_point_service").value
        )
        self._pause_service = str(
            self.get_parameter("pause_service").value
        )
        self._resume_service = str(
            self.get_parameter("resume_service").value
        )
        self._abort_service = str(
            self.get_parameter("abort_service").value
        )
        self._return_home_service = str(
            self.get_parameter("return_home_service").value
        )
        self._clear_service = str(
            self.get_parameter("clear_service").value
        )
        self._clicked_point_topic = str(
            self.get_parameter("clicked_point_topic").value
        )
        self._pause_topic = str(self.get_parameter("pause_topic").value)
        self._resume_topic = str(self.get_parameter("resume_topic").value)
        self._clear_topic = str(self.get_parameter("clear_topic").value)
        self._global_path_topic = str(
            self.get_parameter("global_path_topic").value
        )
        self._actual_path_topic = str(
            self.get_parameter("actual_path_topic").value
        )
        self._local_path_topic = str(
            self.get_parameter("local_path_topic").value
        )
        self._progress_topic = str(
            self.get_parameter("progress_topic").value
        )
        self._waypoint_status_topic = str(
            self.get_parameter("waypoint_status_topic").value
        )
        self._telemetry_topic = str(
            self.get_parameter("telemetry_topic").value
        )
        self._waypoint_reach_status_topic = str(
            self.get_parameter("waypoint_reach_status_topic").value
        )
        status_publish_rate = float(
            self.get_parameter("status_publish_rate").value
        )
        if status_publish_rate <= 0.0:
            self.get_logger().warn("status_publish_rate 必须大于 0，已回退为 2 Hz")
            status_publish_rate = 2.0
        self._localization_topic = str(
            self.get_parameter("localization_topic").value
        )
        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._localization_status_topic = str(
            self.get_parameter("localization_status_topic").value
        )
        self._device_info_topic = str(
            self.get_parameter("device_info_topic").value
        )
        device_info_publish_rate = float(
            self.get_parameter("device_info_publish_rate").value
        )
        if device_info_publish_rate <= 0.0:
            self.get_logger().warn(
                "device_info_publish_rate 必须大于 0，已回退为 1 Hz"
            )
            device_info_publish_rate = 1.0
        self._heartbeat_timeout = float(
            self.get_parameter("heartbeat_timeout").value
        )
        if self._heartbeat_timeout <= 0.0:
            self.get_logger().warn(
                "heartbeat_timeout 必须大于 0，已回退为 2.5 秒"
            )
            self._heartbeat_timeout = 2.5
        self._require_navigation_subscriber = bool(
            self.get_parameter("require_navigation_subscriber").value
        )

        self.navigation_state = self.STATE_IDLE
        self._command_id = ""
        self._command_topic = "/waypoint_sequence/command"
        self._command_pub = self.create_publisher(NavigationTaskCommand, self._command_topic, 100)
        self.create_subscription(NavigationTaskStatus, "/waypoint_sequence/task_status", self._on_task_status, 100)
        self.last_targets: List[PointStamped] = []
        self.reached: List[bool] = []
        self._global_path = Path()
        self._actual_path = Path()
        self._local_path = Path()
        self._progress_percent = 0.0
        self._localization = None
        self._odom = None
        self._localization_receive_time = None
        self._odom_receive_time = None
        self._localization_ok = False

        self._clicked_point_pub = self.create_publisher(
            PointStamped, self._clicked_point_topic, 10
        )
        self._pause_pub = self.create_publisher(Empty, self._pause_topic, 10)
        self._resume_pub = self.create_publisher(Empty, self._resume_topic, 10)
        self._clear_pub = self.create_publisher(Empty, self._clear_topic, 10)
        self._telemetry_pub = self.create_publisher(
            NavigationTelemetry, self._telemetry_topic, 10
        )
        self._device_info_pub = self.create_publisher(
            DeviceInfo, self._device_info_topic, 10
        )
        self._waypoint_reach_status_pub = self.create_publisher(
            WaypointReachStatusArray, self._waypoint_reach_status_topic, 10
        )

        self._single_point_srv = self.create_service(
            NavigateToPoint, self._single_point_service, self._on_single_point
        )
        self._multi_point_srv = self.create_service(
            NavigateToPoints, self._multi_point_service, self._on_multi_point
        )
        self._pause_srv = self.create_service(
            Trigger, self._pause_service, self._on_pause
        )
        self._resume_srv = self.create_service(
            Trigger, self._resume_service, self._on_resume
        )
        self._abort_srv = self.create_service(
            Trigger, self._abort_service, self._on_abort
        )
        self._return_home_srv = self.create_service(
            NavigateToPoint, self._return_home_service, self._on_return_home
        )
        self._clear_srv = self.create_service(
            Trigger, self._clear_service, self._on_clear
        )

        self.create_subscription(
            Path, self._global_path_topic, self._on_global_path, 10
        )
        self.create_subscription(
            Path, self._actual_path_topic, self._on_actual_path, 10
        )
        self.create_subscription(
            Path, self._local_path_topic, self._on_local_path, 10
        )
        self.create_subscription(
            Float32, self._progress_topic, self._on_progress, 10
        )
        self.create_subscription(
            String, self._waypoint_status_topic, self._on_waypoint_status, 10
        )
        self.create_subscription(
            Odometry, self._localization_topic, self._on_localization, 10
        )
        self.create_subscription(Odometry, self._odom_topic, self._on_odom, 10)
        self.create_subscription(
            Bool,
            self._localization_status_topic,
            self._on_localization_status,
            10,
        )
        self._status_timer = self.create_timer(
            1.0 / status_publish_rate, self._publish_status
        )
        self._device_info_timer = self.create_timer(
            1.0 / device_info_publish_rate, self._publish_device_info
        )

        self.get_logger().info(
            "总控导航接口已启动："
            f"single={self._single_point_service}, "
            f"multi={self._multi_point_service}, "
            f"pause={self._pause_service}, "
            f"resume={self._resume_service}, "
            f"abort={self._abort_service}, "
            f"return_home={self._return_home_service}, "
            f"clear={self._clear_service}"
        )

    @staticmethod
    def _is_heartbeat_alive(
        now: float,
        localization_time: Optional[float],
        odom_time: Optional[float],
        timeout: float,
    ) -> bool:
        """根据两路消息的本地接收时间判断设备心跳。"""
        if localization_time is None or odom_time is None:
            return False
        return (
            now - localization_time <= timeout
            and now - odom_time <= timeout
        )

    @staticmethod
    def _validate_target(target: PointStamped) -> Optional[str]:
        if not target.header.frame_id.strip():
            return "header.frame_id 不能为空"

        coordinates = (target.point.x, target.point.y, target.point.z)
        if not all(math.isfinite(value) for value in coordinates):
            return "目标点坐标必须为有限数值"
        return None

    @staticmethod
    def _parse_waypoint_status(status: str) -> Tuple[str, Optional[int]]:
        """将导航模块状态转换为总控到达状态动作。"""
        text = status.strip().lower()
        done = re.match(r"^done:\s*(\d+)\s+waypoint\(s\)", text)
        if done:
            return "done", int(done.group(1))
        if text.startswith("empty"):
            return "empty", None

        match = re.match(
            r"^(?:active|paused)\s*:\s*waypoint\s*=\s*(\d+)\s*/\s*(\d+)",
            text,
        )
        if match is None:
            return "ignore", None
        current = int(match.group(1))
        total = int(match.group(2))
        if current < 1 or total < 1 or current > total:
            return "ignore", None
        return "progress", current - 1

    def _has_navigation_subscriber(self, publisher, topic_name: str) -> bool:
        if not self._require_navigation_subscriber:
            return True
        if publisher.get_subscription_count() > 0:
            return True
        self.get_logger().warn(f"命令未转发：{topic_name} 当前没有订阅者")
        return False

    def _all_publishers_ready(self, publishers) -> bool:
        """组合命令发布前一次性检查全部订阅者，避免只执行前半段。"""
        return all(
            self._has_navigation_subscriber(publisher, topic_name)
            for publisher, topic_name in publishers
        )

    def _append_targets(self, targets) -> None:
        self.last_targets.extend(targets)
        self.reached.extend(False for _ in targets)

    def _clear_targets(self) -> None:
        self.last_targets.clear()
        self.reached.clear()

    def _send_command(self, command, targets=()):
        if not self._has_navigation_subscriber(self._command_pub, self._command_topic):
            return False
        message = NavigationTaskCommand()
        message.command_id = uuid.uuid4().hex
        message.command = command
        message.targets = list(targets)
        self._command_id = message.command_id
        self._command_pub.publish(message)
        return True

    def _on_single_point(self, request, response):
        error = self._validate_target(request.target)
        if error is not None:
            response.success = False
            response.message = f"请求失败：{error}"
            return response

        if not self._has_navigation_subscriber(
            self._clicked_point_pub, self._clicked_point_topic
        ):
            response.success = False
            response.message = f"转发失败：{self._clicked_point_topic} 当前没有订阅者"
            return response

        if not self._send_command("append", [request.target]):
            response.success, response.message = False, "航点管理器有序命令通道不可用"
            return response
        # /clicked_point 是追加语义，总控状态也必须同步追加。
        self._append_targets([request.target])
        self.navigation_state = self.STATE_RUNNING
        response.success = True
        response.message = f"单点导航命令已转发到 {self._command_topic}"
        self.get_logger().info(response.message)
        return response

    def _on_multi_point(self, request, response):
        response.accepted_count = 0
        if not request.targets:
            response.success = False
            response.message = "请求失败：航点列表不能为空"
            return response

        for index, target in enumerate(request.targets):
            error = self._validate_target(target)
            if error is not None:
                response.success = False
                response.message = f"请求失败：第 {index + 1} 个航点无效，{error}"
                return response

        if not self._has_navigation_subscriber(
            self._clicked_point_pub, self._clicked_point_topic
        ):
            response.success = False
            response.message = f"转发失败：{self._clicked_point_topic} 当前没有订阅者"
            return response

        # config_path是预留字段，无论传none或路径，本版本均不加载配置。
        if not self._send_command("append", request.targets):
            response.success, response.message = False, "航点管理器有序命令通道不可用"
            return response

        self._append_targets(request.targets)
        self.navigation_state = self.STATE_RUNNING
        response.success = True
        response.accepted_count = len(request.targets)
        response.message = (
            f"{response.accepted_count} 个航点已按顺序转发到 "
            f"{self._command_topic}"
        )
        self.get_logger().info(response.message)
        return response

    def _on_pause(self, request, response):
        del request
        if not self._has_navigation_subscriber(
            self._pause_pub, self._pause_topic
        ):
            response.success = False
            response.message = f"转发失败：{self._pause_topic} 当前没有订阅者"
            return response

        if not self._send_command("pause"):
            response.success, response.message = False, "航点管理器有序命令通道不可用"
            return response
        self.navigation_state = self.STATE_PAUSED
        response.success = True
        response.message = f"设备临停命令已转发到 {self._command_topic}"
        self.get_logger().info(response.message)
        return response

    def _on_resume(self, request, response):
        del request
        if not self._has_navigation_subscriber(
            self._resume_pub, self._resume_topic
        ):
            response.success = False
            response.message = f"转发失败：{self._resume_topic} 当前没有订阅者"
            return response

        if not self._send_command("resume"):
            response.success, response.message = False, "航点管理器有序命令通道不可用"
            return response
        self.navigation_state = self.STATE_RUNNING
        response.success = True
        response.message = f"恢复任务命令已转发到 {self._command_topic}"
        self.get_logger().info(response.message)
        return response

    def _on_abort(self, request, response):
        del request
        if not self._all_publishers_ready(
            (
                (self._pause_pub, self._pause_topic),
                (self._clear_pub, self._clear_topic),
            )
        ):
            response.success = False
            response.message = "转发失败：暂停或清空 Topic 当前没有订阅者"
            return response

        if not self._send_command("abort"):
            response.success, response.message = False, "航点管理器有序命令通道不可用"
            return response
        self._clear_targets()
        self.navigation_state = self.STATE_IDLE
        response.success = True
        response.message = "中止任务组合命令已转发到有序命令通道"
        self.get_logger().info(response.message)
        return response

    def _on_return_home(self, request, response):
        error = self._validate_target(request.target)
        if error is not None:
            response.success = False
            response.message = f"请求失败：{error}"
            return response

        if not self._all_publishers_ready(
            (
                (self._pause_pub, self._pause_topic),
                (self._clear_pub, self._clear_topic),
                (self._clicked_point_pub, self._clicked_point_topic),
            )
        ):
            response.success = False
            response.message = "转发失败：返航所需 Topic 当前没有订阅者"
            return response

        if not self._send_command("return_home", [request.target]):
            response.success, response.message = False, "航点管理器有序命令通道不可用"
            return response
        # 返航会替换现有任务，因此只保留唯一返航点。
        self.last_targets = [request.target]
        self.reached = [False]
        self.navigation_state = self.STATE_RUNNING
        response.success = True
        response.message = "返航组合命令已转发到有序命令通道"
        self.get_logger().info(response.message)
        return response

    def _on_clear(self, request, response):
        del request
        if not self._has_navigation_subscriber(
            self._clear_pub, self._clear_topic
        ):
            response.success = False
            response.message = f"转发失败：{self._clear_topic} 当前没有订阅者"
            return response

        if not self._send_command("clear"):
            response.success, response.message = False, "航点管理器有序命令通道不可用"
            return response
        self._clear_targets()
        self.navigation_state = self.STATE_IDLE
        response.success = True
        response.message = f"清空航点命令已转发到 {self._command_topic}"
        self.get_logger().info(response.message)
        return response

    def _on_global_path(self, message: Path) -> None:
        self._global_path = sample_telemetry_path(message)

    def _on_actual_path(self, message: Path) -> None:
        self._actual_path = sample_telemetry_path(message)

    def _on_local_path(self, message: Path) -> None:
        self._local_path = sample_telemetry_path(message)

    def _on_progress(self, message: Float32) -> None:
        value = float(message.data)
        self._progress_percent = (
            min(100.0, max(0.0, value)) if math.isfinite(value) else 0.0
        )

    def _on_localization(self, message: Odometry) -> None:
        self._localization = message
        # 使用单调时钟记录本地接收时刻，避免 ROS 消息时间戳异常影响心跳。
        self._localization_receive_time = time.monotonic()

    def _on_odom(self, message: Odometry) -> None:
        self._odom = message
        self._odom_receive_time = time.monotonic()

    def _on_localization_status(self, message: Bool) -> None:
        self._localization_ok = bool(message.data)

    def _on_waypoint_status(self, message: String) -> None:
        # 文本状态没有命令编号，不能作为新接口任务的到达确认。
        if self._command_id:
            return
        action, reached_count = self._parse_waypoint_status(message.data)
        if action == "done" and reached_count is not None:
            for index in range(min(reached_count, len(self.reached))):
                self.reached[index] = True
            if reached_count == len(self.last_targets):
                self.navigation_state = self.STATE_IDLE
        elif action == "empty":
            self._clear_targets()
            self.navigation_state = self.STATE_IDLE
        elif action == "progress" and reached_count is not None:
            # waypoint=N/M 表示第 N 点正在执行，序号小于 N 的点均已到达。
            for index in range(min(reached_count, len(self.reached))):
                self.reached[index] = True
            self.navigation_state = (
                self.STATE_PAUSED
                if message.data.strip().lower().startswith("paused")
                else self.STATE_RUNNING
            )

    def _on_task_status(self, message: NavigationTaskStatus) -> None:
        if not self._command_id or message.command_id != self._command_id:
            return
        # 同编号的明确empty也覆盖RViz直接清空；旧编号empty一律忽略。
        if message.state == "empty" and message.total_count == 0 and message.reached_count == 0:
            self._clear_targets()
            self.navigation_state = self.STATE_IDLE
            return
        total = len(self.last_targets)
        if message.total_count != total or message.reached_count > total:
            return
        if message.state not in ("empty", "active", "paused", "done"):
            return
        if message.state == "empty" and total != 0:
            return
        if message.state == "done" and message.reached_count != total:
            return
        self.reached = [index < message.reached_count for index in range(total)]
        self.navigation_state = (self.STATE_IDLE if message.state in ("empty", "done") else
                                 self.STATE_PAUSED if message.state == "paused" else self.STATE_RUNNING)

    def _publish_device_info(self) -> None:
        device_info = DeviceInfo()
        device_info.header.stamp = self.get_clock().now().to_msg()
        if self._localization is not None:
            device_info.header.frame_id = self._localization.header.frame_id
            device_info.pose = self._localization.pose.pose
        if self._odom is not None:
            device_info.velocity = self._odom.twist.twist
        device_info.heartbeat = self._is_heartbeat_alive(
            time.monotonic(),
            self._localization_receive_time,
            self._odom_receive_time,
            self._heartbeat_timeout,
        )
        device_info.localization_ok = self._localization_ok
        self._device_info_pub.publish(device_info)

    def _publish_status(self) -> None:
        stamp = self.get_clock().now().to_msg()

        telemetry = NavigationTelemetry()
        telemetry.header.stamp = stamp
        telemetry.global_path = self._global_path
        telemetry.actual_path = self._actual_path
        telemetry.local_path = self._local_path
        telemetry.progress_percent = self._progress_percent
        self._telemetry_pub.publish(telemetry)

        status_array = WaypointReachStatusArray()
        status_array.header.stamp = stamp
        for index, (target, reached) in enumerate(
            zip(self.last_targets, self.reached), start=1
        ):
            waypoint = WaypointReachStatus()
            waypoint.index = index
            waypoint.target = target
            waypoint.reached = reached
            status_array.waypoints.append(waypoint)
        self._waypoint_reach_status_pub.publish(status_array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavigationCommandNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
