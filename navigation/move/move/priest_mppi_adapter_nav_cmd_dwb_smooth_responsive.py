#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
try:
    from move.runtime_metrics import Reporter, measured, stamp_age_ms
except ModuleNotFoundError:
    from runtime_metrics import Reporter, measured, stamp_age_ms
from typing import Optional

import numpy as np
import rclpy
from drdds.msg import NavCmd, StdMsgInt32
from master_control_interfaces.msg import NavigationTaskCommand
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Empty, Float32
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

import tf2_ros


def yaw_to_quat(yaw: float):
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


def clamp_abs(value: float, limit: float) -> float:
    if limit <= 0.0 or not math.isfinite(limit):
        return value
    return max(-limit, min(limit, value))


class PriestMppiAdapterNavCmd(Node):
    def __init__(self):
        super().__init__("priest_mppi_adapter_nav_cmd")
        self._perf = Reporter(self, "adapter", ("path", "odom", "cmd_vel", "nav_cmd", "goal"),
            lambda: {"odom_age_ms": stamp_age_ms(self, self.latest_odom),
                     "path_pose_age_ms": stamp_age_ms(self, self.latest_path),
                     "cmd_received_age_ms": (self.get_clock().now()-self.latest_cmd_time).nanoseconds*1e-6
                     if self.latest_cmd_time is not None else -1,
                     "motion_fault": self._motion_fault,
                     "goal_active": self._active_goal_handle is not None,
                     "goal_pending": self._pending_goal,
                     "control_mode": self._control_usage_mode,
                     "global_path_active": self.global_path_active,
                     "localization_ready": self.localization_ready,
                     "cmd_timeout": self._cmd_timeout_active,
                     "queue_mode": "latest_only"})

        self.declare_parameter("priest_path_topic", "local_path")
        self.declare_parameter("global_path_topic", "global_path")
        self.declare_parameter("resume_topic", "waypoint_sequence/resume")
        self.declare_parameter("control_mode_topic", "/chassis/control_usage_mode")
        self.declare_parameter("path_target_frame", "camera_init_footprint")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("tf_timeout", 0.03)
        # The normal local-path period is about 0.2 s. Keep a bounded grace
        # window for one delayed inference cycle without masking a real stall.
        self.declare_parameter("path_timeout", 1.2)
        self.declare_parameter("goal_send_hz", 3.0)
        self.declare_parameter("min_goal_resend_interval", 0.35)
        self.declare_parameter("min_path_points", 4)
        self.declare_parameter("publish_mppi_path", True)
        self.declare_parameter("mppi_path_topic", "mppi_path")
        self.declare_parameter("require_localization_confidence", False)
        self.declare_parameter("localization_confidence_topic", "/localization_3d_confidence")
        self.declare_parameter("localization_confidence_threshold", 0.65)
        self.declare_parameter("localization_confidence_timeout", 1.5)

        self.declare_parameter("follow_path_action", "follow_path")
        self.declare_parameter("controller_id", "FollowPath")
        self.declare_parameter("goal_checker_id", "general_goal_checker")
        self.declare_parameter("progress_checker_id", "progress_checker")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("nav_cmd_topic", "/NAV_CMD")
        self.declare_parameter("cmd_frame_id", 0)
        self.declare_parameter("nav_cmd_publish_hz", 15.0)
        self.declare_parameter("cmd_timeout", 2.0)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_timeout", 0.5)
        self.declare_parameter("motion_feedback_timeout", 3.0)
        self.declare_parameter("scale_x", 1.0)
        self.declare_parameter("scale_y", 1.0)
        self.declare_parameter("scale_yaw", 1.0)
        self.declare_parameter("max_x_vel", 0.8)
        self.declare_parameter("max_y_vel", 0.8)
        self.declare_parameter("max_yaw_vel", 1.0)

        self.priest_path_topic = self.get_parameter("priest_path_topic").value
        self.global_path_topic = self.get_parameter("global_path_topic").value
        self.path_target_frame = self._normalize_frame_id(self.get_parameter("path_target_frame").value)
        self.base_frame = self._normalize_frame_id(self.get_parameter("base_frame").value)
        self.tf_timeout = float(self.get_parameter("tf_timeout").value)
        self.path_timeout = float(self.get_parameter("path_timeout").value)
        self.goal_send_hz = float(self.get_parameter("goal_send_hz").value)
        self.min_goal_resend_interval = float(self.get_parameter("min_goal_resend_interval").value)
        self.min_path_points = int(self.get_parameter("min_path_points").value)
        self.publish_mppi_path = bool(self.get_parameter("publish_mppi_path").value)
        self.mppi_path_topic = self.get_parameter("mppi_path_topic").value
        self.require_localization_confidence = bool(self.get_parameter("require_localization_confidence").value)
        self.localization_confidence_topic = self.get_parameter("localization_confidence_topic").value
        self.localization_confidence_threshold = float(self.get_parameter("localization_confidence_threshold").value)
        self.localization_confidence_timeout = float(self.get_parameter("localization_confidence_timeout").value)

        self.follow_path_action = self.get_parameter("follow_path_action").value
        self.controller_id = self.get_parameter("controller_id").value
        self.goal_checker_id = self.get_parameter("goal_checker_id").value
        self.progress_checker_id = self.get_parameter("progress_checker_id").value

        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.nav_cmd_topic = self.get_parameter("nav_cmd_topic").value
        self.cmd_frame_id = int(self.get_parameter("cmd_frame_id").value)
        self.nav_cmd_publish_hz = float(self.get_parameter("nav_cmd_publish_hz").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        self.odom_timeout = float(self.get_parameter("odom_timeout").value)
        self.motion_feedback_timeout = float(self.get_parameter("motion_feedback_timeout").value)
        self.scale_x = float(self.get_parameter("scale_x").value)
        self.scale_y = float(self.get_parameter("scale_y").value)
        self.scale_yaw = float(self.get_parameter("scale_yaw").value)
        self.max_x_vel = float(self.get_parameter("max_x_vel").value)
        self.max_y_vel = float(self.get_parameter("max_y_vel").value)
        self.max_yaw_vel = float(self.get_parameter("max_yaw_vel").value)

        qos_path = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.path_sub = self.create_subscription(Path, self.priest_path_topic, self._on_path, qos_path)
        self.global_path_sub = self.create_subscription(
            Path, self.global_path_topic, self._on_global_path, qos_path
        )
        self.cmd_vel_sub = self.create_subscription(Twist, self.cmd_vel_topic, self._on_cmd_vel, 10)
        self.odom_sub = self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self._on_odom, 1
        )
        self.resume_sub = self.create_subscription(
            Empty, self.get_parameter("resume_topic").value, self._on_resume, 10
        )
        self.task_command_sub = self.create_subscription(
            NavigationTaskCommand, "/waypoint_sequence/command", self._on_task_command, 100)
        self.control_mode_sub = self.create_subscription(
            StdMsgInt32, self.get_parameter("control_mode_topic").value,
            self._on_control_mode, 10
        )
        self.localization_conf_sub = self.create_subscription(
            Float32, self.localization_confidence_topic, self._on_localization_confidence, 10
        )

        self.mppi_path_pub = self.create_publisher(Path, self.mppi_path_topic, 10) if self.publish_mppi_path else None
        self.nav_cmd_pub = self.create_publisher(NavCmd, self.nav_cmd_topic, 10)

        self.follow_path_client = ActionClient(self, FollowPath, self.follow_path_action)

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.latest_path: Optional[Path] = None
        self.latest_path_time = None
        self.latest_path_seq = 0
        self.last_sent_seq = -1
        self.global_path_active = False
        self._last_global_clear_time = None
        self._minimum_goal_seq = 0
        self.latest_odom = None
        self._odom_received_time = None
        self._motion_goal_key = None
        self._motion_fault = False
        self._motion_anchor = None
        self._last_motion_time = None
        self._control_usage_mode = None

        self.latest_cmd_vel = Twist()
        self.latest_cmd_time = None
        self._cmd_timeout_active = False

        self._last_feedback_log = 0.0
        self._active_goal_handle = None
        self._active_goal_seq = -1
        self._pending_goal = False
        self._cancel_requested = False
        self._last_goal_send_time = None
        self.localization_confidence = 0.0
        self.localization_ready = not self.require_localization_confidence
        self.localization_confidence_time = None
        self._localization_timeout_active = False

        send_period = 1.0 / max(1e-3, self.goal_send_hz)
        self.goal_timer = self.create_timer(send_period, self._on_goal_timer)

        cmd_bridge_period = 1.0 / max(1e-3, self.nav_cmd_publish_hz)
        self.cmd_bridge_timer = self.create_timer(cmd_bridge_period, self._on_cmd_bridge_timer)

        self.get_logger().info(
            "[priest_mppi_adapter_nav_cmd] ready\n"
            f"  priest_path_topic={self.priest_path_topic}\n"
            f"  path_target_frame={self.path_target_frame}\n"
            f"  follow_path_action={self.follow_path_action}\n"
            f"  controller_id={self.controller_id}\n"
            f"  min_goal_resend_interval={self.min_goal_resend_interval}\n"
            f"  mppi_path_topic={self.mppi_path_topic} (publish={self.publish_mppi_path})\n"
            f"  require_localization_confidence={self.require_localization_confidence}\n"
            f"  localization_confidence_topic={self.localization_confidence_topic}\n"
            f"  localization_confidence_threshold={self.localization_confidence_threshold}\n"
            f"  localization_confidence_timeout={self.localization_confidence_timeout}\n"
            f"  cmd_vel_topic={self.cmd_vel_topic} -> nav_cmd_topic={self.nav_cmd_topic}\n"
            f"  nav_cmd_publish_hz={self.nav_cmd_publish_hz}, cmd_timeout={self.cmd_timeout}"
        )

    @staticmethod
    def _normalize_frame_id(frame_id: str) -> str:
        return (frame_id or "").lstrip("/")

    @measured("path")
    def _on_path(self, msg: Path):
        first_path = self.latest_path_seq == 0
        if len(msg.poses) < self.min_path_points:
            self.latest_path = None
            self.latest_path_time = None
            self.latest_path_seq += 1
            self.last_sent_seq = -1
            self.latest_cmd_time = None
            self._publish_nav_cmd(0.0, 0.0, 0.0)
            self._cancel_active_follow_path("local path cleared")
            self._publish_empty_mppi_path()
            return
        # 全局任务清空后，丢弃仍在推理或传输中的旧局部路径。
        if not self.global_path_active:
            return
        now = self.get_clock().now()
        path_stamp = Time.from_msg(msg.header.stamp, clock_type=now.clock_type)
        # 收到旧帧不代表数据新鲜，不能让传输或推理积压刷新控制看门狗。
        if not 0.0 <= (now - path_stamp).nanoseconds * 1e-9 <= self.path_timeout:
            return
        if self._last_global_clear_time is not None:
            path_stamp = Time.from_msg(
                msg.header.stamp, clock_type=self._last_global_clear_time.clock_type
            )
            if path_stamp <= self._last_global_clear_time:
                return
        self.latest_path = msg
        self.latest_path_time = self.get_clock().now()
        self.latest_path_seq += 1
        self._cancel_requested = False
        if first_path:
            self.get_logger().info(
                f"Received first local path: poses={len(msg.poses)} frame={msg.header.frame_id}"
            )

    def _on_global_path(self, msg: Path):
        key = None
        if msg.poses:
            end = msg.poses[-1].pose.position
            key = (msg.header.frame_id, end.x, end.y)
        if key != self._motion_goal_key:
            self._motion_goal_key = key
            self._motion_fault = False
            self._motion_anchor = None
            self._last_motion_time = None
        self.global_path_active = bool(msg.poses)
        if not self.global_path_active:
            # 直接关断速度出口，不等待低频 RL 推理转发空路径。
            self._last_global_clear_time = self.get_clock().now()
            self._on_path(Path())

    def _path_is_fresh(self):
        if self._motion_fault:
            return False
        if not self.global_path_active or self.latest_path is None or self.latest_path_time is None:
            return False
        now = self.get_clock().now()
        age = (now - self.latest_path_time).nanoseconds * 1e-9
        stamp = Time.from_msg(self.latest_path.header.stamp, clock_type=now.clock_type)
        pose_age = (now - stamp).nanoseconds * 1e-9
        return 0.0 <= age <= self.path_timeout and 0.0 <= pose_age <= self.path_timeout

    def _reset_execution(self, reason):
        # 保留航点；作废旧动作、速度和推理结果，等待切换后的新路径与反馈。
        self._motion_fault = False
        self._motion_anchor = None
        self._last_motion_time = None
        self._last_global_clear_time = self.get_clock().now()
        self._on_path(Path())
        self.get_logger().info(f"Navigation execution reset: {reason}; waiting for a fresh path")

    def _on_resume(self, _msg: Empty):
        self._reset_execution("explicit resume of current waypoint")

    def _on_task_command(self, message: NavigationTaskCommand):
        if message.command == "resume":
            self._on_resume(Empty())

    def _on_control_mode(self, msg: StdMsgInt32):
        previous = self._control_usage_mode
        self._control_usage_mode = msg.value
        # 首帧只建立基线；不猜测厂商模式枚举，也不让重复状态刷新保护。
        if previous is not None and previous != msg.value:
            self._reset_execution(f"chassis control mode changed {previous} -> {msg.value}")

    @measured("odom")
    def _on_odom(self, msg: Odometry):
        self.latest_odom = msg
        self._odom_received_time = self.get_clock().now()

    def _motion_feedback_ok(self, x_vel, y_vel, yaw_vel):
        if self._motion_fault:
            return False
        now = self.get_clock().now()
        seconds = now.nanoseconds * 1e-9
        command_size = max(abs(x_vel), abs(y_vel), abs(yaw_vel))
        if command_size < 1e-6:
            self._last_motion_time = None
            return True
        odom = self.latest_odom
        if odom is None or self._odom_received_time is None:
            self._last_motion_time = None
            self.get_logger().warning("Motion gated: no odom feedback", throttle_duration_sec=2.0)
            return False
        stamp = Time.from_msg(odom.header.stamp, clock_type=now.clock_type)
        ages = [(now - t).nanoseconds * 1e-9 for t in (stamp, self._odom_received_time)]
        if any(not 0.0 <= age <= self.odom_timeout for age in ages):
            self._last_motion_time = None
            self.get_logger().warning(
                f"Motion gated: odom age/receive age={ages}, limit={self.odom_timeout}s",
                throttle_duration_sec=2.0
            )
            return False
        p, q = odom.pose.pose.position, odom.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y*q.y + q.z*q.z))
        if not all(math.isfinite(v) for v in (p.x, p.y, yaw)):
            self._last_motion_time = None
            return False
        if command_size < 0.1:
            self._last_motion_time = None
            return True
        if (self._motion_anchor is None or
                (self._last_motion_time is not None and seconds < self._last_motion_time)):
            self._motion_anchor = (0.0, p.x, p.y, yaw)
        effort, x0, y0, yaw0 = self._motion_anchor
        if self._last_motion_time is not None:
            dt = seconds - self._last_motion_time
            if 0.0 <= dt <= 0.5:
                effort += dt
        self._last_motion_time = seconds
        angle = abs(math.atan2(math.sin(yaw-yaw0), math.cos(yaw-yaw0)))
        if math.hypot(p.x-x0, p.y-y0) >= 0.1 or angle >= 0.15:
            self._motion_anchor = (0.0, p.x, p.y, yaw)
        elif effort >= self.motion_feedback_timeout:
            # 累计有效运动指令却无位姿响应，锁停至清任务或切换目标，重规划不能解锁。
            self._motion_fault = True
            self.get_logger().error(
                "Motion feedback stalled: commanded motion without odom progress; "
                "stop current goal, check localization/control; resume via control-mode change "
                "or waypoint_sequence/resume after checking the cause"
            )
            return False
        else:
            self._motion_anchor = (effort, x0, y0, yaw0)
        return True

    def _cancel_follow_path_goal_handle(self, goal_handle, reason: str):
        try:
            goal_handle.cancel_goal_async()
            self.get_logger().debug(f"Cancel FollowPath goal: {reason}")
        except Exception as exc:
            self.get_logger().warn(f"FollowPath cancel failed ({reason}): {exc}")

    def _cancel_active_follow_path(self, reason: str):
        # 即便请求尚未被接受，也必须使清空前的异步回调失效。
        self._minimum_goal_seq = self.latest_path_seq + 1
        self.latest_cmd_time = None
        self._publish_nav_cmd(0.0, 0.0, 0.0)
        goal_handle = self._active_goal_handle
        if goal_handle is None:
            return

        self._active_goal_handle = None
        self._active_goal_seq = -1
        self._cancel_requested = True
        self._cancel_follow_path_goal_handle(goal_handle, reason)

    def _publish_empty_mppi_path(self):
        if self.mppi_path_pub is None:
            return
        empty = Path()
        empty.header.frame_id = self.path_target_frame
        empty.header.stamp = self.get_clock().now().to_msg()
        self.mppi_path_pub.publish(empty)

    @measured("cmd_vel")
    def _on_cmd_vel(self, msg: Twist):
        if not self._path_is_fresh() or self._active_goal_handle is None:
            self.latest_cmd_time = None
            return
        self.latest_cmd_vel = msg
        self.latest_cmd_time = self.get_clock().now()
        self._cmd_timeout_active = False

    def _set_localization_ready(self, ready: bool, reason: str = ""):
        if self.localization_ready == ready:
            return
        self.localization_ready = ready
        if ready:
            self._localization_timeout_active = False
            self.get_logger().debug(
                f"Localization confidence recovered to {self.localization_confidence:.3f}, navigation output enabled"
            )
        else:
            self.last_sent_seq = -1
            self.get_logger().warn(
                f"Localization not ready ({reason}), freeze navigation output"
            )

    def _refresh_localization_state(self):
        if not self.require_localization_confidence:
            if not self.localization_ready:
                self._set_localization_ready(True, "confidence gate disabled")
            return
        if self.localization_confidence_time is None:
            self._set_localization_ready(False, "no confidence message received")
            return
        age = (self.get_clock().now() - self.localization_confidence_time).nanoseconds * 1e-9
        if age > self.localization_confidence_timeout:
            if not self._localization_timeout_active:
                self._localization_timeout_active = True
                self._set_localization_ready(False, f"confidence timeout {age:.2f}s")
            return
        ready = math.isfinite(self.localization_confidence) and (
            self.localization_confidence >= self.localization_confidence_threshold
        )
        self._set_localization_ready(ready, f"confidence {self.localization_confidence:.3f}")

    def _on_localization_confidence(self, msg: Float32):
        self.localization_confidence = float(msg.data)
        self.localization_confidence_time = self.get_clock().now()
        self._localization_timeout_active = False
        self._refresh_localization_state()

    def _lookup_tf(self, target: str, source: str, time=None):
        if time is None:
            time = Time()
        try:
            return self.tf_buffer.lookup_transform(
                target, source, time, timeout=Duration(seconds=self.tf_timeout)
            )
        except Exception as e:
            self.get_logger().debug(f"TF lookup failed: {target} <- {source}: {e}")
            return None

    @staticmethod
    def _apply_tf_xy(points_xy: np.ndarray, tf_msg) -> np.ndarray:
        tx = tf_msg.transform.translation.x
        ty = tf_msg.transform.translation.y
        q = tf_msg.transform.rotation
        yaw = 2.0 * math.atan2(q.z, q.w)
        c, s = math.cos(yaw), math.sin(yaw)
        rot = np.array([[c, -s], [s, c]], dtype=np.float64)
        pts = (rot @ points_xy.T).T
        pts[:, 0] += tx
        pts[:, 1] += ty
        return pts.astype(np.float64)

    def _path_to_target_frame(self, path_msg: Path) -> Optional[Path]:
        if len(path_msg.poses) < self.min_path_points:
            return None

        src_frame = self._normalize_frame_id(path_msg.header.frame_id) or self.base_frame
        pts_src = np.array(
            [(ps.pose.position.x, ps.pose.position.y) for ps in path_msg.poses],
            dtype=np.float64,
        )

        if src_frame == self.path_target_frame:
            pts_target = pts_src
        else:
            tf_msg = self._lookup_tf(self.path_target_frame, src_frame, path_msg.header.stamp)
            if tf_msg is None:
                return None
            pts_target = self._apply_tf_xy(pts_src, tf_msg)

        path_out = Path()
        path_out.header.frame_id = self.path_target_frame
        path_out.header.stamp = self.get_clock().now().to_msg()

        if pts_target.shape[0] < 2:
            return None

        headings = np.zeros((pts_target.shape[0],), dtype=np.float64)
        diffs = pts_target[1:] - pts_target[:-1]
        seg_yaws = np.arctan2(diffs[:, 1], diffs[:, 0])
        headings[:-1] = seg_yaws
        headings[-1] = seg_yaws[-1]

        for i in range(pts_target.shape[0]):
            ps = PoseStamped()
            ps.header = path_out.header
            ps.pose.position.x = float(pts_target[i, 0])
            ps.pose.position.y = float(pts_target[i, 1])
            ps.pose.position.z = 0.0
            qx, qy, qz, qw = yaw_to_quat(float(headings[i]))
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            path_out.poses.append(ps)

        return path_out

    def _publish_nav_cmd(self, x_vel: float, y_vel: float, yaw_vel: float):
        msg = NavCmd()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = int(self.cmd_frame_id)
        msg.data.x_vel = float(x_vel)
        msg.data.y_vel = float(y_vel)
        msg.data.yaw_vel = float(yaw_vel)
        self.nav_cmd_pub.publish(msg)

    @measured("nav_cmd")
    def _on_cmd_bridge_timer(self):
        self._refresh_localization_state()
        x_vel = 0.0
        y_vel = 0.0
        yaw_vel = 0.0

        if (self.localization_ready and self._path_is_fresh()
                and self._active_goal_handle is not None and self.latest_cmd_time is not None):
            age = (self.get_clock().now() - self.latest_cmd_time).nanoseconds * 1e-9
            if age <= self.cmd_timeout:
                x_vel = clamp_abs(self.latest_cmd_vel.linear.x * self.scale_x, self.max_x_vel)
                y_vel = clamp_abs(self.latest_cmd_vel.linear.y * self.scale_y, self.max_y_vel)
                yaw_vel = clamp_abs(self.latest_cmd_vel.angular.z * self.scale_yaw, self.max_yaw_vel)
            elif not self._cmd_timeout_active:
                self._cmd_timeout_active = True
                self.get_logger().warn(
                    f"/cmd_vel timeout ({age:.2f}s > {self.cmd_timeout:.2f}s), publish zero NAV_CMD"
                )
        elif not self.localization_ready:
            self._cmd_timeout_active = False

        if not self._motion_feedback_ok(x_vel, y_vel, yaw_vel):
            x_vel = y_vel = yaw_vel = 0.0
            if self._motion_fault and self._active_goal_handle is not None:
                self._cancel_active_follow_path("motion feedback stalled")
        self._publish_nav_cmd(x_vel, y_vel, yaw_vel)

    @measured("goal")
    def _on_goal_timer(self):
        self._refresh_localization_state()
        if not self.localization_ready:
            self._cancel_active_follow_path("localization not ready")
            return
        if self.latest_path is None or self.latest_path_time is None:
            return

        age = (self.get_clock().now() - self.latest_path_time).nanoseconds * 1e-9
        if not self._path_is_fresh():
            self._cancel_active_follow_path(f"local path timeout {age:.2f}s")
            return

        if self._pending_goal:
            return

        if self.latest_path_seq == self.last_sent_seq:
            return

        if self._last_goal_send_time is not None:
            resend_age = (self.get_clock().now() - self._last_goal_send_time).nanoseconds * 1e-9
            if resend_age < self.min_goal_resend_interval:
                return

        if not self.follow_path_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().debug("follow_path action server not available yet")
            return

        path_goal = self._path_to_target_frame(self.latest_path)
        if path_goal is None:
            return

        goal_msg = FollowPath.Goal()
        goal_msg.path = path_goal
        goal_msg.controller_id = self.controller_id
        goal_msg.goal_checker_id = self.goal_checker_id
        if hasattr(goal_msg, "progress_checker_id"):
            goal_msg.progress_checker_id = self.progress_checker_id

        if self.mppi_path_pub is not None:
            self.mppi_path_pub.publish(path_goal)

        self._pending_goal = True
        self._last_goal_send_time = self.get_clock().now()
        goal_seq = self.latest_path_seq
        future = self.follow_path_client.send_goal_async(goal_msg, feedback_callback=self._on_feedback)
        future.add_done_callback(lambda f, goal_seq=goal_seq: self._on_goal_response(f, goal_seq))

    def _on_goal_response(self, future, goal_seq: int):
        self._pending_goal = False
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().warn(f"FollowPath goal request failed: {e}")
            return
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn("FollowPath goal rejected")
            return
        if (goal_seq < self._minimum_goal_seq or not self._path_is_fresh()
                or not self.localization_ready):
            self._cancel_follow_path_goal_handle(goal_handle, "local path invalidated")
            return

        # A new local path may arrive while this asynchronous request is being
        # accepted. Keep this still-valid goal running until the next timer
        # submits the newer path; canceling it inserts a zero-velocity gap.
        if goal_seq != self.latest_path_seq:
            self.get_logger().debug(
                f"FollowPath goal seq {goal_seq} accepted behind latest path "
                f"seq {self.latest_path_seq}; scheduling a seamless update"
            )

        self._active_goal_handle = goal_handle
        self._active_goal_seq = goal_seq
        self._cancel_requested = False
        self.last_sent_seq = goal_seq
        if goal_seq == 1:
            self.get_logger().info("First FollowPath goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f, goal_handle=goal_handle, goal_seq=goal_seq: self._on_result(f, goal_handle, goal_seq)
        )

    @staticmethod
    def _same_goal_handle(lhs, rhs) -> bool:
        if lhs is None or rhs is None:
            return False
        lhs_goal_id = getattr(lhs, "goal_id", None)
        rhs_goal_id = getattr(rhs, "goal_id", None)
        if lhs_goal_id is not None and rhs_goal_id is not None:
            return lhs_goal_id == rhs_goal_id
        return lhs is rhs

    def _on_result(self, future, goal_handle, goal_seq: int):
        try:
            result_wrap = future.result()
        except Exception as e:
            self.get_logger().warn(f"FollowPath result failed: {e}")
            return

        is_current_goal = self._same_goal_handle(goal_handle, self._active_goal_handle)
        if is_current_goal:
            self._active_goal_handle = None
            self._active_goal_seq = -1
            self._cancel_requested = False
        status = getattr(result_wrap, "status", None)
        result = getattr(result_wrap, "result", None)
        if not is_current_goal:
            return
        if result is not None and hasattr(result, "error_code") and result.error_code != 0:
            self.get_logger().warn(
                f"FollowPath finished with error_code={result.error_code}, "
                f"error_msg='{getattr(result, 'error_msg', '')}' status={status}"
            )

    def _on_feedback(self, feedback_msg):
        now = time.time()
        if now - self._last_feedback_log < 1.0:
            return
        self._last_feedback_log = now
        fb = feedback_msg.feedback
        dist = float(getattr(fb, "distance_to_goal", 0.0))
        speed = float(getattr(fb, "speed", 0.0))
        self.get_logger().debug(f"[follow_path] distance_to_goal={dist:.3f}, speed={speed:.3f}")

    def destroy_node(self):
        try:
            self._publish_nav_cmd(0.0, 0.0, 0.0)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PriestMppiAdapterNavCmd()
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
