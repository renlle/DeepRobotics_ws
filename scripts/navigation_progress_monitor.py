#!/usr/bin/env python3

import math
import os
from pathlib import Path as FilePath
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32

Point = Tuple[float, float]


class NavigationProgressMonitor(Node):
    def __init__(self) -> None:
        super().__init__("navigation_progress_monitor")
        workspace = FilePath(__file__).resolve().parent.parent
        output_dir = workspace / "tmp"
        output_dir.mkdir(parents=True, exist_ok=True)
        self.output_path = output_dir / "navigation_progress.jpg"
        self.temporary_path = output_dir / "navigation_progress.tmp.jpg"

        self.global_path: List[Point] = []
        self.actual_path: List[Point] = []
        self.local_path: List[Point] = []
        self.latest_position: Optional[Point] = None
        self.previous_position: Optional[Point] = None
        self.traveled_distance = 0.0
        self.remaining_distance = 0.0
        self.progress_percent = 0.0
        self.navigation_active = False

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 航点管理器是进度的唯一发布者；绘图不能用空路径覆盖完成状态。
        self.create_subscription(
            Float32, "/navigation/progress_percent", self._on_progress, 10
        )
        self.create_subscription(Path, "/global_path", self._on_global_path, 10)
        self.create_subscription(
            Path, "/global_localization/path", self._on_actual_path, 10
        )
        self.create_subscription(Path, "/local_path", self._on_local_path, 10)
        self.create_subscription(
            Pose,
            "/global_localization/ui_horizontal_pose",
            self._on_position,
            10,
        )
        self.create_timer(0.2, self._draw_image)

        self.get_logger().info(
            "导航进度监控已启动：订阅 /navigation/progress_percent，"
            f"5 Hz 覆盖绘制 {self.output_path}"
        )

    @staticmethod
    def _path_points(message: Path) -> List[Point]:
        return [
            (pose.pose.position.x, pose.pose.position.y) for pose in message.poses
        ]

    def _on_global_path(self, message: Path) -> None:
        points = self._path_points(message)
        if len(points) < 2:
            self.global_path = []
            self.navigation_active = False
            self.remaining_distance = 0.0
            return

        if not self.navigation_active:
            self.traveled_distance = 0.0
            self.previous_position = self.latest_position
        self.navigation_active = True
        self.global_path = points
        self._update_remaining_distance()

    def _on_progress(self, message: Float32) -> None:
        if not math.isfinite(message.data):
            return
        progress = max(0.0, min(100.0, float(message.data)))
        # 新任务或清空由航点管理器明确归零，路径/定位消息不改变任务进度。
        if progress == 0.0 and self.progress_percent != 0.0:
            self.traveled_distance = 0.0
            self.previous_position = self.latest_position
        self.progress_percent = progress

    def _on_actual_path(self, message: Path) -> None:
        self.actual_path = self._path_points(message)

    def _on_local_path(self, message: Path) -> None:
        points = self._path_points(message)
        source_frame = message.header.frame_id or "map"
        if source_frame == "map" or not points:
            self.local_path = points
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                "map", source_frame, Time(), timeout=Duration(seconds=0.05)
            )
        except tf2_ros.TransformException as error:
            self.get_logger().debug(
                f"暂时无法转换蓝色局部路线 {source_frame} -> map: {error}"
            )
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        self.local_path = [
            (
                translation.x + cos_yaw * x - sin_yaw * y,
                translation.y + sin_yaw * x + cos_yaw * y,
            )
            for x, y in points
        ]

    def _on_position(self, message: Pose) -> None:
        position = (message.position.x, message.position.y)
        self.latest_position = position
        if self.navigation_active and self.previous_position is not None:
            self.traveled_distance += math.hypot(
                position[0] - self.previous_position[0],
                position[1] - self.previous_position[1],
            )
        self.previous_position = position
        self._update_remaining_distance()

    def _update_remaining_distance(self) -> None:
        if not self.navigation_active or self.latest_position is None:
            self.remaining_distance = 0.0
            return

        self.remaining_distance = self._remaining_path_distance(
            self.global_path, self.latest_position
        )

    @staticmethod
    def _remaining_path_distance(path: List[Point], position: Point) -> float:
        if len(path) < 2:
            return 0.0

        best_distance_sq = float("inf")
        best_segment = 0
        best_ratio = 0.0
        for index, (start, end) in enumerate(zip(path, path[1:])):
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            length_sq = dx * dx + dy * dy
            if length_sq <= 1e-12:
                ratio = 0.0
            else:
                ratio = max(
                    0.0,
                    min(
                        1.0,
                        ((position[0] - start[0]) * dx + (position[1] - start[1]) * dy)
                        / length_sq,
                    ),
                )
            projected_x = start[0] + ratio * dx
            projected_y = start[1] + ratio * dy
            distance_sq = (position[0] - projected_x) ** 2 + (
                position[1] - projected_y
            ) ** 2
            if distance_sq < best_distance_sq:
                best_distance_sq = distance_sq
                best_segment = index
                best_ratio = ratio

        segment_start = path[best_segment]
        segment_end = path[best_segment + 1]
        remaining = (1.0 - best_ratio) * math.hypot(
            segment_end[0] - segment_start[0],
            segment_end[1] - segment_start[1],
        )
        remaining += sum(
            math.hypot(end[0] - start[0], end[1] - start[1])
            for start, end in zip(path[best_segment + 1 :], path[best_segment + 2 :])
        )
        return remaining

    def _draw_image(self) -> None:
        image = np.full((800, 1000, 3), 245, dtype=np.uint8)
        plot_left, plot_top, plot_right, plot_bottom = 50, 110, 950, 750
        all_points = self.global_path + self.actual_path + self.local_path
        if self.latest_position is not None:
            all_points.append(self.latest_position)

        if all_points:
            xs = [point[0] for point in all_points]
            ys = [point[1] for point in all_points]
            min_x, max_x = min(xs), max(xs)
            min_y, max_y = min(ys), max(ys)
            span_x = max(max_x - min_x, 1.0)
            span_y = max(max_y - min_y, 1.0)
            margin = 0.08
            min_x -= span_x * margin
            max_x += span_x * margin
            min_y -= span_y * margin
            max_y += span_y * margin
            span_x = max_x - min_x
            span_y = max_y - min_y
            scale = min(
                (plot_right - plot_left) / span_x,
                (plot_bottom - plot_top) / span_y,
            )
            center_x = (min_x + max_x) / 2.0
            center_y = (min_y + max_y) / 2.0

            def pixel(point: Point) -> Tuple[int, int]:
                return (
                    int((plot_left + plot_right) / 2 + (point[0] - center_x) * scale),
                    int((plot_top + plot_bottom) / 2 - (point[1] - center_y) * scale),
                )

            self._draw_polyline(image, self.global_path, pixel, (0, 180, 0), 4)
            self._draw_polyline(image, self.actual_path, pixel, (0, 0, 255), 4)
            self._draw_polyline(image, self.local_path, pixel, (255, 0, 0), 3)
            if self.latest_position is not None:
                cv2.circle(image, pixel(self.latest_position), 7, (0, 0, 0), -1)
        else:
            cv2.putText(
                image,
                "Waiting for navigation paths...",
                (300, 420),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (80, 80, 80),
                2,
                cv2.LINE_AA,
            )

        cv2.putText(
            image,
            f"Navigation progress: {self.progress_percent:.1f}%",
            (50, 48),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.05,
            (20, 20, 20),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            f"Traveled: {self.traveled_distance:.2f} m    Remaining: {self.remaining_distance:.2f} m",
            (50, 84),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.68,
            (50, 50, 50),
            2,
            cv2.LINE_AA,
        )
        self._draw_legend(image)

        if cv2.imwrite(str(self.temporary_path), image):
            os.replace(self.temporary_path, self.output_path)
        else:
            self.get_logger().error(f"JPG 写入失败：{self.temporary_path}")

    @staticmethod
    def _draw_polyline(image, points, pixel, color, thickness) -> None:
        if len(points) < 2:
            return
        cv_points = np.array([pixel(point) for point in points], dtype=np.int32)
        cv2.polylines(image, [cv_points], False, color, thickness, cv2.LINE_AA)

    @staticmethod
    def _draw_legend(image) -> None:
        entries = [
            ("Global plan", (0, 180, 0)),
            ("Actual trajectory", (0, 0, 255)),
            ("Local execution plan", (255, 0, 0)),
        ]
        x = 590
        for index, (label, color) in enumerate(entries):
            y = 30 + index * 28
            cv2.line(image, (x, y), (x + 36, y), color, 4, cv2.LINE_AA)
            cv2.putText(
                image,
                label,
                (x + 46, y + 6),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (30, 30, 30),
                1,
                cv2.LINE_AA,
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavigationProgressMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
