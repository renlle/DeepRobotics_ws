#!/usr/bin/env python3

import copy

import rclpy
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class NavigationPathMux(Node):
    def __init__(self):
        super().__init__("navigation_path_mux")
        self.declare_parameter("input_path_topic", "waypoint_global_path")
        self.declare_parameter("output_path_topic", "global_path")
        self.declare_parameter("pure_pursuit_path_topic", "global_path")

        input_topic = str(self.get_parameter("input_path_topic").value)
        output_topic = str(self.get_parameter("output_path_topic").value)
        pure_pursuit_topic = str(self.get_parameter("pure_pursuit_path_topic").value)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._path_pub = self.create_publisher(Path, output_topic, qos)
        self._forwarded_count = 0
        same_output = output_topic.lstrip("/") == pure_pursuit_topic.lstrip("/")
        self._pure_pursuit_path_pub = (
            None if same_output else self.create_publisher(Path, pure_pursuit_topic, qos)
        )
        self.create_subscription(Path, input_topic, self._on_path, qos)
        self.get_logger().info(
            f"navigation path mux ready: input={input_topic} output={output_topic}"
        )

    def _on_path(self, msg: Path):
        selected = copy.deepcopy(msg)
        self._path_pub.publish(selected)
        if self._pure_pursuit_path_pub is not None:
            self._pure_pursuit_path_pub.publish(copy.deepcopy(selected))
        self._forwarded_count += 1
        if self._forwarded_count == 1 or self._forwarded_count % 100 == 0:
            self.get_logger().info(
                f"forwarded path #{self._forwarded_count}: poses={len(msg.poses)} "
                f"frame={msg.header.frame_id}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = NavigationPathMux()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
