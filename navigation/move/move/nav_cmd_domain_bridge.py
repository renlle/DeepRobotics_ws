#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import queue as queue_module
import multiprocessing
import rclpy
from drdds.msg import NavCmd, StdMsgInt32
try:
    from move.runtime_metrics import Reporter
except ModuleNotFoundError:
    from runtime_metrics import Reporter

def subscriber_domain_42(queue, control_queue, metrics=None):
    # 接收端强制运行在我们指定的独立 Domain，动态从 device_parameters.yaml 中加载
    domain_id = "42"
    try:
        import yaml
        yaml_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../config/device_parameters.yaml"))
        if os.path.exists(yaml_path):
            with open(yaml_path, 'r', encoding='utf-8') as f:
                config_data = yaml.safe_load(f)
                if config_data and "/**" in config_data:
                    params = config_data["/**"].get("ros__parameters", {})
                    if "ros_domain_id" in params:
                        domain_id = str(params["ros_domain_id"])
    except Exception as e:
        print(f"Failed to read ros_domain_id in bridge, using default 42: {e}")
    os.environ["ROS_DOMAIN_ID"] = domain_id
    rclpy.init()
    node = rclpy.create_node("nav_cmd_bridge_sub")
    received_count = 0
    # 共享统计只有 Domain 0 单写、Domain 42 每 5 秒读取，允许瞬时采样误差。
    def bridge_state():
        try:
            pending = queue.qsize()
        except (NotImplementedError, OSError):
            pending = -1
        return dict(ipc_pending=pending, received_total=received_count,
                    forwarded_total=metrics[0] if metrics is not None else -1,
                    ipc_wait_ms=metrics[1] if metrics is not None else -1,
                    output_silent_ms=(time.monotonic()-metrics[2])*1000
                    if metrics is not None and metrics[2] else -1)
    perf = Reporter(node, 'bridge', ('enqueue',), bridge_state)
    control_pub = node.create_publisher(StdMsgInt32, "/chassis/control_usage_mode", 10)

    def relay_control_mode():
        # 反向只传控制模式反馈，不向底盘发布模式或恢复指令。
        for _ in range(20):
            try:
                value, received_at = control_queue.get_nowait()
            except queue_module.Empty:
                return
            if time.monotonic() - received_at > 1.0:
                node.get_logger().warning("Discard stale chassis control-mode event")
                continue
            msg = StdMsgInt32()
            msg.value = value
            control_pub.publish(msg)
            node.get_logger().info(f"Chassis control mode feedback: {value}")

    node.create_timer(0.05, relay_control_mode)

    def cb(msg):
        nonlocal received_count
        started = perf.metrics.begin('enqueue')
        # 为了避免跨进程序列化复杂 ROS 消息报错，我们提取核心字段打包为字典
        data = {
            'stamp_sec': msg.header.stamp.sec,
            'stamp_nanosec': msg.header.stamp.nanosec,
            'frame_id': msg.header.frame_id,
            'x_vel': msg.data.x_vel,
            'y_vel': msg.data.y_vel,
            'yaw_vel': msg.data.yaw_vel,
            'queued_at': started
        }
        queue.put(data)
        perf.metrics.finish('enqueue', started)
        received_count += 1
        if received_count == 1 or received_count % 100 == 0:
            node.get_logger().info(
                f"Domain {domain_id} 收到 /NAV_CMD 第 {received_count} 帧: "
                f"x={msg.data.x_vel:.3f}, y={msg.data.y_vel:.3f}, yaw={msg.data.yaw_vel:.3f}"
            )

    node.create_subscription(NavCmd, "/NAV_CMD", cb, 10)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

def publisher_domain_0(queue, control_queue, metrics=None):
    # 发送端运行在底盘域 Domain 0，将算法域导航指令转发给机器狗底盘。
    os.environ["ROS_DOMAIN_ID"] = "0"
    rclpy.init()
    node = rclpy.create_node("nav_cmd_bridge_pub")
    pub = node.create_publisher(NavCmd, "/NAV_CMD", 10)
    forwarded_count = 0
    last_control_mode = None

    def control_mode_cb(msg):
        nonlocal last_control_mode
        if msg.value != last_control_mode:
            last_control_mode = msg.value
            control_queue.put((msg.value, time.monotonic()))

    node.create_subscription(StdMsgInt32, "/CONTROL_USAGE_MODE_MASTER", control_mode_cb, 10)

    while rclpy.ok():
        try:
            # 阻塞获取队列数据，超时 0.01 秒以便让 ROS 事件循环运行
            data = queue.get(timeout=0.01)
            msg = NavCmd()
            msg.header.stamp.sec = data['stamp_sec']
            msg.header.stamp.nanosec = data['stamp_nanosec']
            msg.header.frame_id = data['frame_id']
            msg.data.x_vel = data['x_vel']
            msg.data.y_vel = data['y_vel']
            msg.data.yaw_vel = data['yaw_vel']
            pub.publish(msg)
            forwarded_count += 1
            if metrics is not None:
                metrics[0] = forwarded_count
                metrics[1] = (time.monotonic()-data['queued_at'])*1000
                metrics[2] = time.monotonic()
            if forwarded_count == 1 or forwarded_count % 100 == 0:
                node.get_logger().info(
                    f"已向 Domain 0 转发 /NAV_CMD 第 {forwarded_count} 帧: "
                    f"x={msg.data.x_vel:.3f}, y={msg.data.y_vel:.3f}, yaw={msg.data.yaw_vel:.3f}"
                )
        except queue_module.Empty:
            # 队列为空或超时，属于正常情况。
            pass
        except Exception as exc:
            node.get_logger().error(f"跨域发布 NAV_CMD 失败: {exc}")
        rclpy.spin_once(node, timeout_sec=0)
        
    node.destroy_node()
    rclpy.shutdown()

def main():
    queue = multiprocessing.Queue()
    control_queue = multiprocessing.Queue()
    metrics = multiprocessing.Array('d', 3, lock=False)
    
    p_sub = multiprocessing.Process(target=subscriber_domain_42, args=(queue, control_queue, metrics))
    p_pub = multiprocessing.Process(target=publisher_domain_0, args=(queue, control_queue, metrics))
    
    p_sub.start()
    p_pub.start()
    
    try:
        while True:
            time.sleep(1)
            # 子进程异常退出时立即暴露故障，避免桥接父进程看似仍在运行。
            if not p_sub.is_alive() or not p_pub.is_alive():
                raise RuntimeError(
                    f"跨域桥接子进程退出: sub={p_sub.exitcode}, pub={p_pub.exitcode}"
                )
    except KeyboardInterrupt:
        print("\nShutting down bridge...")
        p_sub.terminate()
        p_pub.terminate()
        p_sub.join()
        p_pub.join()

if __name__ == "__main__":
    main()
