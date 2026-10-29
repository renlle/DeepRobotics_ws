"""仅在 195/196 隔离域测试消息保真、单向性和进程替换，不连接底盘域。"""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest
import rclpy
from drdds.msg import BatteryData, BatteryDataValue, StdMsgInt32
from nav_msgs.msg import Odometry
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.serialization import serialize_message, deserialize_message
from rclpy.signals import SignalHandlerOptions


def test_real_chassis_wire_sample():
    # 2026-10-28 从 Domain 0 只读获取的双电池 CDR 帧；防止两端同错的自发自收测试漏报。
    raw = bytes.fromhex(
        '000100000000000000000000000000000000000002000000281d40ffd900c20163007e31'
        '000000000000373302140500030000003333014233330d42cdccfc410000000000000000'
        '0000000000000000000000000000000000000000000000002c1d8cff1001c20161007e31'
        '00000000000037380214050003000000cdcc004233330d42000000420000000000000000'
        '000000000000000000000000000000000000000000000000'
    )
    message = deserialize_message(raw, BatteryData)
    assert len(message.data) == 2
    first, second = message.data
    assert (first.voltage, first.current, first.battery_level) == (7464, -192, 51)
    assert (second.voltage, second.current, second.battery_level) == (7468, -116, 56)
    assert first.software_version == 55 and first.battery_quantity == 20
    assert list(first.battery_temperature) == pytest.approx([32.3, 35.3, 31.6])
    assert list(second.battery_temperature) == pytest.approx([32.2, 35.3, 32.])
    assert serialize_message(message) == raw


def test_forward_replace_and_shutdown(tmp_path):
    script = Path(__file__).with_name('battery_domain_bridge.py').resolve()
    processes, contexts, nodes, executors, logs = [], [], [], [], []
    env = dict(os.environ, ROS_LOCALHOST_ONLY='1')
    old_localhost = os.environ.get('ROS_LOCALHOST_ONLY')
    os.environ['ROS_LOCALHOST_ONLY'] = '1'

    def start():
        log = (tmp_path / f'bridge-{len(processes)}.log').open('w+')
        logs.append(log)
        process = subprocess.Popen([sys.executable, str(script), '--source-domain', '195',
                                    '--destination-domain', '196'], env=env, stdout=log,
                                   stderr=subprocess.STDOUT)
        processes.append(process)
        return process

    def spin_until(condition, timeout=10, send=lambda: None):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            send()
            for executor in executors:
                executor.spin_once(timeout_sec=.02)
            if condition():
                return
        raise AssertionError('隔离域桥接条件超时')

    try:
        for domain in (195, 196):
            context = Context()
            contexts.append(context)
            rclpy.init(context=context, domain_id=domain,
                       signal_handler_options=SignalHandlerOptions.NO)
            node = rclpy.create_node(f'battery_test_{domain}', context=context)
            nodes.append(node)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            executors.append(executor)
        source, destination = nodes
        charge_received, battery_received, source_received, odom_received = [], [], [], []
        charge_pub = source.create_publisher(StdMsgInt32, '/BATTERY_CHARGE_ENABLE', 10)
        battery_pub = source.create_publisher(BatteryData, '/BATTERY_DATA', 10)
        odom_pub = source.create_publisher(Odometry, '/ODOM', 10)
        destination.create_subscription(Odometry, '/ODOM', odom_received.append, 10)
        destination.create_subscription(StdMsgInt32, '/BATTERY_CHARGE_ENABLE', charge_received.append, 10)
        destination.create_subscription(BatteryData, '/BATTERY_DATA', battery_received.append, 10)
        source.create_subscription(StdMsgInt32, '/BATTERY_CHARGE_ENABLE', source_received.append, 10)
        charge = StdMsgInt32(value=1)
        battery = BatteryData()
        cell = BatteryDataValue(voltage=5300, current=-123, battery_level=73,
                                battery_temperature=[25.5, 26.25])
        battery.data = [cell, BatteryDataValue(voltage=5200, battery_level=68)]
        odom = Odometry()
        odom.header.frame_id = 'odom'
        odom.header.stamp.sec = 123
        odom.header.stamp.nanosec = 456
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = 1.25
        odom.pose.pose.orientation.w = 1.
        odom.twist.twist.linear.x = .75
        odom.twist.twist.angular.z = -.25
        odom.pose.covariance = [float(i) / 10 for i in range(36)]
        odom.twist.covariance = [float(i) / 100 for i in range(36)]

        def send():
            charge_pub.publish(charge)
            battery_pub.publish(battery)
            odom_pub.publish(odom)

        first = start()
        spin_until(lambda: charge_received and battery_received and odom_received, send=send)
        assert serialize_message(charge_received[-1]) == serialize_message(charge)
        assert serialize_message(battery_received[-1]) == serialize_message(battery)
        assert serialize_message(odom_received[-1]) == serialize_message(odom)
        second = start()
        first.wait(timeout=10)
        assert first.returncode == 0
        charge_received.clear()
        battery_received.clear()
        # 用新值排除旧桥已入队的消息，并等待 DDS 图发现新发布者。
        charge.value = 2
        battery.data[0].battery_level = 72
        odom.header.stamp.sec = 124
        spin_until(lambda: charge_received and battery_received
                   and charge_received[-1].value == 2
                   and battery_received[-1].data[0].battery_level == 72
                   and odom_received and odom_received[-1].header.stamp.sec == 124
                   and len(destination.get_publishers_info_by_topic('/BATTERY_DATA')) == 1,
                   send=send)
        assert second.poll() is None
        assert serialize_message(odom_received[-1]) == serialize_message(odom)
        assert len(destination.get_publishers_info_by_topic('/BATTERY_DATA')) == 1
        reverse = destination.create_publisher(StdMsgInt32, '/BATTERY_CHARGE_ENABLE', 10)
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline:
            reverse.publish(StdMsgInt32(value=999))
            for executor in executors:
                executor.spin_once(timeout_sec=.02)
        assert all(message.value != 999 for message in source_received)
        second.send_signal(signal.SIGINT)
        assert second.wait(timeout=5) == 0
    finally:
        for process in processes:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
        for executor in executors:
            executor.shutdown()
        for node in nodes:
            node.destroy_node()
        for context in contexts:
            if context.ok():
                context.shutdown()
        for log in logs:
            log.seek(0)
            print(log.read())
            log.close()
        if old_localhost is None:
            os.environ.pop('ROS_LOCALHOST_ONLY', None)
        else:
            os.environ['ROS_LOCALHOST_ONLY'] = old_localhost
