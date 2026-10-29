#!/usr/bin/env python3
"""电池及 /ODOM 状态单向跨域；保留旧入口以兼容运行实例的锁身份。"""
import argparse
import fcntl
import hashlib
import os
from pathlib import Path
import signal
import tempfile
import time

import rclpy
from drdds.msg import BatteryData, StdMsgInt32
from nav_msgs.msg import Odometry
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions


def acquire_instance(source, destination):
    # 两把锁分别串行化启动和标识存活实例；pidfd 防止 PID 复用导致误杀。
    script = str(Path(__file__).resolve())
    key = hashlib.sha256(f'{script}:{source}:{destination}'.encode()).hexdigest()[:20]
    root = Path(tempfile.gettempdir()) / f'battery-bridge-{os.getuid()}'
    root.mkdir(mode=0o700, exist_ok=True)
    if root.is_symlink() or root.stat().st_uid != os.getuid():
        raise RuntimeError('电池桥接锁目录身份不匹配')
    with (root / f'{key}.startup').open('a+') as startup:
        fcntl.flock(startup, fcntl.LOCK_EX)
        owner = (root / f'{key}.pid').open('a+')
        try:
            fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            owner.seek(0)
            pid = int(owner.read().strip())
            try:
                pidfd = os.pidfd_open(pid)
            except ProcessLookupError:
                pidfd = None
            if pidfd is not None:
                try:
                    process = Path('/proc') / str(pid)
                    argv = process.joinpath('cmdline').read_bytes().split(b'\0')
                    if process.stat().st_uid != os.getuid() or script.encode() not in argv:
                        raise RuntimeError('旧电池桥接 PID 身份不匹配，拒绝终止')
                    print(f'停止旧电池桥接 PID={pid}', flush=True)
                    signal.pidfd_send_signal(pidfd, signal.SIGTERM)
                    deadline = time.monotonic() + 5
                    while True:
                        try:
                            fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
                            break
                        except BlockingIOError:
                            if time.monotonic() >= deadline:
                                signal.pidfd_send_signal(pidfd, signal.SIGKILL)
                                break
                            time.sleep(.05)
                except (ProcessLookupError, FileNotFoundError):
                    pass
                finally:
                    os.close(pidfd)
            # 已退出或已终止后仍有限等待，不允许重复实例并存。
            deadline = time.monotonic() + 2
            while True:
                try:
                    fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    break
                except BlockingIOError:
                    if time.monotonic() >= deadline:
                        owner.close()
                        raise RuntimeError('旧电池桥接未释放锁')
                    time.sleep(.05)
        owner.seek(0)
        owner.truncate()
        owner.write(str(os.getpid()))
        owner.flush()
        return owner


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    # 域参数用于隔离回归；start_app 默认固定 0 -> 42。
    parser.add_argument('--source-domain', type=int, default=0)
    parser.add_argument('--destination-domain', type=int, default=42)
    args = parser.parse_args()
    if args.source_domain == args.destination_domain:
        parser.error('源域与目标域必须不同，避免自循环')
    stopping = False

    def stop(_signum, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    with acquire_instance(args.source_domain, args.destination_domain):
        contexts, nodes = [], []
        executor = None
        counts = {'/BATTERY_CHARGE_ENABLE': 0, '/BATTERY_DATA': 0, '/ODOM': 0}
        try:
            for domain, name in ((args.source_domain, 'domain0to42_bridge_source'),
                                 (args.destination_domain, 'domain0to42_bridge_destination')):
                context = Context()
                contexts.append(context)
                rclpy.init(context=context, domain_id=domain,
                           signal_handler_options=SignalHandlerOptions.NO)
                nodes.append(rclpy.create_node(name, context=context))
            source, destination = nodes
            # BEST_EFFORT 接收同时兼容可靠/尽力发布者；目标侧可靠发布，保持所有原始字段。
            receive_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                                     durability=DurabilityPolicy.VOLATILE)
            for topic, message_type in (('/BATTERY_CHARGE_ENABLE', StdMsgInt32),
                                        ('/BATTERY_DATA', BatteryData),
                                        ('/ODOM', Odometry)):
                publisher = destination.create_publisher(message_type, topic, 10)

                def forward(message, publisher=publisher, topic=topic):
                    publisher.publish(message)
                    counts[topic] += 1

                source.create_subscription(message_type, topic, forward, receive_qos)
            executor = SingleThreadedExecutor(context=contexts[0])
            executor.add_node(source)
            # 就绪只表示已建端点；周期计数明确显示是否真正收到源数据。
            source.create_timer(5., lambda: print(f'跨域累计转发: {counts}', flush=True))
            print(f'跨域转发就绪 PID={os.getpid()}: Domain {args.source_domain} -> '
                  f'{args.destination_domain}', flush=True)
            while not stopping:
                executor.spin_once(timeout_sec=.2)
        finally:
            if executor is not None:
                executor.shutdown()
            for node in reversed(nodes):
                node.destroy_node()
            for context in reversed(contexts):
                if context.ok():
                    context.shutdown()
            print(f'跨域转发退出，累计转发: {counts}', flush=True)


if __name__ == '__main__':
    main()
