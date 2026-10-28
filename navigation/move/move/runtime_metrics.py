"""轻量运行统计：固定计数器、单调时钟，每 5 秒序列化一次。"""
import functools
import json
import threading
import time


class Metrics:
    def __init__(self, names):
        self.lock = threading.Lock()
        self.rows = {name: dict(input=0, output=0, pending=0, total=0.0, maximum=0.0,
                               last_in=None, last_out=None) for name in names}
        self.previous = {name: (0, 0, 0.0) for name in names}
        self.last_report = time.monotonic()

    def begin(self, name):
        now = time.monotonic()
        with self.lock:
            row = self.rows[name]
            row['input'] += 1
            row['pending'] += 1
            row['last_in'] = now
        return now

    def finish(self, name, started):
        now = time.monotonic()
        with self.lock:
            row = self.rows[name]
            row['output'] += 1
            row['pending'] -= 1
            row['last_out'] = now
            row['total'] += now-started
            row['maximum'] = max(row['maximum'], now-started)

    def snapshot(self):
        now = time.monotonic()
        dt = max(1e-9, now-self.last_report)
        result = {}
        with self.lock:
            for name, row in self.rows.items():
                old_in, old_out, old_total = self.previous[name]
                completed = row['output']-old_out
                result[name] = dict(
                    in_hz=(row['input']-old_in)/dt, out_hz=completed/dt,
                    pending=row['pending'], drop=0,
                    work_ms=(row['total']-old_total)*1000/completed if completed else 0.0,
                    max_ms=row['maximum']*1000,
                    input_silent_ms=(now-row['last_in'])*1000 if row['last_in'] is not None else -1,
                    output_silent_ms=(now-row['last_out'])*1000 if row['last_out'] is not None else -1,
                    source_lag_ms=-1)
                self.previous[name] = row['input'], row['output'], row['total']
                row['maximum'] = 0.0
        self.last_report = now
        return dt, result


def measured(name):
    """保留原函数异常/返回语义；测试桩未初始化 Reporter 时直接执行原函数。"""
    def decorate(func):
        @functools.wraps(func)
        def call(self, *args, **kwargs):
            reporter = getattr(self, '_perf', None)
            if reporter is None:
                return func(self, *args, **kwargs)
            started = reporter.metrics.begin(name)
            try:
                return func(self, *args, **kwargs)
            finally:
                reporter.metrics.finish(name, started)
        return call
    return decorate


def stamp_age_ms(node, msg):
    if msg is None or not hasattr(msg, 'header'):
        return -1
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
    age = node.get_clock().now().nanoseconds*1e-9 - stamp
    return age*1000 if stamp > 0 and 0 <= age < 3600 else -1


class Reporter:
    def __init__(self, node, module, names, state=None):
        # 延迟导入，纯计数器可在无 ROS 环境离线测试。
        from std_msgs.msg import String
        from rclpy.clock import Clock, ClockType
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        self.metrics = Metrics(names)
        self.publisher = node.create_publisher(String, '/runtime/performance',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

        def report():
            dt, streams = self.metrics.snapshot()
            payload = dict(module=module, window_s=dt, streams=streams)
            if state:
                try:
                    payload['state'] = state()
                except Exception as exc:
                    payload['state'] = {'unavailable': type(exc).__name__}
            msg = String()
            msg.data = json.dumps(payload, separators=(',', ':'), allow_nan=False)
            self.publisher.publish(msg)
        self.timer = node.create_timer(5.0, report, clock=Clock(clock_type=ClockType.STEADY_TIME))
