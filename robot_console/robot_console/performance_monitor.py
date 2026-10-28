"""只接收低带宽诊断摘要；未启动和停止上报均明确展示。"""
import json
import math
import time

MODULES = ('frontend', 'localization', 'mapping', 'visual_loop', 'waypoints', 'planner', 'adapter', 'bridge')


class PerformanceMonitor:
    def __init__(self):
        self.latest = {}

    def receive(self, data):
        if len(data) > 16384:
            return
        try:
            record = json.loads(data)
            module = record['module']
            if module not in MODULES or not isinstance(record.get('streams'), dict):
                return
            self.latest[module] = (time.monotonic(), record)
        except (ValueError, KeyError, TypeError):
            pass

    def lines(self):
        now = time.monotonic()
        lines = []
        def number(row, key):
            value = row.get(key, -1)
            if not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
                return '-'
            return f'{value:.1f}'
        for module in MODULES:
            entry = self.latest.get(module)
            if entry is None:
                lines.append(f'{module}: 未收到统计（未启动或未连接）')
                continue
            received, record = entry
            age = now-received
            if age > 12:
                lines.append(f'{module}: 统计已停更 {age:.1f}s（节点停止或阻塞）')
                continue
            parts = []
            for name, row in record['streams'].items():
                if not isinstance(row, dict):
                    continue
                parts.append(f'{name}[in/out={number(row,"in_hz")}/{number(row,"out_hz")}Hz '
                             f'q={number(row,"pending")} drop={number(row,"drop")} '
                             f'work/peak={number(row,"work_ms")}/{number(row,"max_ms")}ms '
                             f'in/out_silent={number(row,"input_silent_ms")}/{number(row,"output_silent_ms")}ms '
                             f'source_lag={number(row,"source_lag_ms")}ms]')
            state = record.get('state')
            lines.append(f'{module} (sample_age={age:.1f}s): ' + ' '.join(parts) +
                         (f' state={json.dumps(state, ensure_ascii=False)}' if state else ''))
        return lines
