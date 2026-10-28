"""根据实际规划路线的剩余长度计算航点序列进度，不依赖运行时间。"""

import math


def remaining_path_distance(points, position):
    """当前位置投影到折线后，返回到终点的剩余长度；无有效路线返回None。"""
    if not points or not all(math.isfinite(v) for p in [position, *points] for v in p):
        return None
    if len(points) == 1:
        return math.dist(position, points[0])
    lengths = [math.dist(a, b) for a, b in zip(points, points[1:])]
    remaining = sum(lengths)
    best = float("inf")
    result = None
    for a, b, length in zip(points, points[1:], lengths):
        dx, dy = b[0] - a[0], b[1] - a[1]
        ratio = 0.0 if length <= 1e-9 else max(0.0, min(1.0,
            ((position[0] - a[0]) * dx + (position[1] - a[1]) * dy) / (length * length)))
        projected = (a[0] + ratio * dx, a[1] + ratio * dy)
        distance = math.dist(position, projected)
        if distance < best:
            best = distance
            result = distance + remaining - ratio * length
        remaining -= length
    return max(0.0, result)


class WaypointProgress:
    """各航点等权，段内按路线剩余长度推进；完成信号到达前不报100%。"""

    def __init__(self):
        self.reset()

    def reset(self):
        self.targets = ()
        self.index = 0
        self.start_index = 0
        self.initial_remaining = None
        self.fraction = 0.0
        self.percent = 0.0

    def update(self, targets, index, remaining, *, paused=False, done=False):
        targets = tuple(targets)
        # 新任务、编辑或循环重新开始不能继承旧路线的100%。
        if targets != self.targets or index < self.index:
            self.reset()
            self.targets = targets
            # 航点列表采用追加语义，旧的已到达点不计入新一轮进度。
            self.start_index = index
        if not targets:
            self.reset()
            return 0.0
        if done:
            self.percent = 100.0
            return self.percent
        if index != self.index:
            self.index = index
            self.initial_remaining = None
            self.fraction = 0.0
        if paused:
            return self.percent
        if remaining is not None and math.isfinite(remaining) and remaining >= 0:
            if self.initial_remaining is None:
                self.initial_remaining = remaining
            if self.initial_remaining > 1e-6:
                # 重规划可暂时增加路线长度，显示进度保留高水位，不随每次重规划归零。
                self.fraction = max(self.fraction,
                                    min(0.99, max(0.0, 1.0 - remaining / self.initial_remaining)))
        count = max(1, len(targets) - self.start_index)
        self.percent = max(self.percent, min(99.0,
            100.0 * (index - self.start_index + self.fraction) / count))
        return self.percent
