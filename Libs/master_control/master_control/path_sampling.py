"""仅对上报路径抽稀，保留首末点、原始坐标系及最后1米的原始点。"""
import math

from nav_msgs.msg import Path


def sample_telemetry_path(path, spacing=0.2, dense_tail=1.0):
    if len(path.poses) <= 2:
        return path
    lengths = [math.hypot(b.pose.position.x - a.pose.position.x,
                          b.pose.position.y - a.pose.position.y)
               for a, b in zip(path.poses, path.poses[1:])]
    if not all(math.isfinite(length) for length in lengths):
        return path
    tail_start = len(path.poses) - 1
    tail_length = 0.0
    while tail_start > 0 and tail_length < dense_tail:
        tail_start -= 1
        tail_length += lengths[tail_start]
    output = Path()
    output.header = path.header
    output.poses.append(path.poses[0])
    distance = 0.0
    for index in range(1, tail_start):
        distance += lengths[index - 1]
        if distance + 1e-9 >= spacing:
            output.poses.append(path.poses[index])
            distance = 0.0
    # 尾段边界单独保留，可能与前一个采样点相距不足0.2米。
    output.poses.extend(path.poses[max(1, tail_start):])
    return output
