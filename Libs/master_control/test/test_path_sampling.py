import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from master_control.path_sampling import sample_telemetry_path


def route(count, step):
    path = Path()
    path.header.frame_id = "base_footprint"
    path.header.stamp.sec = 42
    for index in range(count):
        pose = PoseStamped()
        pose.pose.position.x = index * step
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    return path


def test_sampling_keeps_endpoints_frame_tail_and_input_unchanged():
    path = route(201, 0.05)
    output = sample_telemetry_path(path)
    assert len(path.poses) == 201
    assert 65 <= len(output.poses) <= 68
    assert output.header == path.header
    assert output.poses[0] == path.poses[0]
    assert output.poses[-21:] == path.poses[-21:]
    prefix = [p.pose.position.x for p in output.poses if p.pose.position.x < 8.9]
    assert all(math.isclose(b - a, 0.2) for a, b in zip(prefix, prefix[1:]))


def test_short_empty_and_already_sparse_paths_preserve_original_points():
    for count, step in [(0, 0.05), (1, 0.05), (10, 0.05), (10, 1.0)]:
        path = route(count, step)
        assert sample_telemetry_path(path) == path


def test_folded_path_is_sampled_by_arc_length_not_chord_length():
    path = route(61, 0.1)
    for index, pose in enumerate(path.poses):
        pose.pose.position.x = (index % 2) * 0.1
        pose.header.stamp.nanosec = index
    output = sample_telemetry_path(path)
    assert len(output.poses) < len(path.poses)
    assert output.poses[1] == path.poses[2]
    assert output.poses[0].pose.position.x == output.poses[1].pose.position.x
