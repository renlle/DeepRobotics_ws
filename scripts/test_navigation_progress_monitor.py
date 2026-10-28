"""不创建ROS节点，复现完成后的空路径、定位更新和显式归零。"""
import math
from types import MethodType, SimpleNamespace

from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32

from navigation_progress_monitor import NavigationProgressMonitor as Monitor


def monitor():
    node = SimpleNamespace(
        global_path=[], actual_path=[], local_path=[], latest_position=(0.0, 0.0),
        previous_position=(0.0, 0.0), traveled_distance=3.95,
        remaining_distance=0.0, progress_percent=0.0, navigation_active=False,
        _path_points=Monitor._path_points,
        _remaining_path_distance=Monitor._remaining_path_distance)
    node._update_remaining_distance = MethodType(Monitor._update_remaining_distance, node)
    return node


def test_completion_survives_repeated_empty_paths_and_pose_updates():
    node = monitor()
    Monitor._on_progress(node, Float32(data=100.0))
    for _ in range(100):
        Monitor._on_global_path(node, Path())
        Monitor._on_local_path(node, Path())
        Monitor._on_position(node, Pose())
        assert node.progress_percent == 100.0
    assert node.traveled_distance == 3.95


def test_empty_path_before_completion_then_explicit_task_reset():
    node = monitor()
    Monitor._on_progress(node, Float32(data=95.0))
    Monitor._on_global_path(node, Path())
    assert node.progress_percent == 95.0
    Monitor._on_progress(node, Float32(data=100.0))
    Monitor._on_progress(node, Float32(data=0.0))
    Monitor._on_position(node, Pose())
    assert node.progress_percent == 0.0
    assert node.traveled_distance == 0.0
    Monitor._on_progress(node, Float32(data=20.0))
    assert node.progress_percent == 20.0


def test_path_replan_and_invalid_progress_cannot_reset_completion():
    node = monitor()
    Monitor._on_progress(node, Float32(data=100.0))
    route = Path()
    route.poses = [PoseStamped(), PoseStamped()]
    route.poses[-1].pose.position.x = 5.0
    Monitor._on_global_path(node, route)
    Monitor._on_position(node, Pose())
    assert node.progress_percent == 100.0
    for value in [math.nan, math.inf, -math.inf]:
        Monitor._on_progress(node, Float32(data=value))
        assert node.progress_percent == 100.0
