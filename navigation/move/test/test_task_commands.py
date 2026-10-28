from types import MethodType, SimpleNamespace

from geometry_msgs.msg import PointStamped
from master_control_interfaces.msg import NavigationTaskCommand
from std_msgs.msg import Empty
from move.global_path_seq_publisher import GlobalPathSequencePublisher as Sequence, Waypoint
from move.navigation_progress import WaypointProgress


def sequence():
    statuses = []
    actual_path_clears = []
    node = SimpleNamespace(
        _command_id="", _applying_command=False, waypoints=[], current_index=0,
        sequence_done=True, paused=False, sequence_version=0, auto_start_on_click=True,
        _navigation_progress=WaypointProgress(), loop=False, goal_tolerance=1.2,
        edit_radius=1.5, clicked_point_topic="/clicked_point", replan_period=1.0,
        _current_goal_distance=lambda: 0.0, _lookup_robot_xy=lambda: (0.0, 0.0),
        _point_to_waypoint=lambda msg: Waypoint(msg.point.x, msg.point.y, 0.0),
        _invalidate_planning_request=lambda _: None,
        _publish_empty_paths=lambda **kwargs: None, _publish_waypoints=lambda: None,
        status_pub=SimpleNamespace(publish=lambda _: None),
        task_status_pub=SimpleNamespace(publish=statuses.append),
        clear_actual_path_pub=SimpleNamespace(publish=actual_path_clears.append),
        actual_path_clears=actual_path_clears,
        get_logger=lambda: SimpleNamespace(warn=lambda _: None, info=lambda _: None, debug=lambda _: None))
    for name in ["_on_pause", "_on_resume", "_clear_waypoints", "_append_waypoint", "_publish_status", "_format_status"]:
        setattr(node, name, MethodType(getattr(Sequence, name), node))
    return node, statuses


def command(identifier, action, points=()):
    targets = []
    for x, y in points:
        point = PointStamped()
        point.header.frame_id = "map"
        point.point.x, point.point.y = float(x), float(y)
        targets.append(point)
    return NavigationTaskCommand(command_id=identifier, command=action, targets=targets)


def test_return_home_is_one_atomic_status_and_clear_is_effective():
    node, statuses = sequence()
    Sequence._on_task_command(node, command("old", "append", [(0, 0), (5, 5)]))
    statuses.clear()
    Sequence._on_task_command(node, command("return", "return_home", [(-5, -5)]))
    assert [(p.x, p.y) for p in node.waypoints] == [(-5, -5)]
    assert not node.paused
    assert len(statuses) == 1
    assert (statuses[0].command_id, statuses[0].state, statuses[0].total_count, statuses[0].reached_count) == ("return", "active", 1, 0)
    Sequence._on_task_command(node, command("clear", "clear"))
    assert node.waypoints == [] and node._navigation_progress.percent == 0
    assert statuses[-1].state == "empty" and statuses[-1].total_count == 0


def test_duplicate_and_nearby_points_are_reached_one_at_a_time():
    node, statuses = sequence()
    Sequence._on_task_command(node, command("three", "append", [(0, 0), (0, 0), (0.1, 0)]))
    assert len(node.waypoints) == 3
    for reached in [1, 2, 3]:
        Sequence._advance_if_reached(node)
        node._publish_status()
        assert statuses[-1].reached_count == reached
        assert (statuses[-1].state == "done") == (reached == 3)


def test_duplicate_delivery_is_idempotent_and_failed_transform_is_atomic():
    node, statuses = sequence()
    message = command("same", "append", [(0, 0)])
    Sequence._on_task_command(node, message)
    Sequence._on_task_command(node, message)
    assert len(node.waypoints) == 1
    node._point_to_waypoint = lambda _: None
    Sequence._on_task_command(node, command("bad", "return_home", [(5, 5)]))
    assert len(node.waypoints) == 1 and node._command_id == "same"


def test_direct_clear_topic_keeps_command_id_and_reports_empty():
    node, statuses = sequence()
    Sequence._on_task_command(node, command("current", "append", [(0, 0), (5, 5)]))
    Sequence._on_clear(node, Empty())
    assert len(node.actual_path_clears) == 1
    assert node.waypoints == []
    assert statuses[-1].command_id == "current"
    assert statuses[-1].state == "empty"
    assert statuses[-1].total_count == statuses[-1].reached_count == 0
