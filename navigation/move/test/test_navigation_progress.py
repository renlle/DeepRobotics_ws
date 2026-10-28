import math

import pytest

from move.navigation_progress import WaypointProgress, remaining_path_distance


def test_remaining_distance_uses_route_projection_and_off_path_distance():
    route = [(0, 0), (10, 0), (10, 10)]
    assert remaining_path_distance(route, (5, 0)) == pytest.approx(15)
    assert remaining_path_distance(route, (5, 2)) == pytest.approx(17)
    assert remaining_path_distance(route, (10, 5)) == pytest.approx(5)
    assert remaining_path_distance([(0, 0)], (3, 4)) == pytest.approx(5)
    assert remaining_path_distance([(0, 0), (0, 0)], (0, 0)) == 0
    assert remaining_path_distance([], (0, 0)) is None
    assert remaining_path_distance([(math.nan, 0)], (0, 0)) is None


def test_stationary_replans_do_not_invent_progress():
    progress = WaypointProgress()
    for _ in range(100):
        assert progress.update([(5, 5)], 0, 10) == 0
    assert progress.update([(5, 5)], 0, 8) == pytest.approx(20)
    assert progress.update([(5, 5)], 0, 12) == pytest.approx(20)
    assert progress.update([(5, 5)], 0, None) == pytest.approx(20)


def test_three_waypoints_advance_without_reset_and_finish_at_100():
    targets = [(0, 0), (5, 5), (-5, -5)]
    progress = WaypointProgress()
    assert progress.update(targets, 0, 10) == 0
    assert progress.update(targets, 0, 5) == pytest.approx(100 / 6)
    assert progress.update(targets, 1, None) == pytest.approx(100 / 3)
    assert progress.update(targets, 1, 20) == pytest.approx(100 / 3)
    assert progress.update(targets, 1, 10) == pytest.approx(50)
    assert progress.update(targets, 2, 10) == pytest.approx(200 / 3)
    assert progress.update(targets, 2, 0) < 100
    assert progress.update(targets, 2, None, done=True) == 100
    assert progress.update(targets, 2, None, done=True) == 100


def test_pause_resume_clear_and_same_target_restart():
    progress = WaypointProgress()
    targets = [(5, 5)]
    progress.update(targets, 0, 10)
    assert progress.update(targets, 0, 5) == 50
    assert progress.update(targets, 0, 0, paused=True) == 50
    assert progress.update(targets, 0, 4) == 60
    assert progress.update(targets, 0, None, done=True) == 100
    assert progress.update([], 0, None, done=True) == 0
    assert progress.update(targets, 0, 10) == 0


def test_edited_targets_and_loop_reset_old_progress():
    progress = WaypointProgress()
    targets = [(0, 0), (5, 5)]
    progress.update(targets, 0, 10)
    assert progress.update(targets, 1, 5) == 50
    assert progress.update(targets, 0, 10) == 0
    assert progress.update(targets, 0, 5) == 25
    assert progress.update([(0, 0), (-5, -5)], 0, 7) == 0


def test_completed_task_holds_and_appended_waypoints_start_at_zero():
    progress = WaypointProgress()
    targets = [(0, 0), (5, 5)]
    progress.update(targets, 0, 10)
    for _ in range(100):
        assert progress.update(targets, 1, None, done=True) == 100
    targets.append((-5, -5))
    assert progress.update(targets, 2, 20) == 0
    assert progress.update(targets, 2, 10) == 50
    targets.append((0, 0))
    assert progress.update(targets, 2, 10) == 0
    assert progress.update(targets, 2, 5) == 25
    assert progress.update(targets, 3, 10) == 50
    assert progress.update(targets, 3, None, done=True) == 100
    assert progress.update([], 0, None) == 0


@pytest.mark.parametrize("remaining", [None, math.nan, math.inf, -1, 0])
def test_missing_or_zero_length_plan_does_not_claim_completion(remaining):
    progress = WaypointProgress()
    assert progress.update([(0, 0)], 0, remaining) == 0
    assert progress.update([(0, 0)], 0, None, done=True) == 100


def test_production_timer_publishes_motion_pause_completion_and_clear():
    from types import SimpleNamespace
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path
    from move.global_path_seq_publisher import GlobalPathSequencePublisher, Waypoint

    path = Path()
    for x in [0.0, 10.0]:
        pose = PoseStamped()
        pose.pose.position.x = x
        path.poses.append(pose)
    messages = []
    node = SimpleNamespace(
        waypoints=[Waypoint(10.0, 0.0, 0.0)], current_index=0, auto_advance=False,
        paused=False, sequence_done=False, _last_path=path,
        _navigation_progress=WaypointProgress(), _lookup_robot_xy=lambda: (0, 0),
        progress_pub=SimpleNamespace(publish=messages.append))
    tick = GlobalPathSequencePublisher._on_progress_timer
    tick(node)
    assert messages[-1].data == 0
    node._lookup_robot_xy = lambda: (5, 0)
    tick(node)
    assert messages[-1].data == 50
    node.paused = True
    node._last_path = None
    tick(node)
    assert messages[-1].data == 50
    node.paused = False
    node.sequence_done = True
    tick(node)
    assert messages[-1].data == 100
    node.waypoints = []
    tick(node)
    assert messages[-1].data == 0


def test_single_waypoint_loop_resets_progress_even_when_index_stays_zero():
    from types import SimpleNamespace
    from move.global_path_seq_publisher import GlobalPathSequencePublisher, Waypoint

    tracker = WaypointProgress()
    tracker.update([(0, 0, 0)], 0, 10)
    tracker.update([(0, 0, 0)], 0, 5)
    node = SimpleNamespace(
        waypoints=[Waypoint(0.0, 0.0, 0.0)], current_index=0, sequence_version=0,
        loop=True, goal_tolerance=1.2, _navigation_progress=tracker,
        _lookup_robot_xy=lambda: (0, 0),
        _invalidate_planning_request=lambda _: None,
        _publish_empty_paths=lambda: None, _publish_waypoints=lambda: None,
        get_logger=lambda: SimpleNamespace(info=lambda _: None, debug=lambda _: None))
    GlobalPathSequencePublisher._advance_if_reached(node)
    assert node.current_index == 0 and node.sequence_version == 1
    assert tracker.percent == 0
