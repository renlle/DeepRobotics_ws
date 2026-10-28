import math

from geometry_msgs.msg import PointStamped

from master_control.navigation_command_node import NavigationCommandNode


def _target(frame_id="map", x=1.0, y=2.0, z=0.0):
    target = PointStamped()
    target.header.frame_id = frame_id
    target.point.x = x
    target.point.y = y
    target.point.z = z
    return target


def test_heartbeat_alive_when_both_sources_are_fresh():
    assert NavigationCommandNode._is_heartbeat_alive(10.0, 8.0, 9.0, 2.5)


def test_heartbeat_dead_when_one_source_is_missing():
    assert not NavigationCommandNode._is_heartbeat_alive(
        10.0, None, 9.0, 2.5
    )
    assert not NavigationCommandNode._is_heartbeat_alive(
        10.0, 9.0, None, 2.5
    )


def test_heartbeat_dead_when_either_source_times_out():
    assert not NavigationCommandNode._is_heartbeat_alive(
        10.0, 7.0, 9.0, 2.5
    )
    assert not NavigationCommandNode._is_heartbeat_alive(
        10.0, 9.0, 7.0, 2.5
    )


def test_validate_target_accepts_finite_map_point():
    assert NavigationCommandNode._validate_target(_target()) is None


def test_validate_target_rejects_empty_frame():
    error = NavigationCommandNode._validate_target(_target(frame_id=""))
    assert "frame_id" in error


def test_validate_target_rejects_non_finite_coordinate():
    error = NavigationCommandNode._validate_target(_target(x=math.nan))
    assert "有限数值" in error


def test_parse_active_status_marks_previous_waypoints_reached():
    action, reached_count = NavigationCommandNode._parse_waypoint_status(
        "active: waypoint=3/5, dist=1.20m"
    )
    assert action == "progress"
    assert reached_count == 2


def test_parse_paused_status_marks_previous_waypoints_reached():
    action, reached_count = NavigationCommandNode._parse_waypoint_status(
        "paused: waypoint=1/3, dist=unknown"
    )
    assert action == "progress"
    assert reached_count == 0


def test_parse_done_status_marks_all_waypoints_reached():
    assert NavigationCommandNode._parse_waypoint_status(
        "done: 3 waypoint(s)"
    ) == ("done", 3)


def test_parse_empty_status_clears_waypoints():
    assert NavigationCommandNode._parse_waypoint_status(
        "empty: click RViz Publish Point"
    ) == ("empty", None)


def test_parse_invalid_status_is_ignored():
    assert NavigationCommandNode._parse_waypoint_status(
        "active: waypoint=4/3"
    ) == ("ignore", None)
