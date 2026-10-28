from types import MethodType, SimpleNamespace

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from move.global_path_seq_publisher import GlobalPathSequencePublisher as Sequence, Waypoint


def path(points):
    result = Path()
    result.header.frame_id = "map"
    for x, y in points:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x, pose.pose.position.y = float(x), float(y)
        result.poses.append(pose)
    return result


def node():
    published, requests = [], []
    def send(goal):
        requests.append(goal)
        return SimpleNamespace(add_done_callback=lambda _: None)
    instance = SimpleNamespace(
        request_version=7, sequence_version=7, request_in_flight=True,
        _active_goal_version=-1, _planned_segments=[], global_frame="map", planner_id="",
        _planned_targets=[Waypoint(-6, 5, 0), Waypoint(5, 5, 0), Waypoint(0, 0, 0)],
        waypoints=[1, 2, 3], sequence_done=False, paused=False,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: PoseStamped().header.stamp)),
        get_logger=lambda: SimpleNamespace(warn=lambda _: None, error=lambda _: None, debug=lambda _: None),
        path_pub=SimpleNamespace(publish=published.append), pure_pursuit_plan_pub=None,
        compute_path_client=SimpleNamespace(send_goal_async=send),
        _to_pose_stamped=lambda waypoint: path([(waypoint.x, waypoint.y)]).poses[0],
    )
    instance._send_plan_segment = MethodType(Sequence._send_plan_segment, instance)
    return instance, published, requests


def deliver(instance, points, version=7):
    future = SimpleNamespace(result=lambda: SimpleNamespace(result=SimpleNamespace(path=path(points))))
    Sequence._get_result_callback(instance, future, version)


def test_publish_complete_abc_only_after_all_segments_and_keep_current_leg_progress():
    instance, published, requests = node()
    execution = []
    instance.pure_pursuit_plan_pub = SimpleNamespace(publish=execution.append)
    deliver(instance, [(0, 0), (-6, 5)])
    assert not published and instance.request_in_flight
    assert requests[-1].start.pose.position.x == -6
    assert requests[-1].goal.pose.position.x == 5
    deliver(instance, [(-6, 5), (5, 5)])
    assert not published and instance.request_in_flight
    assert requests[-1].goal.pose.position.x == 0
    deliver(instance, [(5, 5), (0, 0)])
    assert [(p.pose.position.x, p.pose.position.y) for p in published[0].poses] == [(0, 0), (-6, 5), (5, 5), (0, 0)]
    assert instance._last_path.poses[-1].pose.position.x == -6
    assert len(execution) == 1 and execution[0] == instance._last_path
    assert not instance.request_in_flight


def test_failed_middle_leg_does_not_publish_partial_route():
    instance, published, _ = node()
    deliver(instance, [(0, 0), (-6, 5)])
    deliver(instance, [])
    assert not published and not instance.request_in_flight


def test_after_a_remaining_route_still_contains_b_and_c():
    instance, published, _ = node()
    instance._planned_targets = instance._planned_targets[1:]
    deliver(instance, [(-5, 5), (5, 5)])
    assert not published
    deliver(instance, [(5, 5), (0, 0)])
    assert [(p.pose.position.x, p.pose.position.y) for p in published[0].poses] == [(-5, 5), (5, 5), (0, 0)]


def test_stale_result_after_clear_cannot_publish_or_request_next_leg():
    instance, published, requests = node()
    deliver(instance, [(0, 0), (-6, 5)])
    instance.sequence_version = 8
    deliver(instance, [(-6, 5), (5, 5)])
    assert not published and len(requests) == 1


def test_waypoint_transition_keeps_display_until_new_route_replaces_it():
    instance, published, _ = node()
    execution = []
    instance.pure_pursuit_plan_pub = SimpleNamespace(publish=execution.append)
    deliver(instance, [(0, 0), (-6, 5)])
    deliver(instance, [(-6, 5), (5, 5)])
    deliver(instance, [(5, 5), (0, 0)])
    instance.waypoints = instance._planned_targets
    instance.current_index = 0
    instance.goal_tolerance = 1.2
    instance._lookup_robot_xy = lambda: (-6, 5)
    instance._invalidate_planning_request = lambda _: None
    instance._publish_waypoints = lambda: None
    instance.get_logger = lambda: SimpleNamespace(info=lambda _: None, debug=lambda _: None)
    instance._publish_empty_paths = MethodType(Sequence._publish_empty_paths, instance)
    Sequence._advance_if_reached(instance)
    assert instance.current_index == 1
    assert len(published) == 1 and published[0].poses
    assert not execution[-1].poses
    instance._planned_targets = instance._planned_targets[1:]
    instance._planned_segments = []
    instance.request_version = instance.sequence_version
    deliver(instance, [(-6, 5), (5, 5)], instance.sequence_version)
    assert len(published) == 1
    deliver(instance, [(5, 5), (0, 0)], instance.sequence_version)
    assert len(published) == 2 and all(message.poses for message in published)


def test_explicit_clear_still_publishes_empty_display_and_execution():
    instance, published, _ = node()
    execution = []
    instance.pure_pursuit_plan_pub = SimpleNamespace(publish=execution.append)
    Sequence._publish_empty_paths(instance)
    assert len(published) == len(execution) == 1
    assert not published[0].poses and not execution[0].poses
