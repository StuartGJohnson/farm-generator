"""Mission action and route handoff contracts for the short road fixture."""

import math
import threading
import time
from dataclasses import replace

import pytest

from route_planning.diagnostic import load_route_json
from route_planning.model import Pose


def test_short_route_sections_and_progress():
    from farm_mission_processor.route_progress import AnnotatedRoute, RouteProgressTracker
    route = load_route_json("debug_out/routes/short/short_route_1.json")
    annotated = AnnotatedRoute(route, "by_length", 5.0)
    assert len(annotated.sections) == 2
    assert annotated.length_m == pytest.approx(10.0, abs=0.05)
    assert annotated.sections[0].end_m == pytest.approx(5.0)
    second_path = annotated.slice(4.0, 10.0)
    assert max(math.hypot(b.x - a.x, b.y - a.y)
               for a, b in zip(second_path, second_path[1:])) <= 0.2 + 1e-6
    assert len(second_path) >= 30
    assert sum(math.hypot(p.x - second_path[0].x, p.y - second_path[0].y) <= 1.0
               for p in second_path) >= 6
    assert math.hypot(second_path[0].x - annotated.pose_at(4).x,
                      second_path[0].y - annotated.pose_at(4).y) < 1e-6
    first_path = annotated.slice(0.0, 5.0)
    route_start = annotated.pose_at(0.0)
    def interior_overlap(path):
        return [p for p in path if 4.000001 < math.hypot(p.x - route_start.x,
                                                         p.y - route_start.y) < 4.999999]
    assert len(interior_overlap(first_path)) >= 4
    assert interior_overlap(first_path) == interior_overlap(second_path)
    tracker = RouteProgressTracker(annotated, forward_search_m=3.0)
    at_two = annotated.pose_at(2.0)
    assert tracker.update(at_two.x, at_two.y) == pytest.approx(2.0, abs=0.02)
    at_nine = annotated.pose_at(9.0)
    assert tracker.update(at_nine.x, at_nine.y) <= 5.0


def test_reeds_shepp_source_edge_is_evenly_subdivided():
    from farm_mission_processor.route_progress import AnnotatedRoute
    route = load_route_json("debug_out/routes/short/short_route_3.json")
    annotated = AnnotatedRoute(route, "by_length", 5.0)
    index = next(i for i, (a, b) in enumerate(zip(annotated.stations, annotated.stations[1:]))
                 if 0.32 < b - a < 0.34)
    a, b = annotated.stations[index:index + 2]
    poses = annotated.slice(a, b)
    assert len(poses) == 3
    distances = [math.hypot(q.x - p.x, q.y - p.y) for p, q in zip(poses, poses[1:])]
    assert distances[0] == pytest.approx(distances[1], abs=1e-9)
    assert distances[0] <= 0.2


@pytest.mark.parametrize("route_number", (2, 3))
def test_overlapping_handoffs_share_edge_anchored_poses(route_number):
    from farm_mission_processor.route_progress import AnnotatedRoute
    route = load_route_json(f"debug_out/routes/short/short_route_{route_number}.json")
    annotated = AnnotatedRoute(route, "by_length", 5.0)
    for previous, current in zip(annotated.sections, annotated.sections[1:]):
        if previous.direction != current.direction:
            continue
        start = max(previous.start_m, current.start_m - 1.0)
        if previous.end_m - start < 0.4:
            continue
        previous_path = annotated.slice(previous.start_m, previous.end_m)
        current_path = annotated.slice(start, current.end_m)
        overlap_interior = annotated.slice(start, previous.end_m)[1:-1]
        assert overlap_interior
        assert all(pose in previous_path and pose in current_path for pose in overlap_interior)


def test_local_sampling_keeps_source_metadata_and_region_boundaries():
    from farm_mission_processor.route_progress import AnnotatedRoute
    from route_planning.model import RouteSegment
    route = load_route_json("debug_out/routes/short/short_route_1.json")
    row = RouteSegment("row", 0, "row", "field_1",
                       (Pose(0, 0, 0), Pose(1, 0, 0)), 1, frozenset(), (1, 1))
    headland = RouteSegment("turn", 0, "headland", "field_1",
                            (Pose(1, 0, 0), Pose(2, 0, 0)), 1, frozenset(), (1, 1))
    route = replace(route, segments=[row, headland])
    by_region = AnnotatedRoute(route, "by_region", 5.0)
    by_activity = AnnotatedRoute(route, "by_activity", 5.0)
    assert [(s.start_m, s.end_m, s.region) for s in by_region.sections] == [(0, 2, "field")]
    assert [(s.start_m, s.end_m, s.region) for s in by_activity.sections] == [
        (0, 1, "row"), (1, 2, "headland")]
    assert by_activity.segment_at(0.5) is row
    assert by_activity.segment_at(1.5) is headland
    assert by_activity.slice(0, 1)[-1] == by_activity.slice(1, 2)[0]


def test_long_source_edge_only_samples_requested_window():
    from farm_mission_processor.route_progress import AnnotatedRoute
    from route_planning.model import RouteSegment
    route = load_route_json("debug_out/routes/short/short_route_1.json")
    edge = RouteSegment("long", 0, "road", None,
                        (Pose(0, 0, 0), Pose(1_000_000, 0, 0)), 1_000_000,
                        frozenset(), (1, 1))
    annotated = AnnotatedRoute(replace(route, segments=[edge]), "by_region", 5.0)
    window = annotated.slice(500_000, 500_001)
    assert window[0] == Pose(500_000, 0, 0)
    assert window[-1] == Pose(500_001, 0, 0)
    assert len(window) <= 8


def test_reversal_is_a_section_boundary():
    from farm_mission_processor.route_progress import AnnotatedRoute
    from route_planning.model import RouteSegment
    route = load_route_json("debug_out/routes/short/short_route_1.json")
    forward = RouteSegment("forward", 0, "road", None,
                           (Pose(0, 0, 0), Pose(5, 0, 0)), 5, frozenset(), (1, 1))
    reverse = RouteSegment("reverse", 0, "road", None,
                           (Pose(5, 0, 0), Pose(0, 0, 0)), 5, frozenset(), (-1, -1))
    route = replace(route, segments=[forward, reverse])
    annotated = AnnotatedRoute(route, "by_region", 5)
    assert [section.direction for section in annotated.sections] == [1, -1]
    assert annotated.sections[0].end_m == annotated.sections[1].start_m


def test_plan_mission_action_short_route(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    import rclpy
    from action_msgs.msg import GoalStatus
    from rclpy.action import ActionClient
    from rclpy.executors import MultiThreadedExecutor
    from farm_mission_interfaces.action import ExecuteMission, PlanMission
    from farm_mission_processor.node import FarmMissionProcessor
    from route_planning import load_farm_map

    rclpy.init()
    node = FarmMissionProcessor()
    client_node = rclpy.create_node("farm_mission_test_client")
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.add_node(client_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    def complete(future, timeout=30):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not future.done():
            time.sleep(0.05)
        assert future.done(), "ROS action timed out"
        return future.result()

    try:
        planner = ActionClient(client_node, PlanMission, "mission_planner")
        executor_client = ActionClient(client_node, ExecuteMission, "mission_executor")
        assert planner.wait_for_server(timeout_sec=5)
        assert executor_client.wait_for_server(timeout_sec=5)
        rejected = complete(executor_client.send_goal_async(ExecuteMission.Goal(mission_id="missing")))
        assert not rejected.accepted
        farm = load_farm_map("debug_out/farms/farm_seed_1_120m2_depot/gis")
        road = farm.roads["redge_0001"].line
        start = road.interpolate(30.0)
        end = road.interpolate(40.0)
        a = road.interpolate(29.95)
        b = road.interpolate(30.05)
        yaw = math.atan2(b.y - a.y, b.x - a.x)
        goal = PlanMission.Goal()
        goal.implement_side = "both"
        goal.start_x, goal.start_y, goal.start_yaw = start.x, start.y, yaw
        goal.end_x, goal.end_y, goal.end_yaw = end.x, end.y, yaw
        handle = complete(planner.send_goal_async(goal))
        assert handle.accepted
        outcome = complete(handle.get_result_async())
        assert outcome.status == GoalStatus.STATUS_SUCCEEDED
        assert outcome.result.success
        assert outcome.result.total_distance_m == pytest.approx(10, abs=0.05)
        assert len(outcome.result.path.poses) >= 2
        assert node._route is not None
    finally:
        executor.shutdown()
        node.destroy_node()
        client_node.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=5)
