"""Directed Euler reconstruction and independent route validation."""

from __future__ import annotations

import math
from collections import Counter

import networkx as nx
from shapely.geometry import Point
from shapely.ops import unary_union
from shapely.strtree import STRtree

from .model import FarmMap, FarmTask, PlannerConfig, Route, RouteSegment, RoutingGraph
from .optimize import TraversalSolution, arc_cost
from .path_feasibility import TractorFootprint, path_feasibility, tree_convex_hulls


def reconstruct_route(graph: RoutingGraph, task: FarmTask,
                      solution: TraversalSolution,
                      config: PlannerConfig = PlannerConfig()) -> Route:
    selected = nx.MultiDiGraph()
    selected.add_node(graph.start_id)
    for arc_id, count in solution.counts.items():
        arc = graph.arcs[arc_id]
        for occurrence in range(count):
            selected.add_edge(arc.u, arc.v, key=(arc_id, occurrence))
    try:
        walk = list(nx.eulerian_path(selected, source=graph.start_id, keys=True))
    except nx.NetworkXError as exc:
        raise AssertionError("CP-SAT counts do not form one Euler route") from exc
    segments = []
    for u, v, (arc_id, occurrence) in walk:
        arc = graph.arcs[arc_id]
        assert (u, v) == (arc.u, arc.v)
        segments.append(RouteSegment(arc_id, occurrence, arc.kind, arc.field_id,
                                     arc.poses, arc.distance_m, arc.service_ids,
                                     arc.driving_directions or tuple(1 for _ in arc.poses),
                                     arc.reverse_distance_m, arc.gear_changes))
    total = sum(s.distance_m for s in segments)
    return Route(task, segments, total, solution.objective_cost,
                 solution.status, solution.counts, solution.saturated_arc_ids,
                 config.reverse_cost_multiplier, config.gear_change_penalty_m,
                 solution.best_bound_cost, solution.solver_wall_time_s)


def validate_route(route: Route, graph: RoutingGraph, farm: FarmMap,
                   config: PlannerConfig = PlannerConfig()) -> None:
    segments = route.segments
    assert segments, "Route has no movement"
    assert graph.arcs[segments[0].arc_id].u == graph.start_id
    assert graph.arcs[segments[-1].arc_id].v == graph.end_id
    assert math.hypot(segments[0].poses[0].x - route.task.start.x,
                      segments[0].poses[0].y - route.task.start.y) <= config.road_pose_tolerance_m + 1e-5
    assert math.hypot(segments[-1].poses[-1].x - route.task.end.x,
                      segments[-1].poses[-1].y - route.task.end.y) <= config.road_pose_tolerance_m + 1e-5
    assert abs((segments[0].poses[0].yaw - route.task.start.yaw + math.pi) % (2 * math.pi) - math.pi) < 1e-5
    assert abs((segments[-1].poses[-1].yaw - route.task.end.yaw + math.pi) % (2 * math.pi) - math.pi) < 1e-5
    for before, after in zip(segments, segments[1:]):
        shared_id = graph.arcs[before.arc_id].v
        assert shared_id == graph.arcs[after.arc_id].u
        assert math.hypot(before.poses[-1].x - after.poses[0].x,
                          before.poses[-1].y - after.poses[0].y) < 1e-3
        if graph.vertices[shared_id].oriented:
            yaw_error = (before.poses[-1].yaw - after.poses[0].yaw + math.pi) % (2 * math.pi) - math.pi
            assert abs(yaw_error) < 1e-3, f"Heading discontinuity at {shared_id}"
    coverage = set().union(*(segment.service_ids for segment in segments))
    assert coverage == graph.requirements
    selected_fields = set(route.task.field_ids)
    footprint = TractorFootprint.from_config(config.tractor)
    channel_union = unary_union(list(farm.irrigation_channels.values()))
    permitted_fields = {fid: farm.fields[fid].difference(channel_union) for fid in selected_fields}
    tree_hulls = tree_convex_hulls(farm, tuple(selected_fields))
    obstacles = {fid: [Point(t.x, t.y).buffer(config.trunk_clearance_m)
                       for t in farm.trees.values() if t.field_id == fid]
                 for fid in selected_fields}
    obstacle_indices = {fid: STRtree(items) for fid, items in obstacles.items()}
    for segment in segments:
        arc = graph.arcs[segment.arc_id]
        assert segment.poses == arc.poses
        for vertex_id, pose in ((arc.u, segment.poses[0]), (arc.v, segment.poses[-1])):
            vertex = graph.vertices[vertex_id]
            if vertex.oriented:
                yaw_error = (vertex.pose.yaw - pose.yaw + math.pi) % (2 * math.pi) - math.pi
                assert abs(yaw_error) < 1e-3, f"Arc {arc.id} mismatches {vertex_id} heading"
        assert segment.service_ids == arc.service_ids
        assert segment.reverse_distance_m == arc.reverse_distance_m
        assert segment.gear_changes == arc.gear_changes
        assert 0 <= segment.reverse_distance_m <= segment.distance_m + 1e-5
        assert len(segment.driving_directions) == len(segment.poses)
        assert all(direction in (-1, 1) for direction in segment.driving_directions)
        if segment.kind in ("rs", "row"):
            assert segment.field_id in selected_fields
        if segment.kind == "rs":
            fid = segment.field_id
            assert path_feasibility(segment.poses, permitted_fields[fid],
                                    obstacles[fid], footprint, obstacle_indices[fid],
                                    tree_hulls[fid])
    assert Counter(s.arc_id for s in segments) == route.traversal_counts
    assert math.isclose(route.total_distance_m,
                        sum(graph.arcs[arc_id].distance_m * count
                            for arc_id, count in route.traversal_counts.items()), abs_tol=1e-5)
    assert route.reverse_cost_multiplier == config.reverse_cost_multiplier
    assert route.gear_change_penalty_m == config.gear_change_penalty_m
    assert route.objective_cost == sum(arc_cost(graph.arcs[arc_id], config) * count
                                       for arc_id, count in route.traversal_counts.items())
