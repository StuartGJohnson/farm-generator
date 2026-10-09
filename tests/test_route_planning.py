"""Synthetic arc-routing cases plus a real GIS-only farm integration case."""

from __future__ import annotations

import math
from dataclasses import replace
from pathlib import Path

import pytest
from shapely.geometry import Point, Polygon, box

from route_planning import (FarmTask, ImplementSide, PlannerConfig, Pose,
                            load_farm_map, plan_route)
from route_planning.diagnostic import (load_route_json, optimality_gap_percent,
                                       save_route_json)
from route_planning.demo import sample_tasks
from route_planning.graph import build_routing_graph
from route_planning.model import Arc, Route, RoutingGraph, Vertex
from route_planning.optimize import optimize_arc_counts
from route_planning.path_feasibility import TractorFootprint, path_feasibility
from route_planning.reconstruct import reconstruct_route
from route_planning.rs import connection


def _toy_graph(start, end, edges, requirements):
    vertices = {}
    arcs = {}
    for arc_id, u, v, service_ids, length in edges:
        for name in (u, v):
            vertices.setdefault(name, Vertex(name, Pose(float(ord(name[0]) - 65), 0.0, 0.0), False))
        arcs[arc_id] = Arc(arc_id, u, v, (vertices[u].pose, vertices[v].pose),
                           length, "row" if service_ids else "road", "field_1" if service_ids else None,
                           frozenset(service_ids))
    return RoutingGraph(vertices, arcs, set(requirements), start, end, {"field_1"})


def test_single_required_row_closed_and_one_sided_deadhead():
    # Only A->B can service a left-mounted operation; B->A is legal transit.
    graph = _toy_graph("A", "A", [
        ("service", "A", "B", {"row_left"}, 10),
        ("reverse_deadhead", "B", "A", set(), 10),
    ], {"row_left"})
    solution = optimize_arc_counts(graph)
    assert solution.counts == {"service": 1, "reverse_deadhead": 1}
    route = reconstruct_route(graph, FarmTask(("field_1",), ImplementSide.LEFT,
                                             Pose(0, 0, 0), Pose(0, 0, 0)), solution)
    assert route.segments[0].arc_id == "service"
    assert route.segments[-1].arc_id == "reverse_deadhead"


def test_shared_two_sided_path_one_traversal():
    graph = _toy_graph("A", "B", [
        ("shared", "A", "B", {"left_row", "right_row"}, 7),
        ("long_way", "A", "B", set(), 20),
    ], {"left_row", "right_row"})
    solution = optimize_arc_counts(graph)
    assert solution.counts == {"shared": 1}


def test_road_bridge_can_be_used_twice():
    # Two distinct required B->A service arcs force two A->B bridge passes.
    graph = _toy_graph("A", "A", [
        ("bridge", "A", "B", set(), 3),
        ("task_one", "B", "A", {"q1"}, 2),
        ("task_two", "B", "A", {"q2"}, 2),
    ], {"q1", "q2"})
    solution = optimize_arc_counts(graph)
    assert solution.counts["bridge"] == 2


def test_open_mission_reconstructs_to_end():
    graph = _toy_graph("A", "C", [
        ("to_row", "A", "B", set(), 2),
        ("service", "B", "C", {"q"}, 4),
        ("return", "C", "B", set(), 4),
    ], {"q"})
    solution = optimize_arc_counts(graph)
    route = reconstruct_route(graph, FarmTask(("field_1",), ImplementSide.BOTH,
                                             Pose(0, 0, 0), Pose(2, 0, 0)), solution)
    assert route.segments[0].arc_id == "to_row"
    assert route.segments[-1].arc_id == "service"


def test_invalid_rs_path_crosses_outside_u_shaped_field():
    field = Polygon([(0, 0), (10, 0), (10, 10), (7, 10), (7, 3),
                     (3, 3), (3, 10), (0, 10)])
    candidate = connection(Pose(1.5, 8, 0), Pose(8.5, 8, 0), field, [],
                           PlannerConfig(turn_radius_m=2, rs_candidate_radius_m=20))
    assert candidate is None


def test_reverse_penalty_prefers_longer_forward_rs_candidate():
    start, end = Pose(0, 0, 0), Pose(2, 3, -math.pi / 2)
    field = box(-20, -20, 20, 20)
    base = PlannerConfig(reverse_cost_multiplier=1, gear_change_penalty_m=0)
    shortest = connection(start, end, field, [], base)
    forward_favored = connection(start, end, field, [],
                                 replace(base, reverse_cost_multiplier=8))
    assert shortest is not None and forward_favored is not None
    assert forward_favored[1] > shortest[1]
    assert forward_favored[3] < shortest[3] - 1


def test_rear_axle_footprint_and_channel_clearance():
    config = PlannerConfig()
    footprint = TractorFootprint.from_config(config.tractor)
    assert config.turn_radius_m == pytest.approx(2.7)
    assert footprint.at(Pose(0, 0, 0)).bounds == pytest.approx((-0.8, -0.95, 3.2, 0.95))
    path = (Pose(0, 0, 0), Pose(2, 0, 0))
    field = box(-2, -2, 7, 2)
    assert path_feasibility(path, field, [], footprint)
    assert not path_feasibility(path, field.difference(box(3.5, -2, 4, 2)),
                                [], footprint)
    assert not path_feasibility(path, box(-0.7, -2, 7, 2), [], footprint)
    long_path = (Pose(0, 0, 0), Pose(10, 0, 0))
    assert not path_feasibility(long_path, box(-2, -2, 15, 2).difference(
        box(5, -2, 5.5, 2)), [], footprint)
    rotating_path = (Pose(0, 0, 0), Pose(0, 0, math.pi / 2))
    assert not path_feasibility(rotating_path, box(-1, -1, 3.3, 3.3),
                                [Point(2.2, 2.2).buffer(0.1)], footprint)


def test_rs_rear_axle_must_stay_outside_tree_hull():
    footprint = TractorFootprint.from_config(PlannerConfig().tractor)
    field = box(-10, -10, 20, 10)
    tree_hull_interior = box(2, -1, 4, 1).buffer(-1e-6)
    across_rows = (Pose(-5, 0, 0), Pose(10, 0, 0))
    along_headland = (Pose(-5, 1, 0), Pose(10, 1, 0))
    assert not path_feasibility(across_rows, field, [], footprint,
                                tree_hull=tree_hull_interior)
    assert path_feasibility(along_headland, field, [], footprint,
                            tree_hull=tree_hull_interior)


def test_route_optimizer_prices_reverse_distance():
    graph = _toy_graph("A", "C", [
        ("short_reverse", "A", "B", set(), 5),
        ("long_forward", "A", "B", set(), 8),
        ("service", "B", "C", {"q"}, 2),
    ], {"q"})
    graph.arcs["short_reverse"] = replace(graph.arcs["short_reverse"],
                                           reverse_distance_m=3)
    cheap_reverse = optimize_arc_counts(graph, PlannerConfig(reverse_cost_multiplier=1))
    expensive_reverse = optimize_arc_counts(graph, PlannerConfig(reverse_cost_multiplier=8))
    assert "short_reverse" in cheap_reverse.counts
    assert "long_forward" in expensive_reverse.counts
    assert "short_reverse" not in expensive_reverse.counts


def test_optimality_gap_uses_found_objective_as_denominator():
    task = FarmTask(("field_1",), ImplementSide.BOTH,
                    Pose(0, 0, 0), Pose(1, 0, 0))
    route = Route(task, [], 0, 120, "FEASIBLE", {}, [], best_bound_cost=108)
    assert optimality_gap_percent(route) == pytest.approx(10.0)
    route.optimality_status = "OPTIMAL"
    assert optimality_gap_percent(route) is None


@pytest.mark.skipif(not Path("debug_out/farms/farm_seed_1_120m2/gis/farm.gpkg").exists(),
                    reason="showcase GIS bundle has not been generated")
def test_showcase_tasks_are_fixed_by_seed():
    farm = load_farm_map("debug_out/farms/farm_seed_1_120m2/gis")
    tasks = sample_tasks(farm, 17, ImplementSide.BOTH)
    reordered = replace(farm, fields=dict(reversed(list(farm.fields.items()))),
                        roads=dict(reversed(list(farm.roads.items()))))
    assert tasks == sample_tasks(farm, 17, ImplementSide.BOTH)
    assert tasks == sample_tasks(reordered, 17, ImplementSide.BOTH)
    assert tasks != sample_tasks(farm, 18, ImplementSide.BOTH)
    assert [task.field_ids for task in tasks] == [
        ("parcel_003",), ("parcel_000", "parcel_002"),
        ("parcel_002", "parcel_003")]
    assert [(task.start.x, task.start.y, task.start.yaw,
             task.end.x, task.end.y, task.end.yaw) for task in tasks] == pytest.approx([
        (35.43745966187153, 78.76323116506582, 3.116137841771624,
         3.5, 26.501613227983544, -math.pi / 2),
        (99.76550843662872, 116.5, -math.pi,
         82.7660847399696, 57.27330179330565, -1.6119671481414803),
        (47.34376853098274, 38.143204796476, 2.9984695121379907,
         97.89033884747457, 3.5, 0.0),
    ])


@pytest.mark.skipif(not Path("debug_out/farms/farm_seed_1_120m2/gis/farm.gpkg").exists(),
                    reason="showcase GIS bundle has not been generated")
def test_real_gis_has_no_unselected_field_transit(tmp_path):
    farm = load_farm_map("debug_out/farms/farm_seed_1_120m2/gis")
    assert farm.irrigation_channels
    road = next(r for r in farm.roads.values() if r.line.length > 5)
    point = road.line.interpolate(road.line.length / 2)
    a, b = road.line.coords[0], road.line.coords[-1]
    pose = Pose(point.x, point.y, math.atan2(b[1] - a[1], b[0] - a[0]))
    task = FarmTask(("parcel_000",), ImplementSide.BOTH, pose, pose)
    graph = build_routing_graph(farm, task)
    selected_rows = {row.id for row in farm.rows.values() if row.field_id in task.field_ids}
    assert graph.requirements == {f"{row_id}:{side}" for row_id in selected_rows
                                  for side in ("L", "R")}
    row_lanes = [a for a in graph.arcs.values() if a.kind == "row"]
    assert len(row_lanes) == 2 * (len(selected_rows) + 1)
    assert sum(len(a.service_ids) == 2 for a in row_lanes) == 2 * (len(selected_rows) - 1)
    assert any(a.kind == "road" and a.field_id not in task.field_ids for a in graph.arcs.values())
    assert all(a.field_id in task.field_ids for a in graph.arcs.values() if a.kind in ("row", "rs"))
    assert any(len(a.service_ids) >= 2 for a in graph.arcs.values() if a.kind == "row")
    from route_planning.path_feasibility import tree_convex_hulls
    from shapely.geometry import LineString
    tree_hull = tree_convex_hulls(farm, task.field_ids)[task.field_ids[0]]
    assert all(not LineString([(p.x, p.y) for p in arc.poses]).intersects(tree_hull)
               for arc in graph.arcs.values() if arc.kind == "rs")
    for arc in graph.arcs.values():
        if arc.kind != "rs":
            continue
        for vertex_id, pose in ((arc.u, arc.poses[0]), (arc.v, arc.poses[-1])):
            vertex = graph.vertices[vertex_id]
            if vertex.field_id is None:
                assert vertex.oriented
                assert abs((vertex.pose.yaw - pose.yaw + math.pi) % (2 * math.pi) - math.pi) < 1e-5
    route = plan_route(farm, task, PlannerConfig(solver_time_s=5))
    assert route.total_distance_m > 0
    assert not route.saturated_arc_ids
    assert {q for segment in route.segments for q in segment.service_ids} == graph.requirements
    saved = tmp_path / "route.json"
    save_route_json(route, farm, saved)
    reloaded = load_route_json(saved)
    assert reloaded.total_distance_m == route.total_distance_m
    assert sum(s.reverse_distance_m for s in reloaded.segments) == sum(
        s.reverse_distance_m for s in route.segments)
    assert reloaded.reverse_cost_multiplier == route.reverse_cost_multiplier
    assert reloaded.traversal_counts == route.traversal_counts
    assert all(len(segment.driving_directions) == len(segment.poses)
               for segment in reloaded.segments)
