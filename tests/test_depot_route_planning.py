"""Depot route checks against a farm GIS bundle made by the generation tests.

Run this file in the route-planning venv; it never generates a farm or imports USD.
"""

from pathlib import Path
from shutil import copy2

import pytest
from shapely.geometry import MultiPoint, Point
from shapely.ops import unary_union

from route_planning.gis import load_farm_map
from route_planning.graph import build_routing_graph
from route_planning.model import (DepotParkingState, FarmTask, ImplementSide,
                                  PlannerConfig, parking_pose, parking_spot_polygon)
from route_planning.path_feasibility import (TractorFootprint, path_feasibility,
                                             tree_concave_hulls)
from route_planning.planner import plan_route


FARM_DIR = Path("debug_out/farms/farm_seed_1_120m2_depot")
TIGHT_FARM_DIR = Path("debug_out/farms/farm_seed_4_120m2_depot")


@pytest.fixture(scope="module")
def depot_farm():
    gis_dir = FARM_DIR / "gis"
    for name in ("farm.gpkg", "map_frame.json"):
        if not (gis_dir / name).is_file():
            pytest.fail(f"Missing route-planning fixture: {gis_dir / name}")
    farm = load_farm_map(gis_dir)
    assert farm.depot_field_id == "parcel_003"
    assert len(farm.parking_spots) >= 2
    return farm


def test_gis_bundle_without_usd_or_orthophoto_loads(tmp_path):
    for name in ("farm.gpkg", "map_frame.json"):
        copy2(FARM_DIR / "gis" / name, tmp_path / name)
    farm = load_farm_map(tmp_path)
    assert farm.depot_field_id == "parcel_003"
    assert farm.parking_spots
    assert farm.orthophoto_path is None


def test_saved_depot_field_concave_tree_boundary(depot_farm):
    farm = depot_farm
    field_id = farm.depot_field_id
    points = [(tree.x, tree.y) for tree in farm.trees.values()
              if tree.field_id == field_id]
    hull = tree_concave_hulls(farm, (field_id,))[field_id]
    wider = tree_concave_hulls(farm, (field_id,), concavity_m=20)[field_id]
    assert hull.is_valid and not hull.is_empty
    assert hull.area < wider.area < MultiPoint(points).convex_hull.area
    assert hull.intersection(farm.depot).area < 1e-6
    with pytest.raises(ValueError, match="concavity_m"):
        tree_concave_hulls(farm, (field_id,), concavity_m=0)


@pytest.mark.parametrize("field_ids", [("parcel_000",), ("parcel_003",)])
def test_saved_depot_graph_connects_parking_spots(depot_farm, field_ids):
    farm = depot_farm
    task = FarmTask(field_ids, ImplementSide.BOTH,
                    DepotParkingState(1, "in"), DepotParkingState(2, "in"))
    config = PlannerConfig()
    graph = build_routing_graph(farm, task, config)
    assert any(arc.kind == "depot" and arc.u == graph.start_id for arc in graph.arcs.values())
    assert any(arc.kind == "depot" and arc.v == graph.end_id for arc in graph.arcs.values())
    assert all(not (arc.kind == "depot" and
                    (graph.vertices[arc.u].field_id or graph.vertices[arc.v].field_id))
               for arc in graph.arcs.values())
    assert all(Point(graph.vertices[arc.u].pose.x, graph.vertices[arc.u].pose.y).distance(
               Point(graph.vertices[arc.v].pose.x, graph.vertices[arc.v].pose.y))
               <= config.rs_candidate_radius_m + 1e-6
               for arc in graph.arcs.values() if arc.kind == "rs")
    assert all(arc.distance_m >= 1 for arc in graph.arcs.values() if arc.kind == "road")
    permitted = unary_union([farm.depot, *farm.road_surfaces.values()])
    footprint = TractorFootprint.from_config(config.tractor)
    for arc in graph.arcs.values():
        if arc.kind != "depot":
            continue
        own_spot = 1 if arc.u == graph.start_id else 2
        blocked = [parking_spot_polygon(spot).buffer(config.depot_parking_clearance_m)
                   for number, spot in farm.parking_spots.items() if number != own_spot]
        assert path_feasibility(arc.poses, permitted, blocked, footprint)


def test_saved_depot_parking_poses(depot_farm):
    for spot in depot_farm.parking_spots.values():
        assert parking_spot_polygon(spot).area == pytest.approx(spot.length_m * spot.width_m)
    assert parking_pose(depot_farm.parking_spots[1], "in").yaw != parking_pose(
        depot_farm.parking_spots[1], "out").yaw
    with pytest.raises(ValueError):
        parking_pose(depot_farm.parking_spots[1], "sideways")


def test_saved_tight_depot_rejects_spot_with_no_clear_exit():
    gis_dir = TIGHT_FARM_DIR / "gis"
    for name in ("farm.gpkg", "map_frame.json"):
        if not (gis_dir / name).is_file():
            pytest.fail(f"Missing route-planning fixture: {gis_dir / name}")
    farm = load_farm_map(gis_dir)
    assert farm.depot_field_id is not None
    task = FarmTask((farm.depot_field_id,), ImplementSide.BOTH,
                    DepotParkingState(1, "in"), DepotParkingState(2, "out"))
    with pytest.raises(ValueError, match="No legal depot connection for start pose"):
        build_routing_graph(farm, task, PlannerConfig())


@pytest.mark.parametrize("field_ids,solver_time_s", [
    (("parcel_000",), 60),
    (("parcel_003",), 90),
    (("parcel_000", "parcel_002"), 120),
])
def test_saved_depot_route_uses_two_parking_spots(
        depot_farm, field_ids, solver_time_s):
    farm = depot_farm
    task = FarmTask(field_ids, ImplementSide.BOTH,
                    DepotParkingState(1, "in"), DepotParkingState(2, "in"))
    config = PlannerConfig(solver_time_s=solver_time_s)
    route = plan_route(farm, task, config)
    assert route.segments[0].kind == "depot"
    assert route.segments[-1].kind == "depot"
    assert route.optimality_status in ("OPTIMAL", "FEASIBLE")
    assert route.task.field_ids == field_ids
    for endpoint, spot in ((route.segments[0].poses[0], farm.parking_spots[1]),
                           (route.segments[-1].poses[-1], farm.parking_spots[2])):
        expected = parking_pose(spot, "in")
        assert (endpoint.x, endpoint.y, endpoint.yaw) == pytest.approx(
            (expected.x, expected.y, expected.yaw))
    required = {f"{row.id}:{side}" for row in farm.rows.values()
                if row.field_id in field_ids for side in ("L", "R")}
    serviced = set().union(*(segment.service_ids for segment in route.segments))
    assert required <= serviced
    permitted_depot = unary_union([farm.depot, *farm.road_surfaces.values()])
    footprint = TractorFootprint.from_config(config.tractor)
    for segment, own_spot in ((route.segments[0], 1), (route.segments[-1], 2)):
        blocked = [parking_spot_polygon(spot).buffer(config.depot_parking_clearance_m)
                   for number, spot in farm.parking_spots.items() if number != own_spot]
        assert path_feasibility(segment.poses, permitted_depot, blocked, footprint)
    if field_ids == (farm.depot_field_id,):
        exit_segment = route.segments[0]
        assert exit_segment.reverse_distance_m < 12
        first_forward = next(pose for pose, direction in zip(
            exit_segment.poses, exit_segment.driving_directions) if direction > 0)
        assert farm.depot.covers(Point(first_forward.x, first_forward.y))
