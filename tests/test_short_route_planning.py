"""Short travel-only routes for route-management and tractor-control development."""

from __future__ import annotations

import math
from pathlib import Path

import pytest
import yaml
from shapely.geometry import LineString

from route_planning import FarmTask, ImplementSide, PlannerConfig, Pose, load_farm_map, plan_route
from route_planning.diagnostic import load_route_json, save_route_json, save_route_plot
from route_planning.graph import build_routing_graph
from route_planning.model import DepotParkingState, depot_parking_dividers, parking_pose
from route_planning.path_feasibility import TractorFootprint, path_feasibility


FARM_GIS = Path("debug_out/farms/farm_seed_1_120m2_depot/gis")
OUTPUT = Path("debug_out/routes/short")


@pytest.fixture(scope="module")
def farm():
    for name in ("farm.gpkg", "map_frame.json"):
        if not (FARM_GIS / name).is_file():
            pytest.fail(f"Missing route-planning fixture: {FARM_GIS / name}")
    return load_farm_map(FARM_GIS)


def _road_pose(road, station: float, direction: int = 1) -> Pose:
    point = road.line.interpolate(station)
    before = road.line.interpolate(max(0.0, station - 0.05))
    after = road.line.interpolate(min(road.line.length, station + 0.05))
    yaw = math.atan2(after.y - before.y, after.x - before.x)
    return Pose(point.x, point.y, yaw + (math.pi if direction < 0 else 0.0))


def _task(farm, number: int) -> FarmTask:
    if number == 1:
        road = farm.roads["redge_0001"]
        start = _road_pose(road, 30.0)
        end = _road_pose(road, 40.0)
    elif number == 2:
        # North on the west road, across channel crossing 18, then right/east.
        incoming = farm.roads["redge_0005"]
        outgoing = farm.roads["redge_0011"]
        start = _road_pose(incoming, incoming.line.length - 10.0)
        end = _road_pose(outgoing, outgoing.line.length - 10.0, -1)
    else:
        return FarmTask((), ImplementSide.BOTH,
                        DepotParkingState(1, "in"), DepotParkingState(2, "in"))
    return FarmTask((), ImplementSide.BOTH, start, end)


def _painted_parking_markers():
    """Read the marker strips using the scene's ground-mesh placement rule."""
    depot = yaml.safe_load((FARM_GIS.parent / "farm.yaml").read_text())["scene"]["depot"]
    markers = []
    for spot in depot["parking_spots"]:
        station = (depot["along_row_length_m"] - depot["parking_inset_m"] -
                   spot["number"] * spot["width_m"])
        start = tuple(depot["origin"][axis] + station * depot["row_direction"][axis] +
                      depot["cross_row_width_m"] * depot["cross_row_direction"][axis]
                      for axis in (0, 1))
        end = tuple(start[axis] - depot["parking_line_length_m"] *
                    depot["cross_row_direction"][axis] for axis in (0, 1))
        markers.append(LineString((start, end)).buffer(0.045, cap_style=2))
    return markers


@pytest.mark.parametrize("number", (1, 2, 3))
def test_short_route_planning(farm, number):
    task = _task(farm, number)
    # This junction's right turn needs the existing 2 m steering configuration.
    config = PlannerConfig(solver_time_s=90 if number == 3 else 30,
                           tractor_config_path=("configs/tractor_route_2m.yaml" if number == 2
                                                else "configs/tractor_default.yaml"))
    graph = build_routing_graph(farm, task, config)
    assert not graph.requirements
    assert all(arc.kind not in ("row", "rs") for arc in graph.arcs.values())
    if number == 3:
        assert any(vertex_id.startswith("depot_aisle_") for vertex_id in graph.vertices)
        assert all(vertex_id.startswith(("terminal_", "depot_aisle_"))
                   for vertex_id in graph.vertices)
        assert all(arc.kind == "depot" for arc in graph.arcs.values())
        assert not any(arc.u == graph.start_id and arc.v == graph.end_id
                       for arc in graph.arcs.values())

    route = plan_route(farm, task, config)
    assert route.optimality_status in ("OPTIMAL", "FEASIBLE")
    assert route.task.field_ids == ()
    assert route.segments
    if number in (1, 2):
        assert sum(segment.reverse_distance_m for segment in route.segments) == pytest.approx(0)
        assert all(direction > 0 for segment in route.segments
                   for direction in segment.driving_directions)
    if number == 1:
        assert route.total_distance_m == pytest.approx(10.0, abs=0.05)
        assert all(segment.kind == "road" for segment in route.segments)
    elif number == 2:
        assert any(segment.kind == "road" and segment.arc_id.startswith("road_redge_0018")
                   for segment in route.segments)
        assert any(segment.kind == "road_turn" for segment in route.segments)
    else:
        assert route.segments[0].kind == route.segments[-1].kind == "depot"
        assert all(segment.kind == "depot" for segment in route.segments)
        assert sum(segment.reverse_distance_m for segment in route.segments) > 0
        assert route.segments[0].poses[0] == parking_pose(farm.parking_spots[1], "in")
        assert route.segments[-1].poses[-1] == parking_pose(farm.parking_spots[2], "in")
        dividers = depot_parking_dividers(farm.parking_spots,
                                          config.depot_parking_clearance_m)
        footprint = TractorFootprint.from_config(config.tractor)
        assert all(path_feasibility(segment.poses, farm.depot, dividers, footprint)
                   for segment in route.segments)
        painted_markers = _painted_parking_markers()
        assert all(path_feasibility(segment.poses, farm.depot,
                                    painted_markers, footprint)
                   for segment in route.segments)

    stem = OUTPUT / f"short_route_{number}"
    save_route_json(route, farm, stem.with_suffix(".json"))
    poses = [pose for segment in route.segments for pose in segment.poses]
    margin = 8.0
    bounds = (min(pose.x for pose in poses) - margin,
              min(pose.y for pose in poses) - margin,
              max(pose.x for pose in poses) + margin,
              max(pose.y for pose in poses) + margin)
    save_route_plot(route, farm, stem.with_suffix(".png"), bounds=bounds)
    reloaded = load_route_json(stem.with_suffix(".json"))
    assert reloaded.total_distance_m == pytest.approx(route.total_distance_m)
    assert reloaded.task.field_ids == ()
