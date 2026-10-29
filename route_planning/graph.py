"""Build a directed farm movement graph from GIS geometry and a task.

No optimization code lives here. Only selected fields receive row or RS arcs;
all GIS road edges remain available to the route.
"""

from __future__ import annotations

import math
from collections import defaultdict
from dataclasses import replace

from shapely.geometry import LineString, Point
from shapely.ops import substring, unary_union
from shapely.strtree import STRtree

from .model import (Arc, DepotParkingState, FarmMap, FarmTask, ImplementSide, PlannerConfig,
                    Pose, RoutingGraph, Vertex, parking_pose, parking_spot_polygon)
from .rs import connection as rs_connection
from .path_feasibility import (TractorFootprint, path_feasibility,
                               road_turn_region, tree_concave_hulls)


def _angle(x, y):
    return math.atan2(y, x)


def _wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _distance(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)


def _tangent(line: LineString, station: float) -> float:
    before = line.interpolate(max(0, station - 0.05))
    after = line.interpolate(min(line.length, station + 0.05))
    return _angle(after.x - before.x, after.y - before.y)


def _pose(point, yaw):
    return Pose(float(point[0]), float(point[1]), _wrap(yaw))


def _setback_station(length: float, at_start: bool, setback: float) -> float:
    """Use the midpoint if two end setbacks would overlap."""
    return min(setback, length / 2) if at_start else max(length - setback, length / 2)


def infer_implement_range(farm: FarmMap, field_ids: tuple[str, ...]) -> float:
    distances = []
    for field_id in field_ids:
        lines = [r.line for r in farm.rows.values() if r.field_id == field_id and r.line is not None]
        for i, line in enumerate(lines):
            neighbors = [line.distance(other) for j, other in enumerate(lines)
                         if i != j and line.distance(other) > 0.1]
            if neighbors:
                distances.append(min(neighbors))
    if not distances:
        raise ValueError("Cannot infer row spacing; specify implement_range_m")
    distances.sort()
    return distances[len(distances) // 2] / 2


def _field_driveable(farm: FarmMap, field_id: str):
    region = farm.fields[field_id]
    if field_id == farm.depot_field_id and farm.depot is not None:
        # GIS round-tripping can leave a sub-millimetre seam at the crop/depot
        # boundary. Close it before eroding for the tractor footprint.
        region = region.union(farm.depot).buffer(1e-3).buffer(-1e-3)
    return region


def _row_candidates(farm: FarmMap, task: FarmTask, config: PlannerConfig):
    offset = task.implement_range_m or infer_implement_range(farm, task.field_ids)
    if offset <= 0:
        raise ValueError("Implement range must be positive")
    candidates = {}
    requirements = set()
    tree_obstacles = {fid: [Point(t.x, t.y).buffer(config.trunk_clearance_m +
                      config.tractor_half_width_m) for t in farm.trees.values() if t.field_id == fid]
                      for fid in task.field_ids}
    tree_indices = {fid: STRtree(obstacles) for fid, obstacles in tree_obstacles.items()}
    for row in farm.rows.values():
        if row.field_id not in task.field_ids:
            continue
        if row.line is None:
            raise ValueError(f"Selected field contains one-tree row {row.id}; no traversal line")
        a, b = row.line.coords[0], row.line.coords[-1]
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy)
        if length <= 0:
            raise ValueError(f"Invalid row {row.id}")
        normal = (-dy / length, dx / length)
        for side, sign in (("L", 1), ("R", -1)):
            # Every tree row has two distinct sides to service. A shared lane
            # can cover facing sides of neighboring rows in one traversal.
            service_id = f"{row.id}:{side}"
            requirements.add(service_id)
            start = (a[0] + sign * offset * normal[0], a[1] + sign * offset * normal[1])
            end = (b[0] + sign * offset * normal[0], b[1] + sign * offset * normal[1])
            line = LineString([start, end])
            region = _field_driveable(farm, row.field_id).buffer(-config.tractor_half_width_m)
            if not region.covers(line):
                continue
            if any(line.intersects(tree_obstacles[row.field_id][int(i)])
                   for i in tree_indices[row.field_id].query(line)):
                continue
            # Equivalent whole paths share one physical arc, retaining all
            # logical requirements. Opposite geometric orientation is folded
            # into the same canonical physical path.
            points = sorted((start, end))
            tol = config.endpoint_tolerance_m
            key = (row.field_id, tuple((round(x / tol), round(y / tol)) for x, y in points))
            entry = candidates.setdefault(key, {"field_id": row.field_id,
                                                "start": start, "end": end,
                                                "requirements": []})
            entry["requirements"].append((service_id, start, end, side))
    # Neighboring rows can define the same lane with slightly staggered tree
    # endpoints. Merge strongly overlapping, collinear paths and extend the
    # shared traversal to cover both full row sides. The two outside lanes
    # remain separate because they have no facing neighbor.
    merged = []
    for candidate in candidates.values():
        start, end = candidate["start"], candidate["end"]
        length = math.dist(start, end)
        combined = False
        for lane in merged:
            if lane["field_id"] != candidate["field_id"]:
                continue
            base, tip = lane["start"], lane["end"]
            base_length = math.dist(base, tip)
            ux, uy = (tip[0] - base[0]) / base_length, (tip[1] - base[1]) / base_length
            vx, vy = (end[0] - start[0]) / length, (end[1] - start[1]) / length
            if abs(ux * vy - uy * vx) > 1e-3:
                continue
            if max(abs(ux * (point[1] - base[1]) - uy * (point[0] - base[0]))
                   for point in (start, end)) > config.endpoint_tolerance_m:
                continue
            stations = [ux * (point[0] - base[0]) + uy * (point[1] - base[1])
                        for point in (start, end)]
            overlap = max(0.0, min(base_length, max(stations)) - max(0.0, min(stations)))
            if overlap < 0.5 * min(base_length, length):
                continue
            lo, hi = min(0.0, *stations), max(base_length, *stations)
            merged_start = (base[0] + lo * ux, base[1] + lo * uy)
            merged_end = (base[0] + hi * ux, base[1] + hi * uy)
            merged_line = LineString([merged_start, merged_end])
            region = _field_driveable(farm, lane["field_id"]).buffer(-config.tractor_half_width_m)
            if not region.covers(merged_line):
                continue
            if any(merged_line.intersects(tree_obstacles[lane["field_id"]][int(i)])
                   for i in tree_indices[lane["field_id"]].query(merged_line)):
                continue
            lane["start"], lane["end"] = merged_start, merged_end
            lane["requirements"].extend(candidate["requirements"])
            combined = True
            break
        if not combined:
            merged.append(candidate)
    return merged, requirements


def build_routing_graph(farm: FarmMap, task: FarmTask,
                        config: PlannerConfig = PlannerConfig()) -> RoutingGraph:
    terminal_spot_numbers = {
        "start": task.start.number if isinstance(task.start, DepotParkingState) else None,
        "end": task.end.number if isinstance(task.end, DepotParkingState) else None,
    }
    def resolve(value):
        if isinstance(value, DepotParkingState):
            if value.number not in farm.parking_spots:
                raise ValueError(f"Unknown depot parking spot {value.number}")
            return parking_pose(farm.parking_spots[value.number], value.direction)
        return value
    task = replace(task, start=resolve(task.start), end=resolve(task.end))
    if not task.field_ids or len(task.field_ids) != len(set(task.field_ids)):
        raise ValueError("Task must list distinct fields")
    if any(field_id not in farm.fields for field_id in task.field_ids):
        raise ValueError("Task names an unknown field")
    if (config.turn_radius_m <= 0 or config.road_turn_setback_m <= 0 or
            config.rs_step_m <= 0 or config.cost_scale_per_m <= 0 or
            config.reverse_cost_multiplier < 1 or config.gear_change_penalty_m < 0):
        raise ValueError("Invalid planner geometry or cost configuration")
    graph = RoutingGraph(start_id="", end_id="", selected_fields=set(task.field_ids))
    paths, graph.requirements = _row_candidates(farm, task, config)
    if len(graph.requirements) == 0:
        raise ValueError("Task has no service requirements")

    def add_arc(arc: Arc):
        if arc.distance_m <= 1e-5:
            raise ValueError(f"Zero-length movement arc {arc.id}")
        if arc.id in graph.arcs:
            raise ValueError(f"Duplicate arc {arc.id}")
        graph.arcs[arc.id] = arc

    # Only truly coincident row pose states are shared. Nearby endpoint
    # positions cannot be collapsed without an explicit movement arc.
    row_state_key = {}

    def row_state(pose: Pose, field_id: str):
        key = (field_id, round(pose.x, 5), round(pose.y, 5), round(_wrap(pose.yaw), 5))
        if key not in row_state_key:
            identifier = f"row_state_{len(row_state_key):04d}"
            row_state_key[key] = identifier
            graph.vertices[identifier] = Vertex(identifier, pose, True, field_id)
        return row_state_key[key]

    for index, path in enumerate(paths):
        field_id = path["field_id"]
        a, b = path["start"], path["end"]
        yaw = _angle(b[0] - a[0], b[1] - a[1])
        forward = (_pose(a, yaw), _pose(b, yaw))
        reverse = (_pose(b, yaw + math.pi), _pose(a, yaw + math.pi))
        for direction, poses in (("f", forward), ("r", reverse)):
            covers = set()
            for service_id, row_a, row_b, side in path["requirements"]:
                same_direction = ((row_b[0] - row_a[0]) * (b[0] - a[0]) +
                                  (row_b[1] - row_a[1]) * (b[1] - a[1])) > 0
                travel_with_row = same_direction if direction == "f" else not same_direction
                row_on_left = (side == "R") == travel_with_row
                if task.implement_side == ImplementSide.BOTH or (row_on_left and task.implement_side == ImplementSide.LEFT) or (
                    not row_on_left and task.implement_side == ImplementSide.RIGHT):
                    covers.add(service_id)
            u = row_state(poses[0], field_id)
            v = row_state(poses[1], field_id)
            add_arc(Arc(f"row_{index:04d}_{direction}", u, v, poses,
                        _distance(poses[0], poses[1]), "row", field_id,
                        frozenset(covers), lane_id=f"row_{index:04d}"))

    # Road travel stops short of each junction. Directed RS arcs below replace
    # position-only switches between incident GIS road centerlines.
    splits = {road.id: {} for road in farm.roads.values()}
    junction_legs = defaultdict(list)
    for road in farm.roads.values():
        for at_start, node_id in ((True, road.node_a), (False, road.node_b)):
            station = _setback_station(road.line.length, at_start,
                                       config.road_turn_setback_m)
            if station not in splits[road.id]:
                identifier = f"setback_{road.id}_{'a' if at_start else 'b'}"
                point = road.line.interpolate(station)
                graph.vertices[identifier] = Vertex(
                    identifier, Pose(point.x, point.y, _tangent(road.line, station)), False)
                splits[road.id][station] = identifier
            junction_legs[node_id].append((road, station,
                                           "r" if at_start else "f",
                                           "f" if at_start else "r"))

    def insert_on_road(road, station, prefix):
        station = max(0.0, min(road.line.length, station))
        for existing, identifier in splits[road.id].items():
            if abs(existing - station) < 0.02:
                return identifier
        identifier = f"{prefix}_{road.id}_{len(splits[road.id]):03d}"
        point = road.line.interpolate(station)
        graph.vertices[identifier] = Vertex(identifier, Pose(point.x, point.y, _tangent(road.line, station)), False)
        splits[road.id][station] = identifier
        return identifier

    road_direction_states = {}

    def road_state(road, station, direction):
        """Preserve travel heading at every road split, including setbacks."""
        station = min(splits[road.id], key=lambda value: abs(value - station))
        base_id = splits[road.id][station]
        key = (road.id, base_id, direction)
        if key not in road_direction_states:
            point = road.line.interpolate(station)
            yaw = _tangent(road.line, station) + (math.pi if direction == "r" else 0)
            identifier = f"{base_id}_{direction}"
            graph.vertices[identifier] = Vertex(identifier, Pose(point.x, point.y, _wrap(yaw)), True)
            road_direction_states[key] = identifier
        return road_direction_states[key]

    def project_terminal(pose, label):
        p = Point(pose.x, pose.y)
        road = min(farm.roads.values(), key=lambda r: r.line.distance(p))
        station = road.line.project(p)
        point = road.line.interpolate(station)
        if p.distance(point) > config.road_pose_tolerance_m:
            raise ValueError(f"{label} pose is not on a road")
        tangent = _tangent(road.line, station)
        if min(abs(_wrap(pose.yaw - tangent)), abs(_wrap(pose.yaw - tangent - math.pi))) > 0.2:
            raise ValueError(f"{label} heading is not road aligned")
        lo = _setback_station(road.line.length, True, config.road_turn_setback_m)
        hi = _setback_station(road.line.length, False, config.road_turn_setback_m)
        if not lo + 0.25 < station < hi - 0.25:
            raise ValueError(f"{label} pose lies inside a road turn setback")
        insert_on_road(road, station, f"{label}_road")
        return road, station, tangent

    def is_depot_pose(pose):
        return farm.depot is not None and farm.depot.covers(Point(pose.x, pose.y))

    start_depot = is_depot_pose(task.start)
    end_depot = is_depot_pose(task.end)
    start_info = None if start_depot else project_terminal(task.start, "start")
    end_info = None if end_depot else project_terminal(task.end, "end")
    closed = (_distance(task.start, task.end) < 1e-5 and
              abs(_wrap(task.start.yaw - task.end.yaw)) < 1e-5)
    graph.start_id = "terminal_start"
    graph.end_id = "terminal_start" if closed else "terminal_end"
    graph.vertices[graph.start_id] = Vertex(graph.start_id, task.start, True)
    if not closed:
        graph.vertices[graph.end_id] = Vertex(graph.end_id, task.end, True)

    def terminal_connection(road, station, tangent, pose, label, outbound):
        sign = 1 if math.cos(_wrap(pose.yaw - tangent)) >= 0 else -1
        target_station = station + (sign if outbound else -sign) * 0.25
        if not 0 <= target_station <= road.line.length:
            raise ValueError(f"{label} pose has no road in its travel direction")
        insert_on_road(road, target_station, f"{label}_interface")
        interface_id = road_state(road, target_station, "f" if sign > 0 else "r")
        interface_pose = graph.vertices[interface_id].pose
        if outbound:
            add_arc(Arc("terminal_start_arc", graph.start_id, interface_id,
                        (pose, interface_pose), _distance(pose, interface_pose), "road", road.field_id, max_uses=1))
        else:
            add_arc(Arc("terminal_end_arc", interface_id, graph.end_id,
                        (interface_pose, pose), _distance(interface_pose, pose), "road", road.field_id, max_uses=1))

    if start_info is not None:
        terminal_connection(*start_info, task.start, "start", True)
    if end_info is not None:
        terminal_connection(*end_info, task.end, "end", False)

    depot_interfaces = []
    if start_depot or end_depot:
        if config.depot_road_sampling_m <= 0:
            raise ValueError("depot_road_sampling_m must be positive")
        nearby = [road for road in farm.roads.values()
                  if road.road_class != "crossing_spur" and
                  road.line.distance(farm.depot) <= max(3.0, config.tractor_half_width_m * 2)]
        for road in nearby:
            lo = _setback_station(road.line.length, True, config.road_turn_setback_m)
            hi = _setback_station(road.line.length, False, config.road_turn_setback_m)
            if hi - lo < 0.5:
                continue
            stations = [lo + 0.2 + k * config.depot_road_sampling_m
                        for k in range(int((hi - lo - 0.4) / config.depot_road_sampling_m) + 1)]
            for station in stations:
                insert_on_road(road, station, "depot_interface")
                depot_interfaces.append((road, station))

    # Each row endpoint sees its own nearby frontage-road states. A projection
    # alone can demand a near U-turn at a short row; offsets along the road
    # permit a forward exit or entry with room to align the tractor.
    interfaces = defaultdict(set)
    for vertex in list(graph.vertices.values()):
        if not vertex.oriented or vertex.field_id is None:
            continue
        frontage = [road for road in farm.roads.values()
                    if road.field_id == vertex.field_id and road.road_class != "crossing_spur"]
        if not frontage:
            raise ValueError(f"Field {vertex.field_id} has no frontage road")
        nearest = sorted(frontage, key=lambda r: r.line.distance(Point(vertex.pose.x, vertex.pose.y)))
        for road in nearest[:config.road_interface_candidates]:
            if road.line.length <= 0.2:
                continue
            projection = road.line.project(Point(vertex.pose.x, vertex.pose.y))
            # Keep row/road interfaces outside both junction turn zones.
            lo = _setback_station(road.line.length, True, config.road_turn_setback_m)
            hi = _setback_station(road.line.length, False, config.road_turn_setback_m)
            for station in (projection,
                            projection - config.road_interface_offset_m,
                            projection + config.road_interface_offset_m):
                if not lo + 0.1 < station < hi - 0.1:
                    continue
                interface_id = insert_on_road(road, station, "interface")
                interfaces[vertex.id].add((interface_id, road.id, station))

    footprint = TractorFootprint.from_config(config.tractor)
    turn_region = road_turn_region(farm)
    turn_obstacles = [Point(t.x, t.y).buffer(config.trunk_clearance_m)
                      for t in farm.trees.values()]
    turn_index = STRtree(turn_obstacles)
    turn_tree_hull = unary_union(list(tree_concave_hulls(farm, tuple(farm.fields)).values()))

    if start_depot or end_depot:
        permitted_depot = unary_union([farm.depot, *farm.road_surfaces.values()])
        spot_polygons = {number: parking_spot_polygon(spot)
                         for number, spot in farm.parking_spots.items()}
        depot_config = replace(config, rs_candidate_radius_m=max(
            config.rs_candidate_radius_m, farm.depot.length + 2 * config.road_turn_setback_m))
        for label, terminal_id, terminal_pose, outbound in (
                ("start", graph.start_id, task.start, True),
                ("end", graph.end_id, task.end, False)):
            if not is_depot_pose(terminal_pose):
                continue
            own_spot = terminal_spot_numbers[label]
            if own_spot is None:
                own_spot = next((number for number, polygon in spot_polygons.items()
                                 if polygon.covers(Point(terminal_pose.x, terminal_pose.y))), None)
            blocked_spots = [polygon.buffer(config.depot_parking_clearance_m)
                             for number, polygon in spot_polygons.items() if number != own_spot]
            blocked_index = STRtree(blocked_spots)
            own = farm.parking_spots.get(own_spot)
            inner_leg_cache = {}

            def depot_rs(a, b):
                return rs_connection(a, b, permitted_depot, blocked_spots,
                    depot_config, blocked_index, permitted_region=permitted_depot,
                    footprint=footprint,
                    reverse_cost_multiplier=config.depot_reverse_cost_multiplier,
                    reverse_event_penalty_m=config.depot_reverse_event_penalty_m,
                    reverse_allowance_m=config.depot_reverse_allowance_m,
                    excess_reverse_cost_multiplier=config.depot_excess_reverse_cost_multiplier)

            connected = 0
            for road, station in depot_interfaces:
                for direction in ("f", "r"):
                    interface_id = road_state(road, station, direction)
                    road_pose = graph.vertices[interface_id].pose
                    source, target = ((terminal_pose, road_pose) if outbound else
                                      (road_pose, terminal_pose))
                    direct = depot_rs(source, target)
                    candidates = [direct] if direct is not None else []
                    if own is not None:
                        outward = (-math.cos(own.yaw_in), -math.sin(own.yaw_in))
                        lateral = (-outward[1], outward[0])
                        side_clearance = (own.width_m / 2 + config.tractor_half_width_m +
                                          config.rs_step_m / 2)
                        # Sample an aisle turn in tractor-radius increments on
                        # both sides of the parked tractor. Every candidate is
                        # checked against the full depot/road surface and spots.
                        for step in range(4, 11):
                            setback = own.length_m / 2 + step * config.turn_radius_m / 4
                            for side in (-1, 1):
                                middle = Pose(terminal_pose.x + setback * outward[0] +
                                              side * side_clearance * lateral[0],
                                              terminal_pose.y + setback * outward[1] +
                                              side * side_clearance * lateral[1], road_pose.yaw)
                                middle_shape = footprint.at(middle)
                                if (not farm.depot.covers(middle_shape) or
                                        any(middle_shape.intersects(spot) for spot in blocked_spots)):
                                    continue
                                key = (round(middle.x, 4), round(middle.y, 4),
                                       round(middle.yaw, 4))
                                if key not in inner_leg_cache:
                                    inner_leg_cache[key] = (depot_rs(terminal_pose, middle)
                                                            if outbound else
                                                            depot_rs(middle, terminal_pose))
                                inner = inner_leg_cache[key]
                                outer = (depot_rs(middle, road_pose) if outbound else
                                         depot_rs(road_pose, middle)) if inner is not None else None
                                first, second = ((inner, outer) if outbound else
                                                 (outer, inner))
                                if first is not None and second is not None:
                                    a_poses, a_length, a_dirs, a_reverse, a_changes, a_events = first
                                    b_poses, b_length, b_dirs, b_reverse, b_changes, b_events = second
                                    changed = a_dirs[-1] != b_dirs[0]
                                    candidates.append((a_poses + b_poses[1:], a_length + b_length,
                                        a_dirs + b_dirs[1:], a_reverse + b_reverse,
                                        a_changes + b_changes + int(changed),
                                        a_events + b_events - int(a_dirs[-1] == b_dirs[0] == -1)))
                    for poses, length, directions, reverse_m, changes, events in candidates:
                        u, v = ((terminal_id, interface_id) if outbound else
                                (interface_id, terminal_id))
                        add_arc(Arc(f"depot_{label}_{connected:04d}", u, v, poses,
                                    length, "depot", driving_directions=directions,
                                    reverse_distance_m=reverse_m, gear_changes=changes,
                                    reverse_events=events, max_uses=1))
                        connected += 1
            if connected == 0:
                raise ValueError(f"No legal depot connection for {label} pose")

    for road in farm.roads.values():
        stations = sorted(splits[road.id])
        for j, (a, b) in enumerate(zip(stations, stations[1:])):
            if b - a < 1e-4:
                continue
            segment = substring(road.line, a, b)
            coords = list(segment.coords)
            for direction, u, v, points in (
                ("f", splits[road.id][a], splits[road.id][b], coords),
                ("r", splits[road.id][b], splits[road.id][a], coords[::-1]),
            ):
                u = road_state(road, a if direction == "f" else b, direction)
                v = road_state(road, b if direction == "f" else a, direction)
                poses = tuple(_pose(point, _angle(points[-1][0] - points[0][0],
                                                 points[-1][1] - points[0][1])) for point in points)
                if graph.vertices[u].oriented:
                    poses = (graph.vertices[u].pose, *poses[1:])
                if graph.vertices[v].oriented:
                    poses = (*poses[:-1], graph.vertices[v].pose)
                if road.road_class == "crossing_spur" and not path_feasibility(
                        poses, turn_region, turn_obstacles, footprint,
                        turn_index, turn_tree_hull):
                    continue
                add_arc(Arc(f"road_{road.id}_{j:03d}_{direction}", u, v, poses,
                            segment.length, "road", road.field_id))

    # Check the swept tractor envelope against the selected field, excluding
    # irrigation channels and tree trunk clearances.
    rs_counter = 0
    tree_geometries = {fid: [Point(t.x, t.y).buffer(config.trunk_clearance_m)
                             for t in farm.trees.values() if t.field_id == fid]
                       for fid in task.field_ids}
    tree_indices = {fid: STRtree(geoms) for fid, geoms in tree_geometries.items()}
    channel_union = unary_union(list(farm.irrigation_channels.values()))
    permitted_fields = {fid: _field_driveable(farm, fid).difference(channel_union)
                        for fid in task.field_ids}
    tree_hulls = tree_concave_hulls(farm, task.field_ids)
    for node_id, legs in sorted(junction_legs.items()):
        for incoming_road, incoming_station, incoming_direction, _ in legs:
            u = road_state(incoming_road, incoming_station, incoming_direction)
            for outgoing_road, outgoing_station, _, outgoing_direction in legs:
                if incoming_road.id == outgoing_road.id:
                    continue
                v = road_state(outgoing_road, outgoing_station, outgoing_direction)
                candidate = rs_connection(graph.vertices[u].pose, graph.vertices[v].pose,
                                          turn_region, turn_obstacles, config, turn_index,
                                          permitted_region=turn_region, footprint=footprint,
                                          tree_hull=turn_tree_hull, forward_only=True)
                if candidate is None:
                    continue
                poses, length, directions, reverse_m, gear_changes, reverse_events = candidate
                add_arc(Arc(f"turn_{node_id}_{incoming_road.id}_{outgoing_road.id}",
                            u, v, poses, length, "road_turn",
                            driving_directions=directions,
                            reverse_distance_m=reverse_m, gear_changes=gear_changes,
                            reverse_events=reverse_events))
    rs_seen = set()

    def add_rs(u, v, start_pose, end_pose, field_id):
        nonlocal rs_counter
        if u == v:
            return
        key = (u, v, round(start_pose.yaw, 4), round(end_pose.yaw, 4))
        if key in rs_seen:
            return
        rs_seen.add(key)
        field_config = (replace(config, rs_candidate_radius_m=max(
            config.rs_candidate_radius_m, farm.depot.length + config.rs_candidate_radius_m))
            if farm.depot is not None and farm.fields[field_id].distance(farm.depot) < 1e-5 else config)
        permitted_region = permitted_fields[field_id]
        hull = tree_hulls[field_id]
        if farm.depot is not None and (graph.vertices[u].field_id is None or graph.vertices[v].field_id is None):
            permitted_region = permitted_region.union(farm.depot).buffer(1e-4)
        candidate = rs_connection(start_pose, end_pose, farm.fields[field_id],
                                  tree_geometries[field_id], field_config, tree_indices[field_id],
                                  permitted_region=permitted_region,
                                  footprint=footprint, tree_hull=hull,
                                  forward_only=config.field_rs_forward_only)
        if candidate is None:
            return
        poses, length, directions, reverse_m, gear_changes, reverse_events = candidate
        add_arc(Arc(f"rs_{rs_counter:05d}", u, v, poses, length,
                    "rs", field_id, driving_directions=directions,
                    reverse_distance_m=reverse_m, gear_changes=gear_changes,
                    reverse_events=reverse_events))
        rs_counter += 1

    row_vertices = [v for v in graph.vertices.values() if v.oriented and v.field_id is not None]
    for vertex in row_vertices:
        search_radius = (config.rs_candidate_radius_m + farm.depot.length
                         if farm.depot is not None and farm.fields[vertex.field_id].distance(farm.depot) < 1e-5
                         else config.rs_candidate_radius_m)
        nearby = sorted((other for other in row_vertices
                         if other.id != vertex.id and other.field_id == vertex.field_id
                         and _distance(vertex.pose, other.pose) <= search_radius),
                        key=lambda other: _distance(vertex.pose, other.pose))
        for other in nearby[:8]:
            add_rs(vertex.id, other.id, vertex.pose, other.pose, vertex.field_id)
        field_interfaces = sorted(interfaces[vertex.id],
                                  key=lambda item: (_distance(vertex.pose,
                                                              graph.vertices[item[0]].pose),
                                                    item[1], item[2], item[0]))
        for interface_id, road_id, station in field_interfaces:
            road_pose = graph.vertices[interface_id].pose
            road = farm.roads[road_id]
            for direction in ("f", "r"):
                state_id = road_state(road, station, direction)
                source = graph.vertices[state_id].pose
                add_rs(state_id, vertex.id, source, vertex.pose, vertex.field_id)
                add_rs(vertex.id, state_id, vertex.pose, source, vertex.field_id)

    # Interior split positions are represented by their directional states.
    # The initial position-only placeholders must not become route junctions.
    incident = {node for arc in graph.arcs.values() for node in (arc.u, arc.v)}
    graph.vertices = {key: value for key, value in graph.vertices.items() if key in incident}

    covered = set().union(*(arc.service_ids for arc in graph.arcs.values()))
    missing = graph.requirements - covered
    if missing:
        raise ValueError(f"No legal service arc for {len(missing)} tasks: {sorted(missing)[:5]}")
    return graph
