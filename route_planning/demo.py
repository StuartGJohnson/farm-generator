"""Plan reproducible random showcase missions from GIS files only."""

from __future__ import annotations

import argparse
import json
import math
import random
from dataclasses import replace
from pathlib import Path

from .diagnostic import save_route_json, save_route_plot
from .gis import load_farm_map
from .model import DEFAULT_TRACTOR_CONFIG_PATH, FarmTask, ImplementSide, PlannerConfig, Pose
from .path_feasibility import TractorFootprint
from .planner import plan_route


def solver_time_for_case(index: int, override_s: float | None) -> float:
    """Give the harder second showcase mission a longer default solve."""
    if index not in (1, 2, 3):
        raise ValueError(f"Unknown showcase case {index}")
    return override_s if override_s is not None else (90.0 if index == 2 else 60.0)


def _road_pose(road, fraction, reverse):
    point = road.line.interpolate(road.line.length * fraction)
    a, b = road.line.coords[0], road.line.coords[-1]
    yaw = math.atan2(b[1] - a[1], b[0] - a[0]) + (math.pi if reverse else 0)
    return Pose(point.x, point.y, (yaw + math.pi) % (2 * math.pi) - math.pi)


def _has_in_place_reversal(route):
    for before, after in zip(route.segments, route.segments[1:]):
        if before.kind != "road" or after.kind != "road":
            continue
        turn = abs((after.poses[0].yaw - before.poses[-1].yaw + math.pi)
                   % (2 * math.pi) - math.pi)
        if turn > math.pi - 0.1:
            return True
    return False


def _pose_faces_fields(pose, x, y, toward_fields):
    dx, dy = (x - pose.x, y - pose.y) if toward_fields else (pose.x - x, pose.y - y)
    distance = math.hypot(dx, dy)
    return distance < 1e-9 or (math.cos(pose.yaw) * dx + math.sin(pose.yaw) * dy) / distance >= -0.5


def sample_tasks(farm, seed: int, implement_side: ImplementSide) -> tuple[FarmTask, ...]:
    """Choose all missions before solving; solver outcomes never advance the RNG."""
    rng = random.Random(seed)
    fields = sorted(farm.fields)
    road_edges = sorted((r for r in farm.roads.values()
                         if r.road_class != "crossing_spur" and r.line.length > 5),
                        key=lambda road: road.id)
    if len(fields) < 2 or len(road_edges) < 2:
        raise ValueError("Showcase tasks need at least two fields and two road edges")
    tasks = []
    for number_of_fields in (1, 2, 2):
        chosen = tuple(sorted(rng.sample(fields, number_of_fields)))
        centers = [farm.fields[field_id].centroid for field_id in chosen]
        center_x = sum(point.x for point in centers) / len(centers)
        center_y = sum(point.y for point in centers) / len(centers)
        for _ in range(8):
            start_road, end_road = rng.sample(road_edges, 2)
            start = _road_pose(start_road, rng.uniform(0.35, 0.65), bool(rng.getrandbits(1)))
            end = _road_pose(end_road, rng.uniform(0.35, 0.65), bool(rng.getrandbits(1)))
            if (_pose_faces_fields(start, center_x, center_y, True) and
                    _pose_faces_fields(end, center_x, center_y, False)):
                tasks.append(FarmTask(chosen, implement_side, start, end))
                break
        else:
            raise ValueError(f"No road-aligned poses for fields {chosen} with seed {seed}")
    return tuple(tasks)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--farm", type=Path, default=Path("debug_out/farms/farm_seed_1_120m2"))
    parser.add_argument("--output-dir", type=Path, default=Path("debug_out/routes/farm_seed_1_120m2"))
    parser.add_argument("--seed", type=int, default=17)
    parser.add_argument("--cases", type=int, choices=(1, 2, 3), default=3,
                        help="number of seeded showcase tasks to plan")
    parser.add_argument("--case-index", type=int, choices=(1, 2, 3),
                        help="plan only this seeded showcase task (within --cases)")
    parser.add_argument("--solver-time", type=float,
                        help="override per-case limits (default: 60/90/60 seconds)")
    parser.add_argument("--reverse-multiplier", type=float, default=8.0)
    parser.add_argument("--reverse-event-penalty", type=float, default=100.0,
                        help="equivalent metres added per reverse episode in an RS path")
    parser.add_argument("--row-repeat-penalty", type=float, default=50.0,
                        help="equivalent metres added per excess physical crop-lane traversal")
    parser.add_argument("--field-rs-forward-only", action="store_true",
                        help="discard field RS connections containing any reverse motion")
    parser.add_argument("--road-turn-setback", type=float, default=2.7,
                        help="distance from each road junction to a turn state, in metres")
    parser.add_argument("--road-interface-offset", type=float, default=2.7,
                        help="road sample offset from each row endpoint projection, in metres")
    parser.add_argument("--turn-radius-step", type=float, default=0.35,
                        help="radius increment for additional RS candidates, in metres")
    parser.add_argument("--turn-radius-expansions", type=int, default=2,
                        help="number of radii above the tractor's minimum radius")
    parser.add_argument("--tractor-config", type=Path, default=DEFAULT_TRACTOR_CONFIG_PATH)
    parser.add_argument("--gear-change-penalty", type=float, default=2.0,
                        help="Penalty per forward/reverse shift, in equivalent meters")
    parser.add_argument("--implement-side", choices=[side.value for side in ImplementSide], default="both")
    args = parser.parse_args()
    config = PlannerConfig(solver_time_s=60.0,
                           reverse_cost_multiplier=args.reverse_multiplier,
                           reverse_event_penalty_m=args.reverse_event_penalty,
                           row_repeat_penalty_m=args.row_repeat_penalty,
                           field_rs_forward_only=args.field_rs_forward_only,
                           road_turn_setback_m=args.road_turn_setback,
                           road_interface_offset_m=args.road_interface_offset,
                           turn_radius_step_m=args.turn_radius_step,
                           turn_radius_expansions=args.turn_radius_expansions,
                           gear_change_penalty_m=args.gear_change_penalty,
                           tractor_config_path=args.tractor_config)
    footprint = TractorFootprint.from_config(config.tractor)
    farm = load_farm_map(args.farm)
    tasks = sample_tasks(farm, args.seed, ImplementSide(args.implement_side))
    if args.case_index is not None and args.case_index > args.cases:
        parser.error("--case-index must be within --cases")
    output = args.output_dir
    output.mkdir(parents=True, exist_ok=True)
    manifest = []
    for index, task in enumerate(tasks[:args.cases], 1):
        if args.case_index is not None and index != args.case_index:
            continue
        case_config = replace(config, solver_time_s=solver_time_for_case(index, args.solver_time))
        try:
            route = plan_route(farm, task, case_config)
        except (ValueError, RuntimeError) as exc:
            raise RuntimeError(f"Mission {index} from seed {args.seed} could not be planned") from exc
        if _has_in_place_reversal(route):
            raise RuntimeError(f"Mission {index} from seed {args.seed} has a road U-turn")
        stem = f"route_{index:02d}"
        save_route_json(route, farm, output / f"{stem}.json")
        save_route_plot(route, farm, output / f"{stem}.png")
        manifest.append({"name": stem, "seed": args.seed, "fields": task.field_ids,
                         "start": vars(task.start), "end": vars(task.end),
                         "distance_m": route.total_distance_m,
                         "reverse_distance_m": sum(s.reverse_distance_m for s in route.segments),
                         "gear_changes": sum(s.gear_changes for s in route.segments),
                         "reverse_events": sum(s.reverse_events for s in route.segments),
                         "reverse_cost_multiplier": route.reverse_cost_multiplier,
                         "gear_change_penalty_m": route.gear_change_penalty_m,
                         "reverse_event_penalty_m": route.reverse_event_penalty_m,
                         "row_repeat_penalty_m": route.row_repeat_penalty_m,
                         "row_repeat_count": route.row_repeat_count,
                         "field_rs_forward_only": config.field_rs_forward_only,
                         "tractor_config_path": str(config.tractor_config_path),
                         "turn_radius_m": config.turn_radius_m,
                         "turn_radius_step_m": config.turn_radius_step_m,
                         "turn_radius_expansions": config.turn_radius_expansions,
                         "road_turn_setback_m": config.road_turn_setback_m,
                         "road_interface_offset_m": config.road_interface_offset_m,
                         "footprint_bounds_rear_axle_m": footprint.at(Pose(0, 0, 0)).bounds,
                         "status": route.optimality_status,
                         "objective_cost": route.objective_cost,
                         "best_bound_cost": route.best_bound_cost,
                         "solver_wall_time_s": route.solver_wall_time_s,
                         "solver_time_limit_s": case_config.solver_time_s,
                         "segments": len(route.segments),
                         "saturated_arc_ids": route.saturated_arc_ids})
        print(f"{stem}: {task.field_ids}, {route.total_distance_m:.1f} m total, "
              f"{sum(s.reverse_distance_m for s in route.segments):.1f} m reverse, "
              f"{route.row_repeat_count} row repeats, {route.optimality_status}, "
              f"{len(route.segments)} segments", flush=True)
    manifest_path = output / "manifest.json"
    if args.case_index is not None and manifest_path.exists():
        previous = json.loads(manifest_path.read_text())
        current_names = {entry["name"] for entry in manifest}
        manifest = sorted([entry for entry in previous if entry["name"] not in current_names] +
                          manifest, key=lambda entry: entry["name"])
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")


if __name__ == "__main__":
    main()
