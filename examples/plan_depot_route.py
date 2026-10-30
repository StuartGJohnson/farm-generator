"""Plan fixed two-spot depot tasks from the generated 120 m showcase bundle."""

from pathlib import Path
import json
import argparse

from route_planning.diagnostic import load_route_json, save_route_json, save_route_plot
from route_planning.gis import load_farm_map
from route_planning.model import DepotParkingState, FarmTask, ImplementSide, PlannerConfig
from route_planning.planner import plan_route


CASES = {
    1: ("parcel_000",),
    2: ("parcel_003",),  # The field containing the depot.
    3: ("parcel_000", "parcel_002"),  # Neither field contains the depot.
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", type=int, choices=CASES,
                        help="run one case (default: all three)")
    parser.add_argument("--reuse-route", action="store_true",
                        help="regenerate plots from the saved route without optimization")
    parser.add_argument("--outlands", action="store_true",
                        help="use the depot farm with outlands and write routes separately")
    parser.add_argument("--output-dir", type=Path,
                        help="override the route output directory")
    args = parser.parse_args()
    stem = "farm_seed_1_120m2_depot" + ("_outlands" if args.outlands else "")
    farm_dir = Path("debug_out/farms") / stem
    farm = load_farm_map(farm_dir)
    if farm.depot_field_id != "parcel_003":
        raise ValueError("Seed 1 depot is expected in parcel_003; regenerate the showcase farm")
    output = args.output_dir or Path("debug_out/routes") / stem
    output.mkdir(parents=True, exist_ok=True)
    x0, y0, x1, y1 = farm.depot.bounds
    for number in ((args.case,) if args.case else (1, 2, 3)):
        stem = f"route_{number:02d}"
        if (number == 1 and args.case is None and not args.reuse_route
                and (output / f"{stem}.json").exists()):
            continue  # Keep the original, previously reviewed route.
        task = FarmTask(CASES[number], ImplementSide.BOTH,
                        DepotParkingState(1, "in"), DepotParkingState(2, "in"))
        config = PlannerConfig(solver_time_s={1: 60, 2: 90, 3: 120}[number])
        route = (load_route_json(output / f"{stem}.json") if args.reuse_route
                 else plan_route(farm, task, config))
        if not args.reuse_route:
            save_route_json(route, farm, output / f"{stem}.json")
        save_route_plot(route, farm, output / f"{stem}.png")
        save_route_plot(route, farm, output / f"{stem}_depot_zoom.png",
                        bounds=(x0 - 18, y0 - 10, x1 + 12, y1 + 8))
        depot_segments = [s for s in route.segments if s.kind == "depot"]
        manifest = {
            "farm": str(farm_dir),
            "fields": task.field_ids,
            "start_spot": 1,
            "end_spot": 2,
            "parking_direction": "in",
            "solver_time_limit_s": config.solver_time_s,
            "depot_reverse_cost_multiplier": config.depot_reverse_cost_multiplier,
            "depot_reverse_allowance_m": config.depot_reverse_allowance_m,
            "depot_excess_reverse_cost_multiplier": config.depot_excess_reverse_cost_multiplier,
            "depot_parking_clearance_m": config.depot_parking_clearance_m,
            "status": route.optimality_status,
            "distance_m": route.total_distance_m,
            "reverse_distance_m": sum(s.reverse_distance_m for s in route.segments),
            "depot_reverse_distances_m": [s.reverse_distance_m for s in depot_segments],
            "row_repeat_count": route.row_repeat_count,
            "solver_wall_time_s": route.solver_wall_time_s,
            "objective_cost": route.objective_cost,
            "best_bound_cost": route.best_bound_cost,
        }
        (output / f"{stem}_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
        if number == 1:
            (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
        print(f"{stem}: {task.field_ids}, {route.optimality_status}, "
              f"{route.total_distance_m:.1f} m, "
              f"{manifest['reverse_distance_m']:.1f} m reverse", flush=True)


if __name__ == "__main__":
    main()
