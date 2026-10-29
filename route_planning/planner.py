"""Public orchestration of graph construction, optimization, and validation."""

from dataclasses import replace

from .graph import build_routing_graph, infer_implement_range
from .model import DepotParkingState, FarmMap, FarmTask, PlannerConfig, Route, parking_pose
from .ordering import ordered_field_graph
from .optimize import optimize_arc_counts
from .reconstruct import reconstruct_route, validate_route


def plan_route(farm: FarmMap, task: FarmTask,
               config: PlannerConfig = PlannerConfig()) -> Route:
    def resolve(value):
        if isinstance(value, DepotParkingState):
            if value.number not in farm.parking_spots:
                raise ValueError(f"Unknown depot parking spot {value.number}")
            return parking_pose(farm.parking_spots[value.number], value.direction)
        return value
    task = replace(task, start=resolve(task.start), end=resolve(task.end))
    if task.implement_range_m is None:
        task = replace(task, implement_range_m=infer_implement_range(farm, task.field_ids))
    graph = ordered_field_graph(build_routing_graph(farm, task, config), task)
    counts = optimize_arc_counts(graph, config)
    route = reconstruct_route(graph, task, counts, config)
    validate_route(route, graph, farm, config)
    return route
