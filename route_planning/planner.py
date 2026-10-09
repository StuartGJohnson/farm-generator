"""Public orchestration of graph construction, optimization, and validation."""

from dataclasses import replace

from .graph import build_routing_graph, infer_implement_range
from .model import FarmMap, FarmTask, PlannerConfig, Route
from .optimize import optimize_arc_counts
from .reconstruct import reconstruct_route, validate_route


def plan_route(farm: FarmMap, task: FarmTask,
               config: PlannerConfig = PlannerConfig()) -> Route:
    if task.implement_range_m is None:
        task = replace(task, implement_range_m=infer_implement_range(farm, task.field_ids))
    graph = build_routing_graph(farm, task, config)
    counts = optimize_arc_counts(graph, config)
    route = reconstruct_route(graph, task, counts, config)
    validate_route(route, graph, farm, config)
    return route
