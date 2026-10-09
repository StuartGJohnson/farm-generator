"""CP-SAT arc multiplicity model; no GIS or path-geometry calculations."""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass

from ortools.sat.python import cp_model

from .model import Arc, PlannerConfig, RoutingGraph


def arc_cost(arc: Arc, config: PlannerConfig) -> int:
    """Quantized travel cost, with reverse travel and shifts priced explicitly."""
    effective_distance = (arc.distance_m +
                          (config.reverse_cost_multiplier - 1) * arc.reverse_distance_m +
                          config.gear_change_penalty_m * arc.gear_changes)
    return max(1, round(config.cost_scale_per_m * effective_distance))


@dataclass(frozen=True)
class TraversalSolution:
    counts: dict[str, int]
    objective_cost: int
    status: str
    saturated_arc_ids: list[str]
    best_bound_cost: float
    solver_wall_time_s: float


def optimize_arc_counts(graph: RoutingGraph,
                        config: PlannerConfig = PlannerConfig()) -> TraversalSolution:
    if not graph.requirements:
        raise ValueError("Routing graph has no service requirements")
    if graph.start_id not in graph.vertices or graph.end_id not in graph.vertices:
        raise ValueError("Routing graph lacks start or end vertex")
    arcs = list(graph.arcs.values())
    vertices = list(graph.vertices)
    outgoing = defaultdict(list)
    incoming = defaultdict(list)
    for arc in arcs:
        outgoing[arc.u].append(arc)
        incoming[arc.v].append(arc)
    for requirement in graph.requirements:
        if not any(requirement in arc.service_ids for arc in arcs):
            raise ValueError(f"No arc can service {requirement}")
    model = cp_model.CpModel()
    default_limit = 2 * len(graph.requirements) + 2
    limits = {a.id: a.max_uses or config.max_arc_uses or default_limit for a in arcs}
    x = {a.id: model.new_int_var(0, limits[a.id], f"x_{a.id}") for a in arcs}
    for requirement in graph.requirements:
        model.add(sum(x[a.id] for a in arcs if requirement in a.service_ids) >= 1)
    for vertex in vertices:
        rhs = (1 if vertex == graph.start_id else -1 if vertex == graph.end_id else 0)
        if graph.start_id == graph.end_id:
            rhs = 0
        model.add(sum(x[a.id] for a in outgoing[vertex]) -
                  sum(x[a.id] for a in incoming[vertex]) == rhs)
    z = {vertex: model.new_bool_var(f"z_{vertex}") for vertex in vertices}
    for arc in arcs:
        model.add(x[arc.id] <= limits[arc.id] * z[arc.u])
        model.add(x[arc.id] <= limits[arc.id] * z[arc.v])
    for vertex in vertices:
        model.add(z[vertex] <= sum(x[a.id] for a in outgoing[vertex] + incoming[vertex]))
    model.add(z[graph.start_id] == 1)
    if graph.start_id != graph.end_id:
        model.add(z[graph.end_id] == 1)
    flow_limit = len(vertices) - 1
    flow = {a.id: model.new_int_var(0, flow_limit, f"flow_{a.id}") for a in arcs}
    for arc in arcs:
        model.add(flow[arc.id] <= flow_limit * x[arc.id])
    for vertex in vertices:
        inflow = sum(flow[a.id] for a in incoming[vertex])
        outflow = sum(flow[a.id] for a in outgoing[vertex])
        if vertex == graph.start_id:
            model.add(outflow - inflow == sum(z[v] for v in vertices if v != vertex))
        else:
            model.add(inflow - outflow == z[vertex])
    costs = {a.id: arc_cost(a, config) for a in arcs}
    model.minimize(sum(costs[a.id] * x[a.id] for a in arcs))
    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = config.solver_time_s
    solver.parameters.num_search_workers = 8
    solver.parameters.random_seed = 1
    status = solver.solve(model)
    if status not in (cp_model.FEASIBLE, cp_model.OPTIMAL):
        raise RuntimeError(f"No farm route found (CP-SAT status: {solver.status_name(status)})")
    counts = {a.id: solver.value(x[a.id]) for a in arcs if solver.value(x[a.id]) > 0}
    saturated = [arc_id for arc_id, count in counts.items()
                 if count == limits[arc_id] and not arc_id.startswith("terminal_")]
    return TraversalSolution(counts, int(solver.objective_value),
                             solver.status_name(status), saturated,
                             solver.best_objective_bound, solver.wall_time)
