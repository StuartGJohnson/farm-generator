"""Expand a movement graph into ordered field-service stages."""

from __future__ import annotations

from dataclasses import replace

from .model import Arc, FarmTask, RoutingGraph


def ordered_field_graph(base: RoutingGraph, task: FarmTask) -> RoutingGraph:
    """Allow service of field i only before the switch to field i + 1.

    A stage switch changes the routing state at an identical road pose. It is
    bookkeeping, not tractor motion, and can be used once at a road state.
    """
    if len(task.field_ids) <= 1:
        return base

    stages = len(task.field_ids)
    graph = RoutingGraph(requirements=set(base.requirements),
                         start_id=f"{base.start_id}@0",
                         end_id=f"{base.end_id}@{stages - 1}",
                         selected_fields=set(base.selected_fields))

    def add_vertex(base_id: str, stage: int) -> str:
        identifier = f"{base_id}@{stage}"
        graph.vertices[identifier] = replace(base.vertices[base_id], id=identifier)
        return identifier

    for stage, field_id in enumerate(task.field_ids):
        for arc in base.arcs.values():
            if (arc.id == "terminal_start_arc" or arc.id.startswith("depot_start_")) and stage != 0:
                continue
            if (arc.id == "terminal_end_arc" or arc.id.startswith("depot_end_")) and stage != stages - 1:
                continue
            if arc.kind in ("row", "rs") and arc.field_id != field_id:
                continue
            u = add_vertex(arc.u, stage)
            v = add_vertex(arc.v, stage)
            copy = replace(arc, id=f"{arc.id}@{stage}", u=u, v=v)
            graph.arcs[copy.id] = copy

    for stage in range(stages - 1):
        field_id = task.field_ids[stage]
        exit_states = sorted({arc.v for arc in base.arcs.values()
                              if arc.kind == "rs" and arc.field_id == field_id
                              and base.vertices[arc.u].field_id == field_id
                              and base.vertices[arc.v].field_id is None})
        for base_id in exit_states:
            vertex = base.vertices[base_id]
            u, v = f"{base_id}@{stage}", f"{base_id}@{stage + 1}"
            if u not in graph.vertices or v not in graph.vertices:
                continue
            arc_id = f"field_switch_{base_id}@{stage}"
            graph.arcs[arc_id] = Arc(arc_id, u, v, (vertex.pose,), 0.0,
                                     "field_switch", task.field_ids[stage + 1],
                                     max_uses=1)
    if not any(arc.kind == "field_switch" for arc in graph.arcs.values()):
        raise ValueError("No road state connects ordered field stages")
    return graph
