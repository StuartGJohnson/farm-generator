"""Reeds–Shepp candidate geometry and field legality checks."""

from __future__ import annotations

import math

from rsplan import planner as rsplanner
from shapely.geometry import Polygon
from shapely.geometry.base import BaseGeometry
from shapely.strtree import STRtree

from .model import PlannerConfig, Pose
from .path_feasibility import TractorFootprint, path_feasibility


def connection(start: Pose, end: Pose, field_polygon: Polygon,
               tree_obstacles: list[Polygon], config: PlannerConfig,
               tree_index: STRtree | None = None, *,
               permitted_region: BaseGeometry | None = None,
               footprint: TractorFootprint | None = None,
               tree_hull: BaseGeometry | None = None,
               forward_only: bool = False) -> tuple[tuple[Pose, ...], float, tuple[int, ...], float, int, int] | None:
    """Return the lowest penalized-cost legal RS candidate, if one exists."""
    if math.hypot(end.x - start.x, end.y - start.y) > config.rs_candidate_radius_m:
        return None
    # The configured radius is a minimum: a wider turn is legal too. Analytic
    # RS families are generated for one exact radius at a time, and an obstacle
    # can invalidate every path at the minimum while leaving a wider turn clear.
    # Sample configured wider radii, then score all legal paths together.
    paths = []
    for expansion in range(config.turn_radius_expansions + 1):
        try:
            paths.extend(rsplanner._solve_path(
                (start.x, start.y, start.yaw), (end.x, end.y, end.yaw),
                config.turn_radius_m + expansion * config.turn_radius_step_m,
                config.rs_step_m))
        except (ValueError, ZeroDivisionError, IndexError):
            continue
    if forward_only:
        paths = [path for path in paths if all(segment.direction > 0
                                               for segment in path.segments)]
    def metrics(path):
        reverse_m = sum(abs(segment.length) for segment in path.segments
                        if segment.direction < 0)
        gear_changes = sum(a.direction != b.direction
                           for a, b in zip(path.segments, path.segments[1:]))
        reverse_events = sum(segment.direction < 0 and
                             (index == 0 or path.segments[index - 1].direction > 0)
                             for index, segment in enumerate(path.segments))
        score = (path.total_length +
                 (config.reverse_cost_multiplier - 1) * reverse_m +
                 config.gear_change_penalty_m * gear_changes +
                 config.reverse_event_penalty_m * reverse_events)
        return score, reverse_m, gear_changes, reverse_events

    ranked = sorted(((metrics(path), path) for path in paths),
                    key=lambda item: (item[0][0], item[1].total_length))
    permitted = permitted_region if permitted_region is not None else field_polygon
    envelope = footprint if footprint is not None else TractorFootprint.from_config(config.tractor)
    index = tree_index if tree_index is not None else STRtree(tree_obstacles)
    for (_, reverse_m, gear_changes, reverse_events), path in ranked:
        if path.total_length <= 1e-5:
            continue
        try:
            waypoints = path.waypoints()
        except (ValueError, ZeroDivisionError, IndexError):
            continue
        if len(waypoints) < 2:
            continue
        poses = tuple(Pose(w.x, w.y, w.yaw) for w in waypoints)
        if not path_feasibility(poses, permitted, tree_obstacles, envelope, index,
                                tree_hull):
            continue
        return ((start, *poses[1:-1], end), path.total_length,
                tuple(w.driving_direction for w in waypoints),
                reverse_m, gear_changes, reverse_events)
    return None
