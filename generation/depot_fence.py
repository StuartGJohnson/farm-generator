"""Lay out nominal 2 m fence sections on the depot's field-facing edges."""

from __future__ import annotations

import math

import numpy as np

from farm_ir.schema import FarmScene


FENCE_SECTION_LENGTH_M = 2.0


def run(scene: FarmScene, config) -> FarmScene:
    depot = scene.depot
    if depot is None or not config.include_depot_fence:
        return scene

    origin = np.asarray(depot.origin, dtype=float)
    row = np.asarray(depot.row_direction, dtype=float)
    cross = np.asarray(depot.cross_row_direction, dtype=float)
    corner = (origin + depot.along_row_length_m * row
              + depot.cross_row_width_m * cross)
    depot.fence_lines = []
    for direction, span in ((row, depot.along_row_length_m),
                            (cross, depot.cross_row_width_m)):
        count = math.floor((span + 1e-8) / FENCE_SECTION_LENGTH_M)
        if count == 0:
            continue
        end = corner - count * FENCE_SECTION_LENGTH_M * direction
        depot.fence_lines.append([
            (float(corner[0]), float(corner[1])),
            (float(end[0]), float(end[1])),
        ])
    return scene


def fence_placements(lines):
    """Return unique post XY positions and panel XY/yaw placements."""
    posts = []
    panels = []
    seen = set()
    for start, end in lines:
        start = np.asarray(start, dtype=float)
        end = np.asarray(end, dtype=float)
        displacement = end - start
        length = float(np.linalg.norm(displacement))
        count = round(length / FENCE_SECTION_LENGTH_M)
        if count < 1 or abs(length - count * FENCE_SECTION_LENGTH_M) > 1e-6:
            raise ValueError("depot fence lines must contain whole 2 m sections")
        direction = displacement / length
        yaw = math.degrees(math.atan2(direction[1], direction[0]))
        for index in range(count + 1):
            point = start + index * FENCE_SECTION_LENGTH_M * direction
            xy = (float(point[0]), float(point[1]))
            key = (round(xy[0], 8), round(xy[1], 8))
            if key not in seen:
                posts.append(xy)
                seen.add(key)
            if index < count:
                panels.append((xy[0], xy[1], yaw))
    return posts, panels
