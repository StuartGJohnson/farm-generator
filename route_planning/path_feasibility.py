"""Swept tractor-envelope checks for rear-axle-centered movement paths."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

from shapely.geometry import LineString, MultiPoint, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.strtree import STRtree
from tractor_generation.config import TractorConfig

from .model import FarmMap, Pose


def tree_convex_hulls(farm: FarmMap, field_ids: Sequence[str]) -> dict[str, BaseGeometry]:
    """Approximate each cultivated block, inset 1 µm to leave its edge usable."""
    return {fid: MultiPoint([(tree.x, tree.y) for tree in farm.trees.values()
                             if tree.field_id == fid]).convex_hull.buffer(-1e-6)
            for fid in field_ids}


@dataclass(frozen=True)
class TractorFootprint:
    """Convex outer envelope of body and straight-ahead tires, at the rear axle."""

    vertices: tuple[tuple[float, float], ...]

    @classmethod
    def from_config(cls, tractor: TractorConfig) -> "TractorFootprint":
        points = []

        def add_box(center_x: float, length: float, center_y: float, width: float) -> None:
            for x in (center_x - length / 2, center_x + length / 2):
                for y in (center_y - width / 2, center_y + width / 2):
                    points.append((x - tractor.rear_axle_x, y))

        for center_x, length, width in tractor.body_segments:
            add_box(center_x, length, 0.0, width)
        for axle_x, track, diameter, width in (
            (tractor.front_axle_x, tractor.front_track_width,
             tractor.front_tire_dia, tractor.front_tire_width),
            (tractor.rear_axle_x, tractor.rear_track_width,
             tractor.rear_tire_dia, tractor.rear_tire_width),
        ):
            for side in (-1, 1):
                add_box(axle_x, diameter, side * track / 2, width)
        hull = MultiPoint(points).convex_hull
        return cls(tuple(hull.exterior.coords[:-1]))

    def at(self, pose: Pose) -> Polygon:
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        return Polygon([(pose.x + c * x - s * y, pose.y + s * x + c * y)
                        for x, y in self.vertices])


def path_feasibility(poses: Sequence[Pose], allowed_region: BaseGeometry,
                     tree_obstacles: Sequence[Polygon], footprint: TractorFootprint,
                     tree_index: STRtree | None = None,
                     tree_hull: BaseGeometry | None = None) -> bool:
    """Check the swept footprint, including outer corners between RS samples.

    Input poses locate the non-steering rear axle. The rear-axle trace must
    stay outside the interior of the tree hull; the footprint may overlap
    the hull at a row endpoint as the tractor enters or leaves that row.
    The tractor geometry is
    linearly interpolated to at most 0.1 m translation or 0.04 rad heading per
    check. A 1 cm buffer around each consecutive-pose convex hull covers the
    small curvature between those checks and gives a conservative test.
    """
    if len(poses) < 2 or allowed_region.is_empty:
        return False
    centerline = LineString([(pose.x, pose.y) for pose in poses])
    if not allowed_region.covers(centerline):
        return False
    if tree_hull is not None and not tree_hull.is_empty and centerline.intersects(tree_hull):
        return False
    index = tree_index if tree_index is not None else STRtree(tree_obstacles)

    def clear(shape: BaseGeometry) -> bool:
        return (allowed_region.covers(shape) and
                not any(shape.intersects(tree_obstacles[int(i)])
                        for i in index.query(shape)))

    previous = footprint.at(poses[0])
    if not clear(previous):
        return False
    for start, end in zip(poses, poses[1:]):
        delta_yaw = (end.yaw - start.yaw + math.pi) % (2 * math.pi) - math.pi
        distance = math.hypot(end.x - start.x, end.y - start.y)
        steps = max(1, math.ceil(distance / 0.1), math.ceil(abs(delta_yaw) / 0.04))
        for step in range(1, steps + 1):
            fraction = step / steps
            pose = Pose(start.x + fraction * (end.x - start.x),
                        start.y + fraction * (end.y - start.y),
                        start.yaw + fraction * delta_yaw)
            current = footprint.at(pose)
            swept = MultiPoint((*previous.exterior.coords[:-1],
                                *current.exterior.coords[:-1])).convex_hull.buffer(0.01)
            if not clear(swept):
                return False
            previous = current
    return True
