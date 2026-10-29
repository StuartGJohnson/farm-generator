"""Swept tractor-envelope checks for rear-axle-centered movement paths."""

from __future__ import annotations

import math
from collections import defaultdict
from dataclasses import dataclass
from statistics import median
from typing import Sequence

from shapely.geometry import LineString, MultiPoint, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import triangulate, unary_union
from shapely.strtree import STRtree
from tractor_generation.config import TractorConfig

from .model import FarmMap, Pose


def tree_concave_hulls(farm: FarmMap, field_ids: Sequence[str],
                       concavity_m: float | None = None) -> dict[str, BaseGeometry]:
    """Bound planted blocks with an alpha shape using a metre-scale radius.

    A triangle is included when its circumradius is at most ``concavity_m``.
    By default that radius is 1.5 times the median same-row tree spacing in
    each field. The small inset leaves row endpoints on the usable boundary.
    """
    if concavity_m is not None and (not math.isfinite(concavity_m) or concavity_m <= 0):
        raise ValueError("concavity_m must be positive and finite")
    result = {}
    for fid in field_ids:
        trees = [tree for tree in farm.trees.values() if tree.field_id == fid]
        points = [(tree.x, tree.y) for tree in trees]
        if len(points) < 3:
            result[fid] = MultiPoint(points).convex_hull.buffer(-1e-6)
            continue
        by_row = defaultdict(list)
        for tree in trees:
            by_row[tree.row_id].append((tree.x, tree.y))
        spacings = [min(math.dist(point, other) for other in row if other != point)
                    for row in by_row.values() if len(row) > 1 for point in row]
        if not spacings:
            spacings = [min(math.dist(point, other) for other in points if other != point)
                        for point in points]
        radius = concavity_m if concavity_m is not None else 1.5 * median(spacings)
        kept = []
        for triangle in triangulate(MultiPoint(points)):
            a, b, c = list(triangle.exterior.coords)[:3]
            area = triangle.area
            if area <= 1e-10:
                continue
            circumradius = math.dist(a, b) * math.dist(b, c) * math.dist(c, a) / (4 * area)
            if circumradius <= radius:
                kept.append(triangle)
        result[fid] = unary_union(kept).buffer(-1e-6) if kept else MultiPoint(points).convex_hull.buffer(-1e-6)
    return result


def road_turn_region(farm: FarmMap) -> BaseGeometry:
    """Land and roads outside channels, bridged only by crossing surfaces."""
    dry_roads = [surface for road_id, surface in farm.road_surfaces.items()
                 if road_id not in farm.roads or farm.roads[road_id].road_class != "crossing_spur"]
    crossings = [surface for road_id, surface in farm.road_surfaces.items()
                 if road_id in farm.roads and farm.roads[road_id].road_class == "crossing_spur"]
    dry_land = unary_union([*farm.fields.values(), *dry_roads])
    channels = unary_union(list(farm.irrigation_channels.values()))
    return dry_land.difference(channels).union(unary_union(crossings))


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
