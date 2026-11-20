"""Distance-based route sections and forward-only pose projection."""

from __future__ import annotations

import math
from bisect import bisect_left, bisect_right
from dataclasses import dataclass

from route_planning.model import Pose, Route


@dataclass(frozen=True)
class Section:
    start_m: float
    end_m: float
    direction: int
    region: str


def _distance(a: Pose, b: Pose) -> float:
    return math.hypot(b.x - a.x, b.y - a.y)


class AnnotatedRoute:
    def __init__(self, route: Route, strategy: str, by_length_m: float):
        if strategy not in ("by_length", "by_region", "by_activity"):
            raise ValueError(f"Unknown mission segmentation strategy {strategy}")
        if by_length_m <= 0:
            raise ValueError("by_length_m must be positive")
        self.poses: list[Pose] = []
        self.stations: list[float] = []
        self.source_segments = tuple(route.segments)
        self.edge_segment_indices: list[int] = []
        intervals = []
        for segment_index, segment in enumerate(route.segments):
            if not segment.poses:
                continue
            directions = (segment.driving_directions[1:] if segment.driving_directions
                          else (1,) * (len(segment.poses) - 1))
            if len(directions) != len(segment.poses) - 1:
                raise ValueError("Driving directions must match route poses")
            region = ("field" if segment.kind in ("row", "field_switch", "headland", "rs")
                      else "depot" if segment.kind == "depot" else "road")
            if not self.poses:
                self.poses.append(segment.poses[0])
                self.stations.append(0.0)
            elif _distance(self.poses[-1], segment.poses[0]) > 0.05:
                raise ValueError("Route segments are disconnected")
            for index, direction in enumerate(directions):
                start = self.stations[-1]
                end = start + _distance(segment.poses[index], segment.poses[index + 1])
                if end <= start + 1e-9:
                    continue
                intervals.append((start, end, 1 if direction > 0 else -1,
                                  region if strategy == "by_region" else
                                  segment.kind if strategy == "by_activity" else "route",
                                  segment_index if strategy == "by_activity" else 0))
                self.poses.append(segment.poses[index + 1])
                self.stations.append(end)
                self.edge_segment_indices.append(segment_index)
        if not intervals:
            raise ValueError("Route has no traversable path")
        self.length_m = self.stations[-1]
        groups = []
        for start, end, direction, region, activity in intervals:
            key = (direction, region, activity)
            if groups and groups[-1][3] == key and abs(groups[-1][1] - start) < 1e-5:
                groups[-1] = (groups[-1][0], end, direction, key)
            else:
                groups.append((start, end, direction, key))
        sections = []
        for start, end, direction, key in groups:
            if strategy == "by_length":
                cursor = start
                while cursor < end - 1e-6:
                    next_end = min(cursor + by_length_m, end)
                    sections.append(Section(cursor, next_end, direction, key[1]))
                    cursor = next_end
            else:
                sections.append(Section(start, end, direction, key[1]))
        self.sections = sections

    def pose_at(self, station: float) -> Pose:
        station = min(max(station, 0.0), self.length_m)
        index = min(len(self.stations) - 2, max(0, bisect_right(self.stations, station) - 1))
        a, b = self.stations[index:index + 2]
        return self._pose_on_edge(index, (station - a) / (b - a))

    def _pose_on_edge(self, index: int, t: float) -> Pose:
        p, q = self.poses[index:index + 2]
        if t <= 0:
            return p
        if t >= 1:
            return q
        yaw_delta = (q.yaw - p.yaw + math.pi) % (2 * math.pi) - math.pi
        return Pose(p.x + t * (q.x - p.x), p.y + t * (q.y - p.y),
                    p.yaw + t * yaw_delta)

    def segment_at(self, station: float):
        """Return the original RouteSegment metadata at a route station."""
        index = min(len(self.edge_segment_indices) - 1,
                    max(0, bisect_right(self.stations, station) - 1))
        return self.source_segments[self.edge_segment_indices[index]]

    def slice(self, start: float, end: float, spacing_m: float = 0.2) -> list[Pose]:
        if end <= start:
            raise ValueError("Empty path slice")
        if not math.isfinite(spacing_m) or spacing_m <= 0:
            raise ValueError("Path pose spacing must be positive and finite")
        if not math.isfinite(start) or not math.isfinite(end) or start < 0 or end > self.length_m + 1e-6:
            raise ValueError("Path slice must lie within the route")
        first_edge = max(0, bisect_right(self.stations, start) - 1)
        last_edge = min(len(self.poses) - 2, bisect_left(self.stations, end) - 1)
        sampled = [self.pose_at(start)]
        for edge_index in range(first_edge, last_edge + 1):
            a, b = self.stations[edge_index:edge_index + 2]
            count = math.ceil((b - a) / spacing_m)
            step = (b - a) / count
            # Anchor fractions to the original edge, even when this goal starts
            # partway through it. Only generate points inside this goal's window.
            first_i = max(1, math.floor((start - a) / step))
            last_i = min(count, math.ceil((end - a) / step))
            for i in range(first_i, last_i + 1):
                station = b if i == count else a + step * i
                if start + 1e-6 < station < end - 1e-6:
                    sampled.append(self._pose_on_edge(edge_index, i / count))
        sampled.append(self.pose_at(end))
        return sampled


class RouteProgressTracker:
    def __init__(self, route: AnnotatedRoute, forward_search_m: float = 10.0):
        self.route = route
        self.station_m = 0.0
        self.forward_search_m = forward_search_m

    def update(self, x: float, y: float) -> float:
        best_distance = math.inf
        best_station = self.station_m
        hi = min(self.route.length_m, self.station_m + self.forward_search_m)
        first = max(0, bisect_left(self.route.stations, self.station_m) - 1)
        last = min(len(self.route.poses) - 1, bisect_right(self.route.stations, hi))
        for index in range(first, last):
            lo, end = self.route.stations[index:index + 2]
            if end < self.station_m or lo > hi:
                continue
            a, b = self.route.poses[index:index + 2]
            dx, dy = b.x - a.x, b.y - a.y
            fraction = max(0.0, min(1.0, ((x - a.x) * dx + (y - a.y) * dy) / (dx * dx + dy * dy)))
            station = max(self.station_m, min(hi, lo + fraction * (end - lo)))
            p = self.route.pose_at(station)
            distance = math.hypot(x - p.x, y - p.y)
            if distance < best_distance:
                best_distance, best_station = distance, station
        if best_distance < 3.0:
            self.station_m = best_station
        return self.station_m
