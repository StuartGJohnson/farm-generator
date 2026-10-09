"""Data contracts independent of GDAL, OR-Tools, and ROS."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Optional

from shapely.geometry import LineString, Polygon
from tractor_generation.config import TractorConfig, load_tractor_config


DEFAULT_TRACTOR_CONFIG_PATH = Path(__file__).resolve().parents[1] / "configs" / "tractor_default.yaml"


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float


class ImplementSide(str, Enum):
    LEFT = "left"
    RIGHT = "right"
    BOTH = "both"


@dataclass(frozen=True)
class FarmTask:
    field_ids: tuple[str, ...]
    implement_side: ImplementSide
    start: Pose
    end: Pose
    implement_range_m: Optional[float] = None


@dataclass(frozen=True)
class PlannerConfig:
    tractor_config_path: str | Path = DEFAULT_TRACTOR_CONFIG_PATH
    turn_radius_m: Optional[float] = None
    rs_step_m: float = 0.35
    tractor_half_width_m: Optional[float] = None
    trunk_clearance_m: float = 0.2
    rs_candidate_radius_m: float = 15.0
    road_interface_candidates: int = 2
    max_arc_uses: Optional[int] = None
    solver_time_s: float = 60.0
    cost_scale_per_m: int = 100
    endpoint_tolerance_m: float = 0.05
    road_pose_tolerance_m: float = 0.05
    reverse_cost_multiplier: float = 8.0
    gear_change_penalty_m: float = 2.0
    tractor: TractorConfig = field(init=False, repr=False, compare=False)

    def __post_init__(self) -> None:
        tractor = load_tractor_config(self.tractor_config_path)
        object.__setattr__(self, "tractor", tractor)
        if self.turn_radius_m is None:
            object.__setattr__(self, "turn_radius_m", tractor.turning_radius)
        if self.tractor_half_width_m is None:
            half_width = max(tractor.tractor_body_width / 2,
                             tractor.front_track_width / 2 + tractor.front_tire_width / 2,
                             tractor.rear_track_width / 2 + tractor.rear_tire_width / 2)
            object.__setattr__(self, "tractor_half_width_m", half_width)


@dataclass(frozen=True)
class Road:
    id: str
    line: LineString
    node_a: str
    node_b: str
    road_class: str
    field_id: Optional[str]
    field_a: Optional[str]
    field_b: Optional[str]


@dataclass(frozen=True)
class Row:
    id: str
    field_id: str
    line: Optional[LineString]
    tree_count: int


@dataclass(frozen=True)
class Tree:
    id: str
    field_id: str
    row_id: str
    x: float
    y: float


@dataclass
class FarmMap:
    fields: dict[str, Polygon]
    field_names: dict[str, str]
    roads: dict[str, Road]
    rows: dict[str, Row]
    trees: dict[str, Tree]
    origin_easting_m: float
    origin_northing_m: float
    orthophoto_path: Optional[str] = None
    orthophoto_extent: Optional[tuple[float, float, float, float]] = None
    irrigation_channels: dict[str, Polygon] = field(default_factory=dict)


@dataclass(frozen=True)
class Vertex:
    id: str
    pose: Pose
    oriented: bool
    field_id: Optional[str] = None


@dataclass(frozen=True)
class Arc:
    id: str
    u: str
    v: str
    poses: tuple[Pose, ...]
    distance_m: float
    kind: str
    field_id: Optional[str] = None
    service_ids: frozenset[str] = frozenset()
    max_uses: Optional[int] = None
    driving_directions: tuple[int, ...] = ()
    reverse_distance_m: float = 0.0
    gear_changes: int = 0


@dataclass
class RoutingGraph:
    vertices: dict[str, Vertex] = field(default_factory=dict)
    arcs: dict[str, Arc] = field(default_factory=dict)
    requirements: set[str] = field(default_factory=set)
    start_id: str = ""
    end_id: str = ""
    selected_fields: set[str] = field(default_factory=set)


@dataclass(frozen=True)
class RouteSegment:
    arc_id: str
    occurrence_index: int
    kind: str
    field_id: Optional[str]
    poses: tuple[Pose, ...]
    distance_m: float
    service_ids: frozenset[str]
    driving_directions: tuple[int, ...] = ()
    reverse_distance_m: float = 0.0
    gear_changes: int = 0


@dataclass
class Route:
    task: FarmTask
    segments: list[RouteSegment]
    total_distance_m: float
    objective_cost: int
    optimality_status: str
    traversal_counts: dict[str, int]
    saturated_arc_ids: list[str]
    reverse_cost_multiplier: float = 1.0
    gear_change_penalty_m: float = 0.0
    best_bound_cost: Optional[float] = None
    solver_wall_time_s: Optional[float] = None

    def continuous_path(self) -> list[Pose]:
        result = []
        for segment in self.segments:
            result.extend(segment.poses if not result else segment.poses[1:])
        return result
