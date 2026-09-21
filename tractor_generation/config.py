"""Typed YAML configuration for tractor assets and simulation bringup."""

from __future__ import annotations

from dataclasses import dataclass, fields
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class TractorConfig:
    livery: str = "tiger_stripes"
    random_seed: int = 1
    wheelbase: float = 2.7
    front_tire_dia: float = 1.0
    front_tire_width: float = 0.2
    # Radial rubber thickness (outer radius minus visual rim radius), cosmetic.
    front_tire_depth: float = 0.2
    rear_tire_dia: float = 1.6
    rear_tire_width: float = 0.3
    rear_tire_depth: float = 0.2
    front_track_width: float = 1.0
    rear_track_width: float = 1.6
    turning_radius: float = 2.7
    sensor_pod_length: float = 2.0
    sensor_pod_width: float = 1.5
    sensor_pod_height: float = 0.2
    sensor_pod_height_above_ground: float = 2.3
    tractor_body_length: float = 3.5
    tractor_body_width: float = 1.5
    tractor_body_height: float = 1.0
    tractor_body_ground_clearance: float = 0.3
    tractor_body_front_axle: float = 0.3
    wheel_well_clearance: float = 0.08
    front_camera: bool = True
    rear_camera: bool = False
    left_camera: bool = False
    right_camera: bool = False
    camera_pitch: float = 10.0
    camera_hfov: float = 90.0
    camera_resolution: tuple[int, int] = (1280, 720)

    def __post_init__(self) -> None:
        positive = [
            "wheelbase", "front_tire_dia", "front_tire_width", "front_tire_depth",
            "rear_tire_dia", "rear_tire_width", "rear_tire_depth",
            "front_track_width", "rear_track_width", "turning_radius",
            "sensor_pod_length", "sensor_pod_width", "sensor_pod_height",
            "sensor_pod_height_above_ground", "tractor_body_length",
            "tractor_body_width", "tractor_body_height",
        ]
        for name in positive:
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive")
        if self.tractor_body_ground_clearance < 0:
            raise ValueError("tractor_body_ground_clearance must be non-negative")
        if self.wheel_well_clearance < 0:
            raise ValueError("wheel_well_clearance must be non-negative")
        if not 0 < self.tractor_body_front_axle < self.tractor_body_length:
            raise ValueError("tractor_body_front_axle must lie inside the tractor body")
        rear_axle = self.front_axle_x - self.wheelbase
        if rear_axle < -self.tractor_body_length * 0.5:
            raise ValueError("wheelbase places the rear axle behind the tractor body")
        if self.sensor_pod_height_above_ground <= self.body_top_z:
            raise ValueError("sensor pod must be above the tractor body")
        if not 0 < self.camera_hfov < 180:
            raise ValueError("camera_hfov must be between 0 and 180 degrees")
        if len(self.camera_resolution) != 2 or any(v <= 0 for v in self.camera_resolution):
            raise ValueError("camera_resolution must contain two positive integers")
        if self.livery not in {"tiger_stripes", "cheetah_spots"}:
            raise ValueError("livery must be tiger_stripes or cheetah_spots")
        if min(width for _, _, width in self.body_segments) <= 0:
            raise ValueError("wheel_well_clearance leaves no body between the tires")

    @property
    def front_axle_x(self) -> float:
        return self.tractor_body_length * 0.5 - self.tractor_body_front_axle

    @property
    def rear_axle_x(self) -> float:
        return self.front_axle_x - self.wheelbase

    @property
    def body_center_z(self) -> float:
        return self.tractor_body_ground_clearance + self.tractor_body_height * 0.5

    @property
    def body_top_z(self) -> float:
        return self.tractor_body_ground_clearance + self.tractor_body_height

    @property
    def max_steer_angle_rad(self) -> float:
        # Bicycle-model inner/nominal steering limit implied by requested radius.
        import math
        return math.atan2(self.wheelbase, self.turning_radius)

    @property
    def enabled_cameras(self) -> tuple[str, ...]:
        return tuple(
            name for name in ("front", "rear", "left", "right")
            if getattr(self, f"{name}_camera")
        )

    @property
    def body_segments(self) -> tuple[tuple[float, float, float], ...]:
        """Return ``(center_x, length, width)`` boxes forming wheel wells.

        Longitudinal regions alongside a tire are narrowed to the space
        between its inner sidewalls. Adjacent regions retain the requested
        full body width. The boxes exactly partition the body length.
        """
        half_length = self.tractor_body_length * 0.5
        wheel_zones = (
            (
                max(-half_length, self.front_axle_x - self.front_tire_dia / 2 - self.wheel_well_clearance),
                min(half_length, self.front_axle_x + self.front_tire_dia / 2 + self.wheel_well_clearance),
                self.front_track_width - self.front_tire_width - 2 * self.wheel_well_clearance,
            ),
            (
                max(-half_length, self.rear_axle_x - self.rear_tire_dia / 2 - self.wheel_well_clearance),
                min(half_length, self.rear_axle_x + self.rear_tire_dia / 2 + self.wheel_well_clearance),
                self.rear_track_width - self.rear_tire_width - 2 * self.wheel_well_clearance,
            ),
        )
        breaks = sorted({-half_length, half_length, *(v for zone in wheel_zones for v in zone[:2])})
        raw: list[tuple[float, float, float]] = []
        for start, end in zip(breaks, breaks[1:]):
            center = (start + end) * 0.5
            active_widths = [width for low, high, width in wheel_zones if low <= center <= high]
            width = min(active_widths, default=self.tractor_body_width)
            raw.append((center, end - start, min(width, self.tractor_body_width)))
        merged: list[tuple[float, float, float]] = []
        for center, length, width in raw:
            if merged and abs(merged[-1][2] - width) < 1e-12:
                old_center, old_length, _ = merged.pop()
                start = old_center - old_length / 2
                end = center + length / 2
                merged.append(((start + end) / 2, end - start, width))
            else:
                merged.append((center, length, width))
        return tuple(merged)


def load_tractor_config(path: str | Path) -> TractorConfig:
    data: dict[str, Any] = yaml.safe_load(Path(path).read_text()) or {}
    known = {field.name for field in fields(TractorConfig)}
    unknown = sorted(set(data) - known)
    if unknown:
        raise ValueError(f"unknown tractor configuration keys: {', '.join(unknown)}")
    if "camera_resolution" in data:
        data["camera_resolution"] = tuple(data["camera_resolution"])
    return TractorConfig(**data)
