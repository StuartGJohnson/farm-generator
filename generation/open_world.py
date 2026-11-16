"""Generate a feature-free driving world with one visual ground surface."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from farm_ir.schema import FarmScene, GenerationProvenance, SceneOrigin
from farm_ir.serialize import from_plain, read, write
from generation.orchestrator import SurfaceFrictionConfig
from generation.surface_friction import generate_surface_friction
from generation.surface_height import generate_surface_height


GENERATOR_VERSION = "open-world-0.1.0"


@dataclass(frozen=True)
class OpenWorldConfig:
    bounds: tuple[float, float, float, float] = (0.0, 0.0, 120.0, 120.0)
    seed: int = 1
    surface: str = "field"  # field or road; both use the farm's visual assets
    origin_lat0: float = 38.2
    origin_lon0: float = -121.5
    origin_alt0: float = 0.0
    utm_zone: str = "10N"
    farm_resolution_m: float = 1.0
    crop_surface: SurfaceFrictionConfig = field(default_factory=lambda: SurfaceFrictionConfig(
        friction_mean=0.5, friction_variation_range=0.90,
        friction_scale_m=10.0, friction_seed=1001))
    road_surface: SurfaceFrictionConfig = field(default_factory=lambda: SurfaceFrictionConfig(
        friction_mean=1.05, friction_variation_range=0.20,
        friction_scale_m=10.0, friction_seed=2001))
    channel_friction: float = 0.30
    height_mean: float = 0.0
    height_variation_range_m: float = 0.5
    height_scale_m: float = 10.0
    height_spectral_slope: float = 4.0
    height_seed: int = 1001
    road_height_variation_range_m: float = 0.125
    road_height_scale_m: float = 10.0
    road_height_spectral_slope: float = 4.0
    road_height_seed: int = 2002
    shoreline_taper_m: float = 1.0
    road_field_blend_m: float = 2.0

    def __post_init__(self):
        x0, y0, x1, y1 = self.bounds
        if not all(math.isfinite(v) for v in self.bounds) or x0 >= x1 or y0 >= y1:
            raise ValueError("bounds must be finite and have positive area")
        if self.surface not in ("field", "road"):
            raise ValueError("surface must be 'field' or 'road'")
        if isinstance(self.seed, bool) or not isinstance(self.seed, int) or self.seed < 0:
            raise ValueError("seed must be a nonnegative integer")
        for name in ("farm_resolution_m", "height_scale_m", "road_height_scale_m"):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive and finite")
        for name in ("height_variation_range_m", "road_height_variation_range_m",
                     "height_spectral_slope", "road_height_spectral_slope"):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) < 0:
                raise ValueError(f"{name} must be nonnegative and finite")
        for name in ("height_seed", "road_height_seed"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ValueError(f"{name} must be a nonnegative integer")

    @property
    def domain_bounds(self) -> tuple[float, float, float, float]:
        return self.bounds

    @property
    def surface_region(self) -> str:
        return "crop" if self.surface == "field" else "road"


def generate_open_world(config: OpenWorldConfig) -> FarmScene:
    """Reuse the farm IR's ENU and sampled fields without farm features."""
    scene = FarmScene(
        origin=SceneOrigin(config.origin_lat0, config.origin_lon0,
                           config.origin_alt0, config.utm_zone),
        provenance=GenerationProvenance(GENERATOR_VERSION, config.seed),
    )
    scene.surface_friction = generate_surface_friction(config)
    scene.surface_height = generate_surface_height(config)
    scene.provenance.generator_params["world_kind"] = "open"
    scene.provenance.generator_params["surface"] = config.surface
    return scene


def save_open_world(scene: FarmScene, config: OpenWorldConfig, path: str) -> None:
    write({"config": config, "scene": scene}, path)


def load_open_world(path: str) -> tuple[FarmScene, OpenWorldConfig]:
    doc = read(path)
    return from_plain(doc["scene"], FarmScene), from_plain(doc["config"], OpenWorldConfig)
