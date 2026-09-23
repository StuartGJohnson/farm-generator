"""
generation/orchestrator.py

`FarmGenerationConfig` and `generate_farm(config) -> FarmScene` -- the
top-level entry point that runs the generation DAG in order (see
CLAUDE.md "Generation order"):

    1. tessellation      (T-tessellation + vertex cleanup -> parcels)
    2. hydrology          (HydrologyNetwork derived from parcel boundaries)
    3. roads              (full perimeter per parcel, via erosion)
    4. crossings          (strict collinear vertex-to-vertex)
    5. crops/weeds        (per CropArea)
    6. surface friction   (independently seeded, output-independent fields)

`generation/material.py` is not called; semantic material patches are separate
from the continuous surface friction fields generated here.
There is no elevation/terrain stage -- see CLAUDE.md "Elevation / terrain:
removed": all features derive directly from the road and hydrology
networks, and a baked heightfield added no information downstream
consumers couldn't already get from those.

Also provides `validate(scene, config) -> list[str]` and
`generate_validated(config) -> (scene, issues, used_seed)`, the
retry-on-validation-failure wrapper described in CLAUDE.md
"Validation-with-retry": generation occasionally (empirically ~2% of
seeds) produces one sharp-angled face or one disconnected parcel from an
inherent, small tension in the pipeline that isn't worth chasing further
by hand -- retrying with seed+1000, +2000, ... is the pragmatic fix.

`save_farm`/`load_farm` write/read a scene TOGETHER WITH the config that
produced it, per CLAUDE.md "Config": "Serialize a FarmGenerationConfig
instance alongside a generated FarmScene for reproducibility". This is
the module that composes them (farm_ir/serialize.py itself knows nothing
about FarmGenerationConfig -- it's generic over any dataclass).
"""

from __future__ import annotations

import dataclasses
from dataclasses import dataclass, field
from typing import Optional

import networkx as nx
import numpy as np
from shapely.geometry import Polygon

from farm_ir.schema import FarmScene, GenerationProvenance, IrrigationType, RoadSurface, SceneOrigin
from farm_ir.serialize import from_plain, read, write
from generation.surface_friction import generate_surface_friction
from generation.surface_height import generate_surface_height

from generation import crops, crossings, hydrology, roads, tessellation
from generation.tessellation import min_interior_angle_deg

GENERATOR_VERSION = "farm-gen-0.1.0"


@dataclass(frozen=True)
class SurfaceFrictionConfig:
    """Dimensionless surface mu, full variation interval, and spatial spectrum."""
    friction_mean: float = 0.65
    friction_variation_range: float = 0.30
    friction_scale_m: float = 5.0
    friction_spectral_slope: float = 4.0
    friction_seed: int = 1001

    def __post_init__(self):
        import math
        values = (self.friction_mean, self.friction_variation_range,
                  self.friction_scale_m, self.friction_spectral_slope)
        if not all(math.isfinite(v) for v in values):
            raise ValueError("surface friction parameters must be finite")
        if self.friction_variation_range < 0 or self.friction_mean < self.friction_variation_range / 2:
            raise ValueError("surface friction interval must be non-negative")
        if self.friction_scale_m <= 0 or self.friction_spectral_slope < 0:
            raise ValueError("friction scale must be positive and spectral slope non-negative")
        if isinstance(self.friction_seed, bool) or not isinstance(self.friction_seed, int) or self.friction_seed < 0:
            raise ValueError("friction_seed must be a non-negative integer")


@dataclass
class FarmGenerationConfig:
    """
    All lengths/areas are meters/square-meters (the schema's ENU frame is
    meters). See CLAUDE.md "Config" for how these interact -- in
    particular, mean parcel size emerges from `bounds` and `max_faces`
    together (`max_faces = domain_area / target_mean_parcel_area_m2`),
    not from a direct "mean size" field.
    """

    # --- domain ---
    bounds: tuple[float, float, float, float]  # (minx, miny, maxx, maxy), meters
    seed: int

    # --- scene origin (ENU anchor; see CLAUDE.md principle 3) ---
    origin_lat0: float = 38.2   # Sacramento Delta, WGS84 degrees
    origin_lon0: float = -121.5
    origin_alt0: float = 0.0
    utm_zone: Optional[str] = "10N"  # informational only -- ENU is canonical

    # --- tessellation (controls parcel count/size/shape) ---
    max_faces: int = 60
    min_interior_angle_deg: float = 50.0     # reject a cut producing any angle below this
    min_area_frac: float = 0.015             # floor: stop splitting below this frac of domain area
    min_subarea_frac: float = 0.3            # reject a cut if either half < this frac of PARENT area
    angle_jitter_deg: float = 6.0            # jitter around perpendicular-to-long-axis cut direction
    corner_margin_frac: float = 0.12         # keeps cut anchors away from existing corners

    # --- erosion / roads ---
    standoff: float = 1.0                    # meters; uniform road setback from parcel boundary
    mitre_limit: float = 2.0                 # caps a corner's erosion extension at mitre_limit * standoff
    road_width: float = 4.0
    road_surface_type: RoadSurface = RoadSurface.GRAVEL
    crop_surface: SurfaceFrictionConfig = field(default_factory=SurfaceFrictionConfig)
    road_surface: SurfaceFrictionConfig = field(default_factory=lambda: SurfaceFrictionConfig(
        friction_mean=1.05, friction_variation_range=0.20, friction_seed=2001))
    farm_resolution_m: float = 1.0
    channel_friction: float = 0.3
    height_mean: float = 0.0
    height_variation_range_m: float = 0.2
    height_scale_m: float = 3.0
    height_spectral_slope: float = 4.0
    height_seed: int = 1001
    shoreline_taper_m: float = 1.0

    # --- hydrology ---
    hydrology_node_merge_tol: Optional[float] = None  # meters; defaults to 0.05 * standoff
    hydrology_top_width: float = 2.0
    hydrology_bottom_width: float = 1.0
    hydrology_depth: float = 0.8
    hydrology_add_water: bool = False
    hydrology_water_depth_fraction: float = 0.5

    # --- crossings ---
    connect_radius: Optional[float] = None    # defaults to standoff * 2.5
    crossing_angle_tol_deg: float = 1e-3      # see CLAUDE.md "Key lessons" -- must stay near the
                                                 # floating-point noise floor, not "looks small enough"

    # --- crops / weeds ---
    # row_spacing/tree_spacing/headland_width are kept small by default so
    # a 60-100m test domain still shows several rows per parcel -- see
    # CLAUDE_update1.md. Row angle is no longer a config input: each
    # parcel picks its own row alignment from its own geometry (see
    # generation/crops.py._pick_row_and_col_dirs). headland_width trims
    # each ROW's ends (turning space, measured along the row direction);
    # sideland_width trims each COLUMN's ends (measured along the column
    # direction) -- see generation/crops.py for the oblique-coordinate
    # trim this drives.
    crop_type: str = "orchard"
    irrigation_type: IrrigationType = IrrigationType.SUB_IRRIGATION
    headland_width: float = 2.0
    sideland_width: float = 2.0
    row_spacing: float = 1.5
    tree_spacing: float = 1.0
    optimize_tree_layout: bool = False
    species_mix: dict[str, float] = field(default_factory=lambda: {"almond": 1.0})
    canopy_radius: float = 1.5
    trunk_dbh: float = 0.15
    tree_height: float = 4.0
    # Expected instances per metre of cultivated row, integrated across
    # the row's lateral profile.  Lateral offsets are Gaussian with this
    # standard deviation and are truncated at 3 sigma.  There is no
    # parcel-wide background weed population.
    weed_row_density_per_m: float = 4.5
    weed_row_falloff_m: float = 0.35

    def __post_init__(self):
        import math
        # Accept the former constructor spelling and old serialized configs.
        if isinstance(self.road_surface, (RoadSurface, str)):
            self.road_surface_type = RoadSurface(self.road_surface)
            self.road_surface = SurfaceFrictionConfig(1.05, 0.20, 5.0, 4.0, 2001)
        for name in ("crop_surface", "road_surface"):
            value = getattr(self, name)
            if isinstance(value, dict):
                setattr(self, name, SurfaceFrictionConfig(**value))
            elif not isinstance(value, SurfaceFrictionConfig):
                raise TypeError(f"{name} must be SurfaceFrictionConfig or a parameter mapping")
        if not math.isfinite(self.farm_resolution_m) or self.farm_resolution_m <= 0:
            raise ValueError("farm_resolution_m must be finite and positive")
        if not math.isfinite(self.channel_friction) or self.channel_friction < 0:
            raise ValueError("channel_friction must be finite and non-negative")
        if not all(math.isfinite(value) for value in (
            self.height_mean, self.height_variation_range_m,
            self.height_scale_m, self.height_spectral_slope,
        )):
            raise ValueError("surface height parameters must be finite")
        if self.height_variation_range_m < 0 or self.height_scale_m <= 0 or self.height_spectral_slope < 0:
            raise ValueError("height range and spectral slope must be non-negative; scale must be positive")
        if isinstance(self.height_seed, bool) or not isinstance(self.height_seed, int) or self.height_seed < 0:
            raise ValueError("height_seed must be a non-negative integer")
        if not math.isfinite(self.shoreline_taper_m) or self.shoreline_taper_m < 0:
            raise ValueError("shoreline_taper_m must be finite and non-negative")
        if not 0.0 <= self.hydrology_water_depth_fraction <= 1.0:
            raise ValueError("hydrology_water_depth_fraction must be between 0 and 1")
        if self.weed_row_density_per_m < 0.0:
            raise ValueError("weed_row_density_per_m must be non-negative")
        if self.weed_row_falloff_m < 0.0:
            raise ValueError("weed_row_falloff_m must be non-negative")
        if self.connect_radius is None:
            self.connect_radius = self.standoff * 2.5
        if self.hydrology_node_merge_tol is None:
            self.hydrology_node_merge_tol = self.standoff * 0.05


def generate_farm(config: FarmGenerationConfig) -> FarmScene:
    """
    Deterministic given (config, config.seed) -- see CLAUDE.md "Config".
    One global seed is split into per-stage sub-seeds so one stage's
    regeneration doesn't reshuffle another's.
    """
    parent_rng = np.random.default_rng(config.seed)
    tess_rng, hydro_rng, road_rng, crossing_rng, crop_rng = parent_rng.spawn(5)

    scene = FarmScene(
        origin=SceneOrigin(
            lat0=config.origin_lat0,
            lon0=config.origin_lon0,
            alt0=config.origin_alt0,
            utm_zone=config.utm_zone,
        ),
        provenance=GenerationProvenance(
            generator_version=GENERATOR_VERSION,
            global_seed=config.seed,
        ),
    )
    scene.hydrology.water_enabled = config.hydrology_add_water
    scene.hydrology.water_depth_fraction = config.hydrology_water_depth_fraction

    scene = tessellation.run(scene, config, tess_rng)
    scene = hydrology.run(scene, config, hydro_rng)
    scene = roads.run(scene, config, road_rng)
    scene = crossings.run(scene, config, crossing_rng)
    scene = crops.run(scene, config, crop_rng)
    scene.surface_friction = generate_surface_friction(config)
    scene.surface_height = generate_surface_height(config)

    scene.provenance.generator_params.update({
        "max_faces": str(config.max_faces),
        "standoff": str(config.standoff),
        "connect_radius": repr(config.connect_radius),
    })

    return scene


def validate(scene: FarmScene, config: FarmGenerationConfig) -> list[str]:
    """
    Returns a list of issue strings; empty means all checks passed. See
    CLAUDE.md "Validation-with-retry" -- this is a fast supplementary
    signal, secondary to visualization/debug_view.py.
    """
    issues: list[str] = []
    domain_area = (config.bounds[2] - config.bounds[0]) * (config.bounds[3] - config.bounds[1])

    for pid, parcel in scene.parcels.items():
        poly = Polygon(parcel.polygon)
        if not poly.is_valid or not poly.is_simple:
            issues.append(f"parcel {pid} is self-intersecting / invalid")

        ang = min_interior_angle_deg(np.array(parcel.polygon))
        if ang < config.min_interior_angle_deg:
            issues.append(f"parcel {pid} has a sharp interior angle ({ang:.1f} deg)")

    # RAW (pre-cleanup) tessellation area should conserve domain area exactly
    # -- an algorithm-correctness check. Post-cleanup area is checked
    # separately, with a loose tolerance, since intentional vertex-cleanup
    # simplification trims small "ears" of area by design (see CLAUDE.md
    # "Key lessons").
    raw_area_str = scene.provenance.generator_params.get("tessellation_raw_area")
    if raw_area_str is not None:
        raw_area = float(raw_area_str)
        if abs(domain_area - raw_area) > 1e-3:
            issues.append(
                f"RAW tessellation area not conserved (algorithm bug): "
                f"domain={domain_area:.2f}, sum={raw_area:.2f}"
            )

    total_area = sum(Polygon(p.polygon).area for p in scene.parcels.values())
    area_tolerance = 0.005 * domain_area
    if abs(domain_area - total_area) > area_tolerance:
        issues.append(
            f"post-cleanup area drifted more than expected: "
            f"domain={domain_area:.2f}, sum_of_parcels={total_area:.2f} "
            f"(tolerance={area_tolerance:.2f})"
        )

    g = nx.Graph()
    g.add_nodes_from(scene.parcels.keys())
    for c in scene.crossings.values():
        if len(c.refs) == 2:
            g.add_edge(c.refs[0], c.refs[1])
    n_components = nx.number_connected_components(g) if g.number_of_nodes() else 0
    if n_components > 1:
        issues.append(f"road network is not fully connected: {n_components} separate components")

    return issues


def generate_validated(config: FarmGenerationConfig, max_seed_tries: int = 10):
    """
    Pragmatic retry wrapper (see CLAUDE.md "Validation-with-retry"): try
    the requested seed, then seed+1000, +2000, ... until validate()
    passes or max_seed_tries is exhausted (in which case the last attempt
    is returned along with its issues, so the caller still gets a
    result). Mirrors examples/farm_mesh_prototype.py's
    generate_validated(), operating on the real IR/config instead of the
    prototype's plain dicts.
    """
    base_seed = config.seed
    scene, issues, trial_seed = None, None, base_seed
    for attempt in range(max_seed_tries):
        trial_seed = base_seed if attempt == 0 else base_seed + attempt * 1000
        trial_config = dataclasses.replace(config, seed=trial_seed)
        scene = generate_farm(trial_config)
        issues = validate(scene, trial_config)
        if not issues:
            return scene, issues, trial_seed
    return scene, issues, trial_seed


def save_farm(scene: FarmScene, config: FarmGenerationConfig, path: str) -> None:
    """
    Write `scene` and the `config` that produced it to a single
    human-readable YAML or JSON file (chosen by `path`'s extension --
    see farm_ir/serialize.py). This is the IR's on-disk form: what
    debug tooling (visualization/debug_view.py) emits for inspection,
    and eventually what USD/Gazebo exporters will read as input.
    """
    write({"config": config, "scene": scene}, path)


def load_farm(path: str) -> tuple[FarmScene, FarmGenerationConfig]:
    """Inverse of save_farm: read a scene + its generating config back
    from `path`."""
    doc = read(path)
    if isinstance(doc["config"].get("road_surface"), str):
        doc["config"]["road_surface_type"] = doc["config"].pop("road_surface")
    # Older scenes embedded the generator model in each field. Preserve their
    # original quantization interval while loading the output-only IR schema.
    friction = doc["scene"].get("surface_friction")
    if friction:
        for region in ("crop", "road"):
            field_data = friction[region]
            parameters = field_data.pop("parameters", None)
            if parameters is not None:
                half_range = parameters["friction_variation_range"] / 2
                field_data["friction_min"] = parameters["friction_mean"] - half_range
                field_data["friction_max"] = parameters["friction_mean"] + half_range
    scene = from_plain(doc["scene"], FarmScene)
    config = from_plain(doc["config"], FarmGenerationConfig)
    if scene.surface_friction is None:
        scene.surface_friction = generate_surface_friction(config)
    if scene.surface_height is None:
        scene.surface_height = generate_surface_height(config)
    return scene, config
