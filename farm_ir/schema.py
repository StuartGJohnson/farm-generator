"""
farm_ir_schema.py

Core intermediate representation (IR) for procedurally generated farms.

Design principles:
  - Vector-first: nearly everything is authored/generated as points, polylines,
    or polygons with parameters. Rasterization (heightfields, material grids)
    happens only at bake/export time, per-simulator, per-resolution.
  - Networks (hydrology, roads) are explicit graphs, not embedded geometry,
    so parcels/crossings/terrain can reference edges rather than duplicate them.
  - Every feature is spatially queryable and taggable, supporting both
    procedural generation and (later) extraction from real imagery/lidar
    without a schema change -- just a different `source`.
  - Coordinates are local ENU meters relative to a single geodetic SceneOrigin.
    This is the frame GNSS synthesis, physics, and visualization all share.

This module defines structure only -- no generation logic, no I/O, no
simulator-specific export. Those are separate concerns layered on top.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional


# ---------------------------------------------------------------------------
# Geometry primitives
# ---------------------------------------------------------------------------
# Deliberately plain tuples, not shapely objects, so the IR has no hard
# geometry-library dependency and serializes trivially to JSON. Convert to
# shapely (or similar) at the generator / visualizer boundary as needed.

Point2 = tuple[float, float]                 # (x, y) in local ENU meters
Point3 = tuple[float, float, float]          # (x, y, z)
Polyline = list[Point2]                      # ordered vertices, no implied closure
Polygon = list[Point2]                       # exterior ring only, for now (no holes yet)


# ---------------------------------------------------------------------------
# Scene-level metadata
# ---------------------------------------------------------------------------

@dataclass
class SceneOrigin:
    """Geodetic anchor for the scene's local ENU frame."""
    lat0: float                    # WGS84 latitude, degrees
    lon0: float                    # WGS84 longitude, degrees
    alt0: float = 0.0              # ellipsoidal or MSL altitude, meters (document which)
    utm_zone: Optional[str] = None # informational only; ENU is canonical, not UTM
    frame: str = "ENU"


# ---------------------------------------------------------------------------
# Provenance -- shared by every feature, cheap now, needed later for
# real-data extraction / hybrid scenes.
# ---------------------------------------------------------------------------

class FeatureSource(str, Enum):
    PROCEDURAL = "procedural"
    MEASURED = "measured"
    HYBRID = "hybrid"


# ---------------------------------------------------------------------------
# Base spatial feature
# ---------------------------------------------------------------------------

class FeatureLayer(str, Enum):
    HYDROLOGY = "hydrology"
    ROAD = "road"
    CROSSING = "crossing"
    PARCEL = "parcel"
    TREE = "tree"
    WEED_ZONE = "weed_zone"
    MATERIAL_PATCH = "material_patch"
    BUILDING = "building"


@dataclass
class SpatialFeature:
    """
    Common base for anything placed in the world with a spatial footprint.
    `layer` defaults to None and is set by each concrete subclass's
    __post_init__ -- required so dataclass field ordering works once
    subclasses add their own defaulted fields (id must stay the only
    field without a default anywhere in the hierarchy).
    """
    id: str
    layer: Optional[FeatureLayer] = None
    tags: dict[str, str] = field(default_factory=dict)
    refs: list[str] = field(default_factory=list)     # ids of correlated/related features
    source: FeatureSource = FeatureSource.PROCEDURAL
    confidence: Optional[float] = None                 # meaningful for measured/hybrid only


# ---------------------------------------------------------------------------
# Generic network graph (shared shape for hydrology and roads)
# ---------------------------------------------------------------------------

@dataclass
class NetworkNode:
    id: str
    position: Point2
    node_type: str              # network-specific: see subclass docstrings
    elevation: Optional[float] = None
    tags: dict[str, str] = field(default_factory=dict)


@dataclass
class NetworkEdge:
    id: str
    node_a: str                 # NetworkNode.id
    node_b: str                 # NetworkNode.id
    polyline: Polyline          # explicit path; endpoints should match node positions
    tags: dict[str, str] = field(default_factory=dict)


# --- Hydrology -------------------------------------------------------------

class HydrologyNodeType(str, Enum):
    CONFLUENCE = "confluence"
    PUMP_STATION = "pump_station"
    TERMINUS = "terminus"
    GATE = "gate"


@dataclass
class HydrologyNode(NetworkNode):
    """node_type should be a HydrologyNodeType value."""
    pass


class ConnectionType(str, Enum):
    GRAVITY_SURFACE = "gravity_surface"  # real visible channel, hydraulically continuous,
                                           # drives the macro elevation gradient (e.g. the trunk)
    PIPED = "piped"                       # buried delivery / locally-maintained level; graph
                                           # connectivity only, not elevation-continuous with
                                           # whatever it's nominally "fed by"
    PUMPED = "pumped"                     # rare/local active lift


@dataclass
class HydrologyEdge(NetworkEdge):
    """A ditch, canal, or levee-bounded channel segment."""
    connection_type: ConnectionType = ConnectionType.GRAVITY_SURFACE
    top_width: float = 0.0
    bottom_width: float = 0.0
    depth: float = 0.0
    side_slope: float = 1.5          # horizontal:vertical, typical earthen ditch
    water_surface_elev_a: Optional[float] = None   # at node_a; meaningful for GRAVITY_SURFACE
    water_surface_elev_b: Optional[float] = None   # at node_b; meaningful for GRAVITY_SURFACE
    target_water_level: Optional[float] = None     # for PIPED/PUMPED: locally-held level,
                                                      # not part of a continuous gradient
    levee_crown_elev_a: Optional[float] = None
    levee_crown_elev_b: Optional[float] = None
    flow_direction: str = "a_to_b"   # "a_to_b" | "b_to_a" | "bidirectional"


@dataclass
class HydrologyNetwork:
    nodes: dict[str, HydrologyNode] = field(default_factory=dict)
    edges: dict[str, HydrologyEdge] = field(default_factory=dict)


# --- Roads -------------------------------------------------------------

class RoadNodeType(str, Enum):
    INTERSECTION = "intersection"
    GATE = "gate"
    DEAD_END = "dead_end"


@dataclass
class RoadNode(NetworkNode):
    """node_type should be a RoadNodeType value."""
    pass


class RoadClass(str, Enum):
    LEVEE_CROWN = "levee_crown"        # runs atop the trunk canal's levee (entrance spine)
    FRONTAGE = "frontage"              # runs alongside a channel edge, parallel, does not cross it
    CROSSING_SPUR = "crossing_spur"    # short perpendicular segment that crosses a channel
                                         # via a culvert/bridge (see CrossingFeature)
    DRIVEWAY = "driveway"


class RoadSurface(str, Enum):
    PAVED = "paved"
    GRAVEL = "gravel"
    DIRT = "dirt"


@dataclass
class RoadEdge(NetworkEdge):
    width: float = 4.0
    surface: RoadSurface = RoadSurface.GRAVEL
    road_class: RoadClass = RoadClass.FRONTAGE


@dataclass
class RoadNetwork:
    nodes: dict[str, RoadNode] = field(default_factory=dict)
    edges: dict[str, RoadEdge] = field(default_factory=dict)


# --- Crossings (road x hydrology intersections) -----------------------

class CrossingType(str, Enum):
    CULVERT = "culvert"     # covered, no navigable span
    BRIDGE = "bridge"


@dataclass
class CrossingFeature(SpatialFeature):
    road_edge_id: str = ""
    hydrology_edge_id: str = ""
    location: Point2 = (0.0, 0.0)
    crossing_type: CrossingType = CrossingType.CULVERT
    span_width: float = 0.0
    structure_params: dict[str, float] = field(default_factory=dict)
    # e.g. {"pipe_diameter": 0.9} for culvert, {"deck_width": 4.5} for bridge

    def __post_init__(self):
        self.layer = FeatureLayer.CROSSING


# ---------------------------------------------------------------------------
# Parcels
# ---------------------------------------------------------------------------

class ParcelType(str, Enum):
    CROP = "crop"
    FARMSTEAD = "farmstead"
    LEVEE = "levee"
    EASEMENT = "easement"


@dataclass
class BoundaryRef:
    """One segment of a parcel's boundary, tied back to the network edge it follows."""
    network: str        # "hydrology" | "road" | "none" (property line, no network tie)
    edge_id: Optional[str] = None


@dataclass
class Parcel(SpatialFeature):
    polygon: Polygon = field(default_factory=list)
    parcel_type: ParcelType = ParcelType.CROP
    boundary_refs: list[BoundaryRef] = field(default_factory=list)

    def __post_init__(self):
        if self.layer is None:
            self.layer = FeatureLayer.PARCEL


@dataclass
class PlantingSpec:
    """Parameters for generative tree placement within a CropArea."""
    row_spacing: float
    tree_spacing: float
    row_angle_deg: float
    headland_width: float
    species_mix: dict[str, float]   # species name -> fraction, sums to 1.0
    seed: int


@dataclass
class WeedSpec:
    """Placeholder tying into the existing weed generator's parameters."""
    density_params: dict[str, float] = field(default_factory=dict)
    seed: int = 0


class IrrigationType(str, Enum):
    DRIP = "drip"                     # young orchard; fine supply lines, not modeled as hydrology
    FLOOD = "flood"                   # bordering channels used to flood-irrigate directly
    SUB_IRRIGATION = "sub_irrigation"  # bordering channels maintain water table via seepage


@dataclass
class CropArea(Parcel):
    crop_type: str = "orchard"
    irrigation_type: IrrigationType = IrrigationType.SUB_IRRIGATION
    planting: Optional[PlantingSpec] = None
    weeds: Optional[WeedSpec] = None
    tree_overrides: list[str] = field(default_factory=list)   # TreeInstance ids, hand-edits
    weed_zone_refs: list[str] = field(default_factory=list)   # WeedZone ids


@dataclass
class Building(SpatialFeature):
    footprint: Polygon = field(default_factory=list)
    building_type: str = "outbuilding"   # house | barn | tractor_shed | outbuilding
    height: float = 4.0
    roof_type: str = "gable"

    def __post_init__(self):
        self.layer = FeatureLayer.BUILDING


@dataclass
class FarmsteadCompound(Parcel):
    buildings: list[str] = field(default_factory=list)   # Building ids
    driveway_edge_ref: Optional[str] = None               # RoadEdge id


# ---------------------------------------------------------------------------
# Trees, weeds, material/traction patches
# ---------------------------------------------------------------------------

@dataclass
class TreeInstance(SpatialFeature):
    position: Point2 = (0.0, 0.0)
    species: str = ""
    age_years: float = 0.0
    canopy_radius: float = 1.0
    trunk_dbh: float = 0.1
    height: float = 3.0

    def __post_init__(self):
        self.layer = FeatureLayer.TREE


@dataclass
class WeedZone(SpatialFeature):
    polygon: Polygon = field(default_factory=list)
    density_params: dict[str, float] = field(default_factory=dict)

    def __post_init__(self):
        self.layer = FeatureLayer.WEED_ZONE


class MaterialClass(str, Enum):
    PAVEMENT = "pavement"
    GRAVEL = "gravel"
    DIRT = "dirt"
    SOFT_MUD = "soft_mud"
    RUT = "rut"
    WET_GRASS = "wet_grass"


@dataclass
class MaterialPatch(SpatialFeature):
    """
    Authored or derived region of non-nominal driving-surface behavior.
    Stays as a semantic class + severity, not raw physics params -- each
    simulator exporter owns its own class -> physics-param lookup table.
    """
    polygon: Polygon = field(default_factory=list)
    material_class: MaterialClass = MaterialClass.DIRT
    severity: float = 0.5                      # 0-1, feeds exporter's param lookup
    roughness_amplitude: Optional[float] = None  # meters, geometric bump height
    roughness_wavelength: Optional[float] = None # meters, spatial frequency of bumps

    def __post_init__(self):
        self.layer = FeatureLayer.MATERIAL_PATCH


# ---------------------------------------------------------------------------
# Elevation / terrain
# ---------------------------------------------------------------------------

class ElevationSource(str, Enum):
    DERIVED_FROM_HYDROLOGY = "derived_from_hydrology"
    GENERATED_TERRAIN = "generated_terrain"
    DERIVED_FROM_TERRAIN_FLOW_ROUTING = "derived_from_terrain_flow_routing"
    MEASURED = "measured"   # future: lidar bare-earth DEM


@dataclass
class ElevationField:
    """
    Baked, resolution-fixed elevation raster. Generated from vector sources
    (hydrology breaklines, terrain generators) -- this is the one part of the
    IR that is raster-native by necessity, since heightfields are what
    simulators actually consume. Treat it as a cached bake product, not the
    authored source of truth.
    """
    resolution: float                 # meters per cell
    origin: Point2                    # world position of grid cell (0,0)
    width_cells: int
    height_cells: int
    values: list[list[float]]         # [row][col] elevation, meters
    source: ElevationSource = ElevationSource.DERIVED_FROM_HYDROLOGY


@dataclass
class TerrainModel:
    elevation: Optional[ElevationField] = None
    breakline_edge_ids: list[str] = field(default_factory=list)   # road/hydrology edges
    # enforced as hard constraints when (re)baking `elevation`


# ---------------------------------------------------------------------------
# Top-level scene container
# ---------------------------------------------------------------------------

@dataclass
class GenerationProvenance:
    """Bookkeeping for reproducibility -- what generated this scene and how."""
    generator_version: str = ""
    global_seed: int = 0
    generator_params: dict[str, str] = field(default_factory=dict)


@dataclass
class FarmScene:
    origin: SceneOrigin
    hydrology: HydrologyNetwork = field(default_factory=HydrologyNetwork)
    roads: RoadNetwork = field(default_factory=RoadNetwork)
    crossings: dict[str, CrossingFeature] = field(default_factory=dict)
    parcels: dict[str, Parcel] = field(default_factory=dict)          # incl. CropArea, FarmsteadCompound
    trees: dict[str, TreeInstance] = field(default_factory=dict)
    weed_zones: dict[str, WeedZone] = field(default_factory=dict)
    material_patches: dict[str, MaterialPatch] = field(default_factory=dict)
    buildings: dict[str, Building] = field(default_factory=dict)
    terrain: TerrainModel = field(default_factory=TerrainModel)
    provenance: GenerationProvenance = field(default_factory=GenerationProvenance)
