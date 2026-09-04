"""Bake the vector hydrology network into a 2.5D triangular USD mesh.

The IR deliberately remains vector-only. This module builds an explicit PSLG
for channel features, then adds an isotropic triangular point distribution in
unconstrained regions before constrained Delaunay triangulation.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import hashlib
import math
import os

import condeltri
import numpy as np
from scipy.stats import qmc
from shapely import node, segmentize, set_precision
from shapely.geometry import LineString, MultiLineString, Point, Polygon, box
from shapely.ops import nearest_points, substring, unary_union

from farm_ir.schema import FarmScene, HydrologyEdge, RoadClass


@dataclass(frozen=True)
class GroundMesh:
    points: list[tuple[float, float, float]]
    triangles: list[tuple[int, int, int]]
    breakline_edges: list[tuple[int, int]]
    road_breakline_edges: list[tuple[int, int]]
    crossing_breakline_edges: list[tuple[int, int]]
    face_classes: list[str]


@dataclass(frozen=True)
class PlanarStraightLineGraph:
    """Triangulator-independent PSLG in the scene's ENU coordinates."""
    vertices: list[tuple[float, float]]
    segments: list[tuple[int, int]]


@dataclass(frozen=True)
class ChannelUndulationConfig:
    """Exporter-side lateral perturbation; the vector IR stays unchanged."""
    max_amplitude: float = 0.10
    min_wavelength: float = 1.0
    sample_spacing: float = 1.0
    seed_offset: int = 0

    def __post_init__(self):
        if self.max_amplitude < 0:
            raise ValueError("max_amplitude must be non-negative")
        if self.min_wavelength <= 0:
            raise ValueError("min_wavelength must be positive")
        if self.sample_spacing <= 0:
            raise ValueError("sample_spacing must be positive")


@dataclass(frozen=True)
class ChannelFace:
    """One fully known sloped surface patch on one parcel side."""
    id: str
    parcel_id: str
    hydrology_edge_id: str
    shoulder: list[tuple[float, float]]
    bottom: list[tuple[float, float]]
    depth: float


@dataclass(frozen=True)
class RoadSurfacePatch:
    """Known planar road footprint used for constraints and face labeling."""
    id: str
    road_edge_ids: tuple[str, ...]
    polygon: object
    is_crossing: bool = False


def _line_segments(geometry):
    if geometry.is_empty:
        return
    if isinstance(geometry, LineString):
        coords = list(geometry.coords)
        for a, b in zip(coords, coords[1:]):
            if a != b:
                yield a, b
    elif isinstance(geometry, MultiLineString) or hasattr(geometry, "geoms"):
        for part in geometry.geoms:
            yield from _line_segments(part)


def _profile_elevation(x: float, y: float, edges: list[HydrologyEdge]) -> float:
    """Lowest elevation contributed by any trapezoidal channel profile."""
    z = 0.0
    for edge in edges:
        if edge.depth <= 0.0 or edge.top_width <= 0.0 or len(edge.polyline) < 2:
            continue
        d = LineString(edge.polyline).distance(Point(x, y))
        bottom_r = min(edge.bottom_width, edge.top_width) * 0.5
        top_r = edge.top_width * 0.5
        if d <= bottom_r:
            edge_z = -edge.depth
        elif d < top_r and top_r > bottom_r:
            edge_z = -edge.depth * (top_r - d) / (top_r - bottom_r)
        else:
            edge_z = 0.0
        z = min(z, edge_z)
    return z


def _canonical_edges(scene: FarmScene) -> list[HydrologyEdge]:
    """Collapse duplicate logical channels without changing IR polylines."""
    unique: dict[tuple[str, str], HydrologyEdge] = {}
    for edge in scene.hydrology.edges.values():
        if len(edge.polyline) < 2:
            continue
        pair = tuple(sorted((edge.node_a, edge.node_b)))
        previous = unique.get(pair)
        if previous is None or edge.depth > previous.depth:
            unique[pair] = replace(edge, polyline=list(edge.polyline))
    return list(unique.values())


def _logical_edge_key(edge: HydrologyEdge) -> tuple[str, str]:
    return tuple(sorted((edge.node_a, edge.node_b)))


def _edge_rng(scene: FarmScene, edge: HydrologyEdge, config: ChannelUndulationConfig):
    global_seed = scene.provenance.global_seed if scene.provenance is not None else 0
    pair = _logical_edge_key(edge)
    payload = f"{global_seed + config.seed_offset}:{pair[0]}:{pair[1]}".encode()
    seed = int.from_bytes(hashlib.blake2b(payload, digest_size=8).digest(), "little")
    return np.random.default_rng(seed)


def _undulate_edge(
    scene: FarmScene,
    edge: HydrologyEdge,
    config: ChannelUndulationConfig,
) -> list[tuple[float, float]]:
    """Return one bounded, band-limited curve in canonical node order."""
    forward = edge.node_a <= edge.node_b
    source = edge.polyline if forward else list(reversed(edge.polyline))
    line = LineString(source)
    length = line.length
    if config.max_amplitude == 0 or length <= config.min_wavelength * 0.5:
        return [(float(x), float(y)) for x, y in source]

    # 1-cos(2*pi*k*s/L) has both zero displacement and zero derivative at
    # the endpoints, preserving the IR's authored corner position and tangent.
    # Its wavelength is L/k; strict truncation keeps wavelengths > the limit.
    max_mode = int(math.ceil(length / config.min_wavelength) - 1)
    if max_mode < 1:
        return [(float(x), float(y)) for x, y in source]
    modes = np.arange(1, max_mode + 1, dtype=float)
    rng = _edge_rng(scene, edge, config)
    coefficients = rng.normal(size=max_mode) / modes**2

    dense_s = np.linspace(0.0, length, max(256, max_mode * 32 + 1))
    dense_wave = (1.0 - np.cos(2.0 * np.pi * np.outer(dense_s / length, modes))) @ coefficients
    peak = float(np.max(np.abs(dense_wave)))
    if peak <= 1e-15:
        return [(float(x), float(y)) for x, y in source]
    target_amplitude = config.max_amplitude * rng.uniform(0.6, 1.0)
    coefficients *= target_amplitude / peak

    spacing = config.sample_spacing
    sample_count = max(2, int(math.ceil(length / spacing)))
    distances = np.linspace(0.0, length, sample_count + 1)
    offsets = (1.0 - np.cos(2.0 * np.pi * np.outer(distances / length, modes))) @ coefficients
    # Keep the first/last sampled segment exactly tangent to the authored IR
    # edge. The continuous basis already has zero endpoint derivative, but a
    # polyline represents that derivative with its first finite chord.
    if len(offsets) >= 4:
        offsets[1] = 0.0
        offsets[-2] = 0.0
    result = []
    tangent_step = min(1e-3, length * 1e-4)
    for distance, offset in zip(distances, offsets):
        point = line.interpolate(float(distance))
        before = line.interpolate(max(0.0, float(distance) - tangent_step))
        after = line.interpolate(min(length, float(distance) + tangent_step))
        tx, ty = after.x - before.x, after.y - before.y
        norm = math.hypot(tx, ty)
        nx, ny = -ty / norm, tx / norm
        result.append((point.x + float(offset) * nx, point.y + float(offset) * ny))
    # Preserve graph junctions exactly, independent of floating-point sine.
    result[0] = (float(source[0][0]), float(source[0][1]))
    result[-1] = (float(source[-1][0]), float(source[-1][1]))
    return result


def build_undulated_hydrology_edges(
    scene: FarmScene,
    config: ChannelUndulationConfig,
) -> dict[tuple[str, str], HydrologyEdge]:
    """Create perturbed export copies, keyed by logical edge; never mutate IR."""
    result = {}
    for edge in _canonical_edges(scene):
        result[_logical_edge_key(edge)] = replace(
            edge, polyline=_undulate_edge(scene, edge, config)
        )
    return result


def _cross(a, b):
    return a[0] * b[1] - a[1] * b[0]


def _miter_offset(previous, point, following, distance, side):
    incoming = (point[0] - previous[0], point[1] - previous[1])
    outgoing = (following[0] - point[0], following[1] - point[1])
    in_len, out_len = math.hypot(*incoming), math.hypot(*outgoing)
    incoming = (incoming[0] / in_len, incoming[1] / in_len)
    outgoing = (outgoing[0] / out_len, outgoing[1] / out_len)
    in_normal = (-side * incoming[1], side * incoming[0])
    out_normal = (-side * outgoing[1], side * outgoing[0])
    a = (point[0] + distance * in_normal[0], point[1] + distance * in_normal[1])
    b = (point[0] + distance * out_normal[0], point[1] + distance * out_normal[1])
    denominator = _cross(incoming, outgoing)
    if abs(denominator) < 1e-10:
        return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5)
    delta = (b[0] - a[0], b[1] - a[1])
    t = _cross(delta, outgoing) / denominator
    return (a[0] + t * incoming[0], a[1] + t * incoming[1])


def _split_offset_ring(ring, corner_candidates, curves):
    """Split a closed GEOS offset at corners and map each arc to an IR edge."""
    stations = sorted({ring.project(Point(point)) for point in corner_candidates})
    arcs = []
    for i, start in enumerate(stations):
        end = stations[(i + 1) % len(stations)]
        if end > start:
            coords = list(substring(ring, start, end).coords)
        else:
            tail = list(substring(ring, start, ring.length).coords)
            head = list(substring(ring, 0.0, end).coords)
            coords = tail + head[1:]
        arc = LineString(coords)
        midpoint = arc.interpolate(0.5, normalized=True)
        edge_index = min(
            range(len(curves)), key=lambda j: LineString(curves[j]).distance(midpoint)
        )
        arcs.append((edge_index, coords))
    if len({edge_index for edge_index, _ in arcs}) != len(curves):
        raise ValueError("could not map offset contour arcs one-to-one to hydrology edges")
    return {edge_index: coords for edge_index, coords in arcs}


def _build_channel_faces(scene, undulated_edges=None) -> list[ChannelFace]:
    """Build paired longitudinal boundaries and endpoint ribs directly."""
    faces = []
    for parcel_id, parcel in scene.parcels.items():
        area2 = sum(
            parcel.polygon[i][0] * parcel.polygon[(i + 1) % len(parcel.polygon)][1]
            - parcel.polygon[(i + 1) % len(parcel.polygon)][0] * parcel.polygon[i][1]
            for i in range(len(parcel.polygon))
        )
        side = 1.0 if area2 > 0 else -1.0
        curves = []
        edges = []
        for i, ref in enumerate(parcel.boundary_refs):
            edge = scene.hydrology.edges[ref.edge_id]
            curve = (
                list(undulated_edges[_logical_edge_key(edge)].polyline)
                if undulated_edges is not None else list(edge.polyline)
            )
            if math.dist(parcel.polygon[i], curve[-1]) < math.dist(parcel.polygon[i], curve[0]):
                curve.reverse()
            curves.append(curve)
            edges.append(edge)
        top_widths = {edge.top_width for edge in edges}
        bottom_widths = {edge.bottom_width for edge in edges}
        depths = {edge.depth for edge in edges}
        if len(top_widths) != 1 or len(bottom_widths) != 1 or len(depths) != 1:
            raise ValueError("POC mesher requires uniform channel dimensions around each parcel")
        top_distance = next(iter(top_widths)) * 0.5
        bottom_distance = next(iter(bottom_widths)) * 0.5
        polygon_ring = [point for curve in curves for point in curve[:-1]]
        polygon = Polygon(polygon_ring)
        shoulder_polygon = polygon.buffer(-top_distance, join_style="mitre")
        bottom_polygon = polygon.buffer(-bottom_distance, join_style="mitre")
        if shoulder_polygon.is_empty or bottom_polygon.is_empty:
            continue
        top_candidates, bottom_candidates = [], []
        for i, curve in enumerate(curves):
            previous_curve = curves[i - 1]
            corner = curve[0]
            top_candidates.append(_miter_offset(previous_curve[-2], corner, curve[1], top_distance, side))
            bottom_candidates.append(_miter_offset(previous_curve[-2], corner, curve[1], bottom_distance, side))
        shoulder_arcs = _split_offset_ring(shoulder_polygon.boundary, top_candidates, curves)
        bottom_arcs = _split_offset_ring(bottom_polygon.boundary, bottom_candidates, curves)
        for i, (curve, edge) in enumerate(zip(curves, edges)):
            shoulder = shoulder_arcs[i]
            bottom = bottom_arcs[i]
            if math.dist(shoulder[0], curve[0]) > math.dist(shoulder[-1], curve[0]):
                shoulder.reverse()
            if math.dist(bottom[0], curve[0]) > math.dist(bottom[-1], curve[0]):
                bottom.reverse()
            # ChannelFace owns corner correspondence. GEOS supplies the valid
            # interior arc, but its independently projected envelope endpoints
            # must not redefine the paired analytical corner stations.
            shoulder[0], shoulder[-1] = top_candidates[i], top_candidates[(i + 1) % len(curves)]
            bottom[0], bottom[-1] = bottom_candidates[i], bottom_candidates[(i + 1) % len(curves)]
            faces.append(ChannelFace(
                id=f"channel_face_{parcel_id}_{i}",
                parcel_id=parcel_id,
                hydrology_edge_id=edge.id,
                shoulder=shoulder,
                bottom=bottom,
                depth=next(iter(depths)),
            ))
    return faces


def build_channel_faces(
    scene: FarmScene,
    undulation: ChannelUndulationConfig | None = None,
) -> list[ChannelFace]:
    """Build the complete known channel-slope patches without changing IR."""
    edges = build_undulated_hydrology_edges(scene, undulation) if undulation else None
    return _build_channel_faces(scene, edges)


def build_road_surface_patches(scene: FarmScene, bounds) -> list[RoadSurfacePatch]:
    """Build joined frontage ribbons and flat-ended crossing connections."""
    domain = box(*bounds)
    road_ribbons = []
    frontage_by_parcel = {}
    for edge in scene.roads.edges.values():
        if edge.road_class == RoadClass.FRONTAGE and edge.tags.get("parcel_id"):
            frontage_by_parcel.setdefault(edge.tags["parcel_id"], []).append(edge)

    # A parcel frontage is one closed road, not a collection of independently
    # square-capped segments.  Buffering the segments separately lets every
    # perpendicular segment's end cap protrude through the opposite road edge.
    for parcel_id, edges in frontage_by_parcel.items():
        nodes = [
            node.position for node in scene.roads.nodes.values()
            if node.tags.get("parcel_id") == parcel_id
        ]
        if len(nodes) >= 3:
            width = edges[0].width
            road_ribbons.append(
                LineString([*nodes, nodes[0]]).buffer(
                    width * 0.5, join_style="mitre"
                )
            )

    for edge in scene.roads.edges.values():
        if len(edge.polyline) < 2 or edge.width <= 0 or edge.road_class == RoadClass.FRONTAGE:
            continue
        # Crossing spurs terminate at the frontage centerline.  A flat cap is
        # fully covered by the receiving frontage ribbon and cannot punch a
        # rectangular notch through its far side.
        cap_style = "flat" if edge.road_class == RoadClass.CROSSING_SPUR else "square"
        road_ribbons.append(
            LineString(edge.polyline).buffer(
                edge.width * 0.5, cap_style=cap_style, join_style="mitre"
            )
        )
    patches = []
    if road_ribbons:
        road_area = unary_union(road_ribbons).intersection(domain)
        patches.append(RoadSurfacePatch(
            id="roads", road_edge_ids=tuple(scene.roads.edges), polygon=road_area
        ))
    for crossing in scene.crossings.values():
        edge = scene.roads.edges.get(crossing.road_edge_id)
        if edge is None or len(edge.polyline) < 2:
            continue
        footprint = LineString(edge.polyline).buffer(
            edge.width * 0.5, cap_style="flat", join_style="mitre"
        ).intersection(domain)
        patches.append(RoadSurfacePatch(
            id=crossing.id,
            road_edge_ids=(edge.id,),
            polygon=footprint,
            is_crossing=True,
        ))
    return patches


def _parcel_channel_geometry(scene, bounds, undulated_edges=None):
    """Derive regions directly from first-class channel faces."""
    domain = box(*bounds)
    faces = _build_channel_faces(scene, undulated_edges)
    by_parcel = {}
    for face in faces:
        by_parcel.setdefault(face.parcel_id, []).append(face)
    shoulder_rings, bottom_rings = [], []
    shoulder_interiors, bottom_interiors = [], []
    for parcel_faces in by_parcel.values():
        shoulder = Polygon([p for face in parcel_faces for p in face.shoulder[:-1]])
        bottom = Polygon([p for face in parcel_faces for p in face.bottom[:-1]])
        shoulder_rings.append(shoulder.boundary)
        bottom_rings.append(bottom.boundary)
        shoulder_interiors.append(shoulder)
        bottom_interiors.append(bottom)
    flat_area = unary_union(shoulder_interiors)
    non_bottom_area = unary_union(bottom_interiors)
    slope_area = non_bottom_area.difference(flat_area)
    return domain, shoulder_rings, bottom_rings, slope_area, faces


def _breakline_elevation_overrides(scene, bounds, undulated_edges=None):
    """Return authored elevation contours; these override distance sampling."""
    shoulder_rings = []
    bottom_levels = []
    for face in _build_channel_faces(scene, undulated_edges):
        shoulder_rings.append(LineString(face.shoulder))
        bottom_levels.append((LineString(face.bottom), -face.depth))
    return shoulder_rings, bottom_levels


def _breaklines(scene, bounds, max_segment_length, undulated_edges=None):
    """Return noded shoulder, bottom, and domain breakline linework."""
    domain, shoulder_rings, bottom_rings, _, faces = _parcel_channel_geometry(
        scene, bounds, undulated_edges
    )
    transverse = [
        LineString([face.shoulder[0], face.bottom[0]])
        for face in faces
    ]
    road_patches = build_road_surface_patches(scene, bounds)
    road_area = next(
        (patch.polygon for patch in road_patches if not patch.is_crossing), None
    )
    channel_lines = [*shoulder_rings, *bottom_rings, *transverse]
    if road_area is not None and not road_area.is_empty:
        channel_lines = [line.difference(road_area) for line in channel_lines]
        road_boundaries = [road_area.boundary]
    else:
        road_boundaries = []
    # Crossing footprints are already part of the unioned road area. Their
    # individual end/side boundaries must not cut across the road surface.
    lines = [domain.boundary, *channel_lines, *road_boundaries]
    # GEOS overlay nodes intersections; explicit precision and a second
    # node pass ensure shared endpoints are bit-identical for PythonCDT.
    # A micrometre topology grid removes numerically distinct copies of an
    # analytic intersection without affecting metre-scale IR geometry.
    precision = 1e-6
    linework = node(set_precision(unary_union(lines), precision))
    linework = node(unary_union(linework))
    return segmentize(linework, max_segment_length=max_segment_length)


def build_hydrology_pslg(
    scene: FarmScene,
    bounds: tuple[float, float, float, float],
    max_segment_length: float = 1.0,
    undulation: ChannelUndulationConfig | None = None,
) -> PlanarStraightLineGraph:
    """Build the domain/channel PSLG before adding any free mesh vertices."""
    if max_segment_length <= 0:
        raise ValueError("max_segment_length must be positive")
    undulated_edges = build_undulated_hydrology_edges(scene, undulation) if undulation else None
    linework = _breaklines(scene, bounds, max_segment_length, undulated_edges)
    road_area = next(
        (
            patch.polygon for patch in build_road_surface_patches(scene, bounds)
            if not patch.is_crossing
        ),
        None,
    )
    road_boundary = (
        set_precision(road_area, 1e-6).boundary
        if road_area is not None and not road_area.is_empty else None
    )
    precision_scale = 1e8

    def canonical_point(point):
        result = (float(point[0]), float(point[1]))
        sample = Point(result)
        if road_boundary is not None and road_boundary.distance(sample) <= 2e-6:
            projected = nearest_points(sample, road_boundary)[1]
            result = (float(projected.x), float(projected.y))
        return result

    def key(point):
        return (round(float(point[0]) * precision_scale), round(float(point[1]) * precision_scale))

    coords = {}
    segment_keys = []
    canonical_lines = [
        LineString([canonical_point(a), canonical_point(b)])
        for a, b in _line_segments(linework)
    ]
    # Projection can place a channel endpoint in the interior of an existing
    # road segment.  Re-node after canonicalization so the PSLG contains only
    # atomic, non-intersecting constraints.
    canonical_linework = node(unary_union(canonical_lines))
    for a, b in _line_segments(canonical_linework):
        ka, kb = key(a), key(b)
        coords.setdefault(ka, (float(a[0]), float(a[1])))
        coords.setdefault(kb, (float(b[0]), float(b[1])))
        if ka != kb:
            segment_keys.append((ka, kb))
    ordered_keys = list(coords)
    indices = {key_: i for i, key_ in enumerate(ordered_keys)}
    segments = sorted({
        tuple(sorted((indices[a], indices[b])))
        for a, b in segment_keys
    })
    return PlanarStraightLineGraph(
        vertices=[coords[key_] for key_ in ordered_keys],
        segments=segments,
    )


def build_ground_mesh(
    scene: FarmScene,
    bounds: tuple[float, float, float, float],
    flat_resolution: float = 1.0,
    undulation: ChannelUndulationConfig | None = None,
) -> GroundMesh:
    """Create a breakline-constrained Delaunay mesh over ``bounds``."""
    if flat_resolution <= 0:
        raise ValueError("flat_resolution must be positive")
    minx, miny, maxx, maxy = bounds
    if not (minx < maxx and miny < maxy):
        raise ValueError("bounds must have positive width and height")

    undulated_edges = build_undulated_hydrology_edges(scene, undulation) if undulation else None
    edges = list(undulated_edges.values()) if undulated_edges else _canonical_edges(scene)
    linework = _breaklines(scene, bounds, flat_resolution, undulated_edges)
    pslg = build_hydrology_pslg(scene, bounds, flat_resolution, undulation)
    _, _, _, slope_area, _ = _parcel_channel_geometry(scene, bounds, undulated_edges)
    sampling_road_area = next(
        (
            patch.polygon for patch in build_road_surface_patches(scene, bounds)
            if not patch.is_crossing
        ),
        None,
    )
    if sampling_road_area is not None and not sampling_road_area.is_empty:
        # Roads replace the underlying channel surface.  Their crossing tops
        # therefore belong to the same connected flat sampling region as the
        # rest of the map, rather than to the excluded channel slopes.
        slope_area = slope_area.difference(sampling_road_area)
    flat_sampling_area = box(*bounds).difference(slope_area)
    precision_scale = 1e8

    def key(point):
        return (round(float(point[0]) * precision_scale), round(float(point[1]) * precision_scale))

    coords = {key(point): point for point in pslg.vertices}
    pslg_keys = [key(point) for point in pslg.vertices]

    # Blue-noise samples have no preferred axis or repeated lattice pattern.
    # Their Delaunay triangulation is the dual of an amorphous Voronoi mesh.
    seed = scene.provenance.global_seed if scene.provenance is not None else 0
    sampler = qmc.PoissonDisk(
        d=2,
        radius=0.62 * flat_resolution,
        ncandidates=40,
        rng=np.random.default_rng(seed + 0x5EED),
        l_bounds=np.array([minx, miny]),
        u_bounds=np.array([maxx, maxy]),
    )
    for x, y in sampler.fill_space():
        sample = Point(float(x), float(y))
        if flat_sampling_area.covers(sample) and linework.distance(sample) >= 0.15 * flat_resolution:
            coords.setdefault(key((x, y)), (float(x), float(y)))

    ordered_keys = list(coords)
    vertex_index = {k: i for i, k in enumerate(ordered_keys)}
    xy_input = [coords[k] for k in ordered_keys]
    vertices = [condeltri.V2d(x, y) for x, y in xy_input]
    constraints = {
        tuple(sorted((vertex_index[pslg_keys[a]], vertex_index[pslg_keys[b]])))
        for a, b in pslg.segments
    }

    triangulation = condeltri.Triangulation(
        condeltri.VertexInsertionOrder.AUTO,
        condeltri.IntersectingConstraintEdges.NOT_ALLOWED,
        0.0,
    )
    triangulation.insert_vertices(vertices)
    triangulation.insert_edges([condeltri.Edge(a, b) for a, b in sorted(constraints)])
    triangulation.erase_super_triangle()

    xy = [(float(v.x), float(v.y)) for v in triangulation.vertices_iter()]
    shoulders, bottom_levels = _breakline_elevation_overrides(
        scene, bounds, undulated_edges
    )
    points = []
    for x, y in xy:
        sample = Point(x, y)
        z = _profile_elevation(x, y, edges)
        for ring, bottom_z in bottom_levels:
            if ring.distance(sample) <= 2e-6:
                z = bottom_z
                break
        else:
            if any(ring.distance(sample) <= 2e-6 for ring in shoulders):
                z = 0.0
        points.append((x, y, z))
    triangles = []
    for triangle in triangulation.triangles_iter():
        tri = tuple(int(i) for i in triangle.vertices)
        a, b, c = (xy[i] for i in tri)
        twice_area = abs((b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1]))
        # Defensive guard for downstream physics/rendering clients.
        if twice_area > 1e-12:
            triangles.append(tri)
    breakline_edges = [(int(edge.v1), int(edge.v2)) for edge in triangulation.fixed_edges_iter()]

    road_patches = build_road_surface_patches(scene, bounds)
    road_area = next((patch.polygon for patch in road_patches if not patch.is_crossing), None)
    if road_area is not None and not road_area.is_empty:
        # Match the precision grid used to create the PSLG.  Classifying road
        # faces against the unsnapped polygon can leave micron-scale terrain
        # slivers beside a crossing, with no corresponding road-top vertex.
        road_area = set_precision(road_area, 1e-6)
    crossing_supports = []
    for crossing in scene.crossings.values():
        road_edge = scene.roads.edges.get(crossing.road_edge_id)
        channel_edge = scene.hydrology.edges.get(crossing.hydrology_edge_id)
        if road_edge is None or channel_edge is None:
            continue
        export_channel = (
            undulated_edges[_logical_edge_key(channel_edge)]
            if undulated_edges is not None else channel_edge
        )
        road_ribbon = LineString(road_edge.polyline).buffer(
            road_edge.width * 0.5 + 2e-6,
            cap_style="square",
            join_style="mitre",
        )
        channel_ribbon = LineString(export_channel.polyline).buffer(
            channel_edge.top_width * 0.5 + 2e-6,
            cap_style="flat",
            join_style="mitre",
        )
        support = road_ribbon.intersection(channel_ribbon)
        if not support.is_empty:
            crossing_supports.append(support)
    crossing_support = unary_union(crossing_supports) if crossing_supports else None

    def triangle_center(triangle):
        return Point(
            sum(points[i][0] for i in triangle) / 3.0,
            sum(points[i][1] for i in triangle) / 3.0,
        )

    face_classes = ["ground"] * len(triangles)
    road_index = {}
    crossing_breakline_edges = []
    if road_area is not None and not road_area.is_empty:
        inside = [road_area.covers(triangle_center(tri)) for tri in triangles]
        edge_sides = {}
        for triangle_index, tri in enumerate(triangles):
            for i in range(3):
                edge = tuple(sorted((tri[i], tri[(i + 1) % 3])))
                edge_sides.setdefault(edge, []).append(triangle_index)
        boundary_edges = {
            edge for edge, adjacent in edge_sides.items()
            if any(inside[i] for i in adjacent) and not all(inside[i] for i in adjacent)
        }
        # The road boundary is itself a PSLG constraint.  Use those constraints
        # as the authoritative wall footprint as well as topology transitions.
        # In particular, a constrained edge can have only one retained planar
        # neighbour after the road split, which made the old adjacency-only
        # test omit a wall triangle in seed 3.
        road_boundary_tolerance = road_area.boundary.buffer(2e-6)
        for edge, adjacent in edge_sides.items():
            a, b = edge
            edge_line = LineString([xy[a], xy[b]])
            if (
                road_boundary_tolerance.covers(edge_line)
                and any(not inside[i] for i in adjacent)
            ):
                boundary_edges.add(edge)
        road_vertices = {
            i for triangle_index, tri in enumerate(triangles)
            if inside[triangle_index] for i in tri
        }
        # CDT may place a channel-profile vertex on a constrained road edge
        # without using it in the triangle on the road side.  That legal 2-D
        # T-junction becomes a hole once the road vertices are lifted.  Find
        # every such terrain-side boundary vertex and split the road triangle
        # edge at exactly the same station.
        outside_vertices = {
            i for triangle_index, tri in enumerate(triangles)
            if not inside[triangle_index] for i in tri
        }
        boundary_candidates = [
            i for i in outside_vertices
            if road_area.boundary.distance(Point(xy[i])) <= 2e-6
        ]
        road_edge_splits = {}
        for triangle_index, tri in enumerate(triangles):
            if not inside[triangle_index]:
                continue
            for side_index in range(3):
                a, b = tri[side_index], tri[(side_index + 1) % 3]
                segment = LineString([xy[a], xy[b]])
                if (
                    not road_boundary_tolerance.covers(segment)
                ):
                    continue
                split_vertices = [
                    i for i in boundary_candidates
                    if i not in (a, b)
                    and segment.distance(Point(xy[i])) <= 2e-6
                ]
                if split_vertices:
                    split_vertices.sort(key=lambda i: segment.project(Point(xy[i])))
                    road_edge_splits[(triangle_index, side_index)] = split_vertices
                    road_vertices.update(split_vertices)
        for index in sorted(road_vertices):
            road_index[index] = len(points)
            x, y, _ = points[index]
            points.append((x, y, 0.0))
        # Replace, rather than overlay, every road-interior terrain face.
        base_triangle_count = len(triangles)
        original_triangles = triangles[:]
        for triangle_index in range(base_triangle_count):
            tri = triangles[triangle_index]
            if inside[triangle_index]:
                perimeter = []
                for side_index in range(3):
                    perimeter.append(tri[side_index])
                    perimeter.extend(road_edge_splits.get((triangle_index, side_index), ()))
                road_triangles = []
                for i in range(1, len(perimeter) - 1):
                    candidate = tuple(road_index[j] for j in (perimeter[0], perimeter[i], perimeter[i + 1]))
                    pa, pb, pc = (points[j] for j in candidate)
                    twice_area = abs(
                        (pb[0] - pa[0]) * (pc[1] - pa[1])
                        - (pc[0] - pa[0]) * (pb[1] - pa[1])
                    )
                    if twice_area > 1e-12:
                        road_triangles.append(candidate)
                triangles[triangle_index] = road_triangles[0]
                face_classes[triangle_index] = "road"
                triangles.extend(road_triangles[1:])
                face_classes.extend(["road"] * (len(road_triangles) - 1))
        for (triangle_index, side_index), split_vertices in road_edge_splits.items():
            original = original_triangles[triangle_index]
            a, b = original[side_index], original[(side_index + 1) % 3]
            boundary_edges.discard(tuple(sorted((a, b))))
            chain = [a, *split_vertices, b]
            boundary_edges.update(
                tuple(sorted((u, v))) for u, v in zip(chain, chain[1:])
            )
        for a, b in sorted(boundary_edges):
            if a not in road_index or b not in road_index:
                continue
            ta, tb = road_index[a], road_index[b]
            if points[a][2] < -1e-9 or points[b][2] < -1e-9:
                # This terrain-side edge is the exact intersection between a
                # vertical crossing face and a sloping channel face.
                edge_midpoint = Point(
                    (points[a][0] + points[b][0]) * 0.5,
                    (points[a][1] + points[b][1]) * 0.5,
                )
                if (
                    abs(points[a][2] - points[b][2]) > 1e-9
                    and crossing_support is not None
                    and crossing_support.covers(edge_midpoint)
                ):
                    crossing_breakline_edges.append((a, b))
                for wall_triangle in ((a, b, tb), (a, tb, ta)):
                    pa, pb, pc = (points[i] for i in wall_triangle)
                    u = tuple(pb[i] - pa[i] for i in range(3))
                    v = tuple(pc[i] - pa[i] for i in range(3))
                    cross3 = (
                        u[1] * v[2] - u[2] * v[1],
                        u[2] * v[0] - u[0] * v[2],
                        u[0] * v[1] - u[1] * v[0],
                    )
                    if sum(component * component for component in cross3) > 1e-20:
                        triangles.append(wall_triangle)
                        face_classes.append("crossing_wall")

    final_edges = {
        tuple(sorted((tri[i], tri[(i + 1) % 3])))
        for tri in triangles for i in range(3)
    }
    surviving_breaklines = []
    for a, b in breakline_edges:
        edge = tuple(sorted((a, b)))
        if edge in final_edges:
            surviving_breaklines.append(edge)
        elif a in road_index and b in road_index:
            road_edge = tuple(sorted((road_index[a], road_index[b])))
            if road_edge in final_edges:
                surviving_breaklines.append(road_edge)

    road_breakline_edges = []
    if road_area is not None and not road_area.is_empty:
        road_boundary = road_area.boundary.buffer(2e-6)
        road_edge_counts = {}
        for triangle, face_class in zip(triangles, face_classes):
            if face_class != "road":
                continue
            for i in range(3):
                edge = tuple(sorted((triangle[i], triangle[(i + 1) % 3])))
                road_edge_counts[edge] = road_edge_counts.get(edge, 0) + 1
        for (a, b), count in road_edge_counts.items():
            if count != 1:
                continue
            segment = LineString([points[a][:2], points[b][:2]])
            if road_boundary.covers(segment):
                road_breakline_edges.append((a, b))

    return GroundMesh(
        points=points,
        triangles=triangles,
        breakline_edges=surviving_breaklines,
        road_breakline_edges=road_breakline_edges,
        crossing_breakline_edges=crossing_breakline_edges,
        face_classes=face_classes,
    )


def write_ground_mesh_usda(mesh: GroundMesh, path: str, prim_name: str = "Ground") -> None:
    """Write a self-contained USDA containing one double-sided mesh."""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    min_pt = tuple(min(p[i] for p in mesh.points) for i in range(3))
    max_pt = tuple(max(p[i] for p in mesh.points) for i in range(3))
    face_colors = []
    for face_class, triangle in zip(mesh.face_classes, mesh.triangles):
        if face_class == "road":
            color = (0.30, 0.24, 0.17)
        elif face_class == "crossing_wall":
            color = (0.24, 0.24, 0.22)
        elif min(mesh.points[i][2] for i in triangle) < -1e-9:
            color = (0.34, 0.20, 0.08)
        else:
            color = (0.18, 0.38, 0.08)
        face_colors.extend([color, color, color])

    normals = []
    for ia, ib, ic in mesh.triangles:
        a, b, c = mesh.points[ia], mesh.points[ib], mesh.points[ic]
        ux, uy, uz = (b[i] - a[i] for i in range(3))
        vx, vy, vz = (c[i] - a[i] for i in range(3))
        normal = (uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx)
        length = math.sqrt(sum(component * component for component in normal))
        normal = tuple(component / length for component in normal)
        normals.extend([normal, normal, normal])

    def vec(values):
        return ",\n        ".join(f"({a:.9g}, {b:.9g}, {c:.9g})" for a, b, c in values)

    counts = ", ".join("3" for _ in mesh.triangles)
    indices = ", ".join(str(i) for tri in mesh.triangles for i in tri)
    road_faces = ", ".join(
        str(i) for i, kind in enumerate(mesh.face_classes)
        if kind == "road"
    )
    wall_faces = ", ".join(str(i) for i, kind in enumerate(mesh.face_classes) if kind == "crossing_wall")
    text = f'''#usda 1.0
(
    defaultPrim = "{prim_name}"
    metersPerUnit = 1
    upAxis = "Z"
)

def Mesh "{prim_name}"
{{
    uniform bool doubleSided = 1
    float3[] extent = [({min_pt[0]:.9g}, {min_pt[1]:.9g}, {min_pt[2]:.9g}), ({max_pt[0]:.9g}, {max_pt[1]:.9g}, {max_pt[2]:.9g})]
    int[] faceVertexCounts = [{counts}]
    int[] faceVertexIndices = [{indices}]
    normal3f[] normals = [
        {vec(normals)}
    ] (
        interpolation = "faceVarying"
    )
    uniform token orientation = "rightHanded"
    point3f[] points = [
        {vec(mesh.points)}
    ]
    color3f[] primvars:displayColor = [
        {vec(face_colors)}
    ] (
        interpolation = "faceVarying"
    )
    uniform token subdivisionScheme = "none"

    def GeomSubset "RoadFaces"
    {{
        uniform token elementType = "face"
        int[] indices = [{road_faces}]
    }}

    def GeomSubset "CrossingWalls"
    {{
        uniform token elementType = "face"
        int[] indices = [{wall_faces}]
    }}
}}
'''
    with open(path, "w", encoding="utf-8") as stream:
        stream.write(text)


def save_ground_mesh_wireframe(mesh: GroundMesh, path: str, title: str | None = None) -> None:
    """Save plan and perspective wireframe diagnostics with all vertices."""
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection

    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    triangle_edges = {
        tuple(sorted((tri[i], tri[(i + 1) % 3])))
        for tri in mesh.triangles
        for i in range(3)
    }
    xy_segments = [
        [(mesh.points[a][0], mesh.points[a][1]), (mesh.points[b][0], mesh.points[b][1])]
        for a, b in triangle_edges
    ]
    fixed = {tuple(sorted(edge)) for edge in mesh.breakline_edges}
    road_fixed = {tuple(sorted(edge)) for edge in mesh.road_breakline_edges}
    channel_fixed = fixed - road_fixed
    channel_fixed_xy = [
        [(mesh.points[a][0], mesh.points[a][1]), (mesh.points[b][0], mesh.points[b][1])]
        for a, b in channel_fixed
    ]
    road_fixed_xy = [
        [(mesh.points[a][0], mesh.points[a][1]), (mesh.points[b][0], mesh.points[b][1])]
        for a, b in road_fixed
    ]
    transverse_xy = [
        [(mesh.points[a][0], mesh.points[a][1]), (mesh.points[b][0], mesh.points[b][1])]
        for a, b in fixed
        if abs(mesh.points[a][2] - mesh.points[b][2]) > 0.5
    ]
    crossing_xy = [
        [(mesh.points[a][0], mesh.points[a][1]), (mesh.points[b][0], mesh.points[b][1])]
        for a, b in mesh.crossing_breakline_edges
    ]

    fig = plt.figure(figsize=(14, 6), constrained_layout=True)
    plan = fig.add_subplot(1, 2, 1)
    plan.add_collection(LineCollection(xy_segments, colors="0.55", linewidths=0.35))
    plan.add_collection(LineCollection(channel_fixed_xy, colors="#ff8c00", linewidths=0.8))
    plan.add_collection(LineCollection(road_fixed_xy, colors="#e31a1c", linewidths=1.2))
    plan.add_collection(LineCollection(transverse_xy, colors="#7a0177", linewidths=2.0))
    plan.scatter(
        [segment[0][0] for segment in crossing_xy],
        [segment[0][1] for segment in crossing_xy],
        s=22, c="#7a0177", zorder=5, linewidths=0,
    )
    plan.scatter(
        [p[0] for p in mesh.points], [p[1] for p in mesh.points],
        s=4, c="#0868ac", zorder=3, linewidths=0,
    )
    plan.autoscale()
    plan.set_aspect("equal")
    plan.set_title("plan: roads (red), channels (orange), crossings (purple)")
    plan.set_xlabel("East [m]")
    plan.set_ylabel("North [m]")

    perspective = fig.add_subplot(1, 2, 2, projection="3d")
    perspective.plot_trisurf(
        [p[0] for p in mesh.points],
        [p[1] for p in mesh.points],
        [p[2] for p in mesh.points],
        triangles=mesh.triangles,
        color="#b8d89b", edgecolor="0.25", linewidth=0.25,
        alpha=0.35, shade=False,
    )
    road_triangles = [
        tri for tri, kind in zip(mesh.triangles, mesh.face_classes)
        if kind == "road"
    ]
    if road_triangles:
        perspective.plot_trisurf(
            [p[0] for p in mesh.points],
            [p[1] for p in mesh.points],
            [p[2] for p in mesh.points],
            triangles=road_triangles,
            color="#78634a", edgecolor="0.2", linewidth=0.2,
            alpha=0.75, shade=False,
        )
    for a, b in mesh.crossing_breakline_edges:
        perspective.plot(
            [mesh.points[a][0], mesh.points[b][0]],
            [mesh.points[a][1], mesh.points[b][1]],
            [mesh.points[a][2], mesh.points[b][2]],
            color="#7a0177", linewidth=2.5, zorder=6,
        )
    perspective.scatter(
        [p[0] for p in mesh.points],
        [p[1] for p in mesh.points],
        [p[2] for p in mesh.points],
        s=2, c="#0868ac", depthshade=False,
    )
    perspective.set_title("perspective wireframe")
    perspective.set_xlabel("East [m]")
    perspective.set_ylabel("North [m]")
    perspective.set_zlabel("Elevation [m]")
    perspective.set_zlim(-0.85, 0.05)
    perspective.set_zticks([-0.8, -0.4, 0.0])
    perspective.view_init(elev=32, azim=-55)
    perspective.set_box_aspect((1, 1, 0.3))
    if title:
        fig.suptitle(title)
    fig.savefig(path, dpi=220)
    plt.close(fig)


def export_scene_ground(
    scene: FarmScene,
    bounds: tuple[float, float, float, float],
    path: str,
    flat_resolution: float = 1.0,
    undulation: ChannelUndulationConfig | None = None,
) -> GroundMesh:
    mesh = build_ground_mesh(scene, bounds, flat_resolution, undulation)
    write_ground_mesh_usda(mesh, path)
    return mesh
