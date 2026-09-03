"""Bake the vector hydrology network into a 2.5D triangular USD mesh.

The IR deliberately remains vector-only. This module builds an explicit PSLG
for channel features, then adds an isotropic triangular point distribution in
unconstrained regions before constrained Delaunay triangulation.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import os

import condeltri
import numpy as np
from scipy.stats import qmc
from shapely import node, segmentize, set_precision
from shapely.geometry import LineString, MultiLineString, Point, Polygon, box
from shapely.ops import unary_union

from farm_ir.schema import FarmScene, HydrologyEdge


@dataclass(frozen=True)
class GroundMesh:
    points: list[tuple[float, float, float]]
    triangles: list[tuple[int, int, int]]
    breakline_edges: list[tuple[int, int]]


@dataclass(frozen=True)
class PlanarStraightLineGraph:
    """Triangulator-independent PSLG in the scene's ENU coordinates."""
    vertices: list[tuple[float, float]]
    segments: list[tuple[int, int]]


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


def _parcel_channel_geometry(scene, bounds):
    """Derive channel regions and breaklines from parcel-side IR edges."""
    domain = box(*bounds)
    shoulder_rings = []
    bottom_rings = []
    shoulder_interiors = []
    bottom_interiors = []
    for parcel in scene.parcels.values():
        referenced = [
            scene.hydrology.edges[ref.edge_id]
            for ref in parcel.boundary_refs
            if ref.network == "hydrology" and ref.edge_id in scene.hydrology.edges
        ]
        top_widths = {edge.top_width for edge in referenced}
        bottom_widths = {edge.bottom_width for edge in referenced}
        if len(top_widths) != 1 or len(bottom_widths) != 1:
            raise ValueError("POC mesher requires uniform channel widths around each parcel")
        polygon = Polygon(parcel.polygon)
        shoulder = polygon.buffer(-next(iter(top_widths)) * 0.5, join_style="mitre")
        bottom = polygon.buffer(-next(iter(bottom_widths)) * 0.5, join_style="mitre")
        if not shoulder.is_empty:
            shoulder_rings.append(shoulder.boundary)
            shoulder_interiors.append(shoulder)
        if not bottom.is_empty:
            bottom_rings.append(bottom.boundary)
            bottom_interiors.append(bottom)
    flat_area = unary_union(shoulder_interiors)
    non_bottom_area = unary_union(bottom_interiors)
    slope_area = non_bottom_area.difference(flat_area)
    return domain, shoulder_rings, bottom_rings, slope_area


def _breaklines(scene, bounds, max_segment_length):
    """Return noded shoulder, bottom, and domain breakline linework."""
    domain, shoulder_rings, bottom_rings, _ = _parcel_channel_geometry(scene, bounds)
    lines = [domain.boundary, *shoulder_rings, *bottom_rings]
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
) -> PlanarStraightLineGraph:
    """Build the domain/channel PSLG before adding any free mesh vertices."""
    if max_segment_length <= 0:
        raise ValueError("max_segment_length must be positive")
    linework = _breaklines(scene, bounds, max_segment_length)
    precision_scale = 1e8

    def key(point):
        return (round(float(point[0]) * precision_scale), round(float(point[1]) * precision_scale))

    coords = {}
    segment_keys = []
    for a, b in _line_segments(linework):
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
) -> GroundMesh:
    """Create a breakline-constrained Delaunay mesh over ``bounds``."""
    if flat_resolution <= 0:
        raise ValueError("flat_resolution must be positive")
    minx, miny, maxx, maxy = bounds
    if not (minx < maxx and miny < maxy):
        raise ValueError("bounds must have positive width and height")

    edges = _canonical_edges(scene)
    linework = _breaklines(scene, bounds, flat_resolution)
    pslg = build_hydrology_pslg(scene, bounds, flat_resolution)
    _, _, _, slope_area = _parcel_channel_geometry(scene, bounds)
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
        if not slope_area.covers(sample) and linework.distance(sample) >= 0.15 * flat_resolution:
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
    points = [(x, y, _profile_elevation(x, y, edges)) for x, y in xy]
    triangles = []
    for triangle in triangulation.triangles_iter():
        tri = tuple(int(i) for i in triangle.vertices)
        a, b, c = (xy[i] for i in tri)
        twice_area = abs((b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1]))
        # Defensive guard for downstream physics/rendering clients.
        if twice_area > 1e-12:
            triangles.append(tri)
    breakline_edges = [(int(edge.v1), int(edge.v2)) for edge in triangulation.fixed_edges_iter()]
    return GroundMesh(points=points, triangles=triangles, breakline_edges=breakline_edges)


def write_ground_mesh_usda(mesh: GroundMesh, path: str, prim_name: str = "Ground") -> None:
    """Write a self-contained USDA containing one double-sided mesh."""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    min_pt = tuple(min(p[i] for p in mesh.points) for i in range(3))
    max_pt = tuple(max(p[i] for p in mesh.points) for i in range(3))
    colors = [(0.18, 0.38, 0.08) if p[2] >= -1e-9 else (0.34, 0.20, 0.08) for p in mesh.points]

    def vec(values):
        return ",\n        ".join(f"({a:.9g}, {b:.9g}, {c:.9g})" for a, b, c in values)

    counts = ", ".join("3" for _ in mesh.triangles)
    indices = ", ".join(str(i) for tri in mesh.triangles for i in tri)
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
    point3f[] points = [
        {vec(mesh.points)}
    ]
    color3f[] primvars:displayColor = [
        {vec(colors)}
    ] (
        interpolation = "vertex"
    )
    uniform token subdivisionScheme = "none"
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
    fixed_xy = [
        [(mesh.points[a][0], mesh.points[a][1]), (mesh.points[b][0], mesh.points[b][1])]
        for a, b in fixed
    ]

    fig = plt.figure(figsize=(14, 6), constrained_layout=True)
    plan = fig.add_subplot(1, 2, 1)
    plan.add_collection(LineCollection(xy_segments, colors="0.55", linewidths=0.35))
    plan.add_collection(LineCollection(fixed_xy, colors="#e31a1c", linewidths=1.0))
    plan.scatter(
        [p[0] for p in mesh.points], [p[1] for p in mesh.points],
        s=4, c="#0868ac", zorder=3, linewidths=0,
    )
    plan.autoscale()
    plan.set_aspect("equal")
    plan.set_title("plan: vertices, triangles, and breaklines (red)")
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
) -> GroundMesh:
    mesh = build_ground_mesh(scene, bounds, flat_resolution)
    write_ground_mesh_usda(mesh, path)
    return mesh
