"""
farm_mesh_prototype.py

Simplified, trunk-free version. A main trunk channel (real-world hydraulic
head, gravity gradient, etc.) is a genuinely useful later refinement, but
combining a fixed external boundary with T-tessellation proved to be a
deep, recurring source of sliver-parcel bugs -- tried from two different
directions (build the trunk into the tessellation's own boundary; tessellate
freely and clip by the trunk afterward), and both hit distinct, serious
failure modes rooted in the same general problem: robustly overlaying two
independently-generated planar geometries without near-miss slivers is a
hard computational-geometry problem that needs tolerance-aware snapping as
a first-class part of the overlay operation (what GEOS's OverlayNG /
shapely.set_precision provide) -- not achievable by hand-rolled patches in
this sandbox. See CLAUDE.md "Key lessons" for the full account.

DECISION: defer the trunk/main-channel concept to later digital-twin work.
This version tessellates the WHOLE domain directly, with every channel
edge treated equivalently (no distinguished gravity-fed trunk). Because
there is no external fixed geometry to combine with, every tessellation
face stays convex throughout (a straight chord cut of a convex polygon is
always convex) -- this removes an entire class of complexity (non-convex
splitting, protected-vertex cleanup) by construction, not by patching.

STATUS: still a sandbox prototype with no network access, so polygon
erosion is hand-rolled (mitre-limit clamp, not a true bevel -- see
erode_polygon_variable's docstring) and should be replaced with
shapely.buffer in the real repo. The networkx-based logic (MST, connected
components) already uses the real library and can be ported directly.

Run this file directly to generate and save plots for several random
seeds in one batch -- always review multiple seeds, not one.
"""

import os

import numpy as np
import networkx as nx
from scipy.spatial import cKDTree
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon
from matplotlib.collections import PatchCollection


# ============================================================================
# Geometry primitives
# ============================================================================

def polygon_area(verts):
    x, y = verts[:, 0], verts[:, 1]
    return 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def signed_area(verts):
    x, y = verts[:, 0], verts[:, 1]
    return 0.5 * (np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def point_in_polygon(pt, verts):
    x, y = pt
    n = len(verts)
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = verts[i]
        xj, yj = verts[j]
        if (yi > y) != (yj > y):
            x_int = (xj - xi) * (y - yi) / (yj - yi + 1e-15) + xi
            if x < x_int:
                inside = not inside
        j = i
    return inside


def _line_edge_intersection(p, d, a, b):
    e = b - a
    denom = d[0] * e[1] - d[1] * e[0]
    if abs(denom) < 1e-12:
        return None
    diff = a - p
    t_edge = (diff[0] * d[1] - diff[1] * d[0]) / denom
    if not (0.0 <= t_edge <= 1.0):
        return None
    return t_edge


def _line_line_intersection(a1, b1, a2, b2):
    d1, d2 = b1 - a1, b2 - a2
    denom = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(denom) < 1e-9:
        return None
    t = ((a2[0] - a1[0]) * d2[1] - (a2[1] - a1[1]) * d2[0]) / denom
    return a1 + t * d1


def min_interior_angle(verts):
    """Smallest interior angle (degrees) over all vertices of a polygon."""
    n = len(verts)
    angles = []
    for i in range(n):
        prev_v, cur, nxt = verts[i - 1], verts[i], verts[(i + 1) % n]
        v1, v2 = prev_v - cur, nxt - cur
        denom = np.linalg.norm(v1) * np.linalg.norm(v2)
        if denom < 1e-12:
            continue
        cos_a = np.dot(v1, v2) / denom
        angles.append(np.degrees(np.arccos(np.clip(cos_a, -1, 1))))
    return min(angles) if angles else 180.0


# ============================================================================
# T-tessellation: recursive chord-splitting with T-junctions
# ============================================================================
#
# Chosen over Voronoi/CVT (too much cell-size/shape variance even after
# Lloyd relaxation) and over plain axis-aligned BSP-grid-of-channels (an
# earlier, abandoned design that generated the hydrology graph first and
# derived parcels from it). Every face here stays CONVEX throughout, since
# the only input is the domain rectangle and every cut is a straight chord
# of a convex polygon (always yields two convex polygons) -- this is a
# provable invariant, not something that needs runtime checking.

def cut_angle_for_face(verts):
    """Perpendicular to the face's long axis -- cutting ALONG the long
    axis (a genuine bug found during prototyping) just produces more
    strips in the same direction forever and never converges to
    square-ish faces."""
    minx, miny = verts.min(axis=0)
    maxx, maxy = verts.max(axis=0)
    fw, fh = maxx - minx, maxy - miny
    return np.pi / 2 if fw >= fh else 0.0


def sample_anchor_inside(verts, rng, corner_margin_frac, max_tries=30):
    minx, miny = verts.min(axis=0)
    maxx, maxy = verts.max(axis=0)
    fw, fh = maxx - minx, maxy - miny
    mx, my = fw * corner_margin_frac, fh * corner_margin_frac
    for _ in range(max_tries):
        p = np.array([rng.uniform(minx + mx, maxx - mx), rng.uniform(miny + my, maxy - my)])
        if point_in_polygon(p, verts):
            return p
    return None


def split_convex_polygon(verts, anchor, angle):
    """
    Split a CONVEX polygon with the line through `anchor` at `angle`.
    Assumes exactly two boundary crossings -- guaranteed for a convex
    polygon and a generic line, which holds throughout this design since
    there is no external fixed geometry to introduce concavity. (An
    earlier trunk-integrated design needed a more general, non-convex-
    capable splitter here -- that whole class of complexity is gone now
    that faces are provably convex.)
    """
    n = len(verts)
    d = np.array([np.cos(angle), np.sin(angle)])
    crossings = []
    for i in range(n):
        a, b = verts[i], verts[(i + 1) % n]
        t_edge = _line_edge_intersection(anchor, d, a, b)
        if t_edge is not None:
            point = a + t_edge * (b - a)
            crossings.append((i, point))
    if len(crossings) != 2:
        return None
    (i, pi), (j, pj) = crossings
    if i == j:
        return None
    if i > j:
        i, pi, j, pj = j, pj, i, pi

    poly_a = [pi] + [verts[k] for k in range(i + 1, j + 1)] + [pj]
    poly_b = [pj] + [verts[k] for k in list(range(j + 1, n)) + list(range(0, i + 1))] + [pi]
    return np.array(poly_a), np.array(poly_b)


def t_tessellate_faces(root_faces, seed, min_area_frac=0.015, min_subarea_frac=0.3,
                        angle_jitter_deg=6.0, max_faces=60, corner_margin_frac=0.12,
                        min_interior_angle_deg=50.0):
    """
    Recursively split the largest remaining face until every face is
    below min_area_frac of the total root area, or max_faces is reached.

    min_subarea_frac directly prevents low-AREA slivers: a candidate cut
    is rejected if either resulting sub-face would be smaller than this
    fraction of its PARENT's area.

    min_interior_angle_deg additionally rejects a candidate cut if EITHER
    resulting sub-face would have any interior angle below this threshold
    -- area alone doesn't catch a narrow, acute WEDGE shape (can have
    plenty of area while still being a practically unusable sliver).
    Checking ALL vertices of each candidate sub-face (not just the two new
    cut points) is deliberate: a pre-existing sharp angle inherited
    unchanged from the parent should also block further cutting that
    direction, so the face is left larger (absorbing the corner) rather
    than compounding the problem with smaller sharp pieces.

    LESSON: vertex cleanup applied AFTER tessellation (see
    remove_degenerate_vertices) can still reintroduce a sharp angle this
    check never sees, by collapsing a short-but-not-degenerate edge and
    removing the vertex that was buffering a corner. validate() re-checks
    angles post-cleanup for exactly this reason -- this tessellation-time
    check alone is necessary but not sufficient.
    """
    total_area = sum(polygon_area(f) for f in root_faces)
    rng = np.random.default_rng(seed)
    faces = list(root_faces)

    while len(faces) < max_faces:
        areas = [polygon_area(f) for f in faces]
        idx = int(np.argmax(areas))
        face = faces[idx]
        if areas[idx] < min_area_frac * total_area:
            break

        split_result = None
        for _ in range(25):
            anchor = sample_anchor_inside(face, rng, corner_margin_frac)
            if anchor is None:
                continue
            base_angle = cut_angle_for_face(face)
            angle = base_angle + np.radians(rng.normal(0, angle_jitter_deg))
            result = split_convex_polygon(face, anchor, angle)
            if result is None:
                continue
            poly_a, poly_b = result
            area_a, area_b = polygon_area(poly_a), polygon_area(poly_b)
            if area_a < min_subarea_frac * areas[idx] or area_b < min_subarea_frac * areas[idx]:
                continue
            if min_interior_angle(poly_a) < min_interior_angle_deg or min_interior_angle(poly_b) < min_interior_angle_deg:
                continue
            split_result = (poly_a, poly_b)
            break

        if split_result is None:
            areas[idx] = -1
            if max(areas) < 0:
                break
            continue

        faces.pop(idx)
        faces.extend(split_result)

    return faces


# ============================================================================
# Vertex cleanup
# ============================================================================
#
# Needed for erosion stability (see erode_polygon_variable): a degenerate
# near-duplicate vertex, or a near-collinear one, causes numerically
# unstable mitre-line intersections. No "protected points" concept is
# needed here anymore -- that machinery existed specifically to stop this
# cleanup from deleting genuine trunk bend vertices, which no longer exist
# in this design.

def remove_degenerate_vertices(verts, min_edge_len):
    """
    Collapse near-duplicate consecutive vertices (edges shorter than
    min_edge_len) by MERGING them to their midpoint, not by deleting one
    endpoint outright. Deleting one endpoint asymmetrically favors
    whichever vertex survives -- confirmed empirically to measurably
    shrink the polygon's area and create an artificially sharp angle at
    the surviving vertex (a 4-vertex face lost 20 of its 150 sq units and
    gained a 29 degree corner from a single dropped vertex). Merging to
    the midpoint preserves area far better and doesn't privilege either
    original edge's direction.

    min_edge_len should scale with the erosion standoff (e.g.
    2 * standoff) -- a fixed constant threshold works by luck on some
    seeds and fails on others, since any edge shorter than roughly twice
    the erosion distance will cause local-feature-size problems in
    erode_polygon_variable() regardless of the specific seed.
    """
    n = len(verts)
    if n <= 3:
        return verts
    result = [verts[0].copy()]
    for k in range(1, n):
        if np.linalg.norm(result[-1] - verts[k]) < min_edge_len:
            result[-1] = (result[-1] + verts[k]) / 2.0
        else:
            result.append(verts[k].copy())
    if len(result) > 3 and np.linalg.norm(result[-1] - result[0]) < min_edge_len:
        result[0] = (result[0] + result[-1]) / 2.0
        result.pop()
    return np.array(result) if len(result) >= 3 else verts


def remove_near_collinear_vertices(verts, angle_tol_deg):
    """Drop vertices whose interior angle is within angle_tol_deg of 180
    (nearly straight -- a chord-cutting artifact, not a real corner)."""
    n = len(verts)
    cleaned = []
    for i in range(n):
        cur = verts[i]
        prev_v, nxt = verts[i - 1], verts[(i + 1) % n]
        v1, v2 = prev_v - cur, nxt - cur
        denom = np.linalg.norm(v1) * np.linalg.norm(v2)
        if denom < 1e-12:
            continue
        cos_a = np.dot(v1, v2) / denom
        angle = np.degrees(np.arccos(np.clip(cos_a, -1, 1)))
        if abs(180 - angle) < angle_tol_deg:
            continue
        cleaned.append(cur)
    return np.array(cleaned) if len(cleaned) >= 3 else verts


# ============================================================================
# Erosion (road setback)
# ============================================================================
#
# See CLAUDE.md "Library usage": replace with
# shapely.buffer(-standoff, join_style="mitre", mitre_limit=...) in the
# real repo. Every distinct bug found here (corner-mitre mismatch,
# degenerate-vertex instability, near-collinear-vertex instability,
# local-feature-size violation) is a known, solved problem for a real
# offsetting library and NOT something to keep hand-patching.

def erode_polygon_variable(verts, edge_distances, mitre_limit=2.0):
    """
    Per-edge offset + consecutive-edge mitre intersection, with a mitre-
    limit CLAMP: if the mitre point would land further than
    mitre_limit * standoff from the original vertex, pull it back along
    the same direction instead. This bounds how far any one corner's
    erosion can extend.

    NOTE: a true BEVEL (replacing the mitre point with two points, the
    standard approach in real offsetting libraries) was tried and
    introduced NEW self-intersections at exactly the sharpest corners it
    was meant to fix. The simple clamp used here is a lower-risk
    approximation for this hand-rolled prototype (no new vertices or
    edges, so it can't introduce a new crossing) -- it is NOT a
    substitute for a real bevel; port to shapely instead of refining
    this further.

    Takes a per-edge distance array (currently always called with a
    uniform value) so a real, robust variable setback (e.g. wider near a
    future reintroduced trunk) can be added later without changing call
    sites.
    """
    if signed_area(verts) < 0:
        verts = verts[::-1]
        edge_distances = edge_distances[::-1]
    n = len(verts)
    offset_edges = []
    for i in range(n):
        a, b = verts[i], verts[(i + 1) % n]
        edge_dir = (b - a) / np.linalg.norm(b - a)
        inward_normal = np.array([-edge_dir[1], edge_dir[0]])
        d = edge_distances[i]
        offset_edges.append((a + inward_normal * d, b + inward_normal * d))

    new_verts = []
    for i in range(n):
        a1, b1 = offset_edges[i - 1]
        a2, b2 = offset_edges[i]
        p = _line_line_intersection(a1, b1, a2, b2)
        d = edge_distances[i]
        if p is None:
            new_verts.append(a2)
            continue
        dist = np.linalg.norm(p - verts[i])
        if dist > mitre_limit * d and dist > 1e-9:
            direction = (p - verts[i]) / dist
            p = verts[i] + direction * (mitre_limit * d)
        new_verts.append(p)
    return np.array(new_verts)


# ============================================================================
# Roads: every parcel gets its own full eroded perimeter
# ============================================================================
#
# LESSON: an earlier one-sided-frontage design (pick one side per shared
# channel via adjacency detection) repeatedly produced T-junction bugs
# where some channels ended up with a road on NEITHER side. Full-
# perimeter-per-parcel can't produce a coverage gap by construction, at
# the cost of a doubled road along every shared channel.

def build_full_perimeter_roads(eroded_faces):
    road_segments, segment_face_id = [], []
    for face_idx, ef in enumerate(eroded_faces):
        n = len(ef)
        for i in range(n):
            road_segments.append((ef[i], ef[(i + 1) % n]))
            segment_face_id.append(face_idx)
    return road_segments, segment_face_id


# ============================================================================
# Crossings: STRICT collinearity, vertex-to-vertex only -- no new points
# ============================================================================
#
# A crossing is only valid if it connects two EXISTING within-parcel road
# vertices (never a constructed point -- no edge midpoints, no nearest-
# point-on-edge projections), and it must be collinear with a real road
# edge on BOTH sides it connects: the crossing continues, in a genuine
# straight line, an edge that already exists at each endpoint.
#
# LESSON: two earlier versions of this got it wrong in different ways.
# (1) A version that projected a straight line to whatever point on the
# opposite boundary that lands (via edge_fraction_point at a detected
# overlap's midpoint) introduced NEW points not present in the road
# network at all -- rejected, since every crossing vertex must be a
# subset of the actual within-parcel road vertices.
# (2) A version that allowed landing on a non-vertex point along a plain
# "supercell" edge (checking perpendicularity there instead of
# collinearity, since a mid-edge point has no corner to be collinear
# with) was ALSO rejected -- it's still constructing a point that isn't
# an existing road vertex. The correct behavior: if no existing vertex
# lines up collinearly on both sides, there simply is no valid crossing
# there. This is a real, accepted consequence -- some parcels may end up
# with fewer, or occasionally no, straight crossings to a given neighbor,
# and generate_validated()'s retry wrapper handles the resulting rare
# full-disconnection case by trying a different seed, the same way it
# already handles the rare sharp-angle case.

def incident_edge_directions(eroded_face, idx):
    """Unit directions of the two eroded-polygon edges meeting at vertex
    idx: the direction arriving at idx, and the direction already
    leaving idx."""
    n = len(eroded_face)
    prev_v, cur, next_v = eroded_face[idx - 1], eroded_face[idx], eroded_face[(idx + 1) % n]
    dir_in = cur - prev_v
    dir_out = next_v - cur
    dir_in = dir_in / (np.linalg.norm(dir_in) + 1e-12)
    dir_out = dir_out / (np.linalg.norm(dir_out) + 1e-12)
    return dir_in, dir_out


def is_collinear(dir_a, dir_b, angle_tol_deg=1e-3):
    """
    True if unit vectors dir_a and dir_b lie along the same line --
    parallel OR anti-parallel, direction-agnostic (collinearity doesn't
    care which way either vector points, only whether they share a line).
    Uses the 2D cross product (signed perpendicular component) rather
    than a signed dot product specifically so both directions along the
    line count as collinear without needing separate +/- cases.
    """
    cross = dir_a[0] * dir_b[1] - dir_a[1] * dir_b[0]
    return abs(cross) < np.sin(np.radians(angle_tol_deg))


def straight_vertex_crossings(eroded_faces, connect_radius, angle_tol_deg=1e-3):
    """
    Every pair of EXISTING eroded-boundary vertices (never a constructed
    point) belonging to different parcels, within connect_radius of each
    other, kept only if the connecting segment is collinear with an
    incident road edge at BOTH endpoints. O(V^2) over all vertices --
    fine at this scale (a few hundred vertices), not meant to scale
    without a spatial index.
    """
    all_verts = []  # (face_idx, local_idx, point)
    for fi, ef in enumerate(eroded_faces):
        for vi in range(len(ef)):
            all_verts.append((fi, vi, ef[vi]))

    incident = {(fi, vi): incident_edge_directions(eroded_faces[fi], vi) for fi, vi, _ in all_verts}

    crossings, crossing_faces = [], []
    n = len(all_verts)
    for a in range(n):
        fi, vi, V = all_verts[a]
        dir_in_v, dir_out_v = incident[(fi, vi)]
        for b in range(a + 1, n):
            fj, vj, W = all_verts[b]
            if fi == fj:
                continue
            d = np.linalg.norm(W - V)
            if d < 1e-9 or d > connect_radius:
                continue
            crossing_dir = (W - V) / d

            ok_v = is_collinear(crossing_dir, dir_in_v, angle_tol_deg) or is_collinear(crossing_dir, dir_out_v, angle_tol_deg)
            if not ok_v:
                continue
            dir_in_w, dir_out_w = incident[(fj, vj)]
            ok_w = is_collinear(crossing_dir, dir_in_w, angle_tol_deg) or is_collinear(crossing_dir, dir_out_w, angle_tol_deg)
            if not ok_w:
                continue

            crossings.append((V, W))
            crossing_faces.append((fi, fj))

    return crossings, crossing_faces


def parcel_components(n_faces, crossing_faces):
    """Connected components of the whole-farm parcel-adjacency graph,
    built directly from which faces each crossing actually connects."""
    g = nx.Graph()
    g.add_nodes_from(range(n_faces))
    g.add_edges_from(crossing_faces)
    return nx.number_connected_components(g)


# ============================================================================
# Orchestration

# ============================================================================

def generate(bounds, seed, standoff=1.0, max_faces=60,
             min_interior_angle_deg=50.0, connect_radius=None, crossing_angle_tol_deg=1e-3):
    if connect_radius is None:
        connect_radius = standoff * 2.5

    minx, miny, maxx, maxy = bounds
    domain = np.array([[minx, miny], [maxx, miny], [maxx, maxy], [minx, maxy]])

    faces = t_tessellate_faces([domain], seed=seed, max_faces=max_faces,
                                 min_interior_angle_deg=min_interior_angle_deg)

    min_edge_len = 2.0 * standoff
    faces = [remove_degenerate_vertices(f, min_edge_len) for f in faces]
    faces = [remove_near_collinear_vertices(f, 10.0) for f in faces]

    eroded_faces = [erode_polygon_variable(f, np.full(len(f), standoff)) for f in faces]
    road_segments, segment_face_id = build_full_perimeter_roads(eroded_faces)

    crossings, crossing_faces = straight_vertex_crossings(
        eroded_faces, connect_radius, angle_tol_deg=crossing_angle_tol_deg)

    return dict(faces=faces, eroded_faces=eroded_faces,
                road_segments=road_segments, crossings=crossings,
                crossing_faces=crossing_faces)


# ============================================================================
# Validation
# ============================================================================

def _seg_intersect(p1, p2, p3, p4):
    d1, d2 = p2 - p1, p4 - p3
    denom = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(denom) < 1e-12:
        return False
    t = ((p3[0] - p1[0]) * d2[1] - (p3[1] - p1[1]) * d2[0]) / denom
    u = ((p3[0] - p1[0]) * d1[1] - (p3[1] - p1[1]) * d1[0]) / denom
    return 1e-9 < t < 1 - 1e-9 and 1e-9 < u < 1 - 1e-9


def polygon_self_intersects(verts):
    n = len(verts)
    for i in range(n):
        for j in range(i + 1, n):
            if j == i or (j + 1) % n == i or (i + 1) % n == j:
                continue
            if _seg_intersect(verts[i], verts[(i + 1) % n], verts[j], verts[(j + 1) % n]):
                return True
    return False


def validate(result, bounds, faces_raw=None, min_interior_angle_deg=50.0):
    """
    Returns a list of issue strings; empty means all checks passed.

    NOTE on area conservation: post-cleanup area is checked against a
    LOOSE tolerance (0.5% of domain area), not near-zero. Removing a
    near-degenerate vertex from a polygon necessarily trims a small "ear"
    of area -- this is an intentional, expected side effect of the
    erosion-stability cleanup (see remove_degenerate_vertices), not a
    bug. If faces_raw (the pre-cleanup tessellation output) is provided,
    that IS checked with a tight tolerance, since the tessellation
    algorithm itself should conserve area exactly.
    """
    issues = []
    faces = result["faces"]
    eroded_faces = result["eroded_faces"]

    for i, ef in enumerate(eroded_faces):
        if polygon_self_intersects(ef):
            issues.append(f"eroded face {i} is self-intersecting")

    # LESSON: tessellation-time angle rejection alone is not sufficient --
    # post-cleanup re-check catches sharp angles introduced by collapsing
    # a short edge and removing a corner-buffering vertex. This is a rare
    # (~2% of seeds observed) but real residual tension between erosion-
    # stability cleanup and angle-niceness that isn't worth chasing
    # further by hand -- see generate_validated()'s retry-on-failure
    # wrapper for the pragmatic mitigation.
    for i, f in enumerate(faces):
        ang = min_interior_angle(f)
        if ang < min_interior_angle_deg:
            issues.append(f"face {i} has a sharp interior angle ({ang:.1f} deg)")

    domain_area = (bounds[2] - bounds[0]) * (bounds[3] - bounds[1])

    if faces_raw is not None:
        raw_area = sum(polygon_area(f) for f in faces_raw)
        if abs(domain_area - raw_area) > 1e-3:
            issues.append(f"RAW tessellation area not conserved (algorithm bug): "
                           f"domain={domain_area:.2f}, sum={raw_area:.2f}")

    total_area = sum(polygon_area(f) for f in faces)
    area_tolerance = 0.005 * domain_area
    if abs(domain_area - total_area) > area_tolerance:
        issues.append(f"post-cleanup area drifted more than expected: "
                       f"domain={domain_area:.2f}, sum_of_faces={total_area:.2f} "
                       f"(tolerance={area_tolerance:.2f})")

    n_components = parcel_components(len(faces), result["crossing_faces"])
    if n_components > 1:
        issues.append(f"road network is not fully connected: {n_components} separate components")

    return issues


def generate_validated(bounds, seed, max_seed_tries=10, **kwargs):
    """
    Pragmatic wrapper: generate() occasionally (rarely, ~2% of seeds
    observed) produces one sharp-angled face from an inherent, small
    tension between erosion-stability vertex cleanup and angle-niceness
    (see validate()'s docstring) -- not worth chasing further by hand.
    Retrying with a nearby seed is cheap and reliable: try the requested
    seed first, then seed+1000, +2000, ... until validate() passes or
    max_seed_tries is exhausted (in which case the last attempt is
    returned along with its issues, so the caller still gets a result).
    """
    for attempt in range(max_seed_tries):
        trial_seed = seed if attempt == 0 else seed + attempt * 1000
        result = generate(bounds, trial_seed, **kwargs)
        issues = validate(result, bounds)
        if not issues:
            return result, issues, trial_seed
    return result, issues, trial_seed


# ============================================================================
# Plotting
# ============================================================================

def plot_result(result, seed_label=None):
    faces, eroded_faces = result["faces"], result["eroded_faces"]
    fig, ax = plt.subplots(figsize=(10, 10))
    patches = [MplPolygon(f, closed=True) for f in faces]
    rng = np.random.default_rng(0)
    colors = rng.uniform(0.75, 0.95, size=len(patches))
    collection = PatchCollection(patches, array=colors, cmap="Greens", edgecolor="gray", linewidth=0.4, alpha=0.5)
    ax.add_collection(collection)

    for ef in eroded_faces:
        ax.add_patch(plt.Polygon(ef, closed=True, fill=False, edgecolor="darkorange", linewidth=0.7, zorder=3))

    for i, (p0, p1) in enumerate(result["road_segments"]):
        ax.plot([p0[0], p1[0]], [p0[1], p1[1]], color="saddlebrown", linewidth=1.5, zorder=4,
                 label="road" if i == 0 else None)

    for i, (p0, p1) in enumerate(result["crossings"]):
        ax.plot([p0[0], p1[0]], [p0[1], p1[1]], color="crimson", linewidth=1.6, zorder=5,
                 label="crossing" if i == 0 else None)

    ax.autoscale_view()
    ax.set_aspect("equal")
    ax.legend(loc="upper right", fontsize=9)
    title = "Farm mesh prototype: T-tessellation + full-perimeter roads + minimum crossings (no trunk)"
    if seed_label is not None:
        title += f"\nseed={seed_label}"
    ax.set_title(title)
    return fig, ax


# ============================================================================
# Batch entry point -- ALWAYS generate multiple seeds, never just one.
# ============================================================================

if __name__ == "__main__":
    bounds = (0, 0, 100, 100)
    seeds = [1, 2, 3, 4, 5]

    for seed in seeds:
        result, issues, used_seed = generate_validated(bounds, seed)

        status = "OK" if not issues else f"{len(issues)} ISSUE(S)"
        retry_note = f" (used seed {used_seed} after retry)" if used_seed != seed else ""
        print(f"seed={seed}{retry_note}: {len(result['faces'])} parcels, "
              f"{len(result['road_segments'])} road edges, "
              f"{len(result['crossings'])} crossings -- {status}")
        for issue in issues:
            print(f"    ! {issue}")

        fig, ax = plot_result(result, seed_label=seed)
        out_dir = os.path.join(os.path.dirname(__file__), "..", "debug_out", "examples")
        os.makedirs(out_dir, exist_ok=True)
        plt.savefig(os.path.join(out_dir, f"farm_mesh_seed_{seed}.png"), dpi=150, bbox_inches="tight")
        plt.close(fig)
