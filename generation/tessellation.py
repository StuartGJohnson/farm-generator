"""
generation/tessellation.py

Stages 1-2 of the generation DAG (see CLAUDE.md "Generation order"):
T-tessellation of the domain rectangle into convex parcel faces, followed
by vertex cleanup for erosion stability. Populates `scene.parcels` with
`CropArea` instances (this phase has no farmstead/other parcel type --
see CLAUDE.md "Farmstead: deferred"). `boundary_refs` are left empty here
and filled in by generation/hydrology.py once hydrology edges exist.

The recursive split-selection policy (which face to cut, cut-direction
bias, rejection criteria) is farm-generation POLICY and is hand-rolled by
design. The actual polygon split is delegated to shapely.ops.split (GEOS)
rather than hand-rolled line/edge intersection -- see CLAUDE.md "Library
usage".
"""

from __future__ import annotations

import numpy as np
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import split as shapely_split

from farm_ir.schema import CropArea, FarmScene, ParcelType

from generation._geom_utils import exterior_coords


def min_interior_angle_deg(coords: np.ndarray) -> float:
    """Smallest interior angle (degrees) over all vertices of a polygon
    ring given as an (N, 2) array (no closing duplicate)."""
    n = len(coords)
    angles = []
    for i in range(n):
        prev_v, cur, nxt = coords[i - 1], coords[i], coords[(i + 1) % n]
        v1, v2 = prev_v - cur, nxt - cur
        denom = np.linalg.norm(v1) * np.linalg.norm(v2)
        if denom < 1e-12:
            continue
        cos_a = np.dot(v1, v2) / denom
        angles.append(np.degrees(np.arccos(np.clip(cos_a, -1, 1))))
    return min(angles) if angles else 180.0


def _cut_angle_for_face(coords: np.ndarray) -> float:
    """Perpendicular to the face's long axis. Cutting ALONG the long axis
    is a genuine bug found during prototyping -- it just produces more
    strips in the same direction forever and never converges to
    square-ish faces."""
    minx, miny = coords.min(axis=0)
    maxx, maxy = coords.max(axis=0)
    fw, fh = maxx - minx, maxy - miny
    return np.pi / 2 if fw >= fh else 0.0


def _sample_anchor_inside(poly: Polygon, rng: np.random.Generator,
                           corner_margin_frac: float, max_tries: int = 30):
    minx, miny, maxx, maxy = poly.bounds
    fw, fh = maxx - minx, maxy - miny
    mx, my = fw * corner_margin_frac, fh * corner_margin_frac
    for _ in range(max_tries):
        x = rng.uniform(minx + mx, maxx - mx)
        y = rng.uniform(miny + my, maxy - my)
        if poly.contains(Point(x, y)):
            return np.array([x, y])
    return None


def _split_convex_polygon(poly: Polygon, anchor: np.ndarray, angle: float):
    """
    Split a convex polygon with the infinite line through `anchor` at
    `angle`. Because the only input is the domain rectangle and every cut
    is a straight chord of a convex polygon, this always yields two
    convex polygons -- a provable invariant of this design, not something
    requiring a runtime check (see CLAUDE.md "Tessellation").
    """
    minx, miny, maxx, maxy = poly.bounds
    diag = float(np.hypot(maxx - minx, maxy - miny)) * 2.0 + 1.0
    d = np.array([np.cos(angle), np.sin(angle)])
    p0 = anchor - d * diag
    p1 = anchor + d * diag
    line = LineString([tuple(p0), tuple(p1)])
    pieces = [g for g in shapely_split(poly, line).geoms
              if isinstance(g, Polygon) and g.area > 0]
    if len(pieces) != 2:
        return None
    return pieces[0], pieces[1]


def t_tessellate(bounds, rng: np.random.Generator, max_faces: int = 60,
                  min_area_frac: float = 0.015, min_subarea_frac: float = 0.3,
                  angle_jitter_deg: float = 6.0, corner_margin_frac: float = 0.12,
                  min_interior_angle_deg_thresh: float = 50.0) -> list[Polygon]:
    """
    Recursively split the largest remaining face until every face is
    below min_area_frac of the total root area, or max_faces is reached.

    min_subarea_frac directly prevents low-AREA slivers: a candidate cut
    is rejected if either resulting sub-face would be smaller than this
    fraction of its PARENT's area.

    min_interior_angle_deg_thresh additionally rejects a candidate cut if
    EITHER resulting sub-face would have any interior angle below this
    threshold -- area alone doesn't catch a narrow, acute wedge shape.
    Checking ALL vertices of each candidate sub-face (not just the two
    new cut points) is deliberate: a pre-existing sharp angle inherited
    unchanged from the parent should also block further cutting that
    direction, so the face is left larger (absorbing the corner) rather
    than compounding the problem.
    """
    minx, miny, maxx, maxy = bounds
    domain = Polygon([(minx, miny), (maxx, miny), (maxx, maxy), (minx, maxy)])
    total_area = domain.area
    faces = [domain]

    while len(faces) < max_faces:
        areas = [f.area for f in faces]
        idx = int(np.argmax(areas))
        face = faces[idx]
        if areas[idx] < min_area_frac * total_area:
            break

        split_result = None
        for _ in range(25):
            anchor = _sample_anchor_inside(face, rng, corner_margin_frac)
            if anchor is None:
                continue
            base_angle = _cut_angle_for_face(exterior_coords(face))
            angle = base_angle + np.radians(rng.normal(0, angle_jitter_deg))
            result = _split_convex_polygon(face, anchor, angle)
            if result is None:
                continue
            poly_a, poly_b = result
            area_a, area_b = poly_a.area, poly_b.area
            if area_a < min_subarea_frac * areas[idx] or area_b < min_subarea_frac * areas[idx]:
                continue
            if (min_interior_angle_deg(exterior_coords(poly_a)) < min_interior_angle_deg_thresh or
                    min_interior_angle_deg(exterior_coords(poly_b)) < min_interior_angle_deg_thresh):
                continue
            split_result = (poly_a, poly_b)
            break

        if split_result is None:
            # This face couldn't be split this pass; next iteration recomputes
            # areas fresh and will pick it again unless another face is now
            # larger. See examples/farm_mesh_prototype.py for the identical
            # (empirically 100/100-seed-clean) behavior this ports.
            if len(faces) == 1:
                break
            continue

        faces.pop(idx)
        faces.extend(split_result)

    return faces


# ---------------------------------------------------------------------------
# Vertex cleanup
# ---------------------------------------------------------------------------
# Needed for erosion stability (see generation/roads.py): a degenerate
# near-duplicate vertex, or a near-collinear one, causes numerically
# unstable mitre-line offsets even in shapely's own buffer implementation
# at extreme ratios. No "protected points" concept is needed here -- that
# machinery existed specifically to stop this cleanup from deleting
# genuine trunk bend vertices, which don't exist in this (no-trunk) phase.

def remove_degenerate_vertices(coords: np.ndarray, min_edge_len: float) -> np.ndarray:
    """
    Collapse near-duplicate consecutive vertices (edges shorter than
    min_edge_len) by MERGING them to their midpoint, not by deleting one
    endpoint outright. Deleting one endpoint asymmetrically favors
    whichever vertex survives -- confirmed empirically during prototyping
    to measurably shrink the polygon's area and create an artificially
    sharp angle at the surviving vertex. Merging to the midpoint is a
    better approximation, though it does not fully eliminate a resulting
    sharp angle when going from 4 vertices to 3 -- that residual is
    handled by the retry-on-validation-failure wrapper in
    generation/orchestrator.py, not chased further here.

    min_edge_len should scale with the erosion standoff (e.g.
    2 * standoff) -- a fixed constant threshold works by luck on some
    seeds and fails on others.
    """
    n = len(coords)
    if n <= 3:
        return coords
    result = [coords[0].copy()]
    for k in range(1, n):
        if np.linalg.norm(result[-1] - coords[k]) < min_edge_len:
            result[-1] = (result[-1] + coords[k]) / 2.0
        else:
            result.append(coords[k].copy())
    if len(result) > 3 and np.linalg.norm(result[-1] - result[0]) < min_edge_len:
        result[0] = (result[0] + result[-1]) / 2.0
        result.pop()
    return np.array(result) if len(result) >= 3 else coords


def remove_near_collinear_vertices(coords: np.ndarray, angle_tol_deg: float) -> np.ndarray:
    """Drop vertices whose interior angle is within angle_tol_deg of 180
    (nearly straight -- a chord-cutting artifact, not a real corner)."""
    n = len(coords)
    cleaned = []
    for i in range(n):
        cur = coords[i]
        prev_v, nxt = coords[i - 1], coords[(i + 1) % n]
        v1, v2 = prev_v - cur, nxt - cur
        denom = np.linalg.norm(v1) * np.linalg.norm(v2)
        if denom < 1e-12:
            continue
        cos_a = np.dot(v1, v2) / denom
        angle = np.degrees(np.arccos(np.clip(cos_a, -1, 1)))
        if abs(180 - angle) < angle_tol_deg:
            continue
        cleaned.append(cur)
    return np.array(cleaned) if len(cleaned) >= 3 else coords


# Fixed collinear-cleanup tolerance -- distinct from, and much looser
# than, crossings.py's collinearity tolerance. This one is about
# discarding a chord-cutting artifact vertex (nearly-straight, "looks
# like 180 degrees"), not about verifying a genuine construction
# relationship between two separately-generated edges.
_COLLINEAR_CLEANUP_ANGLE_TOL_DEG = 10.0


def run(scene: FarmScene, config, rng: np.random.Generator) -> FarmScene:
    faces_raw = t_tessellate(
        config.bounds, rng,
        max_faces=config.max_faces,
        min_area_frac=config.min_area_frac,
        min_subarea_frac=config.min_subarea_frac,
        angle_jitter_deg=config.angle_jitter_deg,
        corner_margin_frac=config.corner_margin_frac,
        min_interior_angle_deg_thresh=config.min_interior_angle_deg,
    )

    min_edge_len = 2.0 * config.standoff
    cleaned_coords = []
    for f in faces_raw:
        c = exterior_coords(f)
        c = remove_degenerate_vertices(c, min_edge_len)
        c = remove_near_collinear_vertices(c, _COLLINEAR_CLEANUP_ANGLE_TOL_DEG)
        cleaned_coords.append(c)

    for i, coords in enumerate(cleaned_coords):
        pid = f"parcel_{i:03d}"
        parcel = CropArea(
            id=pid,
            polygon=[(float(x), float(y)) for x, y in coords],
            parcel_type=ParcelType.CROP,
            crop_type=config.crop_type,
            irrigation_type=config.irrigation_type,
        )
        scene.parcels[pid] = parcel

    # RAW (pre-cleanup) total area, stashed for orchestrator.validate()'s
    # tight algorithm-correctness check -- see CLAUDE.md "Area conservation
    # should only be checked strictly on RAW tessellation output".
    raw_area = sum(f.area for f in faces_raw)
    scene.provenance.generator_params["tessellation_raw_area"] = repr(raw_area)
    scene.provenance.generator_params["tessellation_raw_face_count"] = str(len(faces_raw))

    return scene
