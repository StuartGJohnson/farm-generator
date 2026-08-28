"""
generation/crops.py

Stage 7 of the generation DAG: trees + weeds per `CropArea`. Retargets
the existing grid-orchard generator's design to consume an arbitrary
polygon (a tessellated parcel) instead of a fixed rectangle with a
global origin -- clipping against the actual polygon is required since
T-tessellation parcels are not clean rectangles.

Row alignment (see CLAUDE_update1.md): rows are NOT laid out at a fixed
global compass angle. Real farmers align rows to the field's own edges,
so for each parcel a random vertex of the parcel polygon is picked as
the planting grid's anchor corner, and one of that vertex's two incident
edges is randomly chosen as the row direction -- the other incident edge
(at the SAME vertex) becomes the column direction. Because every
tessellated face is convex (see CLAUDE.md "Tessellation"), the entire
polygon lies within the wedge swept between those two edge directions
from that vertex.

Headland/sideland trim: `row_dir`/`col_dir` are unit vectors, so a
point's (i, j) coordinates in that (generally non-orthogonal) basis are
each a literal along-axis distance in meters, even though the two axes
aren't perpendicular to each other. `headland_width` is the minimum
clearance from the parcel boundary measured along the row direction (row
ends -- tractor turning space); `sideland_width` is the minimum clearance
from the parcel boundary measured along the column direction (the
outermost row's clearance from the field edge).

Both must be measured against the LOCAL boundary each individual row/
column actually hits, not a single global figure -- an earlier version
trimmed against the parcel's global max (i, j) reach (the single farthest
vertex), which under-trimmed every row/column that hit the real boundary
sooner than that one extreme vertex, letting trees land at the true edge
(and inside the road standoff strip just beyond it). The fix: for each
row, intersect that row's actual line with the polygon (shapely) to get
ITS OWN local entry/exit before trimming by headland_width; symmetrically
per column with sideland_width. This is exact regardless of parcel shape
and still additionally clipped against the real polygon as a final
safety net.

Weeds are a simple uniform low-density zone over the whole parcel this
phase (headland + row-gap correlation is a later phase -- see CLAUDE.md
"Generation order" #7).

Boundary-tie tolerance: a row's near-side entry is mathematically exactly
0 whenever that row still lies along the anchor vertex's own row_dir edge
(the common case for the first several rows), but `_local_span`'s real
shapely intersection returns that as ~1e-15-level noise around zero, not
a bit-exact 0. Because `headland_width`/`sideland_width`/`tree_spacing`/
`row_spacing` are typically round numbers, the first candidate tree often
lands EXACTLY on `i_lo`/`j_lo` (an exact tie), so which way that noise
happens to fall for a given row/column flips inclusion -- confirmed
directly: `row_span[0]` alternated between -1e-15 and +1e-15 across
consecutive rows of one real parcel, correctly dropping the first tree
of a row whenever it landed on the positive side. This produced a
periodic "missing first tree" pattern at row/column starts, distinct
from the expected, unavoidable staircase clipping at row/column ENDS
(see above) -- that one is real geometry (parcel edges generally aren't
perpendicular to row_dir/col_dir), this one was a floating-point tie.
`_BOUNDARY_EPS` below absorbs exactly that noise without loosening the
clearance guarantee by anything a farmer would notice.
"""

from __future__ import annotations

import numpy as np
from shapely.geometry import LineString, Point, Polygon

from farm_ir.schema import CropArea, FarmScene, PlantingSpec, TreeInstance, WeedSpec, WeedZone

# Absorbs GEOS intersection noise (~1e-15 m observed) at exact-tie
# boundary comparisons, without loosening the real headland/sideland
# clearance guarantee by anything physically meaningful.
_BOUNDARY_EPS = 1e-6


def _pick_row_and_col_dirs(parcel_coords: np.ndarray, rng: np.random.Generator):
    """
    Anchor the planting grid to a random corner of the parcel: pick a
    random vertex, then randomly assign one of its two incident edges as
    the row direction and the other as the column direction. Returns
    (origin, row_dir, col_dir) with row_dir/col_dir unit vectors pointing
    from the vertex into the polygon along its own edges.
    """
    n = len(parcel_coords)
    v_idx = int(rng.integers(0, n))
    origin = parcel_coords[v_idx]
    prev_v = parcel_coords[v_idx - 1]
    next_v = parcel_coords[(v_idx + 1) % n]

    dir_to_prev = prev_v - origin
    dir_to_next = next_v - origin
    dir_to_prev = dir_to_prev / (np.linalg.norm(dir_to_prev) + 1e-12)
    dir_to_next = dir_to_next / (np.linalg.norm(dir_to_next) + 1e-12)

    if rng.random() < 0.5:
        row_dir, col_dir = dir_to_next, dir_to_prev
    else:
        row_dir, col_dir = dir_to_prev, dir_to_next

    return origin, row_dir, col_dir


def _oblique_coord(point: np.ndarray, origin: np.ndarray, basis_inv: np.ndarray, axis: int) -> float:
    return float(((point - origin) @ basis_inv.T)[axis])


def _local_span(poly: Polygon, through_point: np.ndarray, direction: np.ndarray,
                 origin: np.ndarray, basis_inv: np.ndarray, axis: int):
    """
    Intersect the infinite line through `through_point` in `direction`
    with `poly`, and return the (lo, hi) oblique coordinate (axis 0 = the
    row_dir/i component, axis 1 = the col_dir/j component) that
    intersection spans -- i.e. THIS row's (or column's) own real entry/
    exit against the actual parcel boundary, not a global figure. Returns
    None if the line misses the polygon entirely.
    """
    minx, miny, maxx, maxy = poly.bounds
    reach = float(np.hypot(maxx - minx, maxy - miny)) * 2.0 + 1.0
    line = LineString([tuple(through_point - direction * reach), tuple(through_point + direction * reach)])
    inter = poly.intersection(line)
    if inter.is_empty:
        return None

    coords = []
    if hasattr(inter, "geoms"):
        for g in inter.geoms:
            if hasattr(g, "coords"):
                coords.extend(list(g.coords))
    elif hasattr(inter, "coords"):
        coords = list(inter.coords)
    if not coords:
        return None

    vals = [_oblique_coord(np.array(c), origin, basis_inv, axis) for c in coords]
    return min(vals), max(vals)


def _generate_tree_grid(origin: np.ndarray, row_dir: np.ndarray, col_dir: np.ndarray,
                         poly: Polygon, spec: PlantingSpec, rng: np.random.Generator):
    basis_inv = np.linalg.inv(np.column_stack([row_dir, col_dir]))

    # Generous OUTER loop bounds only -- how far a row/column could
    # possibly reach. Exact trimming happens per-row/per-column below via
    # real polygon intersections against THIS row's/column's own line,
    # not from these bounds.
    minx, miny, maxx, maxy = poly.bounds
    diag = float(np.hypot(maxx - minx, maxy - miny))
    k_max = int(np.ceil(diag / spec.tree_spacing)) + 2
    l_max = int(np.ceil(diag / spec.row_spacing)) + 2

    species_names = list(spec.species_mix.keys())
    species_probs = np.array(list(spec.species_mix.values()), dtype=float)
    species_probs = species_probs / species_probs.sum()

    # A column's local (sideland) span depends only on k (the row_dir
    # offset), shared across every row -- compute once per k and reuse.
    col_span_cache: dict[int, tuple[float, float] | None] = {}

    def col_span(k: int):
        if k not in col_span_cache:
            i = spec.headland_width + k * spec.tree_spacing
            col_span_cache[k] = _local_span(poly, origin + row_dir * i, col_dir, origin, basis_inv, axis=1)
        return col_span_cache[k]

    trees = []
    for l in range(l_max + 1):
        # Phase the grid at (headland_width, sideland_width) from the
        # anchor vertex -- NOT at (0, 0) filtered afterward by those
        # margins. The two give the same row-axis result here only by
        # coincidence (tree_spacing=1.0 divides headland_width=2.0
        # evenly); row_spacing=1.5 does NOT divide sideland_width=2.0
        # evenly, so phasing from 0 and filtering rounded the first
        # surviving column up to 3.0m instead of landing on 2.0m -- a
        # real bug, not intended quantization. Anchoring the phase at the
        # margin itself makes the first tree land exactly at
        # (headland_width, sideland_width) whenever that row/column's own
        # local boundary allows it (see CLAUDE.md-style corner geometry
        # above), matching the standoff-eroded road's own corner offset.
        j = spec.sideland_width + l * spec.row_spacing
        row_span = _local_span(poly, origin + col_dir * j, row_dir, origin, basis_inv, axis=0)
        if row_span is None:
            continue
        i_lo, i_hi = row_span[0] + spec.headland_width, row_span[1] - spec.headland_width
        if i_hi < i_lo:
            continue  # this row is too short for headland clearance at both ends

        for k in range(k_max + 1):
            i = spec.headland_width + k * spec.tree_spacing
            if i < i_lo - _BOUNDARY_EPS or i > i_hi + _BOUNDARY_EPS:
                continue

            cs = col_span(k)
            if cs is None:
                continue
            j_lo, j_hi = cs[0] + spec.sideland_width, cs[1] - spec.sideland_width
            if j < j_lo - _BOUNDARY_EPS or j > j_hi + _BOUNDARY_EPS:
                continue  # this column is too short for sideland clearance at both ends

            p = origin + row_dir * i + col_dir * j
            if not poly.covers(Point(p[0], p[1])):
                continue  # defensive final check against the real polygon (covers, not contains,
                          # so a point that lands exactly on the boundary isn't spuriously dropped)
            species = rng.choice(species_names, p=species_probs)
            trees.append(dict(
                position=(float(p[0]), float(p[1])),
                species=str(species),
                age=float(rng.uniform(2.0, 15.0)),
            ))
    return trees


def run(scene: FarmScene, config, rng: np.random.Generator) -> FarmScene:
    tree_idx = 0
    weed_idx = 0

    for pid, parcel in scene.parcels.items():
        if not isinstance(parcel, CropArea):
            continue

        poly = Polygon(parcel.polygon)
        parcel_coords = np.array(parcel.polygon)

        parcel_seed = int(rng.integers(0, 2**63 - 1))
        parcel_rng = np.random.default_rng(parcel_seed)

        origin, row_dir, col_dir = _pick_row_and_col_dirs(parcel_coords, parcel_rng)
        row_angle_deg = float(np.degrees(np.arctan2(row_dir[1], row_dir[0])))

        spec = PlantingSpec(
            row_spacing=config.row_spacing,
            tree_spacing=config.tree_spacing,
            row_angle_deg=row_angle_deg,
            headland_width=config.headland_width,
            sideland_width=config.sideland_width,
            species_mix=dict(config.species_mix),
            seed=parcel_seed,
        )
        parcel.planting = spec

        for t in _generate_tree_grid(origin, row_dir, col_dir, poly, spec, parcel_rng):
            tid = f"tree_{tree_idx:05d}"
            tree_idx += 1
            scene.trees[tid] = TreeInstance(
                id=tid,
                position=t["position"],
                species=t["species"],
                age_years=t["age"],
                canopy_radius=config.canopy_radius,
                trunk_dbh=config.trunk_dbh,
                height=config.tree_height,
                tags={"parcel_id": pid},
            )
            parcel.refs.append(tid)

        wid = f"weed_{weed_idx:04d}"
        weed_idx += 1
        scene.weed_zones[wid] = WeedZone(
            id=wid,
            polygon=parcel.polygon,
            density_params=dict(config.weed_density_params),
            tags={"parcel_id": pid},
        )
        parcel.weeds = WeedSpec(density_params=dict(config.weed_density_params), seed=parcel_seed)
        parcel.weed_zone_refs.append(wid)

    return scene
