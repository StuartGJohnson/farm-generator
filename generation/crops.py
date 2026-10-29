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

Weed zones retain the exact cultivated row centerlines.  Exporters sample
weeds from a lateral probability distribution around those rows; there is
no uniform parcel-wide background population.

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

from farm_ir.schema import CropArea, Depot, FarmScene, ParkingSpot, PlantingSpec, TreeInstance, WeedSpec, WeedZone

# Absorbs GEOS intersection noise (~1e-15 m observed) at exact-tie
# boundary comparisons, without loosening the real headland/sideland
# clearance guarantee by anything physically meaningful.
_BOUNDARY_EPS = 1e-6


def _pick_row_and_col_dirs(
    parcel_coords: np.ndarray,
    rng: np.random.Generator,
    optimize_tree_layout: bool = False,
):
    """
    Anchor the planting grid to a random corner of the parcel: pick a
    random vertex, then assign one of its two incident edges as the row
    direction and the other as the column direction.  With optimized
    layout the longer incident edge is always the row; otherwise the
    assignment is random. Returns
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
    prev_length = np.linalg.norm(dir_to_prev)
    next_length = np.linalg.norm(dir_to_next)
    dir_to_prev = dir_to_prev / (prev_length + 1e-12)
    dir_to_next = dir_to_next / (next_length + 1e-12)

    if optimize_tree_layout and not np.isclose(prev_length, next_length):
        if prev_length > next_length:
            row_dir, col_dir = dir_to_prev, dir_to_next
        else:
            row_dir, col_dir = dir_to_next, dir_to_prev
    elif rng.random() < 0.5:
        row_dir, col_dir = dir_to_next, dir_to_prev
    else:
        row_dir, col_dir = dir_to_prev, dir_to_next

    return origin, row_dir, col_dir


def _oblique_coord(point: np.ndarray, origin: np.ndarray, basis_inv: np.ndarray, axis: int) -> float:
    return float(((point - origin) @ basis_inv.T)[axis])


def _local_spans(poly: Polygon, through_point: np.ndarray, direction: np.ndarray,
                 origin: np.ndarray, basis_inv: np.ndarray, axis: int,
                 excluded_polygon: Polygon | None = None):
    """
    Intersect the infinite line through `through_point` in `direction`
    with `poly`, and return each connected interval in oblique coordinates.
    A depot makes a hole in the crop polygon, so taking the minimum and
    maximum across all components would falsely plant through the depot.
    """
    minx, miny, maxx, maxy = poly.bounds
    reach = float(np.hypot(maxx - minx, maxy - miny)) * 2.0 + 1.0
    line = LineString([tuple(through_point - direction * reach), tuple(through_point + direction * reach)])
    inter = poly.intersection(line)
    if excluded_polygon is not None:
        inter = inter.difference(excluded_polygon)
    if inter.is_empty:
        return []
    parts = inter.geoms if hasattr(inter, "geoms") else [inter]
    spans = []
    for part in parts:
        if not isinstance(part, LineString):
            continue
        vals = [_oblique_coord(np.array(c), origin, basis_inv, axis) for c in part.coords]
        if vals:
            spans.append((min(vals), max(vals)))
    return sorted(spans)


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
    col_span_cache: dict[int, list[tuple[float, float]]] = {}

    def col_span(k: int):
        if k not in col_span_cache:
            i = spec.headland_width + k * spec.tree_spacing
            col_span_cache[k] = _local_spans(
                poly, origin + row_dir * i, col_dir, origin, basis_inv, axis=1)
        return col_span_cache[k]

    trees = []
    row_centerlines = []
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
        for row_span in _local_spans(poly, origin + col_dir * j, row_dir,
                                     origin, basis_inv, axis=0):
            i_lo = row_span[0] + spec.headland_width
            i_hi = row_span[1] - spec.headland_width
            if i_hi < i_lo:
                continue

            row_tree_count = 0
            row_number = len(row_centerlines) + 1
            for k in range(k_max + 1):
                i = spec.headland_width + k * spec.tree_spacing
                if i < i_lo - _BOUNDARY_EPS or i > i_hi + _BOUNDARY_EPS:
                    continue
                if not any(j >= lo + spec.sideland_width - _BOUNDARY_EPS and
                           j <= hi - spec.sideland_width + _BOUNDARY_EPS
                           for lo, hi in col_span(k)):
                    continue
                p = origin + row_dir * i + col_dir * j
                if not poly.covers(Point(p[0], p[1])):
                    continue
                species = rng.choice(species_names, p=species_probs)
                trees.append(dict(
                    position=(float(p[0]), float(p[1])),
                    species=str(species),
                    age=float(rng.uniform(2.0, 15.0)),
                    row_number=row_number,
                ))
                row_tree_count += 1

            if row_tree_count:
                start = origin + col_dir * j + row_dir * i_lo
                end = origin + col_dir * j + row_dir * i_hi
                row_centerlines.append([
                    (float(start[0]), float(start[1])),
                    (float(end[0]), float(end[1])),
                ])
    return trees, row_centerlines


def _depot_exclusion_polygon(depot_poly: Polygon, row_dir: np.ndarray,
                             col_dir: np.ndarray, headland_m: float,
                             sideland_m: float) -> Polygon:
    """Expand the depot rectangle in planting coordinates into a parallelogram."""
    anchor = np.asarray(depot_poly.exterior.coords[0], dtype=float)
    basis_inv = np.linalg.inv(np.column_stack([row_dir, col_dir]))
    local = np.asarray([basis_inv @ (np.asarray(xy) - anchor)
                        for xy in depot_poly.exterior.coords[:-1]])
    i_lo, i_hi = local[:, 0].min() - headland_m, local[:, 0].max() + headland_m
    j_lo, j_hi = local[:, 1].min() - sideland_m, local[:, 1].max() + sideland_m
    return Polygon([tuple(anchor + i * row_dir + j * col_dir)
                    for i, j in ((i_lo, j_lo), (i_hi, j_lo),
                                 (i_hi, j_hi), (i_lo, j_hi))])


def _exclude_depot_from_grid(trees: list[dict], row_centerlines: list,
                             exclusion: Polygon) -> tuple[list[dict], list]:
    """Keep the field-edge grid and remove only tree centers in the exclusion."""
    retained = []
    clipped_rows = []
    for old_number, row in enumerate(row_centerlines, 1):
        surviving = [tree for tree in trees if tree["row_number"] == old_number
                     and not exclusion.covers(Point(tree["position"]))]
        if not surviving:
            continue
        remainder = LineString(row).difference(exclusion)
        parts = (list(remainder.geoms) if hasattr(remainder, "geoms") else [remainder])
        for part in parts:
            if not isinstance(part, LineString) or part.is_empty:
                continue
            group = [tree for tree in surviving
                     if part.distance(Point(tree["position"])) < 1e-5]
            if not group:
                continue
            number = len(clipped_rows) + 1
            for tree in group:
                retained.append({**tree, "row_number": number})
            clipped_rows.append([tuple(part.coords[0]), tuple(part.coords[-1])])
    return retained, clipped_rows


def run(scene: FarmScene, config, rng: np.random.Generator) -> FarmScene:
    tree_idx = 0
    weed_idx = 0
    layouts = {}
    for pid, parcel in scene.parcels.items():
        if not isinstance(parcel, CropArea):
            continue
        poly = Polygon(parcel.polygon)
        parcel_coords = np.array(parcel.polygon)
        parcel_seed = int(rng.integers(0, 2**63 - 1))
        parcel_rng = np.random.default_rng(parcel_seed)
        origin, row_dir, col_dir = _pick_row_and_col_dirs(
            parcel_coords, parcel_rng, config.optimize_tree_layout
        )
        layouts[pid] = (poly, parcel_seed, parcel_rng, origin, row_dir, col_dir)

    if config.depot_enabled:
        candidates = []
        nominal_length = config.depot_along_row_size * config.tree_spacing
        nominal_width = config.depot_cross_row_size * config.row_spacing
        spot_length = config.depot_spot_length_multiple * config.depot_tractor_length_m
        spot_width = config.depot_spot_width_multiple * config.depot_tractor_width_m
        inset = 0.1
        for pid, (poly, parcel_seed, _, origin, row_dir, col_dir) in layouts.items():
            # Roads already exist. The inner boundary of their closed
            # frontage ring is the outer boundary of the depot/crop area.
            road_nodes = [node.position for node in scene.roads.nodes.values()
                          if node.tags.get("parcel_id") == pid]
            if len(road_nodes) < 3:
                continue
            road_edges = [edge for edge in scene.roads.edges.values()
                          if edge.tags.get("parcel_id") == pid and edge.road_class.value == "frontage"]
            if not road_edges:
                continue
            crop_base = Polygon(road_nodes).buffer(-road_edges[0].width / 2,
                                                  join_style="mitre")
            if crop_base.geom_type != "Polygon" or crop_base.is_empty:
                continue
            crop_corner = np.asarray(min(crop_base.exterior.coords[:-1],
                                         key=lambda xy: np.linalg.norm(np.asarray(xy) - origin)))
            basis_inv = np.linalg.inv(np.column_stack([row_dir, col_dir]))
            i0, j0 = basis_inv @ (crop_corner - origin)
            # The inner edges pass through positions of the original tree
            # grid, before the depot removes any crop area.
            k = int(np.ceil((nominal_length - config.headland_width) /
                            config.tree_spacing - 1e-9))
            l = int(np.ceil((nominal_width - config.sideland_width) /
                            config.row_spacing - 1e-9))
            i_inner = config.headland_width + k * config.tree_spacing
            j_inner = config.sideland_width + l * config.row_spacing
            length, width = i_inner - i0, j_inner - j0
            if length <= 0 or width <= 0:
                continue
            count = int(np.floor((length - 2 * inset) / spot_width)) if width >= spot_length + 2 * inset else 0
            if count < 1:
                continue
            reach = max(poly.bounds[2] - poly.bounds[0],
                        poly.bounds[3] - poly.bounds[1]) * 3
            wedge = Polygon([origin - reach * row_dir - reach * col_dir,
                             origin + i_inner * row_dir - reach * col_dir,
                             origin + i_inner * row_dir + j_inner * col_dir,
                             origin - reach * row_dir + j_inner * col_dir])
            depot_poly = crop_base.intersection(wedge)
            remaining = crop_base.difference(depot_poly)
            if (depot_poly.geom_type != "Polygon" or depot_poly.is_empty or
                    remaining.geom_type != "Polygon" or remaining.area <= depot_poly.area):
                continue
            spec = PlantingSpec(config.row_spacing, config.tree_spacing,
                float(np.degrees(np.arctan2(row_dir[1], row_dir[0]))),
                config.headland_width, config.sideland_width,
                dict(config.species_mix), parcel_seed, config.optimize_tree_layout)
            original_trees, original_rows = _generate_tree_grid(
                origin, row_dir, col_dir, poly, spec, np.random.default_rng(parcel_seed))
            row_counts = [sum(t["row_number"] == index for t in original_trees)
                          for index in range(1, len(original_rows) + 1)]
            if (len(original_rows) <= config.depot_cross_row_size or
                    max(row_counts, default=0) <= config.depot_along_row_size):
                continue
            exclusion = _depot_exclusion_polygon(
                depot_poly, row_dir, col_dir,
                config.depot_headland_m, config.depot_sideland_m)
            remaining_trees, remaining_rows = _exclude_depot_from_grid(
                original_trees, original_rows, exclusion)
            if len(remaining_rows) < 2 or len(remaining_trees) < len(original_trees) / 2:
                continue
            candidates.append((pid, depot_poly, crop_base, crop_corner,
                               row_dir, col_dir, length, width, count, i_inner, j_inner))
        if not candidates:
            raise ValueError("No field can fit a depot with at least one parking spot; increase crop spacing or depot dimensions")
        (pid, depot_poly, crop_base, depot_origin, row_dir, col_dir,
         length, width, count, i_inner, j_inner) = candidates[int(rng.integers(len(candidates)))]
        spots = []
        for number in range(1, count + 1):
            center = (depot_origin + (length - inset - (number - 0.5) * spot_width) * row_dir
                      + (width - inset - spot_length / 2) * col_dir)
            spots.append(ParkingSpot(number, (float(center[0]), float(center[1])),
                                     float(np.arctan2(col_dir[1], col_dir[0])), spot_length, spot_width))
        scene.depot = Depot(pid, [(float(x), float(y)) for x, y in depot_poly.exterior.coords[:-1]],
                            tuple(map(float, row_dir)), tuple(map(float, col_dir)), spots,
                            config.draw_parking_spaces, tuple(map(float, depot_origin)), float(length), float(width),
                            spot_length, inset)

    for pid, (original_poly, parcel_seed, parcel_rng, origin, row_dir, col_dir) in layouts.items():
        parcel = scene.parcels[pid]
        poly = (original_poly.difference(Polygon(scene.depot.polygon))
                if scene.depot and scene.depot.field_id == pid else original_poly)
        row_angle_deg = float(np.degrees(np.arctan2(row_dir[1], row_dir[0])))

        spec = PlantingSpec(
            row_spacing=config.row_spacing,
            tree_spacing=config.tree_spacing,
            row_angle_deg=row_angle_deg,
            headland_width=config.headland_width,
            sideland_width=config.sideland_width,
            species_mix=dict(config.species_mix),
            seed=parcel_seed,
            optimize_tree_layout=config.optimize_tree_layout,
        )
        parcel.planting = spec

        trees, row_centerlines = _generate_tree_grid(
            origin, row_dir, col_dir, original_poly, spec, parcel_rng)
        original_tree_count = len(trees)
        exclusion = None
        if scene.depot is not None and scene.depot.field_id == pid:
            exclusion = _depot_exclusion_polygon(
                Polygon(scene.depot.polygon), row_dir, col_dir,
                config.depot_headland_m, config.depot_sideland_m)
            trees, row_centerlines = _exclude_depot_from_grid(
                trees, row_centerlines, exclusion)
        for t in trees:
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
                tags={"parcel_id": pid, "row_number": str(t["row_number"])},
            )
            parcel.refs.append(tid)

        if scene.depot is not None and scene.depot.field_id == pid:
            # Reserve the IDs the pre-depot field would have consumed so tree
            # IDs in every later field remain unchanged.
            tree_idx += max(0, original_tree_count - len(trees))

        wid = f"weed_{weed_idx:04d}"
        weed_idx += 1
        scene.weed_zones[wid] = WeedZone(
            id=wid,
            polygon=[(float(x), float(y)) for x, y in poly.exterior.coords[:-1]],
            excluded_polygons=[list(exclusion.exterior.coords[:-1])] if exclusion is not None else [],
            row_centerlines=row_centerlines,
            density_params={
                "row_density_per_m": config.weed_row_density_per_m,
                "row_falloff_m": config.weed_row_falloff_m,
            },
            tags={"parcel_id": pid},
        )
        parcel.weeds = WeedSpec(
            density_params={
                "row_density_per_m": config.weed_row_density_per_m,
                "row_falloff_m": config.weed_row_falloff_m,
            },
            seed=parcel_seed,
        )
        parcel.weed_zone_refs.append(wid)

    return scene
