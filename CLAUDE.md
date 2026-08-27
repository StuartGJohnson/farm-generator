# farm-gen — Architecture & Generation Design

This document is the authoritative design reference for this repository.
Read it before making structural changes. It captures decisions reached
during design discussion and prototyping — several choices here were
deliberate rejections of simpler alternatives (some tried and found to be
buggy in practice), and the reasoning is included so it doesn't get
silently reversed.

## Purpose

Procedurally generate synthetic farms — initially flat Sacramento Delta
orchard land — as an intermediate representation (IR), for developing and
validating a ROS2 SW stack (perception/planning/control) on an autonomous
electric tractor. The IR is simulator-agnostic; exporters translate it to
USD/IsaacSim, Gazebo/SDF, or others. This repo owns IR schema, generation,
and visualization. Simulator export and USD asset generation are separate,
later concerns — some asset-generation code will be strategically ported in
from an existing (simpler, grid-only orchard) repo, but that repo is not a
dependency of this one and nothing here should assume its presence.

## Core architectural principles

1. **Vector-first.** Nearly everything is authored/generated as points,
   polylines, or polygons with parameters — not rasters. The one exception
   is `ElevationField`, which is raster-native by necessity (simulators
   consume heightfields directly) but is treated as a *baked cache product*
   derived from vector sources, not a source of truth in itself. Material/
   traction variation is also vector (`MaterialPatch` polygons + semantic
   `material_class`), rasterized only at simulator-export bake time, so the
   IR stays resolution-independent and re-bakeable at different simulator
   grid resolutions without re-authoring.

2. **Networks are explicit graphs, but DERIVED from tessellation, not
   authored first.** An earlier design iteration generated the hydrology
   mesh as an independent graph (BSP-style channel grid) and derived
   parcels from it. That was superseded: parcels are now generated directly
   via T-tessellation (see Generation Order below), and the hydrology graph
   (`HydrologyNetwork` nodes/edges) is populated FROM parcel boundaries
   afterward — every parcel edge becomes a `HydrologyEdge`, every shared
   corner a `HydrologyNode`. The schema itself (graph of nodes/edges) is
   unchanged; only which direction the dependency runs changed. Parcels
   still reference network edges via `boundary_refs`, not duplicated
   geometry.

3. **Single ENU working frame.** All IR geometry is local East-North-Up
   Cartesian meters, anchored to one WGS84 `SceneOrigin`. This is required
   for synthetic GNSS generation (ENU → ECEF → WGS84 transform + noise
   model) and matches what ROS localization stacks expect as a `map` frame.
   UTM zone is stored as an informational label only; ENU is canonical.

4. **Provenance is cheap, add it everywhere.** Every `SpatialFeature`
   carries `source` (`procedural` | `measured` | `hybrid`) and `confidence`.
   Not used yet — this repo is procedural-only for now — but the schema
   already supports later fusion with real aerial imagery/lidar extraction
   (road vectorization, canopy-height-model tree detection, lidar bare-earth
   DEM) without a schema migration. Don't remove these fields even though
   they're inert today.

5. **Elevation is derived, with the derivation strategy explicit and
   swappable.** For this (flat Delta) phase, `ElevationField.source =
   derived_from_hydrology`. A future vineyard/topographic phase would
   generate terrain first and derive hydrology from it via flow-routing —
   the *interface* (`ElevationField` consumed by everything downstream) is
   designed to support that without consumers caring which direction the
   dependency ran. Never let downstream code (tree placement, road
   profiles, exporters) read elevation from anywhere but `ElevationField`.
   With no distinguished trunk this phase (see below), there is no real
   macro gradient source either — treat the whole domain as flat/uniform
   for now; a gradient-driving feature returns with the trunk.

6. **Prefer established libraries over hand-rolled computational geometry
   and graph algorithms.** See "Library usage" below — this is not a
   generic aspiration, it's backed by a specific, repeated experience
   during prototyping (see "Key lessons"), including a case where
   hand-rolling was tried hard, from two different directions, and still
   couldn't reliably solve the problem (see "Trunk / main channel:
   deferred").

## Trunk / main channel: deferred to later digital-twin work

**This version has no distinguished main/trunk channel.** Every hydrology
edge is generated equivalently by T-tessellation; there is no gravity-fed
head, no macro elevation gradient, no `HydrologyEdge.connection_type =
GRAVITY_SURFACE` edges produced this phase (the schema field still exists
and is meaningful once the trunk returns — see the schema section below).

This is a deliberate simplification, not an oversight. A trunk (real
hydraulic head, single pump station, elevation gradient) is a genuinely
useful realism feature and is **not being abandoned** — it's deferred
until it can be implemented against a real computational geometry library
in the actual repo, for a specific, empirically-grounded reason:

Combining a fixed external boundary (the trunk polyline) with T-tessellation
was tried from two different directions during prototyping, and both
produced serious, recurring sliver-parcel bugs:
- **Building the trunk into the tessellation's own boundary** (splitting
  the domain into two root faces along the trunk, then tessellating each
  side with the trunk's points as fixed/protected vertices) repeatedly
  produced acute-angle wedge parcels and erosion instabilities specifically
  near the trunk, because cuts landing near a *fixed, unmovable* vertex have
  less geometric freedom than a free interior cut. Multiple targeted fixes
  (angle-rejection thresholds, mitre-limit clamps, protected-vertex cleanup)
  each fixed one manifestation and reliably uncovered another.
- **Tessellating freely first, then clipping the result by the trunk as a
  post-process** (the reverse order) was tried as the direct alternative.
  It did NOT avoid the problem, even after adding explicit vertex snapping —
  testing across 10 seeds still showed non-conserved area and near-degenerate
  angles (down to ~2°) in most seeds. The failure mode is symmetric: instead
  of a tessellation cut landing near a fixed trunk vertex, the trunk itself
  can cross near an existing tessellation vertex — same underlying problem,
  opposite direction.

**Root cause:** robustly overlaying two independently-generated planar
geometries without producing near-miss slivers is a genuinely hard,
well-studied computational-geometry problem. It requires tolerance-aware
vertex snapping as a first-class part of the overlay operation itself —
not a cleverer choice of cut, not a post-hoc cleanup pass. This is exactly
what GEOS's `OverlayNG` engine and `shapely.set_precision()` provide, and
is not something a few more hours of hand-patching reliably solves — this
was tested from both directions before reaching that conclusion.

**When the trunk returns** (a real digital-twin phase, presumably with
shapely available): implement the combination via `shapely.ops.split`
combined with an explicit `shapely.set_precision()` snapping step before
the split, not via hand-rolled geometry. Budget real validation time for
this specifically — it is not a routine polygon operation, based on
direct experience.

## Library usage — load-bearing, not optional

Prefer established library operations over custom geometric/graph code:

- **shapely (GEOS)** for: polygon buffering/erosion (`polygon.buffer
  (-standoff, join_style="mitre", mitre_limit=...)`), boolean ops (union/
  difference/intersection), `shapely.voronoi_polygons`, `shapely.ops.split`
  for splitting a polygon by a line/polyline. When reintroducing the trunk
  (see above), combine `shapely.ops.split` with `shapely.set_precision()`
  for vertex snapping — this combination is a specific, necessary
  requirement, not general advice.
- **networkx** for: connected-component analysis (`connected_components`
  — used to detect full-network connectivity in `validate()`), minimum
  spanning trees where an actual minimum is wanted (not crossings — see
  Key Lessons), and any other standard graph algorithm. Ad hoc "connect
  everything nearby" proximity logic was a repeated source of bugs during
  prototyping; a real graph-theoretic check (component analysis) is what
  actually caught them.
- **scipy.spatial.cKDTree** for nearest-neighbor / proximity queries.
- **numpy.random.default_rng** for all seeded randomness, split into
  per-stage sub-seeds so one stage's regeneration doesn't reshuffle
  another's.

Custom code should implement farm-generation POLICY (tessellation
parameters, which crossings are structurally necessary) — not re-derive
general-purpose computational geometry or graph algorithms a well-tested
library already provides correctly.

## Repository layout

```
farm-gen/
  farm_ir/
    schema.py              # farm_ir_schema.py — the IR itself. Pure data, no logic.
  generation/
    tessellation.py          # T-tessellation over the domain -> parcels
    hydrology.py               # HydrologyNetwork derived from parcel boundaries
    elevation.py
    roads.py                    # full-perimeter erosion-based road generation
    crossings.py                  # strict collinear vertex-to-vertex crossings
    crops.py                       # trees + weeds
    material.py                     # later
    orchestrator.py                  # generate_farm(config) -> FarmScene
  visualization/
    debug_view.py                     # PRIMARY dev tool — see below
  export/                             # stubbed for now; not built this phase
    usd/
    gazebo/
  examples/                           # reference prototypes — see "Example reference code"
    farm_mesh_prototype.py
  tests/
  CLAUDE.md                           # this file
  pyproject.toml
```

## The IR schema

Lives at `farm_ir/schema.py` (developed as `farm_ir_schema.py`, drop it in
as the first commit). Do not restructure it casually — it was iterated on
deliberately. Key shapes to know before writing generation code:

- `FarmScene` is the top-level container: `origin`, `hydrology`
  (`HydrologyNetwork`), `roads` (`RoadNetwork`), `crossings`, `parcels`
  (dict, holds `CropArea`/plain `Parcel` — see "Farmstead: deferred"
  below), `trees`, `weed_zones`, `material_patches`, `buildings`,
  `terrain`, `provenance`.
- `HydrologyEdge.connection_type`: this phase, every generated edge is
  `PIPED` (locally-maintained, not hydraulically continuous with
  anything). `GRAVITY_SURFACE` is not produced this phase — it's reserved
  for when the trunk returns (see above); leave the enum value in place.
- `RoadEdge.road_class`: with the full-perimeter road design (see below),
  every parcel edge produces a `FRONTAGE` road on its own side —
  `CROSSING_SPUR` marks the short connectors generated by the crossing
  stage. `LEVEE_CROWN` and `DRIVEWAY` remain in the enum for later use
  (trunk entrance spine, farmstead driveway) but are not populated this
  phase.
- `CropArea.irrigation_type` (`DRIP` | `FLOOD` | `SUB_IRRIGATION`) — affects
  nothing structurally yet but is read by later material/moisture
  correlation logic. All crop parcels are still bounded on every side by a
  hydrology channel.
- Polygons have exterior rings only — no holes yet. Not needed until
  farmstead cutouts are reintroduced (see below).

If you need a field that isn't here, add it to the schema deliberately and
update this document's relevant section — don't route around the schema
with ad hoc dicts in generation code.

## Generation order (DAG)

```
1. tessellation      (T-tessellation over the whole domain rectangle)
2. vertex cleanup     (degenerate + near-collinear -- see Key Lessons on merging vs deleting)
3. hydrology graph     (derived from parcel adjacency: edges + nodes from boundaries)
4. elevation            (flat/uniform this phase -- see principle 5 above)
5. roads                 (full perimeter per parcel, via erosion)
6. crossings              (strict collinear vertex-to-vertex, not MST-minimized)
7. crops/weeds             (per CropArea, ports existing grid generator)
[deferred] trunk/main channel (see above)
[deferred] farmstead        (see below)
[later]    material patches
```

Each stage should be a function `(scene, config, rng) -> scene`, as close
to pure as practical. Orchestration lives in `generation/orchestrator.py`
as `generate_farm(config: FarmGenerationConfig) -> FarmScene`.

**Seeding:** one global seed, split into per-stage sub-seeds (e.g.
`numpy.random.default_rng(seed).spawn(n)`).

**Validation-with-retry:** generation occasionally (empirically, ~2% of
seeds) produces one sharp-angled face from an inherent, small tension
between erosion-stability vertex cleanup and angle-niceness (see Key
Lessons) — not worth chasing further by hand. Wrap generation in a cheap
retry-on-validation-failure loop (try the requested seed, then
`seed + 1000`, `seed + 2000`, ... until `validate()` passes or a small
try-budget is exhausted) rather than treating this as a hard requirement
to eliminate entirely. `examples/farm_mesh_prototype.py`'s
`generate_validated()` demonstrates this — tested clean across 100/100
seeds with at most one retry needed.

### 1. Tessellation (T-tessellation)

Chosen over Voronoi/CVT (too much cell-size/shape variance even after
Lloyd relaxation) and over plain axis-aligned BSP-grid-of-channels (the
original design — abandoned once the full-mesh-first architecture proved
harder to keep bug-free than tessellating parcels directly).

Recursive chord-splitting: pick the largest remaining face, cut it with a
straight chord between two points on ITS OWN current boundary (a
T-junction — the cut's endpoints land mid-edge on the neighbor's
still-intact edge, not at a shared vertex — this is what real cadastral/
field subdivision looks like, and what a uniform X-junction grid does
not). Cut direction is **perpendicular to the face's long axis** (cutting
ALONG the long axis is a genuine bug found during prototyping — it just
produces more strips in the same direction forever and never converges to
square-ish faces).

Two rejection criteria on every candidate cut, both necessary:
- `min_subarea_frac`: reject if either resulting sub-face would be below
  this fraction of the *parent's* area — prevents low-AREA slivers.
- `min_interior_angle_deg` (50°, empirically tuned): reject if either
  resulting sub-face would have any interior angle below this threshold —
  area alone doesn't catch a narrow, acute WEDGE shape (can have plenty of
  area while still being a practically unusable sliver). Check ALL
  vertices of each candidate sub-face, not just the two new cut points —
  a pre-existing sharp angle inherited unchanged from the parent should
  also block further cutting that direction, so the face is left larger
  (absorbing the corner) rather than compounding the problem.

Because the only input is the domain rectangle and every cut is a
straight chord of a convex polygon (always yields two convex polygons),
**every face stays convex throughout** — this is a provable invariant of
this design, not something requiring runtime checking, and it's a direct
benefit of dropping the trunk: a non-convex-capable polygon splitter
(needed when a fixed concave boundary is involved) is real, avoidable
complexity that this version doesn't need at all.

### 2. Vertex cleanup

Needed for erosion stability (see Roads below): a degenerate near-
duplicate vertex, or a near-collinear one, causes numerically unstable
mitre-line intersections in per-edge polygon offsetting.

**Collapse near-duplicate vertices by MERGING to their midpoint, not by
deleting one endpoint.** A real bug found during prototyping: deleting one
endpoint of a short edge asymmetrically favors whichever vertex survives —
measurably shrinks the polygon's area and can create an artificially sharp
angle at the surviving vertex (observed: a 4-vertex face lost ~13% of its
area and gained a 29° corner from a single dropped vertex). Merging to the
midpoint is a better approximation, though note it does NOT fully solve
the sharp-angle side effect (see Key Lessons) — going from 4 vertices to 3
inherently changes the resulting angle regardless of exactly where the
merged point sits, since the sharpness comes from the original polygon's
own edge directions. This residual is rare and small enough to handle via
the retry-on-validation-failure wrapper rather than further hand-patching.

`min_edge_len` for the degenerate-vertex pass should scale with the
erosion standoff (e.g. `2 * standoff`) — a fixed constant threshold works
by luck on some seeds and fails on others.

### 3. Hydrology graph (derived)

Build `HydrologyNetwork` from the cleaned tessellation output: every
parcel boundary edge becomes a `HydrologyEdge` (`connection_type = PIPED`
this phase — see above), every shared vertex a `HydrologyNode`.

### 4. Elevation

`ElevationField.source = derived_from_hydrology`, baked as a raster with
breaklines enforced from hydrology + road edges. With no trunk this
phase, there's no macro gradient source — treat the domain as flat/
uniform; every channel is a shallow, locally-flat depression at roughly
constant relative depth.

### 5. Roads — full perimeter per parcel

Every parcel's road is its own complete eroded perimeter
(`polygon.buffer(-standoff)`), not a selectively-one-sided pick via
adjacency detection. **This replaces an earlier one-sided-frontage
design** (a dual-graph spanning tree over parcels, one road per shared
channel) — abandoned after repeated T-junction bugs where some channels
ended up with a road on NEITHER side, because adjacency-based side-
selection has to get every T-junction sub-segment right, and it kept not
doing so. Full-perimeter can't produce a coverage gap by construction, at
the cost of a doubled road along every shared channel — an accepted
simplification.

Use a UNIFORM standoff distance for every edge. A distinct (e.g. larger)
setback for some future distinguished channel was tried and reintroduced
local-feature-size erosion instability — if a variable setback is wanted
later, apply it as a separate road-planning step downstream of a
uniformly-eroded base, not as a different distance fed into the same
offset operation.

A mitre-limit CLAMP (cap how far any single corner's erosion can extend,
pulling the point back along the same direction rather than a true bevel)
is used for robustness — see Key Lessons on why a true bevel was tried
and rejected.

### 6. Crossings — strict collinearity, vertex-to-vertex only, no new points

**Every crossing connects two EXISTING within-parcel road vertices, never
a constructed point — and must be collinear with a real road edge on
BOTH ends.** This went through three iterations before landing here, and
the two rejected ones are worth knowing so they aren't reintroduced:

1. *Raw proximity matching* (connect the nearest cross-parcel points from
   a `cKDTree` search over all eroded-boundary points) broke down at
   T-tessellation "supercells": since tessellation is recursive, one side
   of a shared channel can be a single long uncut edge while the other
   has been recursively subdivided into several smaller faces with
   T-junction vertices along that same physical line. The nearest point
   across such a boundary is not necessarily the one directly opposite,
   producing a diagonal, unrealistic crossing.
2. *Detect the shared-boundary overlap and place the crossing at its
   midpoint*, projected onto each side via fractional interpolation —
   this fixed the diagonal problem but **introduced brand-new points that
   aren't part of the actual road network**, and collapsed the result to
   a bare connectivity-minimum spanning tree when a denser, more
   realistic set of crossings was wanted. Both were real regressions, not
   improvements — rejected.
3. *A version allowing a crossing to land on a non-vertex point along a
   plain supercell edge* (checking perpendicularity there instead of
   collinearity, since a mid-edge point has no corner to align with) —
   still constructing a point that isn't an existing road vertex, and
   inconsistent (a different test at each end). Also rejected.

**The correct rule, implemented in `straight_vertex_crossings`:** for
every pair of *existing* eroded-boundary vertices belonging to different
parcels within `connect_radius`, keep the pair only if the connecting
segment is collinear (a real straight line, checked via the 2D cross
product so both directions along the line count — not a signed
dot-product test) with an incident road edge at **both** endpoints. No
midpoint construction, no projection, no perpendicularity fallback.

**Tolerance must be tight, not just "small-looking."** The angle
tolerance for this check defaults to `1e-3` degrees, not a few degrees —
measured empirically (see Key Lessons) that edges sharing a genuine
construction relationship (a real T-junction) stay collinear to ~1e-13
degrees after erosion, while T-tessellation's own axis-biased cutting
means unrelated edges are routinely within several degrees of each other
just by coincidence. A loose tolerance (tried first, at 8°) was
measuring against the wrong reference scale and let coincidentally
axis-aligned pairs through. This was verified directly, twice: every
crossing endpoint is confirmed to be a member of the actual
`eroded_faces` vertex set (no constructed points), and every accepted
crossing's collinearity deviation is confirmed to be at the floating-
point noise floor (~1e-13 degrees), not merely "close."

**Accepted consequence:** where no existing vertex lines up collinearly
on both sides, there is no valid crossing there — full stop. This can
occasionally leave a parcel disconnected from the rest of the network
(rare — 2/100 seeds in testing, the same order as the existing rare
sharp-angle cleanup issue). Handle it the same way: via
`generate_validated()`'s retry-on-validation-failure wrapper, not by
loosening the geometric rule to force a connection that isn't really
there.

**Deliberately not reduced to a minimum spanning tree.** Every valid
collinear pair is kept (typically ~120 for a 60-parcel domain, not the
graph-theoretic minimum of 59) — this is intentional, not an
optimization left undone: a later redundant-road-elimination pass (not
built yet — see Purpose/DAG) will need real, valid crossing options to
work with when it identifies and removes over-built parallel roads
between adjacent parcels. Fewer crossings now would mean less to work
with then.

Default `CrossingType.CULVERT` everywhere (buried pipe, road continues at
grade). `BRIDGE` is not expected to trigger at current parcel/channel
sizes without a trunk; reserve it for spans wider than a
`min_span_for_bridge` threshold against the crossed channel's `top_width`
if that changes later.

Headland turning space (`PlantingSpec.headland_width`) is **not** a
road — open field inside the parcel polygon, never a `RoadEdge`.

### 7. Crops / weeds

Per `CropArea`: consume `PlantingSpec`, inset the boundary by
`headland_width`, generate the row/tree grid inside the inset region,
**clip against the actual polygon** (T-tessellation parcels are not clean
rectangles), assign species/age noise. This is a retarget of the existing
grid-orchard generator (linked repo) — main change is consuming an
arbitrary polygon with a parcel-derived origin/angle instead of a fixed
rectangle with a global origin. Weeds start as a simple uniform
low-density zone over headland + row gaps; ditch-proximity correlation is
a later phase.

## Farmstead: deferred to a later version

**Not built this phase.** There is enough to debug in culvert/crossing
placement and IsaacSim integration without adding building generation on
top. The schema (`FarmsteadCompound`, `Building`) stays in place —
don't remove it — but no generation stage invokes it yet, and
`RoadClass.DRIVEWAY` is unused until it returns. When it's reintroduced
(likely alongside the trunk, since a real farmstead is normally sited near
the main access), revisit whether polygon holes are needed for a
farmstead cutout inside a larger crop parcel.

## Key lessons from prototyping

Every one of these was found empirically while building
`examples/farm_mesh_prototype.py`, not anticipated in advance — treat this
list as a set of specific failure modes to avoid reintroducing, not
abstract advice:

- **Hand-rolled polygon offsetting is not robust**, in four distinct,
  separately-discovered ways: (1) independent per-edge offsets don't
  converge at a shared corner; (2) degenerate near-duplicate vertices
  cause numerically unstable mitre-line intersections; (3) near-collinear
  vertices make adjacent offset lines nearly parallel, same instability;
  (4) an edge shorter than roughly twice the erosion distance is a
  genuine local-feature-size violation no amount of vertex cleanup fixes.
  All four are exactly what `shapely.buffer` (GEOS, mitre-limit + bevel
  fallback) solves robustly. Use it; don't hand-patch further.
- **A true bevel fallback (replacing an over-extended mitre point with
  two points) was tried by hand and introduced NEW self-intersections**
  at exactly the sharpest corners it was meant to fix. A simple clamp
  (pull the mitre point back along its own direction, no new
  vertices/edges) is a lower-risk approximation for a hand-rolled
  prototype, but is not a substitute for a real bevel — this is
  specifically a case where replicating a library's exact mechanism by
  hand is harder than it looks, not just tedious.
- **Combining two independently-generated planar geometries (e.g. a fixed
  external boundary with a free tessellation) is a fundamentally hard
  problem, not solvable by cut-selection cleverness in either
  direction.** See "Trunk / main channel: deferred" above — this was
  tested thoroughly enough (from both directions, with explicit snapping
  added) to be confident it needs real tooling (GEOS OverlayNG /
  `shapely.set_precision`), not more hand-patching.
- **When collapsing a near-duplicate vertex, merge to the midpoint, not
  delete one endpoint.** Deletion asymmetrically favors whichever vertex
  survives and measurably distorts area/angle. Note that even the merge
  fix doesn't fully eliminate a resulting sharp angle when going from 4
  vertices to 3 — that residual is inherent to reducing vertex count, not
  a bug to keep chasing; handle it with retry-on-validation-failure
  instead (see Generation Order above).
- **Vertex-simplification's angle-safety check must be re-verified AFTER
  cleanup, not just enforced during generation.** Tessellation's own
  `min_interior_angle_deg` rejection only guards candidate cuts at the
  time of splitting — it cannot see a sharp angle introduced later by
  cleanup collapsing a short edge. `validate()` re-checks angles
  post-cleanup for exactly this reason.
- **Cleanup thresholds should derive from the relevant physical
  parameter** (e.g. `2 * standoff`), never a hardcoded constant — a
  constant works by luck on some seeds and fails on others.
- **Crossings must connect only EXISTING road vertices, never a
  constructed point, and must be collinear (not just "roughly aligned")
  with a real road edge on BOTH ends.** This took three iterations to get
  right. Raw nearest-point proximity matching produced diagonal crossings
  at T-tessellation "supercells" (one side of a shared channel can be a
  single long uncut edge while the other has been recursively
  subdivided, so the nearest point across isn't necessarily the one
  directly opposite). The first fix — detect the real shared-boundary
  overlap and place the crossing at its midpoint, projected onto each
  side — solved the diagonal problem but introduced brand-new points
  that aren't part of the actual road network, which is not allowed.
  A second attempt allowed landing on a non-vertex point with a
  perpendicularity check instead of collinearity there — still
  constructing a non-existent point, still rejected. The correct version
  only pairs real vertices and applies the identical collinearity test
  (via 2D cross product, direction-agnostic) at both ends — verified
  directly that every crossing endpoint is a member of the actual vertex
  set, not just visually plausible. Where no valid collinear pair exists,
  there simply is no crossing there — an accepted, rare (~2% of seeds)
  cause of parcel disconnection, handled by the same retry wrapper as the
  sharp-angle case, not by loosening the geometric rule.
- **The collinearity tolerance must be tight enough to reject coincidental
  axis alignment, not just "roughly straight."** The first working version
  of the collinearity check used an 8° tolerance, which looked reasonable
  in isolation but was wrong by many orders of magnitude: T-tessellation's
  own cut-angle bias (`cut_angle_for_face` always starts from exactly 0°
  or 90°, plus a jitter of only a few degrees) means nearly every edge in
  the mesh is within a few degrees of horizontal or vertical *regardless
  of which parcel it belongs to* — so an 8° tolerance was mostly detecting
  "this edge is roughly axis-aligned like everything else," not "these
  two specific edges share a genuine construction relationship." Measured
  directly: edges that DO share a genuine ancestor (a real T-junction,
  where the shared point is the literal same computed value reused in
  both child faces) stay collinear to ~1e-13 degrees after erosion —
  floating-point noise, not an approximation. The tolerance was tightened
  to `1e-3` degrees accordingly: comfortably above the observed noise
  floor, comfortably below the scale of coincidental axis bias. Always
  measure the actual precision floor for a "should be exact" geometric
  relationship empirically before picking a tolerance — don't guess a
  tolerance that merely "sounds reasonably tight."
- **Crossing generation should NOT always be reduced to a minimum
  spanning tree.** An earlier version minimized to the graph-theoretic
  minimum (`n_parcels - 1`) — correct for connectivity, but a later
  redundant-road-elimination pass (removing over-built parallel roads
  between adjacent parcels, not built yet) will need a denser set of real
  candidate crossings to work with than the bare minimum provides. Keep
  every geometrically valid (collinear, vertex-to-vertex) crossing found,
  not just enough for connectivity.
- **Full-perimeter-per-parcel roads are more robust than one-sided
  frontage-via-adjacency**, at the cost of doubled road density. Adjacency-
  based side selection has to get every T-junction sub-segment right; it
  kept not doing so.
- **Area conservation should only be checked strictly on RAW tessellation
  output, not after intentional vertex-cleanup simplification.** Removing
  a near-degenerate vertex necessarily trims a small "ear" of area — this
  is expected, not a bug. `validate()` checks raw tessellation area with a
  tight tolerance (algorithm correctness) and post-cleanup area with a
  loose one (0.5% of domain — catches real bugs, not expected drift).

## Visualization — primary tool, build this early

**The visualizer is more important than automated validation checks for
this phase.** Build `visualization/debug_view.py` before or alongside the
early generation stages. It should:

- Render the current `FarmScene` to a **static image file (PNG)** via
  matplotlib.
- Support **per-layer toggles** (roads, parcels, crossings, trees, weed
  zones) with distinct colors per type.
- Be a **pure consumer of the IR** — no simulator-specific concepts.
- **Always generate output across multiple random seeds, never just
  one.** Run and save plots for at least 5 different seeds in a single
  batch (`examples/farm_mesh_prototype.py`'s `__main__` block does this),
  as the default behavior of any debug CLI/script, not an opt-in flag.
  Several of the bugs in "Key lessons" only appeared for SOME seeds —
  reviewing a single seed's plot will miss this entire class of bug.

Automated `validate(scene) -> list[Issue]` checks (self-intersecting
parcels, sharp interior angles, area conservation, full network
connectivity) are a fast supplementary signal —
`examples/farm_mesh_prototype.py`'s `validate()` function demonstrates the
minimum useful set — but remain secondary to the visualizer, not a
replacement.

## Example reference code

`examples/farm_mesh_prototype.py` is a consolidated, self-contained
prototype distilled from iterative design/debugging — it demonstrates the
full validated pipeline end to end (T-tessellation → vertex cleanup →
full-perimeter erosion roads → strict collinear vertex-to-vertex
crossings → plotting), with a `generate_validated()` retry wrapper and a
`__main__` block that generates and validates 5 seeds in one run.
Stress-tested clean across 100/100 seeds (2 needed one cheap retry).

**Read it as a specification of BEHAVIOR and INVARIANTS, not as code to
port verbatim.** It was written in a sandbox with no network access, so it
hand-rolls geometry (polygon erosion) that the "Library usage" section
above says to replace with shapely in the real implementation. The
`networkx`-based connected-components logic already uses the real
library and can be ported directly. Any docstring in that file starting
with "LESSON" or "NOTE" documents a specific bug found and fixed during
prototyping — cross-reference against "Key lessons" above.

This file does NOT demonstrate the trunk/main-channel concept — see
"Trunk / main channel: deferred" above for why, and what's needed before
attempting it again.

## Config

**All lengths/areas below are meters/square-meters** (the schema's ENU
frame is meters — see `farm_ir_schema.py`'s `Point2` docstring). This
section was previously a bare list of parameter names with no units and
no explanation of how they interact — not sufficient for an implementer
to make good choices. It's spelled out concretely here — and, checked
directly against `examples/farm_mesh_prototype.py` while writing this,
several of these are NOT currently reachable as arguments to `generate()`
even though they're real, meaningful tunables — they're stuck at
hardcoded defaults one level down, inside `t_tessellate_faces()` /
`erode_polygon_variable()`. **Wiring all of these through into one real
config object, all the way from `generate_farm(config)` down to where
they're actually used, is real work the example prototype does not yet
do — don't assume it's already done just because the parameter exists
somewhere in the file.**

```python
@dataclass
class FarmGenerationConfig:
    # --- domain ---
    bounds: tuple[float, float, float, float]  # (minx, miny, maxx, maxy), meters
    seed: int

    # --- tessellation (controls parcel count/size/shape) ---
    max_faces: int = 60             # target parcel COUNT -- currently a real
                                       # generate() parameter. See "controlling
                                       # mean parcel size" below -- this is the
                                       # primary lever, not min_area_frac.
    min_interior_angle_deg: float = 50.0  # reject a cut producing any angle
                                             # below this (wedge prevention).
                                             # Currently a real generate()
                                             # parameter.
    min_area_frac: float = 0.015    # NOT currently exposed by generate() --
                                       # hardcoded as t_tessellate_faces()'s
                                       # own default. Floor: stop splitting a
                                       # face once it's already below this
                                       # fraction of total domain area, even
                                       # if max_faces isn't reached yet -- a
                                       # safety floor, not the primary size
                                       # control.
    min_subarea_frac: float = 0.3   # NOT currently exposed by generate() --
                                       # hardcoded default. Reject a candidate
                                       # cut if either half would be below
                                       # this fraction of the PARENT's area --
                                       # controls SIZE VARIANCE (higher = more
                                       # even parcel sizes, more cut-rejection
                                       # retries), not the mean.
    angle_jitter_deg: float = 6.0   # NOT currently exposed by generate() --
                                       # hardcoded default. Random jitter
                                       # around the perpendicular-to-long-axis
                                       # cut direction.
    corner_margin_frac: float = 0.12  # NOT currently exposed by generate() --
                                         # hardcoded default. Keeps cut anchors
                                         # away from existing corners (sliver
                                         # prevention).

    # --- erosion / roads ---
    standoff: float = 1.0           # meters -- road setback from each
                                       # parcel's own boundary (uniform
                                       # everywhere this phase, see "Trunk"
                                       # section on why not variable).
                                       # Currently a real generate() parameter.
    mitre_limit: float = 2.0        # NOT currently exposed by generate() --
                                       # hardcoded as erode_polygon_variable()'s
                                       # own default. Caps a corner's erosion
                                       # extension at mitre_limit * standoff.

    # --- crossings ---
    connect_radius: float = None    # meters -- max distance between two
                                       # vertices to be considered for a
                                       # crossing; defaults to standoff * 2.5
                                       # if left None. Currently a real
                                       # generate() parameter.
    crossing_angle_tol_deg: float = 1e-3  # collinearity tolerance -- see Key
                                             # Lessons on why this must stay
                                             # near the floating-point noise
                                             # floor (~1e-13 deg observed),
                                             # NOT a "looks small enough"
                                             # value like a few degrees.
                                             # Currently a real generate()
                                             # parameter.

    # --- planting/weed params: not yet finalized, see Generation Order #7 ---
```

**Controlling mean parcel size** (this came up explicitly during design
and isn't obvious from the parameter list alone): there is no direct
"mean parcel size" field. It emerges from `bounds` and `max_faces`
together — `max_faces` is the primary lever, since tessellation stops
once that many parcels exist (or once `min_area_frac` is hit first,
whichever comes first). To target a specific mean parcel area:

```
max_faces = domain_area / target_mean_parcel_area_m2
```

E.g. a 100m × 100m domain (10,000 m²) targeting ~400 m² parcels →
`max_faces = 25`. This gives the *mean*, not uniform sizing — real
variance around it is intentional (matches realistic field-size variety),
controlled separately by `min_subarea_frac` if it needs narrowing.
`min_area_frac` is a floor, not a primary control: it only binds if
`max_faces` is set unrealistically high for the domain size (asking for
parcels smaller than 1.5% of the domain), in which case generation stops
early with fewer, larger parcels than requested — lower it deliberately
only if genuinely small parcels relative to the domain are wanted.

Serialize a `FarmGenerationConfig` instance alongside a generated
`FarmScene` for reproducibility — given the same config and seed,
generation should be deterministic (modulo the retry-on-validation-
failure wrapper's seed perturbation on the rare cases it triggers on).

## Explicitly out of scope this phase

- Trunk / main hydrology channel (see "Trunk / main channel: deferred"
  above — deferred to later digital-twin work, not abandoned).
- Farmstead / buildings (see "Farmstead: deferred" above).
- USD/Gazebo export (stub the `export/` dirs, don't build them yet).
- Material patches / traction variability (schema supports it, generation
  does not yet populate it).
- Real imagery/lidar extraction pipeline (schema's `source`/`confidence`
  fields anticipate it; no extraction code this phase).
- Topography-first / vineyard-style generation (would derive hydrology
  from generated terrain via flow-routing — a second `ElevationField`
  source strategy implementing the same interface, not needed now).
- Polygon holes (e.g. a farmstead cutout inside a larger crop parcel —
  revisit when farmstead returns).
