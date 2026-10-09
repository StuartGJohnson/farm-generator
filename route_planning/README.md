# GIS task route planning

Run from the repository root:

```bash
conda activate isaacsim61-cu13
python -m route_planning --farm debug_out/farms/farm_seed_1_120m2/gis
```

This reads `farm.gpkg`, `map_frame.json`, and optionally `orthophoto.tif` for
plots. It also loads the tractor dimensions from `configs/tractor_default.yaml`;
it never reads `farm.yaml` or USD. The default demo writes three seeded
random missions to `debug_out/routes/farm_seed_1_120m2/`. Each has a PNG route
plot and a JSON route with ordered arcs, local ENU poses, service IDs, distance,
and forward/reverse samples for RS maneuvers. `manifest.json` records tasks and
solver status, best objective bound, solve time, and reverse distance. Each
mission gets a 60-second CP-SAT limit by default. `--seed` fixes all three
field lists and road-aligned start/stop poses before planning starts. Pose
candidates that point more than 120 degrees away from the selected fields
are discarded during task sampling. Solver results never cause a new task
to be drawn. Use `--seed`, `--solver-time`,
`--reverse-multiplier`, `--gear-change-penalty`, `--output-dir`, and
`--tractor-config`, `--implement-side {left,right,both}` to vary the diagnostic run. Reverse
stretches are dashed red on the route plots. If a solved route contains a
zero-distance 180-degree road reversal, the demo reports that failure for the
fixed task.

The package follows `writeup/route_planning.tex` in four modules:

1. `gis.py` reads GeoPackage features and translates EPSG:32610 to the local
   map frame using `map_frame.json`.
2. `graph.py` constructs road, crop-row, and field-constrained Reeds–Shepp
   movement arcs. Every tree row has separate left- and right-side service
   requirements. With a two-sided implement, a lane between neighboring rows
   can cover the facing side of each row in one pass. The outer sides of the
   field also require passes. Slightly staggered but overlapping row-side
   centerlines are merged into one lane with both requirements. One-sided
   tasks restrict service to the
   direction that places the trees on the mounted implement side.
   Road interface states retain travel heading so field maneuvers connect to
   the direction in which the tractor arrived or will depart.
3. `optimize.py` solves directed arc multiplicities with CP-SAT service,
   balance, and connectivity constraints.
4. `reconstruct.py` builds an ordered Euler route and checks it independently.

The tractor configuration supplies the default 2.7 m minimum turn radius.
RS poses track the non-steering rear axle. `path_feasibility.py` builds the
convex outer envelope of the straight-ahead body and tires and checks its
swept footprint against the field polygon minus the GIS irrigation channels,
with 0.2 m clearance from tree points. The rear-axle RS trace is also barred
from the interior of each field's tree convex hull, so row-to-row maneuvers
stay in headlands or sidelands. The tractor footprint may overlap the hull
at a row endpoint while the tractor leaves or enters that row. Road crossings over channels are not
yet exempted. Other defaults include 0.35 m RS sampling and a 15 m RS
candidate radius. Reeds–Shepp alternatives are ranked using
an 8x reverse-distance multiplier and a 2 m equivalent penalty per gear
change; the same costs are used by the route optimizer. The default implement
range is half the median nearest-row spacing measured from GIS. One-tree rows
have no navigable line and cannot currently be selected for service.
Road-intersection heading changes follow the specification's position-only
road state allowance; sharp junction turns still need vehicle-feasible
geometry before execution.
The planner checks ranked RS candidates against the allowed field region and
tree clearance, then keeps the lowest-cost legal path for each connection.
These choices are
potential planning-policy refinements before vehicle deployment.
