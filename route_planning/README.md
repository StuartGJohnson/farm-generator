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
ordered field lists and road-aligned start/stop poses before planning starts.
The route completes every tree-side service requirement in one field before
switching to the next field. A switch occurs at a road pose and adds no
tractor movement; the ordered JSON records it as `field_switch`. Pose
candidates that point more than 120 degrees away from the selected fields
are discarded during task sampling. Solver results never cause a new task
to be drawn. Use `--cases 1` to regenerate only the first task. Use `--seed`, `--solver-time`,
`--reverse-multiplier`, `--road-turn-setback`, `--turn-radius-step`,
`--turn-radius-expansions`, `--road-interface-offset`,
`--reverse-event-penalty`, `--field-rs-forward-only`,
`--gear-change-penalty`, `--output-dir`,
`--tractor-config`, and `--implement-side {left,right,both}` to vary the diagnostic run. Reverse
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
   the direction in which the tractor arrived or will depart. At each road
   junction, directed states are placed `road_turn_setback_m` from the node
   on every incident edge. RS arcs connect each incoming leg to each other
   outgoing leg; only all-forward RS candidates are admitted for road turns,
   and the CP-SAT solver pays for the selected turn. A crossing
   shorter than twice the setback uses its midpoint at both ends. Row-to-road
   projections inside a junction setback are omitted. For every row endpoint,
   road interface states are also sampled 2.7 m on either side of its road
   projection by default. This gives a short or partial row room to connect
   forward to a road. `--road-interface-offset` changes that distance.
3. `ordering.py` duplicates road states by field stage. Only the current
   field's row and field-maneuver arcs exist in each stage; a zero-motion
   switch at a road exit interface from that field advances to the next
   stage. This is a copy of graph state, not a second physical road. `optimize.py` solves
   directed arc multiplicities with CP-SAT service, balance, and connectivity
   constraints over this staged graph.
4. `reconstruct.py` builds an ordered Euler route and checks it independently.

The tractor configuration supplies the default 2.7 m minimum turn radius.
For the 2.0 m route-planning trial, run
`python -m route_planning --cases 1 --solver-time 60 --tractor-config configs/tractor_route_2m.yaml`.
This changes only the turning radius; the road turn setback stays at 2.7 m.
RS poses track the non-steering rear axle. `path_feasibility.py` builds the
convex outer envelope of the straight-ahead body and tires and checks its
swept footprint against the field polygon minus the GIS irrigation channels,
with 0.2 m clearance from tree points. The rear-axle RS trace is also barred
from the interior of each field's tree convex hull, so row-to-row maneuvers
stay in headlands or sidelands. The tractor footprint may overlap the hull
at a row endpoint while the tractor leaves or enters that row. Road turns use
the same swept-footprint check against all trees and channels. The GIS road
surface over each crossing is the only permitted bridge across a channel;
adjacent field headlands and sidelands remain available to the turn. The
default road turn setback is 2.7 m. Other defaults include 0.35 m RS sampling and a 15 m RS
candidate radius. Reeds–Shepp alternatives are ranked using
an 8x reverse-distance multiplier, a 2 m equivalent penalty per internal gear
change, and a 50 m equivalent penalty per contiguous reverse episode. The
same costs are used by the route optimizer. `--reverse-event-penalty` controls
the last cost. `--field-rs-forward-only` discards every reversing field RS
candidate while road junctions remain forward-only in either mode. The default implement
range is half the median nearest-row spacing measured from GIS. One-tree rows
have no navigable line and cannot currently be selected for service.
The planner checks ranked RS candidates against the allowed field region and
tree clearance, then keeps the lowest-cost legal path for each connection.
Because the tractor's turning radius is a minimum, candidates are generated
at that radius and at `turn_radius_expansions` wider radii separated by
`turn_radius_step_m`. The defaults are two expansions at 0.35 m intervals,
so the 2.0 m trial searches 2.0, 2.35, and 2.7 m. Set
`--turn-radius-expansions 0` to search only the minimum radius. Road turns
must remain forward; field maneuvers can still reverse.
These choices are
potential planning-policy refinements before vehicle deployment.
