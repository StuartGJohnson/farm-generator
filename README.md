# farm-generator
Procedural farm generation via intermediate representation for robotics simulations. Tools for conversion to (minimally) IsaacSim/USD.

See `CLAUDE.md` for the full architecture/design reference.

## Setup

This repo uses the `usd2508` conda environment (shared with the USD/IsaacSim
tooling this project eventually exports to), not a Python venv.

```bash
conda activate usd2508
pip install -e ".[dev]"
```

That installs this package (editable) plus its runtime dependencies (numpy,
shapely, networkx, scipy, matplotlib, pyyaml) and dev dependencies (pytest),
all pinned in `pyproject.toml`. Re-run the `pip install` after pulling
changes that touch `pyproject.toml`.

If `usd2508` doesn't exist yet on your machine: `conda create -n usd2508
python=3.12` (any Python >= 3.10 works), then activate and `pip install` as
above.

## Running the tests

```bash
conda activate usd2508
python -m pytest tests/ -q
```

## Running the debug visualizer

`visualization/debug_view.py` is the primary dev tool (see CLAUDE.md
"Visualization"). Run it as a module from the repo root:

```bash
conda activate usd2508
python -m visualization.debug_view
```

This generates a farm for 5 seeds and writes, per seed, into
`debug_out/farm_scenes/`:
- `farm_seed_<N>.png` — a static plot (parcels/roads/crossings/trees/weed
  zones), at 300 DPI so tree-level detail is actually legible.
- `farm_seed_<N>.yaml` — the full `FarmScene` IR plus the
  `FarmGenerationConfig` that produced it (see `farm_ir/serialize.py`),
  human-readable and round-trippable. This is what a future USD/Gazebo
  exporter will read as input.

`debug_out/` is scratch output (gitignored, aside from whatever's already
checked in for reference) — safe to delete and regenerate at any time.

## Running the example prototype

`examples/farm_mesh_prototype.py` is a self-contained reference prototype
(see CLAUDE.md "Example reference code") — read it as a spec of behavior,
not production code. Run directly:

```bash
conda activate usd2508
python examples/farm_mesh_prototype.py
```

It generates and validates 5 seeds and saves plots to
`debug_out/examples/`.

## Generating hydrology ground meshes

Run `python examples/generate_ground_meshes.py` to generate five 16 m by
16 m farm IRs and corresponding USDA ground meshes in `debug_out/mesh/`.
The exporter uses deterministic Poisson-disc (blue-noise) points, producing
an amorphous Delaunay/Voronoi-dual mesh, plus exact constrained breaklines at
the shoulders and flat bottoms of the trapezoidal irrigation channels.
Each seed also produces a wireframe PNG showing all vertices, triangle edges,
and the constrained breaklines highlighted in red.

The example enables exporter-side lateral channel undulation with a 10 cm
maximum amplitude, wavelengths strictly greater than 1 m, and 20 cm curve
sampling. `ChannelUndulationConfig` controls these values and an optional seed
offset. Perturbations are deterministic per farm seed and logical hydrology
edge, taper to zero at network nodes, and are applied before deriving both the
channel shoulder and bottom breaklines. The `FarmScene` IR is not modified.

Before constructing the PSLG, the exporter creates first-class `ChannelFace`
objects. Each face corresponds to one parcel-side hydrology edge and owns its
paired shoulder and bottom arcs, exact shared corner endpoints, depth, and
endpoint cross-slope ribs. GEOS computes valid offset contours; those contours
are split at analytically determined corner stations and mapped one-to-one to
their source hydrology edges. This avoids recovering corners by nearby-vertex
indices and provides a reusable surface-patch representation for later roads
and bridge approaches.
At every authored parcel corner, a transverse PSLG breakline connects the
shoulder to the bottom contour. This prevents constrained Delaunay from
choosing its own miter across the channel slope and establishes a reusable
surface-patch convention for later roads and structures.

## Generating a farm programmatically

```python
from generation.orchestrator import FarmGenerationConfig, generate_validated, save_farm

config = FarmGenerationConfig(bounds=(0.0, 0.0, 100.0, 100.0), seed=1)
scene, issues, used_seed = generate_validated(config)
save_farm(scene, config, "my_farm.yaml")
```
