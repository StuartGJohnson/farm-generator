"""
generation/elevation.py

Stage 4 of the generation DAG. `ElevationField.source =
DERIVED_FROM_HYDROLOGY` this phase (see CLAUDE.md principle 5). With no
trunk and no macro elevation gradient this phase, the domain is treated
as flat/uniform -- baking is trivial (a constant-value raster), but the
raster is still generated (not skipped) since downstream consumers
(exporters, tree placement) should always read elevation from
`ElevationField`, never assume flatness themselves.

Breaklines: the generation order runs elevation (stage 4) before roads
(stage 5), so only hydrology edges exist yet to record as breaklines.
Since the terrain is flat this phase, breaklines have no actual effect on
the baked values -- this is a documented ordering quirk, not a bug: a
real gradient-driving trunk phase would need to re-bake elevation once
road edges exist too (see CLAUDE.md principle 5).
"""

from __future__ import annotations

import numpy as np

from farm_ir.schema import ElevationField, ElevationSource, FarmScene


def run(scene: FarmScene, config, rng) -> FarmScene:
    minx, miny, maxx, maxy = config.bounds
    res = config.elevation_resolution
    width_cells = max(1, int(np.ceil((maxx - minx) / res)))
    height_cells = max(1, int(np.ceil((maxy - miny) / res)))

    values = [[config.base_elevation for _ in range(width_cells)] for _ in range(height_cells)]

    scene.terrain.elevation = ElevationField(
        resolution=res,
        origin=(minx, miny),
        width_cells=width_cells,
        height_cells=height_cells,
        values=values,
        source=ElevationSource.DERIVED_FROM_HYDROLOGY,
    )
    scene.terrain.breakline_edge_ids = list(scene.hydrology.edges.keys())

    return scene
