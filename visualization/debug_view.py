"""
visualization/debug_view.py

Primary dev tool for this phase (see CLAUDE.md "Visualization -- primary
tool, build this early"). Pure consumer of `FarmScene` -- no
simulator-specific concepts. Renders a static PNG per scene via
matplotlib, with per-layer toggles (roads, parcels, crossings, trees,
weed zones).

Run directly to generate and save plots for multiple seeds in one batch
-- ALWAYS review several seeds, never just one, since several classes of
bug (see CLAUDE.md "Key lessons") only appear for some seeds.
"""

from __future__ import annotations

import os

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import PatchCollection
from matplotlib.patches import Polygon as MplPolygon

from farm_ir.schema import FarmScene

DEFAULT_LAYERS = {"parcels", "roads", "crossings", "trees", "weed_zones"}


def plot_scene(scene: FarmScene, layers: set[str] | None = None, title: str | None = None):
    layers = DEFAULT_LAYERS if layers is None else set(layers)
    fig, ax = plt.subplots(figsize=(10, 10))

    if "weed_zones" in layers and scene.weed_zones:
        for wz in scene.weed_zones.values():
            ax.add_patch(MplPolygon(wz.polygon, closed=True, facecolor="khaki",
                                     edgecolor="none", alpha=0.3, zorder=1))

    if "parcels" in layers and scene.parcels:
        parcels = list(scene.parcels.values())
        patches = [MplPolygon(p.polygon, closed=True) for p in parcels]
        color_rng = np.random.default_rng(0)
        colors = color_rng.uniform(0.75, 0.95, size=len(patches))
        coll = PatchCollection(patches, array=colors, cmap="Greens",
                                edgecolor="gray", linewidth=0.4, alpha=0.5, zorder=2)
        ax.add_collection(coll)

    if "roads" in layers and scene.roads.edges:
        for i, edge in enumerate(scene.roads.edges.values()):
            xs = [p[0] for p in edge.polyline]
            ys = [p[1] for p in edge.polyline]
            ax.plot(xs, ys, color="saddlebrown", linewidth=1.2, zorder=4,
                    label="road" if i == 0 else None)

    if "crossings" in layers and scene.crossings:
        for i, c in enumerate(scene.crossings.values()):
            edge = scene.roads.edges.get(c.road_edge_id)
            if edge is None:
                continue
            xs = [p[0] for p in edge.polyline]
            ys = [p[1] for p in edge.polyline]
            ax.plot(xs, ys, color="crimson", linewidth=1.6, zorder=5,
                    label="crossing" if i == 0 else None)

    if "trees" in layers and scene.trees:
        xs = [t.position[0] for t in scene.trees.values()]
        ys = [t.position[1] for t in scene.trees.values()]
        ax.scatter(xs, ys, s=6, color="darkgreen", zorder=6, label="tree")

    ax.autoscale_view()
    ax.set_aspect("equal")
    ax.legend(loc="upper right", fontsize=9)
    if title:
        ax.set_title(title)
    return fig, ax


def save_scene_png(scene: FarmScene, out_path: str, layers: set[str] | None = None,
                    title: str | None = None, dpi: int = 300) -> None:
    """
    dpi defaults to 300, not matplotlib's 100-150 norm, because this tool
    is used to eyeball tree-level detail (see module docstring): at
    dpi=150 a 100m-wide domain renders at only ~15 px/m, and a ~6px tree
    marker's center can only be placed to ~0.2m precision on that pixel
    grid -- since rows follow each parcel's own (generally oblique) edge
    angle rather than the pixel grid, that sub-pixel rounding shows up as
    a visible wobble along rows. Confirmed by re-rendering the same scene
    at 600 dpi and comparing an identical crop: the wobble disappeared,
    while the underlying tree position data was already full float64
    precision either way -- this was a rasterization artifact, not a
    generation bug.
    """
    fig, ax = plot_scene(scene, layers=layers, title=title)
    out_dir = os.path.dirname(out_path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    fig.savefig(out_path, dpi=dpi, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    from generation.orchestrator import FarmGenerationConfig, generate_validated, save_farm

    out_dir = os.path.join(os.path.dirname(__file__), "..", "debug_out", "farm_scenes")
    bounds = (0.0, 0.0, 50.0, 50.0)
    seeds = [1, 2, 3, 4, 5]

    for seed in seeds:
        config = FarmGenerationConfig(bounds=bounds,
                                      seed=seed,
                                      max_faces=4,
                                      standoff=3.5,
                                      headland_width=7.5,
                                      sideland_width=6.5)
        scene, issues, used_seed = generate_validated(config)

        status = "OK" if not issues else f"{len(issues)} ISSUE(S)"
        retry_note = f" (used seed {used_seed} after retry)" if used_seed != seed else ""
        print(f"seed={seed}{retry_note}: {len(scene.parcels)} parcels, "
              f"{len(scene.roads.edges)} road edges, {len(scene.crossings)} crossings, "
              f"{len(scene.trees)} trees -- {status}")
        for issue in issues:
            print(f"    ! {issue}")

        save_scene_png(scene, os.path.join(out_dir, f"farm_seed_{seed}.png"),
                        title=f"farm-gen debug view\nseed={seed}")
        save_farm(scene, config, os.path.join(out_dir, f"farm_seed_{seed}.yaml"))
