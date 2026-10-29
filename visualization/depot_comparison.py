"""Compare the original planting grid with a depot cut in one field."""

from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as PolygonPatch
from shapely.geometry import Polygon


def save_depot_tree_comparison(scene, original, path):
    if scene.depot is None:
        raise ValueError("scene has no depot")
    field_id = scene.depot.field_id
    boundary = Polygon(scene.parcels[field_id].polygon)
    depot = Polygon(scene.depot.polygon)
    views = ((original, "Original field"), (scene, "Field with depot"))
    fig, axes = plt.subplots(1, 2, figsize=(14, 7), constrained_layout=True)
    x0, y0, x1, y1 = boundary.bounds
    for ax, (farm, title) in zip(axes, views):
        ax.add_patch(PolygonPatch(list(boundary.exterior.coords), closed=True,
                                  facecolor="#edf6e7", edgecolor="#415840", linewidth=1.2))
        trees = [tree for tree in farm.trees.values()
                 if tree.tags.get("parcel_id") == field_id]
        if trees:
            ax.scatter([tree.position[0] for tree in trees],
                       [tree.position[1] for tree in trees], s=13,
                       color="#15783a", zorder=3)
        if farm.depot is not None:
            ax.add_patch(PolygonPatch(list(depot.exterior.coords), closed=True,
                                      facecolor="#777777", edgecolor="white", zorder=2))
        ax.set_xlim(x0 - 3, x1 + 3)
        ax.set_ylim(y0 - 3, y1 + 3)
        ax.set_aspect("equal")
        ax.set_title(f"{title}: {len(trees)} trees")
        ax.set_xlabel("East [m]")
        ax.set_ylabel("North [m]")
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=180)
    plt.close(fig)
