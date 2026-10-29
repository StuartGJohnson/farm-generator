"""Diagnostic view of the generated height field and meshed ground."""

from pathlib import Path

import numpy as np


def save_surface_height_plot(mesh, path):
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection
    from matplotlib.colors import Normalize

    field = mesh.surface_height
    if field is None:
        raise ValueError("mesh has no generated surface height field")
    values = np.asarray(field.values)
    ground = np.asarray(mesh.points)
    xy = ground[:, :2]
    z = ground[:, 2]
    polygons = xy[np.asarray(mesh.triangles)]
    means = z[np.asarray(mesh.triangles)].mean(axis=1)

    figure, axes = plt.subplots(1, 3, figsize=(18, 5), constrained_layout=True)
    extent = (field.bounds[0], field.bounds[2], field.bounds[1], field.bounds[3])
    road_min = field.road_height_min if field.road_height_min is not None else field.height_min
    road_max = field.road_height_max if field.road_height_max is not None else field.height_max
    low = min(field.height_min, road_min)
    high = max(field.height_max, road_max)
    scale = Normalize(low, high) if low < high else None
    raw = axes[0].imshow(values, origin="lower", extent=extent, cmap="terrain", norm=scale)
    figure.colorbar(raw, ax=axes[0], label="Height offset [m]")
    axes[0].set_title("Field ground height")
    road_values = np.asarray(field.road_values if field.road_values is not None else field.values)
    road = axes[1].imshow(road_values, origin="lower", extent=extent, cmap="terrain", norm=scale)
    figure.colorbar(road, ax=axes[1], label="Road height offset [m]")
    axes[1].set_title("Road height (same scale)")
    mesh_colors = PolyCollection(polygons, array=means, cmap="terrain", edgecolors="none", rasterized=True)
    axes[2].add_collection(mesh_colors)
    axes[2].set_xlim(xy[:, 0].min(), xy[:, 0].max())
    axes[2].set_ylim(xy[:, 1].min(), xy[:, 1].max())
    figure.colorbar(mesh_colors, ax=axes[2], label="Ground elevation [m]")
    axes[2].set_title("Blended ground and channels")
    for axis in axes:
        axis.set_aspect("equal")
        axis.set_xlabel("East [m]")
        axis.set_ylabel("North [m]")
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=160)
    plt.close(figure)
