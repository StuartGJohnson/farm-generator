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

    figure, axes = plt.subplots(1, 2, figsize=(13, 5), constrained_layout=True)
    extent = (field.bounds[0], field.bounds[2], field.bounds[1], field.bounds[3])
    scale = Normalize(field.height_min, field.height_max) if field.height_min < field.height_max else None
    raw = axes[0].imshow(values, origin="lower", extent=extent, cmap="terrain", norm=scale)
    figure.colorbar(raw, ax=axes[0], label="Height offset [m]")
    axes[0].set_title("Generated height field")
    mesh_colors = PolyCollection(polygons, array=means, cmap="terrain", edgecolors="none", rasterized=True)
    axes[1].add_collection(mesh_colors)
    axes[1].set_xlim(xy[:, 0].min(), xy[:, 0].max())
    axes[1].set_ylim(xy[:, 1].min(), xy[:, 1].max())
    figure.colorbar(mesh_colors, ax=axes[1], label="Ground elevation [m]")
    axes[1].set_title("Meshed ground (channels included)")
    for axis in axes:
        axis.set_aspect("equal")
        axis.set_xlabel("East [m]")
        axis.set_ylabel("North [m]")
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=160)
    plt.close(figure)
