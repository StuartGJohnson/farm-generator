"""Plan-view diagnostics of continuous centroid mu and exported bin values."""

from pathlib import Path
import numpy as np

from generation.surface_friction import face_surface_regions


def save_surface_friction_plot(mesh, path, quantized=None):
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection
    from matplotlib.colors import Normalize

    if mesh.face_friction is None:
        raise ValueError("mesh has no surface friction samples")
    xy = np.asarray(mesh.points)[:, :2]
    polygons = xy[np.asarray(mesh.triangles)]
    regions = np.asarray(mesh.face_surface_regions if mesh.face_surface_regions is not None else
                         face_surface_regions(mesh.points, mesh.triangles, mesh.face_classes))
    values = np.asarray(mesh.face_friction)
    figure, axes = plt.subplots(1, 4 if quantized is not None else 3, figsize=(20 if quantized is not None else 15, 5), constrained_layout=True)
    panels = [("Crop field (masked)", regions == "crop", values),
              ("Road field (masked)", regions == "road", values),
              ("All surfaces; channels included", np.ones(len(values), dtype=bool), values)]
    if quantized is not None:
        panels.append(("USD midpoint bins", np.ones(len(values), dtype=bool), np.asarray(quantized)))
    for ax, (title, mask, data) in zip(axes, panels):
        limits = (data[mask].min(), data[mask].max()) if mask.any() else (0, 1)
        if limits[0] == limits[1]:
            limits = (limits[0]-0.01, limits[1]+0.01)
        if title.startswith(("All", "USD")):
            limits = (values.min(), values.max())
        colors = PolyCollection(polygons[mask], array=data[mask], cmap="viridis",
                                norm=Normalize(*limits), edgecolors="none", rasterized=True)
        ax.add_collection(colors)
        ax.set_xlim(xy[:, 0].min(), xy[:, 0].max())
        ax.set_ylim(xy[:, 1].min(), xy[:, 1].max())
        ax.set_aspect("equal")
        ax.set_facecolor("#dddddd")
        ax.set_title(title)
        ax.set_xlabel("East [m]")
        ax.set_ylabel("North [m]")
        figure.colorbar(colors, ax=ax, shrink=0.65, label="Surface friction μ")
    figure.suptitle("Seeded surface friction at triangle centroids (gray = other surface types)")
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=160)
    plt.close(figure)
