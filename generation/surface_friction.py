"""Seeded surface fields and centroid sampling, independent of any output format."""

import numpy as np

from farm_ir.schema import SurfaceFriction, SurfaceFrictionField
from generation.spectral_field import generate_values, sample_values


def generate_field(parameters, bounds, resolution):
    """Filter white noise using sqrt(P), in cycles/m, with a radial Nyquist cut.

    A single affine normalization preserves the discrete spectrum and zero-mean
    variation. Max-absolute scaling guarantees the requested interval. Bilinear
    reconstruction is continuous and bounded (unlike overshooting cubic fits).
    The FFT domain is periodic; interpolation introduces the usual grid-scale
    attenuation, so the specified power law describes the stored grid.
    """
    low = parameters.friction_mean - parameters.friction_variation_range / 2
    high = parameters.friction_mean + parameters.friction_variation_range / 2
    values = generate_values(bounds, resolution, parameters.friction_mean,
                             parameters.friction_variation_range,
                             parameters.friction_scale_m,
                             parameters.friction_spectral_slope,
                             parameters.friction_seed)
    return SurfaceFrictionField(tuple(bounds), values.tolist(), low, high)


def generate_surface_friction(config):
    return SurfaceFriction(
        generate_field(config.crop_surface, config.bounds, config.farm_resolution_m),
        generate_field(config.road_surface, config.bounds, config.farm_resolution_m),
        config.channel_friction, config.farm_resolution_m,
    )


def sample_field(field, xy):
    """Bounded periodic bilinear evaluation at arbitrary ENU coordinates."""
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    if field.friction_min == field.friction_max:
        return np.full(len(xy), field.friction_min)
    return sample_values(field.bounds, field.values, xy)


def face_surface_regions(points, triangles, face_classes):
    """Road decks win over channels; slopes, bottoms, and crossing walls are channel."""
    return ["road" if kind == "road" else
            "channel" if kind == "crossing_wall" or min(points[i][2] for i in tri) < -1e-9
            else "crop" for tri, kind in zip(triangles, face_classes)]


def sample_mesh_friction(fields, points, triangles, face_classes, regions=None):
    centroids = np.asarray(points)[np.asarray(triangles)].mean(axis=1)[:, :2]
    regions = np.asarray(regions if regions is not None else
                         face_surface_regions(points, triangles, face_classes))
    result = np.full(len(triangles), fields.channel_friction, dtype=float)
    for name in ("crop", "road"):
        mask = regions == name
        result[mask] = sample_field(getattr(fields, name), centroids[mask])
    return result.tolist()
