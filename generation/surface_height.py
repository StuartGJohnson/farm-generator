"""Deterministic ground elevation offsets generated before mesh export."""

import numpy as np

from farm_ir.schema import SurfaceHeightField
from generation.spectral_field import generate_values, sample_values


def generate_surface_height(config):
    values = generate_values(config.bounds, config.farm_resolution_m,
                             config.height_mean, config.height_variation_range_m,
                             config.height_scale_m, config.height_spectral_slope,
                             config.height_seed)
    half_range = config.height_variation_range_m / 2
    return SurfaceHeightField(tuple(config.bounds), values.tolist(),
                              config.farm_resolution_m,
                              config.height_mean - half_range,
                              config.height_mean + half_range,
                              config.shoreline_taper_m)


def sample_height(field, xy):
    """Sample the IR field at arbitrary ENU coordinates, including mesh vertices."""
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    if field.height_min == field.height_max:
        return np.full(len(xy), field.height_min)
    return sample_values(field.bounds, field.values, xy)
