"""Deterministic ground elevation offsets generated before mesh export."""

import numpy as np

from farm_ir.schema import SurfaceHeightField
from generation.spectral_field import generate_values, sample_values
from shapely.geometry import Point


def generate_surface_height(config):
    values = generate_values(config.domain_bounds, config.farm_resolution_m,
                             config.height_mean, config.height_variation_range_m,
                             config.height_scale_m, config.height_spectral_slope,
                             config.height_seed)
    half_range = config.height_variation_range_m / 2
    road = generate_values(config.domain_bounds, config.farm_resolution_m,
                           config.height_mean, config.road_height_variation_range_m,
                           config.road_height_scale_m, config.road_height_spectral_slope,
                           config.road_height_seed)
    road_half_range = config.road_height_variation_range_m / 2
    return SurfaceHeightField(tuple(config.domain_bounds), values.tolist(),
                              config.farm_resolution_m,
                              config.height_mean - half_range,
                              config.height_mean + half_range,
                              config.shoreline_taper_m, road.tolist(),
                              config.height_mean - road_half_range,
                              config.height_mean + road_half_range,
                              config.road_field_blend_m)


def sample_height(field, xy):
    """Sample the IR field at arbitrary ENU coordinates, including mesh vertices."""
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    if field.height_min == field.height_max:
        return np.full(len(xy), field.height_min)
    return sample_values(field.bounds, field.values, xy)


def sample_ground_height(field, xy, road_area=None):
    """Blend the crop spectrum into the road spectrum inside crop land.

    The road value wins exactly at the shared breakline, so the two adjacent
    meshes have identical elevations there.
    """
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    crop = sample_height(field, xy)
    if field.road_values is None or road_area is None or road_area.is_empty:
        return crop
    road = sample_values(field.bounds, field.road_values, xy)
    distance = np.asarray([0.0 if road_area.covers(Point(*point)) else
                           road_area.distance(Point(*point)) for point in xy])
    if field.road_field_blend_m <= 0:
        mix = (distance > 1e-8).astype(float)
    else:
        mix = np.clip(distance / field.road_field_blend_m, 0.0, 1.0)
        mix = mix * mix * (3.0 - 2.0 * mix)
    return road + mix * (crop - road)
