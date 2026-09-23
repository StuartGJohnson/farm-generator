"""Periodic, band-limited scalar fields sampled in the farm's ENU frame."""

import math

import numpy as np


def generate_values(bounds, resolution, mean, variation_range, scale_m, slope, seed):
    """FFT-filter white noise to P(f) ∝ [1 + (f*scale_m)^2]^(-slope/2)."""
    minx, miny, maxx, maxy = bounds
    if not math.isfinite(resolution) or resolution <= 0:
        raise ValueError("farm_resolution_m must be finite and positive")
    if not all(math.isfinite(v) for v in bounds) or maxx <= minx or maxy <= miny:
        raise ValueError("field bounds must have positive finite dimensions")
    nx = max(2, math.ceil((maxx - minx) / resolution))
    ny = max(2, math.ceil((maxy - miny) / resolution))
    if variation_range == 0:
        return np.full((ny, nx), mean)
    noise = np.random.default_rng(seed).standard_normal((ny, nx))
    fx = np.fft.fftfreq(nx, d=(maxx - minx) / nx)
    fy = np.fft.fftfreq(ny, d=(maxy - miny) / ny)
    frequency = np.hypot(fx[None, :], fy[:, None])
    amplitude = (1 + (frequency * scale_m)**2)**(-slope / 4)
    amplitude[frequency > 1 / (2 * resolution)] = 0
    amplitude[0, 0] = 0
    variation = np.fft.ifft2(np.fft.fft2(noise) * amplitude).real
    variation -= variation.mean()
    maximum = np.max(np.abs(variation))
    if maximum > 0:
        variation /= maximum
    return mean + variation_range / 2 * variation


def sample_values(bounds, values, xy):
    """Continuous, bounded periodic bilinear interpolation of a row-major grid."""
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    values = np.asarray(values)
    ny, nx = values.shape
    minx, miny, maxx, maxy = bounds
    q = (xy - (minx, miny)) / (maxx - minx, maxy - miny) * (nx, ny)
    ij = np.floor(q).astype(np.int64)
    t = q - ij
    x, y = ij[:, 0] % nx, ij[:, 1] % ny
    xx, yy = (x + 1) % nx, (y + 1) % ny
    return ((1 - t[:, 1]) * ((1 - t[:, 0]) * values[y, x] + t[:, 0] * values[y, xx])
            + t[:, 1] * ((1 - t[:, 0]) * values[yy, x] + t[:, 0] * values[yy, xx]))
