"""Surface-height spectrum, IR persistence, and ground/water separation."""

from dataclasses import replace
import json

import numpy as np
import pytest

from generation.orchestrator import FarmGenerationConfig, generate_farm, load_farm, save_farm
from generation.surface_height import generate_surface_height, sample_height
from export.usd import build_ground_mesh


def test_height_field_spectrum_and_bounds():
    config = FarmGenerationConfig(bounds=(0, 0, 128, 128), seed=1)
    field = generate_surface_height(config)
    values = np.asarray(field.values)
    assert field == generate_surface_height(config)
    assert field != generate_surface_height(replace(config, height_seed=1002))
    assert values.mean() == pytest.approx(config.height_mean)
    assert values.min() >= field.height_min - 1e-12
    assert values.max() <= field.height_max + 1e-12
    white = np.random.default_rng(config.height_seed).standard_normal(values.shape)
    f = np.fft.fftfreq(128, d=1)
    frequency = np.hypot(f[:, None], f[None, :])
    expected = (np.abs(np.fft.fft2(white))**2
                * (1 + (frequency * config.height_scale_m)**2)**(-config.height_spectral_slope / 2))
    observed = np.abs(np.fft.fft2(values - values.mean()))**2
    band = (frequency > 0) & (frequency <= .5) & (expected > 1e-8)
    ratio = observed[band] / expected[band]
    assert np.std(ratio) / np.mean(ratio) < 1e-9
    assert observed[frequency > .5].max() < 1e-20
    samples = sample_height(field, [(0.3, 0.7), (128.3, 128.7)])
    assert samples[0] == pytest.approx(samples[1])
    assert field.height_min <= samples[0] <= field.height_max


def test_height_applies_to_ground_but_not_water_and_survives_round_trip(tmp_path):
    config = FarmGenerationConfig(bounds=(0, 0, 50, 50), seed=1, max_faces=4,
                                  standoff=3.5, hydrology_add_water=True)
    scene = generate_farm(config)
    terrain = build_ground_mesh(scene, config.bounds)
    flat_scene = replace(scene, surface_height=None)
    flat = build_ground_mesh(flat_scene, config.bounds)
    assert terrain.triangles == flat.triangles
    assert terrain.face_surface_regions == flat.face_surface_regions
    assert terrain.face_friction == flat.face_friction
    offsets = sample_height(scene.surface_height, [point[:2] for point in flat.points])
    assert np.allclose(np.asarray(terrain.points)[:, 2],
                       np.asarray(flat.points)[:, 2] + offsets * terrain.height_weights)
    assert any(abs(offset) > .01 for offset in offsets)
    assert min(terrain.height_weights) < 1e-8
    road_vertices = {i for triangle, kind in zip(terrain.triangles, terrain.face_classes)
                     if kind == "road" for i in triangle}
    assert road_vertices and all(terrain.height_weights[i] == 1 for i in road_vertices)
    # Matching road/crop breakline edges must use the very same vertices.
    by_region = {"road": {}, "crop": {}}
    for triangle, region in zip(terrain.triangles, terrain.face_surface_regions):
        if region not in by_region:
            continue
        for k in range(3):
            a, b = triangle[k], triangle[(k + 1) % 3]
            xy_key = tuple(sorted((tuple(round(v, 6) for v in terrain.points[a][:2]),
                                   tuple(round(v, 6) for v in terrain.points[b][:2]))))
            by_region[region].setdefault(xy_key, set()).add(tuple(sorted((a, b))))
    shared_xy = by_region["road"].keys() & by_region["crop"].keys()
    assert shared_xy
    assert all(by_region["road"][edge] == by_region["crop"][edge] for edge in shared_xy)
    assert terrain.water_surfaces == flat.water_surfaces
    # The actual shoreline is pinned to the water plane; artificial perimeter
    # edges at roads and the domain boundary are handled separately.
    from scipy.spatial import cKDTree
    ground_xy = np.asarray(flat.points)[:, :2]
    tree = cKDTree(ground_xy)
    pinned = 0
    for surface in terrain.water_surfaces:
        for a, b in surface.boundary_edges:
            for vertex in (a, b):
                water_point = surface.points[vertex]
                for index in tree.query_ball_point(water_point[:2], 1e-5):
                    if abs(flat.points[index][2] - water_point[2]) < 1e-5:
                        assert terrain.points[index][2] == pytest.approx(water_point[2], abs=1e-5)
                        pinned += 1
    assert pinned > 0
    path = tmp_path / "scene.json"
    save_farm(scene, config, str(path))
    restored, restored_config = load_farm(str(path))
    assert restored.surface_height == scene.surface_height
    assert restored_config == config
    # Old scenes acquire the deterministic field when loaded.
    doc = json.loads(path.read_text())
    doc["scene"].pop("surface_height")
    path.write_text(json.dumps(doc))
    migrated, _ = load_farm(str(path))
    assert migrated.surface_height == scene.surface_height


@pytest.mark.parametrize("change", [
    {"height_variation_range_m": -0.1}, {"height_scale_m": 0},
    {"height_spectral_slope": -1}, {"height_mean": float("nan")},
    {"height_seed": -1},
    {"shoreline_taper_m": -1},
])
def test_invalid_height_parameters(change):
    with pytest.raises(ValueError):
        FarmGenerationConfig(bounds=(0, 0, 8, 8), seed=1, **change)
