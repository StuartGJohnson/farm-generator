"""
tests/test_generation.py

Smoke tests for the full generation DAG (generation/orchestrator.py).
Uses small parcel counts for speed, but still exercises tessellation ->
hydrology -> elevation -> roads -> crossings -> crops end to end.
"""

from farm_ir.schema import CropArea, HydrologyEdge, RoadEdge
from generation.orchestrator import FarmGenerationConfig, generate_farm, generate_validated, validate


def _small_config(seed: int) -> FarmGenerationConfig:
    return FarmGenerationConfig(bounds=(0.0, 0.0, 60.0, 60.0), seed=seed, max_faces=8)


def test_generate_farm_produces_all_layers():
    scene = generate_farm(_small_config(seed=1))

    assert len(scene.parcels) >= 1
    assert all(isinstance(p, CropArea) for p in scene.parcels.values())
    for parcel in scene.parcels.values():
        assert len(parcel.polygon) >= 3
        assert len(parcel.boundary_refs) == len(parcel.polygon)

    assert len(scene.hydrology.edges) > 0
    assert all(isinstance(e, HydrologyEdge) for e in scene.hydrology.edges.values())

    assert scene.terrain.elevation is not None
    assert scene.terrain.elevation.width_cells > 0
    assert scene.terrain.elevation.height_cells > 0

    assert len(scene.roads.edges) > 0
    assert all(isinstance(e, RoadEdge) for e in scene.roads.edges.values())

    assert len(scene.trees) > 0
    assert len(scene.weed_zones) == len(scene.parcels)


def test_generate_farm_is_deterministic():
    config = _small_config(seed=42)
    scene_a = generate_farm(config)
    scene_b = generate_farm(config)

    assert sorted(scene_a.parcels.keys()) == sorted(scene_b.parcels.keys())
    for pid in scene_a.parcels:
        assert scene_a.parcels[pid].polygon == scene_b.parcels[pid].polygon


def test_generate_validated_multiple_seeds():
    for seed in range(1, 8):
        config = _small_config(seed=seed)
        scene, issues, used_seed = generate_validated(config, max_seed_tries=10)
        assert issues == [], f"seed {seed} (tried up to {used_seed}): {issues}"


def test_validate_returns_list_of_strings():
    config = _small_config(seed=3)
    scene = generate_farm(config)
    issues = validate(scene, config)
    assert isinstance(issues, list)
    assert all(isinstance(issue, str) for issue in issues)
