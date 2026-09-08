"""
tests/test_generation.py

Smoke tests for the full generation DAG (generation/orchestrator.py).
Uses small parcel counts for speed, but still exercises tessellation ->
hydrology -> roads -> crossings -> crops end to end.
"""

import pytest
import numpy as np
from shapely.geometry import LineString, Point

from farm_ir.schema import CropArea, HydrologyEdge, RoadEdge
from generation.orchestrator import FarmGenerationConfig, generate_farm, generate_validated, validate
from generation.crops import _pick_row_and_col_dirs


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

    assert not hasattr(scene, "terrain")  # removed -- see CLAUDE.md "Elevation / terrain: removed"

    assert len(scene.roads.edges) > 0
    assert all(isinstance(e, RoadEdge) for e in scene.roads.edges.values())

    assert scene.crossings
    for crossing in scene.crossings.values():
        channel = scene.hydrology.edges[crossing.hydrology_edge_id]
        assert LineString(channel.polyline).distance(Point(crossing.location)) < 1e-7

    assert len(scene.trees) > 0
    assert len(scene.weed_zones) == len(scene.parcels)
    for zone in scene.weed_zones.values():
        assert zone.row_centerlines
        assert zone.density_params == {
            "row_density_per_m": 4.5,
            "row_falloff_m": 0.35,
        }
        parcel_trees = [
            tree for tree in scene.trees.values()
            if tree.tags.get("parcel_id") == zone.tags.get("parcel_id")
        ]
        rows = [LineString(row) for row in zone.row_centerlines]
        assert all(
            min(row.distance(Point(tree.position)) for row in rows) < 1e-7
            for tree in parcel_trees
        )


def test_weed_row_parameters_are_validated():
    with pytest.raises(ValueError, match="weed_row_density_per_m"):
        FarmGenerationConfig(bounds=(0.0, 0.0, 10.0, 10.0), seed=1,
                             weed_row_density_per_m=-0.1)
    with pytest.raises(ValueError, match="weed_row_falloff_m"):
        FarmGenerationConfig(bounds=(0.0, 0.0, 10.0, 10.0), seed=1,
                             weed_row_falloff_m=-0.1)


def test_optimized_tree_layout_uses_long_incident_side():
    rectangle = np.array([(0.0, 0.0), (12.0, 0.0), (12.0, 4.0), (0.0, 4.0)])
    for seed in range(20):
        _, row_dir, col_dir = _pick_row_and_col_dirs(
            rectangle, np.random.default_rng(seed), optimize_tree_layout=True
        )
        assert abs(row_dir[0]) > 1.0 - 1e-10
        assert abs(row_dir[1]) < 1e-10
        assert abs(col_dir[0]) < 1e-10
        assert abs(col_dir[1]) > 1.0 - 1e-10


def test_tree_layout_choice_is_recorded_in_ir():
    scene = generate_farm(FarmGenerationConfig(
        bounds=(0.0, 0.0, 60.0, 60.0), seed=2, max_faces=4,
        optimize_tree_layout=True,
    ))
    assert all(parcel.planting.optimize_tree_layout for parcel in scene.parcels.values())


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
