"""
tests/test_generation.py

Smoke tests for the full generation DAG (generation/orchestrator.py).
Uses small parcel counts for speed, but still exercises tessellation ->
hydrology -> roads -> crossings -> crops end to end.
"""

import pytest
import numpy as np
from dataclasses import replace
from shapely.geometry import LineString, Point, Polygon, box
from shapely.ops import unary_union

from farm_ir.schema import CropArea, HydrologyEdge, ParcelGroup, RoadEdge
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


def test_outlands_extend_domain_with_crops_channels_and_no_roads():
    config = FarmGenerationConfig(
        bounds=(0.0, 0.0, 60.0, 60.0), seed=1, max_faces=4,
        outlands_m=10.0, standoff=3.5, headland_width=7.5,
        sideland_width=6.5, outlands_headland_width=2.0,
        outlands_sideland_width=1.5, tree_spacing=4.0, row_spacing=5.0,
    )
    scene, issues, _ = generate_validated(config)
    assert issues == []
    expected = {
        "outlands_north": (-10, 60, 70, 70),
        "outlands_south": (-10, -10, 70, 0),
        "outlands_east": (60, 0, 70, 60),
        "outlands_west": (-10, 0, 0, 60),
    }
    for pid, bounds in expected.items():
        parcel = scene.parcels[pid]
        assert parcel.parcel_type.value == "crop"
        assert parcel.parcel_group == ParcelGroup.OUTLAND
        assert parcel.planting.headland_width == config.outlands_headland_width
        assert parcel.planting.sideland_width == config.outlands_sideland_width
        assert Polygon(parcel.polygon).equals(box(*bounds))
        assert any(t.tags.get("parcel_id") == pid for t in scene.trees.values())
        assert any(z.tags.get("parcel_id") == pid and z.row_centerlines
                   for z in scene.weed_zones.values())
        assert len(parcel.boundary_refs) == 4
        assert not any(e.tags.get("parcel_id") == pid for e in scene.roads.edges.values())
    assert all(p.planting.headland_width == config.headland_width and
               p.planting.sideland_width == config.sideland_width
               for p in scene.parcels.values() if p.parcel_group == ParcelGroup.FARM)
    covered = unary_union([Polygon(p.polygon) for p in scene.parcels.values()])
    assert box(*config.domain_bounds).difference(covered).area < 0.005 * 6400
    assert scene.surface_height.bounds == config.domain_bounds
    assert scene.surface_friction.crop.bounds == config.domain_bounds
    assert scene.surface_friction.road.bounds == config.domain_bounds


def test_nonpositive_outlands_preserve_inner_domain():
    base = _small_config(seed=1)
    for distance in (0.0, -5.0):
        config = FarmGenerationConfig(bounds=base.bounds, seed=base.seed,
                                      max_faces=base.max_faces, outlands_m=distance)
        scene = generate_farm(config)
        assert config.domain_bounds == base.bounds
        assert all(p.parcel_group == ParcelGroup.FARM for p in scene.parcels.values())
        assert scene.surface_height.bounds == base.bounds


def test_outlands_clearances_leave_inner_farm_coordinates_fixed():
    config = FarmGenerationConfig(
        bounds=(0, 0, 120, 120), seed=1, max_faces=4,
        outlands_m=10, headland_width=7.5, sideland_width=6.5,
        row_spacing=5, tree_spacing=4,
    )
    first = generate_farm(config)
    changed = generate_farm(replace(config, outlands_headland_width=3,
                                    outlands_sideland_width=2))
    without_outlands = generate_farm(replace(config, outlands_m=0))
    for pid, parcel in first.parcels.items():
        if parcel.parcel_group != ParcelGroup.FARM:
            continue
        assert parcel.polygon == changed.parcels[pid].polygon
        assert parcel.polygon == without_outlands.parcels[pid].polygon
        positions = lambda scene: sorted(t.position for t in scene.trees.values()
                                         if t.tags.get("parcel_id") == pid)
        assert positions(first) == positions(changed)
        assert positions(first) == positions(without_outlands)
