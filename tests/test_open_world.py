"""Open driving worlds reuse the farm's terrain, physics, and GIS frame."""

import sqlite3

import numpy as np
import pytest

from export.usd import build_ground_mesh, write_ground_mesh_usda
from farm_ir.schema import FarmScene
from generation.open_world import (OpenWorldConfig, generate_open_world,
                                   load_open_world, save_open_world)
from generation.surface_height import sample_ground_height
from gis import export_gis
from shapely.geometry import box


@pytest.mark.parametrize("surface,region,material", [
    ("field", "crop", "GrassMaterial"),
    ("road", "road", "GravelRoadMaterial"),
])
def test_open_world_mesh_and_usd(tmp_path, surface, region, material):
    config = OpenWorldConfig(bounds=(0, 0, 16, 16), surface=surface)
    scene = generate_open_world(config)
    assert isinstance(scene, FarmScene)
    assert not scene.parcels and not scene.roads.edges and not scene.hydrology.edges
    assert not scene.trees and not scene.crossings
    assert scene == generate_open_world(config)
    mesh = build_ground_mesh(scene, config.bounds, surface_region=config.surface_region)
    assert set(mesh.face_surface_regions) == {region}
    assert set(mesh.face_classes) == {"ground"}
    assert not mesh.water_surfaces
    field = getattr(scene.surface_friction, region)
    assert min(mesh.face_friction) >= field.friction_min
    assert max(mesh.face_friction) <= field.friction_max
    expected = sample_ground_height(scene.surface_height,
                                    [point[:2] for point in mesh.points],
                                    box(*config.bounds) if surface == "road" else None)
    assert np.allclose([point[2] for point in mesh.points], expected)
    path = tmp_path / "farm.usda"
    write_ground_mesh_usda(mesh, str(path), scene=scene)
    text = path.read_text()
    assert f"rel material:binding = </World/Looks/{material}>" in text
    assert "PhysicsCollisionAPI" in text
    assert "PhysxVehicleTireFrictionTable" in text
    saved = tmp_path / "farm.yaml"
    save_open_world(scene, config, str(saved))
    assert load_open_world(str(saved)) == (scene, config)


def test_open_world_gis_ground_extent(tmp_path):
    config = OpenWorldConfig(bounds=(0, 0, 16, 16), surface="road")
    scene = generate_open_world(config)
    export_gis(scene, config.bounds, tmp_path, tiles=False,
               open_surface=config.surface)
    with sqlite3.connect(tmp_path / "gis" / "farm.gpkg") as database:
        assert database.execute("SELECT id, surface FROM ground_surface").fetchall() == [
            ("open_ground", "road")]
        assert database.execute(
            "SELECT COUNT(*) FROM gpkg_contents WHERE table_name='fields'").fetchone()[0] == 0
    assert (tmp_path / "gis" / "map_frame.json").is_file()
    assert (tmp_path / "gis" / "farm.qgz").is_file()


def test_open_world_rejects_invalid_surface():
    with pytest.raises(ValueError, match="surface"):
        OpenWorldConfig(surface="gravel")
