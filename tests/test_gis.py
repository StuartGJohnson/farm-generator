"""Integration check: independent GDAL reload of generated semantic layers."""

import json
import sqlite3
import subprocess
import zipfile
from pathlib import Path

from PIL import Image

from generation.orchestrator import FarmGenerationConfig, generate_validated
from gis import export_gis


def test_gis_roundtrip(tmp_path):
    config = FarmGenerationConfig(bounds=(0, 0, 80, 80), seed=4, max_faces=3,
                                  row_spacing=5, tree_spacing=4,
                                  headland_width=6, sideland_width=5,
                                  standoff=3)
    scene, issues, _ = generate_validated(config)
    assert not issues
    rgb = tmp_path / "isaac_rgb.png"
    Image.new("RGB", (8, 8), (48, 96, 32)).save(rgb)
    export_gis(scene, config.bounds, tmp_path, rgb_path=rgb, gsd_m=10, tiles=True)
    subprocess.run(["/usr/bin/python3", str(Path(__file__).resolve().parents[1] / "gis" / "verify.py"),
                    str(tmp_path / "gis")], check=True)
    info = json.loads(subprocess.check_output(["gdalinfo", "-json", str(tmp_path / "gis" / "orthophoto.tif")]))
    assert info["size"] == [8, 8]
    assert info["geoTransform"][1] == 10
    assert info["geoTransform"][5] == -10
    assert all((tmp_path / "gis" / "tiles" / str(z)).is_dir() for z in range(16, 22))
    with zipfile.ZipFile(tmp_path / "gis" / "farm.qgz") as archive:
        qgs = archive.read(next(name for name in archive.namelist() if name.endswith(".qgs"))).decode()
    assert "./orthophoto.tif" in qgs
    assert "./farm.gpkg|layername=fields" in qgs


def test_outlands_field_semantics(tmp_path):
    config = FarmGenerationConfig(
        bounds=(0, 0, 60, 60), seed=1, max_faces=4,
        outlands_m=10, standoff=3.5,
        headland_width=7.5, sideland_width=6.5,
        outlands_headland_width=2, outlands_sideland_width=1.5,
        row_spacing=5, tree_spacing=4,
    )
    scene, issues, _ = generate_validated(config)
    assert not issues
    output = tmp_path / "outlands_gis"
    export_gis(scene, config.domain_bounds, output, tiles=False)
    with sqlite3.connect(output / "gis" / "farm.gpkg") as gpkg:
        fields = gpkg.execute(
            "SELECT id, name, kind, parcel_group FROM fields WHERE parcel_group = 'outland'"
        ).fetchall()
        assert sorted((field_id, name) for field_id, name, _, _ in fields) == [
            ("outlands_east", "Outlands East"),
            ("outlands_north", "Outlands North"),
            ("outlands_south", "Outlands South"),
            ("outlands_west", "Outlands West"),
        ]
        assert all(kind == "crop" for _, _, kind, _ in fields)
        farm_names = [name for (name,) in gpkg.execute(
            "SELECT name FROM fields WHERE parcel_group = 'farm' ORDER BY id"
        )]
        assert farm_names == [f"Field {i:03d}" for i in range(1, len(farm_names) + 1)]
        assert gpkg.execute("SELECT COUNT(*) FROM trees WHERE field_id LIKE 'outlands_%'").fetchone()[0] > 0
        assert gpkg.execute("SELECT COUNT(*) FROM roads WHERE field_id LIKE 'outlands_%'").fetchone()[0] == 0
