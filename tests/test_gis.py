"""Integration check: independent GDAL reload of generated semantic layers."""

import json
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
    assert all((tmp_path / "gis" / "tiles" / str(z)).is_dir() for z in (16, 17, 18))
    with zipfile.ZipFile(tmp_path / "gis" / "farm.qgz") as archive:
        qgs = archive.read(next(name for name in archive.namelist() if name.endswith(".qgs"))).decode()
    assert "./orthophoto.tif" in qgs
    assert "./farm.gpkg|layername=fields" in qgs
