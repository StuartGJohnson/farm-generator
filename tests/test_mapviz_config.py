"""Check farm georeferencing and Mapviz's generated offline tile source."""

import json
import sys
from pathlib import Path

from pyproj import Transformer
import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "src" / "bringup"
sys.path.insert(0, str(PACKAGE_ROOT))
from bringup_ros.mapviz_config import farm_map_origin, write_mapviz_config  # noqa: E402


def test_generated_mapviz_config_uses_farm_georeference_and_tiles(tmp_path):
    farm = tmp_path / "farm"
    gis = farm / "gis"
    tile = gis / "tiles" / "21" / "340792" / "807440.png"
    tile.parent.mkdir(parents=True)
    tile.write_bytes(b"png")
    longitude, latitude, altitude = -121.5, 38.2, 12.0
    east, north = Transformer.from_crs("EPSG:4326", "EPSG:32610", always_xy=True).transform(
        longitude, latitude
    )
    (gis / "map_frame.json").write_text(json.dumps({
        "crs": "EPSG:32610", "origin_easting_m": east, "origin_northing_m": north,
    }))
    (farm / "farm.yaml").write_text(yaml.safe_dump({
        "scene": {"origin": {"alt0": altitude}},
    }))

    lat, lon, alt, heading = farm_map_origin(farm)
    assert (lat, lon, alt) == pytest.approx((latitude, longitude, altitude), abs=1e-9)
    assert heading == pytest.approx(-0.9277, abs=0.001)

    config_path = write_mapviz_config(farm, PACKAGE_ROOT / "config" / "tractor.mviz")
    assert config_path == gis / "mapviz.generated.mviz"
    config = yaml.safe_load(config_path.read_text())
    assert config["fixed_frame"] == "map"
    assert config["target_frame"] == "base_link"
    tile_config = config["displays"][0]["config"]["custom_sources"][0]
    assert tile_config["base_url"] == (gis / "tiles").as_uri() + "/{level}/{x}/{y}.png"
    assert tile_config["max_zoom"] == 21
    assert [(display["config"]["topic"], display["config"]["draw_style"])
            for display in config["displays"][1:]] == [
                ("/ground_truth/odom", "arrows"), ("/gps/fix", "points")
            ]
