"""Build a Mapviz view for a generated farm's georeferenced XYZ tiles."""

from __future__ import annotations

import json
from pathlib import Path

from pyproj import Geod, Transformer
import yaml


def farm_map_origin(farm_dir: str | Path) -> tuple[float, float, float, float]:
    """Return latitude, longitude, altitude and Mapviz's ENU X-axis heading."""
    farm_dir = Path(farm_dir)
    frame = json.loads((farm_dir / "gis" / "map_frame.json").read_text())
    east = float(frame["origin_easting_m"])
    north = float(frame["origin_northing_m"])
    to_wgs84 = Transformer.from_crs(frame["crs"], "EPSG:4326", always_xy=True)
    longitude, latitude = to_wgs84.transform(east, north)
    east_longitude, east_latitude = to_wgs84.transform(east + 100.0, north)
    azimuth, _, _ = Geod(ellps="WGS84").inv(
        longitude, latitude, east_longitude, east_latitude
    )
    # Mapviz's local XY axes are true ENU. The farm's axes are projected grid
    # east/north, so account for meridian convergence at the map origin.
    heading = 90.0 - azimuth
    farm = yaml.safe_load((farm_dir / "farm.yaml").read_text())
    altitude = float(farm["scene"]["origin"]["alt0"])
    return latitude, longitude, altitude, heading


def write_mapviz_config(
    farm_dir: str | Path, template_path: str | Path, output_path: str | Path | None = None
) -> Path:
    """Write a farm-specific .mviz file beside its GIS assets."""
    farm_dir = Path(farm_dir).expanduser().resolve()
    tiles = farm_dir / "gis" / "tiles"
    frame = farm_dir / "gis" / "map_frame.json"
    if not frame.is_file():
        raise FileNotFoundError(f"Missing farm map frame: {frame}")
    levels = (
        sorted(int(p.name) for p in tiles.iterdir() if p.is_dir() and p.name.isdigit())
        if tiles.is_dir() else []
    )
    if not levels or not any((tiles / str(level)).rglob("*.png") for level in levels):
        raise FileNotFoundError(f"Missing XYZ PNG tiles under {tiles}")
    config = yaml.safe_load(Path(template_path).read_text())
    tile_display = next(d for d in config["displays"] if d["type"] == "mapviz_plugins/tile_map")
    source = tile_display["config"]["custom_sources"][0]
    source["base_url"] = tiles.as_uri() + "/{level}/{x}/{y}.png"
    source["max_zoom"] = levels[-1]
    output = Path(output_path) if output_path is not None else farm_dir / "gis" / "mapviz.generated.mviz"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(yaml.safe_dump(config, sort_keys=False))
    return output
