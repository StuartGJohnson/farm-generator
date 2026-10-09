"""Load route-planning input exclusively from the generated GIS bundle."""

from __future__ import annotations

import json
import subprocess
from pathlib import Path

from shapely import affinity
from shapely.geometry import shape

from .model import FarmMap, Road, Row, Tree


def load_farm_map(path: str | Path) -> FarmMap:
    path = Path(path)
    gis_dir = path / "gis" if (path / "gis").is_dir() else path
    helper = Path(__file__).with_name("_read_gpkg.py")
    result = subprocess.run(["/usr/bin/python3", str(helper), str(gis_dir)],
                            check=True, text=True, capture_output=True)
    doc = json.loads(result.stdout)
    frame = doc["frame"]
    east, north = frame["origin_easting_m"], frame["origin_northing_m"]

    def local(geometry):
        return affinity.translate(shape(geometry), xoff=-east, yoff=-north) if geometry else None

    layers = doc["layers"]
    fields = {f["properties"]["id"]: local(f["geometry"])
              for f in layers["fields"]}
    field_names = {f["properties"]["id"]: f["properties"]["name"]
                   for f in layers["fields"]}
    roads = {}
    for feature in layers["road_centerlines"]:
        p = feature["properties"]
        roads[p["id"]] = Road(p["id"], local(feature["geometry"]), p["node_a"], p["node_b"],
                              p["road_class"], p["field_id"], p["field_a"], p["field_b"])
    rows = {}
    for feature in layers["row_centerlines"]:
        p = feature["properties"]
        rows[p["id"]] = Row(p["id"], p["field_id"], local(feature["geometry"]), p["tree_count"])
    trees = {}
    for feature in layers["trees"]:
        p = feature["properties"]
        point = local(feature["geometry"])
        trees[p["id"]] = Tree(p["id"], p["field_id"], p["row_id"], point.x, point.y)
    channels = {feature["properties"]["id"]: local(feature["geometry"])
                for feature in layers["irrigation_channels"]}
    raster = gis_dir / "orthophoto.tif"
    return FarmMap(fields, field_names, roads, rows, trees, east, north,
                   str(raster) if raster.exists() else None,
                   tuple(doc["raster_extent"]) if doc["raster_extent"] else None,
                   channels)
