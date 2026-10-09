"""GDAL bridge run with /usr/bin/python3; writes local-map GeoJSON to stdout."""

import json
import sys
from pathlib import Path

from osgeo import gdal, ogr


LAYERS = ("fields", "row_centerlines", "road_centerlines", "trees",
          "irrigation_channels", "crossings")


def main(gis_dir):
    gis_dir = Path(gis_dir)
    frame = json.loads((gis_dir / "map_frame.json").read_text())
    if frame["crs"] != "EPSG:32610":
        raise ValueError("Route planning requires EPSG:32610 farm GIS")
    ds = ogr.Open(str(gis_dir / "farm.gpkg"))
    if ds is None:
        raise ValueError("Cannot open farm GeoPackage")
    layers = {}
    for name in LAYERS:
        layer = ds.GetLayerByName(name)
        if layer is None or layer.GetSpatialRef().GetAuthorityCode(None) != "32610":
            raise ValueError(f"Missing or incorrectly projected GIS layer {name}")
        layers[name] = []
        for feature in layer:
            geometry = feature.GetGeometryRef()
            properties = {name: feature.GetField(name) for name in feature.keys()}
            layers[name].append({"properties": properties,
                                 "geometry": json.loads(geometry.ExportToJson()) if geometry else None})
    raster_extent = None
    raster = gdal.Open(str(gis_dir / "orthophoto.tif"))
    if raster is not None:
        transform = raster.GetGeoTransform()
        raster_extent = [transform[0] - frame["origin_easting_m"],
                         transform[3] + raster.RasterYSize * transform[5] - frame["origin_northing_m"],
                         transform[0] + raster.RasterXSize * transform[1] - frame["origin_easting_m"],
                         transform[3] - frame["origin_northing_m"]]
    print(json.dumps({"frame": frame, "layers": layers,
                      "raster_extent": raster_extent}, separators=(",", ":")))


if __name__ == "__main__":
    main(sys.argv[1])
