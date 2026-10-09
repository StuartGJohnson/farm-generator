"""System Python helper: GDAL and QGIS are installed with Ubuntu Python.

Called by gis.export from the Isaac Sim conda environment, which uses a
different Python ABI and cannot import the system GDAL extension module.
"""

from __future__ import annotations

import json
import math
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
from PIL import Image
from osgeo import gdal, ogr, osr


LAYER_SPECS = {
    "fields": (ogr.wkbPolygon, ["id", "name", "label", "kind", "crop_type"]),
    "row_centerlines": (ogr.wkbLineString, ["id", "name", "label", "field_id", "row_number", "tree_count"]),
    "roads": (ogr.wkbPolygon, ["id", "name", "node_a", "node_b", "road_class", "width_m", "surface", "field_id", "field_a", "field_b"]),
    "road_centerlines": (ogr.wkbLineString, ["id", "name", "node_a", "node_b", "road_class", "width_m", "surface", "field_id", "field_a", "field_b"]),
    "trees": (ogr.wkbPoint, ["id", "name", "field_id", "row_id", "species", "canopy_radius_m"]),
    "irrigation_channels": (ogr.wkbPolygon, ["id", "name", "node_a", "node_b", "top_width_m", "depth_m"]),
    "channel_centerlines": (ogr.wkbLineString, ["id", "name", "node_a", "node_b", "top_width_m", "depth_m"]),
    "crossings": (ogr.wkbPolygon, ["id", "name", "road_edge_id", "channel_edge_id", "field_a", "field_b", "crossing_type", "span_width_m"]),
}


def _utm_origin(origin):
    src = osr.SpatialReference()
    src.ImportFromEPSG(4326)
    src.SetAxisMappingStrategy(osr.OAMS_TRADITIONAL_GIS_ORDER)
    dst = osr.SpatialReference()
    dst.ImportFromEPSG(32610)
    dst.SetAxisMappingStrategy(osr.OAMS_TRADITIONAL_GIS_ORDER)
    x, y, _ = osr.CoordinateTransformation(src, dst).TransformPoint(origin["lon"], origin["lat"])
    return x, y, dst


def _translate(geometry, east, north):
    if geometry.GetGeometryCount():
        for i in range(geometry.GetGeometryCount()):
            _translate(geometry.GetGeometryRef(i), east, north)
    else:
        for i in range(geometry.GetPointCount()):
            x, y, _ = geometry.GetPoint(i)
            geometry.SetPoint_2D(i, x + east, y + north)


def _write_gpkg(path, layers, east, north, srs):
    path.unlink(missing_ok=True)
    ds = ogr.GetDriverByName("GPKG").CreateDataSource(str(path))
    if ds is None:
        raise RuntimeError("Could not create GeoPackage")
    for name, (geometry_type, fields) in LAYER_SPECS.items():
        layer = ds.CreateLayer(name, srs, geometry_type)
        if layer is None:
            raise RuntimeError(f"Could not create {name} layer")
        records = layers[name]
        for field in fields:
            sample = next((r["properties"][field] for r in records if r["properties"].get(field) is not None), "")
            field_type = ogr.OFTInteger if isinstance(sample, int) else ogr.OFTReal if isinstance(sample, float) else ogr.OFTString
            layer.CreateField(ogr.FieldDefn(field, field_type))
        for record in records:
            feature = ogr.Feature(layer.GetLayerDefn())
            for field in fields:
                value = record["properties"].get(field)
                if value is not None:
                    feature.SetField(field, value)
            if record["wkt"] is not None:
                geometry = ogr.CreateGeometryFromWkt(record["wkt"])
                _translate(geometry, east, north)
                feature.SetGeometry(geometry)
            if layer.CreateFeature(feature) != 0:
                raise RuntimeError(f"Could not write {name} feature")
        layer = None
    ds = None


def _write_orthophoto(path, rgb_path, bounds, gsd, east, north, srs):
    width = math.ceil((bounds[2] - bounds[0]) / gsd)
    height = math.ceil((bounds[3] - bounds[1]) / gsd)
    rgb = np.asarray(Image.open(rgb_path).convert("RGB"))
    if rgb.shape[:2] != (height, width):
        raise ValueError(f"Isaac image is {rgb.shape[:2]}, expected {(height, width)}")
    driver = gdal.GetDriverByName("GTiff")
    ds = driver.Create(str(path), width, height, 3, gdal.GDT_Byte,
                       options=["TILED=YES", "COMPRESS=DEFLATE", "PREDICTOR=2"])
    ds.SetProjection(srs.ExportToWkt())
    ds.SetGeoTransform((east + bounds[0], (bounds[2] - bounds[0]) / width, 0,
                        north + bounds[3], 0, -(bounds[3] - bounds[1]) / height))
    for band in range(3):
        ds.GetRasterBand(band + 1).WriteArray(rgb[:, :, band])
        ds.GetRasterBand(band + 1).SetColorInterpretation((gdal.GCI_RedBand,
             gdal.GCI_GreenBand, gdal.GCI_BlueBand)[band])
    ds.FlushCache()
    ds = None


def _write_qgis(path, gpkg, tif, extent):
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from qgis.core import (Qgis, QgsApplication, QgsFillSymbol, QgsLineSymbol,
                           QgsMarkerSymbol, QgsPalLayerSettings, QgsProject,
                           QgsRasterLayer, QgsRectangle, QgsReferencedRectangle,
                           QgsRelation, QgsTextBufferSettings, QgsTextFormat,
                           QgsVectorLayer, QgsVectorLayerSimpleLabeling)
    from qgis.PyQt.QtGui import QColor, QFont
    app = QgsApplication([], False)
    app.initQgis()
    project = QgsProject.instance()
    project.setFileName(str(path))
    project.setCrs(QgsVectorLayer(f"{gpkg}|layername=fields", "Fields", "ogr").crs())
    project.setFilePathStorage(Qgis.FilePathType.Relative)
    if tif is not None and tif.exists():
        raster = QgsRasterLayer(str(tif), "Orthophoto")
        if not raster.isValid():
            raise RuntimeError("QGIS could not read orthophoto")
        project.addMapLayer(raster)
    styles = {
        "fields": ("Fields", "fill", "255,215,80,0", "115,90,20,255"),
        "row_centerlines": ("Tree rows", "line", "240,220,65,255", None),
        "roads": ("Road surfaces", "fill", "160,120,80,55", "90,65,45,170"),
        "road_centerlines": ("Road graph", "line", "110,65,35,255", None),
        "trees": ("Trees", "marker", "20,115,35,255", None),
        "irrigation_channels": ("Irrigation channels", "fill", "35,135,220,115", "20,80,165,200"),
        "channel_centerlines": ("Channel graph", "line", "15,75,180,255", None),
        "crossings": ("Crossings", "fill", "235,55,45,170", "135,20,20,255"),
    }
    # Add in draw order: channels and roads under row lines and trees.
    vector_layers = {}
    for key in styles:
        label, kind, color, outline = styles[key]
        layer = QgsVectorLayer(f"{gpkg}|layername={key}", label, "ogr")
        if not layer.isValid():
            raise RuntimeError(f"QGIS could not read {key}")
        if kind == "fill":
            symbol = QgsFillSymbol.createSimple({"color": color, "outline_color": outline, "outline_width": "0.3"})
        elif kind == "line":
            symbol = QgsLineSymbol.createSimple({"line_color": color, "line_width": "0.35"})
        else:
            symbol = QgsMarkerSymbol.createSimple({"name": "circle", "color": color, "size": "1.5"})
        layer.renderer().setSymbol(symbol)
        if key in ("fields", "row_centerlines"):
            settings = QgsPalLayerSettings()
            settings.fieldName = "label"
            settings.isExpression = False
            if key == "row_centerlines":
                settings.placement = QgsPalLayerSettings.Line
            text = QgsTextFormat()
            text.setFont(QFont("Noto Sans", 9 if key == "fields" else 8, QFont.Bold))
            text.setSize(9 if key == "fields" else 8)
            text.setColor(QColor("#27221c" if key == "fields" else "#fff5c8"))
            buffer = QgsTextBufferSettings()
            buffer.setEnabled(True)
            buffer.setSize(0.9)
            buffer.setColor(QColor("#fff7de" if key == "fields" else "#29251d"))
            text.setBuffer(buffer)
            settings.setFormat(text)
            layer.setLabeling(QgsVectorLayerSimpleLabeling(settings))
            layer.setLabelsEnabled(True)
        project.addMapLayer(layer)
        vector_layers[key] = layer
    # Foreign-key relations expose fields -> roads/rows and rows -> trees in
    # QGIS forms and the "Select Related Features" tools without duplicating
    # feature records into a layer per field or row.
    relation_specs = [
        ("field_rows", "Rows in field", "row_centerlines", "field_id", "fields"),
        ("field_trees", "Trees in field", "trees", "field_id", "fields"),
        ("row_trees", "Trees in row", "trees", "row_id", "row_centerlines"),
        ("field_roads", "Road surfaces in field", "roads", "field_id", "fields"),
        ("field_road_graph", "Road edges in field", "road_centerlines", "field_id", "fields"),
        ("crossing_field_a", "Crossings from field", "crossings", "field_a", "fields"),
        ("crossing_field_b", "Crossings to field", "crossings", "field_b", "fields"),
    ]
    for identifier, name, child, foreign_key, parent in relation_specs:
        relation = QgsRelation()
        relation.setId(identifier)
        relation.setName(name)
        relation.setReferencingLayer(vector_layers[child].id())
        relation.setReferencedLayer(vector_layers[parent].id())
        relation.addFieldPair(foreign_key, "id")
        if not relation.isValid():
            raise RuntimeError(f"Invalid QGIS relation {identifier}")
        project.relationManager().addRelation(relation)
    project.viewSettings().setDefaultViewExtent(
        QgsReferencedRectangle(QgsRectangle(*extent), project.crs()))
    if not project.write():
        raise RuntimeError("Could not write QGIS project")


def main(input_path, output_dir):
    data = json.loads(Path(input_path).read_text())
    output_dir = Path(output_dir)
    east, north, srs = _utm_origin(data["origin"])
    gpkg = output_dir / "farm.gpkg"
    _write_gpkg(gpkg, data["layers"], east, north, srs)
    (output_dir / "map_frame.json").write_text(json.dumps({
        "crs": "EPSG:32610", "origin_easting_m": east,
        "origin_northing_m": north,
        "ros2_map_x": "easting - origin_easting_m",
        "ros2_map_y": "northing - origin_northing_m",
    }, indent=2) + "\n")
    tif = output_dir / "orthophoto.tif"
    if data["rgb_path"]:
        _write_orthophoto(tif, data["rgb_path"], data["bounds"], data["gsd_m"], east, north, srs)
    if tif.exists() and data["tiles"]:
        subprocess.run(["/usr/bin/gdal2tiles.py", "--xyz", "--zoom=16-18", "--processes=1",
                        "--webviewer=none", str(tif), str(output_dir / "tiles")], check=True)
    bounds = data["bounds"]
    _write_qgis(output_dir / "farm.qgz", gpkg, tif if data["rgb_path"] else None,
                (east + bounds[0], north + bounds[1], east + bounds[2], north + bounds[3]))
    print(f"EPSG:32610 origin=({east:.3f}, {north:.3f}); {gpkg}")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
