"""Reload a GeoPackage with GDAL and verify saved farm semantics.

This module runs under /usr/bin/python3 because system GDAL has a different
ABI from the Isaac Sim conda Python.
"""

from __future__ import annotations

import json
import sys
from collections import Counter
from pathlib import Path

from osgeo import ogr


def verify(gis_dir: Path) -> None:
    expected = json.loads((gis_dir / "semantic_reference.json").read_text())
    ds = ogr.Open(str(gis_dir / "farm.gpkg"))
    if ds is None:
        raise AssertionError("GeoPackage cannot be opened")
    required = {"fields", "row_centerlines", "roads", "road_centerlines", "trees",
                "irrigation_channels", "channel_centerlines", "crossings"}
    actual = {ds.GetLayerByIndex(i).GetName() for i in range(ds.GetLayerCount())}
    assert actual == required, (actual, required)
    field_geometries = {f.GetField("id"): f.GetGeometryRef().Clone()
                        for f in ds.GetLayerByName("fields")}
    tree_counts = Counter()
    row_tree_counts = Counter()
    trees_by_row = {}
    for tree in ds.GetLayerByName("trees"):
        matches = [pid for pid, geom in field_geometries.items()
                   if geom.Intersects(tree.GetGeometryRef())]
        assert len(matches) == 1, (tree.GetField("id"), matches)
        assert tree.GetField("field_id") == matches[0]
        row_id = tree.GetField("row_id")
        assert row_id
        trees_by_row.setdefault(row_id, []).append(tree.GetGeometryRef().Clone())
        row_tree_counts[row_id] += 1
        tree_counts[matches[0]] += 1
    row_counts = Counter()
    for row in ds.GetLayerByName("row_centerlines"):
        pid = row.GetField("field_id")
        assert pid in field_geometries
        row_id = row.GetField("id")
        trees = trees_by_row.get(row_id, [])
        assert len(trees) == row.GetField("tree_count")
        line = row.GetGeometryRef()
        if len(trees) == 1:
            assert line is None
        else:
            assert field_geometries[pid].Intersects(line)
            assert line.GetPointCount() == 2
            endpoints = [line.GetPoint_2D(0), line.GetPoint_2D(1)]
            assert all(any(abs(point.GetX() - x) < 1e-6 and abs(point.GetY() - y) < 1e-6
                           for point in trees) for x, y in endpoints)
            assert all(line.Distance(point) < 1e-6 for point in trees)
        row_counts[pid] += 1
    assert {pid: tree_counts[pid] for pid in field_geometries} == expected["field_tree_counts"]
    assert {pid: row_counts[pid] for pid in field_geometries} == expected["field_row_counts"]
    assert dict(row_tree_counts) == expected["row_tree_counts"]
    road_counts = Counter()
    for road in ds.GetLayerByName("road_centerlines"):
        if road.GetField("road_class") == "crossing_spur":
            assert road.GetField("field_id") is None
            assert road.GetField("field_a") in field_geometries
            assert road.GetField("field_b") in field_geometries
            assert road.GetField("field_a") != road.GetField("field_b")
        else:
            pid = road.GetField("field_id")
            assert pid in field_geometries
            road_counts[pid] += 1
    assert {pid: road_counts[pid] for pid in field_geometries} == expected["field_road_counts"]
    for crossing in ds.GetLayerByName("crossings"):
        assert crossing.GetField("field_a") in field_geometries
        assert crossing.GetField("field_b") in field_geometries
    assert ds.GetLayerByName("roads").GetFeatureCount() == expected["road_edges"]
    assert ds.GetLayerByName("road_centerlines").GetFeatureCount() == expected["road_edges"]
    assert ds.GetLayerByName("irrigation_channels").GetFeatureCount() == expected["channel_edges"]
    assert ds.GetLayerByName("channel_centerlines").GetFeatureCount() == expected["channel_edges"]
    assert ds.GetLayerByName("crossings").GetFeatureCount() == expected["crossings"]
    for i in range(ds.GetLayerCount()):
        layer = ds.GetLayerByIndex(i)
        assert layer.GetSpatialRef().GetAuthorityCode(None) == "32610"
    from qgis.core import QgsApplication, QgsProject
    import os
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    app = QgsApplication([], False)
    app.initQgis()
    project = QgsProject.instance()
    assert project.read(str(gis_dir / "farm.qgz"))
    qgis_layers = {layer.name(): layer for layer in project.mapLayers().values()}
    assert all(layer.isValid() for layer in qgis_layers.values())
    assert qgis_layers["Fields"].labelsEnabled()
    assert qgis_layers["Tree rows"].labelsEnabled()
    assert len(project.relationManager().relations()) >= 7
    print(json.dumps({"field_tree_counts": dict(tree_counts), "field_row_counts": dict(row_counts)}))


if __name__ == "__main__":
    verify(Path(sys.argv[1]))
