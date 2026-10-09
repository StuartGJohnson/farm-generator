"""Prepare semantic farm geometry and hand it to the system GDAL/QGIS runtime.

The scene is in local ENU metres. For this small synthetic Delta site the map
frame is translated to UTM 10N at the farm origin; its axis directions are
East and North. The exact offset is recorded in the GeoPackage metadata.
"""

from __future__ import annotations

import json
import subprocess
from collections import Counter
from pathlib import Path

from shapely.geometry import LineString, Point, Polygon


def _row_id(field_id: str, row_number: int) -> str:
    return f"{field_id}_row_{row_number:03d}"


def _row_assignments(scene):
    """Return per-tree row membership, including older scenes without row tags."""
    rows = {zone.tags.get("parcel_id"): [LineString(coords) for coords in zone.row_centerlines]
            for zone in scene.weed_zones.values()}
    result = {}
    for tree in scene.trees.values():
        field_id = tree.tags.get("parcel_id")
        field_rows = rows.get(field_id, [])
        if not field_rows:
            raise ValueError(f"Tree {tree.id} has no cultivated row in {field_id}")
        tagged = tree.tags.get("row_number")
        number = int(tagged) if tagged is not None else min(
            range(1, len(field_rows) + 1),
            key=lambda n: field_rows[n - 1].distance(Point(tree.position)))
        if not 1 <= number <= len(field_rows):
            raise ValueError(f"Tree {tree.id} has invalid row number {number}")
        result[tree.id] = (field_id, number)
    return result


def semantic_reference(scene) -> dict:
    """Facts computed before GIS serialization for independent reload checks."""
    tree_counts = Counter(t.tags.get("parcel_id") for t in scene.trees.values())
    row_counts = Counter(z.tags.get("parcel_id") for z in scene.weed_zones.values()
                         for _ in z.row_centerlines)
    assignments = _row_assignments(scene)
    row_tree_counts = Counter(_row_id(*assignments[t.id]) for t in scene.trees.values())
    road_counts = Counter(e.tags.get("parcel_id") for e in scene.roads.edges.values()
                          if e.road_class.value != "crossing_spur")
    return {
        "field_tree_counts": {pid: tree_counts[pid] for pid in scene.parcels},
        "field_row_counts": {pid: row_counts[pid] for pid in scene.parcels},
        "row_tree_counts": dict(sorted(row_tree_counts.items())),
        "field_road_counts": {pid: road_counts[pid] for pid in scene.parcels},
        "road_edges": len(scene.roads.edges),
        "channel_edges": len(scene.hydrology.edges),
        "crossings": len(scene.crossings),
    }


def _feature(geometry, **properties):
    return {"wkt": geometry.wkt if geometry is not None else None, "properties": properties}


def _features(scene):
    layers = {name: [] for name in (
        "fields", "row_centerlines", "roads", "road_centerlines", "trees",
        "irrigation_channels", "channel_centerlines", "crossings",
    )}
    field_names = {pid: f"Field {i:03d}" for i, pid in enumerate(sorted(scene.parcels), 1)}
    assignments = _row_assignments(scene)
    trees_by_row = {}
    for tree in scene.trees.values():
        trees_by_row.setdefault(assignments[tree.id], []).append(tree)
    for pid, parcel in sorted(scene.parcels.items()):
        layers["fields"].append(_feature(Polygon(parcel.polygon), id=pid,
                                         name=field_names[pid], label=field_names[pid],
                                         kind=str(parcel.parcel_type.value),
                                         crop_type=getattr(parcel, "crop_type", "")))
    for zone in sorted(scene.weed_zones.values(), key=lambda z: z.id):
        pid = zone.tags.get("parcel_id", "")
        for field_row_number, coords in enumerate(zone.row_centerlines, 1):
            row_trees = trees_by_row.get((pid, field_row_number), [])
            if not row_trees:
                raise ValueError(f"Cultivated row {pid}/{field_row_number} has no trees")
            axis = LineString(coords)
            ordered = sorted(row_trees, key=lambda t: axis.project(Point(t.position)))
            # A lone tree has no navigable row segment. Keep its row record
            # and relation, with NULL geometry, instead of an invalid
            # zero-length LineString.
            line = (LineString([ordered[0].position, ordered[-1].position])
                    if len(ordered) > 1 else None)
            layers["row_centerlines"].append(_feature(
                line, id=_row_id(pid, field_row_number),
                name=f"{field_names.get(pid, pid)} Row {field_row_number:03d}",
                label=f"{field_names[pid].replace('Field ', 'F')}-R{field_row_number:03d}",
                field_id=pid,
                row_number=field_row_number, tree_count=len(row_trees)))
    for edge in sorted(scene.roads.edges.values(), key=lambda e: e.id):
        line = LineString(edge.polyline)
        is_crossing = edge.road_class.value == "crossing_spur"
        field_id = None if is_crossing else edge.tags.get("parcel_id")
        field_a = scene.roads.nodes[edge.node_a].tags.get("parcel_id") if is_crossing else None
        field_b = scene.roads.nodes[edge.node_b].tags.get("parcel_id") if is_crossing else None
        props = dict(id=edge.id, name=f"Road {edge.id}", node_a=edge.node_a,
                     node_b=edge.node_b, road_class=edge.road_class.value,
                     width_m=float(edge.width), surface=edge.surface.value,
                     field_id=field_id, field_a=field_a, field_b=field_b)
        layers["road_centerlines"].append(_feature(line, **props))
        layers["roads"].append(_feature(line.buffer(edge.width / 2, cap_style="flat", join_style="mitre"), **props))
    for tree in sorted(scene.trees.values(), key=lambda t: t.id):
        pid, row_number = assignments[tree.id]
        layers["trees"].append(_feature(Point(tree.position), id=tree.id,
                                        name=f"Tree {tree.id}", field_id=pid,
                                        row_id=_row_id(pid, row_number),
                                        species=tree.species, canopy_radius_m=float(tree.canopy_radius)))
    for edge in sorted(scene.hydrology.edges.values(), key=lambda e: e.id):
        line = LineString(edge.polyline)
        props = dict(id=edge.id, name=f"Channel {edge.id}", node_a=edge.node_a,
                     node_b=edge.node_b, top_width_m=float(edge.top_width),
                     depth_m=float(edge.depth))
        layers["channel_centerlines"].append(_feature(line, **props))
        layers["irrigation_channels"].append(_feature(
            line.buffer(edge.top_width / 2, cap_style="flat", join_style="mitre"), **props))
    road_by_id = scene.roads.edges
    for crossing in sorted(scene.crossings.values(), key=lambda c: c.id):
        road = road_by_id[crossing.road_edge_id]
        footprint = LineString(road.polyline).buffer(road.width / 2, cap_style="flat", join_style="mitre")
        if footprint.is_empty:
            footprint = Point(crossing.location).buffer(road.width / 2)
        layers["crossings"].append(_feature(
            footprint, id=crossing.id, name=f"Crossing {crossing.id}",
            road_edge_id=crossing.road_edge_id,
            channel_edge_id=crossing.hydrology_edge_id,
            field_a=crossing.refs[0] if len(crossing.refs) > 0 else None,
            field_b=crossing.refs[1] if len(crossing.refs) > 1 else None,
            crossing_type=crossing.crossing_type.value,
            span_width_m=float(crossing.span_width)))
    return layers


def export_gis(scene, bounds, farm_dir: Path, *, rgb_path: Path | None = None,
               gsd_m: float = 0.05, tiles: bool = True) -> dict:
    """Write GeoPackage and optional rendered orthophoto, QGIS project, and tiles.

    rgb_path is an Isaac Sim RGB image whose top row is geographic north.
    """
    if gsd_m <= 0:
        raise ValueError("gsd_m must be positive")
    farm_dir = Path(farm_dir).resolve()
    gis_dir = farm_dir / "gis"
    gis_dir.mkdir(parents=True, exist_ok=True)
    reference = semantic_reference(scene)
    (gis_dir / "semantic_reference.json").write_text(json.dumps(reference, indent=2) + "\n")
    payload = {
        "origin": {"lat": scene.origin.lat0, "lon": scene.origin.lon0},
        "bounds": bounds,
        "gsd_m": gsd_m,
        "layers": _features(scene),
        "rgb_path": str(Path(rgb_path).resolve()) if rgb_path else None,
        "tiles": tiles,
    }
    input_path = gis_dir / "_export_input.json"
    input_path.write_text(json.dumps(payload))
    try:
        result = subprocess.run(["/usr/bin/python3", str(Path(__file__).with_name("gdal_export.py")),
                                 str(input_path), str(gis_dir)], check=True, text=True,
                                capture_output=True)
    except subprocess.CalledProcessError as exc:
        raise RuntimeError(f"GDAL/QGIS export failed:\n{exc.stdout}\n{exc.stderr}") from exc
    finally:
        input_path.unlink(missing_ok=True)
    return {"reference": reference, "gdal_output": result.stdout.strip()}
