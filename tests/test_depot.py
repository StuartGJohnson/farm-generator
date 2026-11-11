"""Farm generation, GIS export, and depot geometry checks."""

from dataclasses import replace

import pytest
import numpy as np
from shapely.geometry import LineString, Point, Polygon

from generation.orchestrator import FarmGenerationConfig, generate_farm, load_farm, save_farm
from generation.crops import _depot_exclusion_polygon
from export.usd import build_ground_mesh, write_ground_mesh_usda
from gis.export import export_gis
from route_planning.gis import load_farm_map


def depot_config():
    return FarmGenerationConfig(bounds=(0, 0, 120, 120), seed=4, max_faces=3,
        row_spacing=5, tree_spacing=4, headland_width=6, sideland_width=5,
        standoff=3, depot_enabled=True, depot_cross_row_size=4)


def test_outlands_do_not_move_existing_depot_or_inner_trees():
    config = FarmGenerationConfig(
        bounds=(0, 0, 120, 120), seed=1, max_faces=4,
        row_spacing=5, tree_spacing=4, headland_width=7.5,
        sideland_width=6.5, standoff=3.5,
        optimize_tree_layout=True, depot_enabled=True, depot_cross_row_size=4,
    )
    original = generate_farm(config)
    expanded = generate_farm(replace(config, outlands_m=10))
    assert expanded.depot == original.depot
    assert expanded.roads == original.roads
    assert expanded.crossings == original.crossings
    for pid in original.parcels:
        assert expanded.parcels[pid].polygon == original.parcels[pid].polygon
        assert [t.position for t in expanded.trees.values() if t.tags.get("parcel_id") == pid] == [
            t.position for t in original.trees.values() if t.tags.get("parcel_id") == pid]


def test_depot_clearances_default_from_crop_spacing_and_only_trim_adjacent_trees():
    config = depot_config()
    assert config.depot_headland_m == pytest.approx(config.headland_width)
    assert config.depot_sideland_m == pytest.approx(config.row_spacing / 2)
    wider_road = FarmGenerationConfig(bounds=(0, 0, 120, 120), seed=4, road_width=5)
    assert wider_road.depot_headland_m == pytest.approx(wider_road.headland_width)
    assert wider_road.depot_sideland_m == pytest.approx(wider_road.row_spacing / 2)
    for name in ("depot_headland_m", "depot_sideland_m"):
        with pytest.raises(ValueError, match=name):
            FarmGenerationConfig(bounds=(0, 0, 120, 120), seed=4, **{name: -0.1})

    regular = generate_farm(config)
    widened = generate_farm(replace(config, depot_headland_m=12, depot_sideland_m=12))
    assert regular.depot == widened.depot
    region = Polygon(regular.depot.polygon)
    field_id = regular.depot.field_id
    regular_positions = {tree.position for tree in regular.trees.values()
                         if tree.tags.get("parcel_id") == field_id}
    wide_positions = {tree.position for tree in widened.trees.values()
                      if tree.tags.get("parcel_id") == field_id}
    removed = regular_positions - wide_positions
    assert removed
    wide_exclusion = _depot_exclusion_polygon(
        region, np.asarray(regular.depot.row_direction),
        np.asarray(regular.depot.cross_row_direction), 12, 12)
    assert removed <= {position for position in regular_positions
                       if wide_exclusion.covers(Point(position))}
    assert regular.roads == widened.roads
    assert regular.hydrology == widened.hydrology


def test_depot_cuts_crop_and_round_trips(tmp_path):
    config = depot_config()
    scene = generate_farm(config)
    depot = scene.depot
    assert depot is not None and depot.field_id in scene.parcels
    assert depot == generate_farm(config).depot
    assert len(depot.parking_spots) >= 2
    region = Polygon(depot.polygon)
    assert Polygon(scene.parcels[depot.field_id].polygon).covers(region)
    road_nodes = [node.position for node in scene.roads.nodes.values()
                  if node.tags.get("parcel_id") == depot.field_id]
    crop_base = Polygon(road_nodes).buffer(-config.road_width / 2, join_style="mitre")
    assert crop_base.buffer(1e-7).covers(region)
    assert region.boundary.intersection(crop_base.boundary.buffer(1e-6)).length > 1
    assert min(LineString(edge.polyline).distance(region) for edge in scene.roads.edges.values()
               if edge.tags.get("parcel_id") == depot.field_id) == pytest.approx(config.road_width / 2)
    parcel_corner = min(scene.parcels[depot.field_id].polygon,
                        key=lambda xy: Point(xy).distance(Point(depot.origin)))
    basis_inv = np.linalg.inv(np.column_stack([depot.row_direction,
                                               depot.cross_row_direction]))
    i0, j0 = basis_inv @ (np.asarray(depot.origin) - np.asarray(parcel_corner))
    assert (i0 + depot.along_row_length_m - config.headland_width) / config.tree_spacing == pytest.approx(
        round((i0 + depot.along_row_length_m - config.headland_width) / config.tree_spacing))
    assert (j0 + depot.cross_row_width_m - config.sideland_width) / config.row_spacing == pytest.approx(
        round((j0 + depot.cross_row_width_m - config.sideland_width) / config.row_spacing))
    assert all(not region.covers(Point(tree.position)) for tree in scene.trees.values())
    assert all(not region.intersects(Point(xy)) for zone in scene.weed_zones.values()
               for row in zone.row_centerlines for xy in row)
    assert all(region.intersection(LineString(row)).length < 1e-7
               for zone in scene.weed_zones.values() for row in zone.row_centerlines)
    assert [spot.number for spot in depot.parking_spots] == list(range(1, len(depot.parking_spots) + 1))
    row = np.asarray(depot.row_direction)
    column = np.asarray(depot.cross_row_direction)
    for spot in depot.parking_spots:
        assert spot.length_m == pytest.approx(8.0)
        assert spot.width_m == pytest.approx(4.0)
        center = np.asarray(spot.center)
        corners = [center + a * spot.width_m / 2 * row + b * spot.length_m / 2 * column
                   for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
        assert region.contains(Polygon(corners))
    path = tmp_path / "farm.json"
    save_farm(scene, config, str(path))
    restored, restored_config = load_farm(str(path))
    assert restored.depot == depot and restored_config == config
    plain = generate_farm(replace(config, depot_enabled=False))
    assert plain.depot is None
    assert scene.roads == plain.roads
    assert scene.hydrology == plain.hydrology
    assert scene.crossings == plain.crossings
    cultivated = {tuple(round(v, 5) for v in tree.position)
                  for tree in scene.trees.values() if tree.tags.get("parcel_id") == depot.field_id}
    previous = [tree for tree in plain.trees.values()
                if tree.tags.get("parcel_id") == depot.field_id]
    exclusion = _depot_exclusion_polygon(
        region, np.asarray(depot.row_direction), np.asarray(depot.cross_row_direction),
        config.depot_headland_m, config.depot_sideland_m)
    base_local = np.asarray([basis_inv @ (np.asarray(xy) - np.asarray(depot.origin))
                             for xy in depot.polygon])
    excluded_local = np.asarray([basis_inv @ (np.asarray(xy) - np.asarray(depot.origin))
                                 for xy in exclusion.exterior.coords[:-1]])
    assert excluded_local[:, 0].min() == pytest.approx(
        base_local[:, 0].min() - config.depot_headland_m)
    assert excluded_local[:, 0].max() == pytest.approx(
        base_local[:, 0].max() + config.depot_headland_m)
    assert excluded_local[:, 1].min() == pytest.approx(
        base_local[:, 1].min() - config.depot_sideland_m)
    assert excluded_local[:, 1].max() == pytest.approx(
        base_local[:, 1].max() + config.depot_sideland_m)
    depot_roadside_j = min((basis_inv @ (np.asarray(xy) - np.asarray(parcel_corner)))[1]
                           for xy in depot.polygon)
    def roadward(position):
        return (basis_inv @ (np.asarray(position) - np.asarray(parcel_corner)))[1] <= depot_roadside_j + 1e-6
    lost = [tree for tree in previous
            if tuple(round(v, 5) for v in tree.position) not in cultivated]
    assert cultivated == {tuple(round(v, 5) for v in tree.position)
                          for tree in previous if not exclusion.covers(Point(tree.position))}
    assert len(cultivated) >= len(previous) / 2
    assert lost and all(region.distance(Point(tree.position)) <= max(
        config.headland_width, config.sideland_width) + 1e-5 for tree in lost)
    roadward_original = [tree for tree in previous if roadward(tree.position)]
    roadward_kept = [tree for tree in roadward_original
                     if tuple(round(v, 5) for v in tree.position) in cultivated]
    roadward_lost = [tree for tree in roadward_original
                     if tuple(round(v, 5) for v in tree.position) not in cultivated]
    assert roadward_kept and roadward_lost
    assert len(roadward_kept) == 8 and len(roadward_lost) == 7
    assert all(region.distance(Point(tree.position)) > config.depot_sideland_m
               for tree in roadward_kept)
    original_zone = next(zone for zone in plain.weed_zones.values()
                         if zone.tags.get("parcel_id") == depot.field_id)
    unaffected_rows = [str(index) for index, row in enumerate(original_zone.row_centerlines, 1)
                       if LineString(row).distance(region) > config.depot_sideland_m + 1e-5]
    assert unaffected_rows
    # Even a row just outside the depot must keep its original trees.
    nearest = min(unaffected_rows,
                  key=lambda index: LineString(original_zone.row_centerlines[int(index) - 1]).distance(region))
    assert any(tree.tags["row_number"] == nearest for tree in previous)
    assert all(tuple(round(v, 5) for v in tree.position) in cultivated
               for tree in previous if tree.tags["row_number"] in unaffected_rows)
    with pytest.raises(ValueError, match="No field can fit a depot"):
        generate_farm(replace(config, depot_along_row_size=100))
    for pid in scene.parcels:
        if pid != depot.field_id:
            assert [(t.id, t.position, t.species) for t in scene.trees.values()
                    if t.tags.get("parcel_id") == pid] == [
                    (t.id, t.position, t.species) for t in plain.trees.values()
                    if t.tags.get("parcel_id") == pid]


def test_generated_depot_gis_export(tmp_path):
    config = FarmGenerationConfig(bounds=(0, 0, 120, 120), seed=1, max_faces=4,
        standoff=3.5, headland_width=7.5, sideland_width=6.5,
        tree_spacing=4, row_spacing=5, optimize_tree_layout=True,
        hydrology_add_water=True, depot_enabled=True, depot_cross_row_size=4)
    scene = generate_farm(config)
    export_gis(scene, config.bounds, tmp_path, tiles=False)
    farm = load_farm_map(tmp_path)
    assert farm.depot is not None
    assert len(farm.parking_spots) == len(scene.depot.parking_spots)
    assert farm.fields[scene.depot.field_id].intersection(farm.depot).area < 1e-6


def test_depot_ground_uses_road_surface_and_optional_lines(tmp_path):
    config = FarmGenerationConfig(bounds=(0, 0, 55, 55), seed=1, max_faces=1,
        row_spacing=5, tree_spacing=4, headland_width=6, sideland_width=5,
        standoff=3, depot_enabled=True, depot_cross_row_size=3)
    scene = generate_farm(config)
    mesh = build_ground_mesh(scene, config.bounds)
    center = Point(scene.depot.parking_spots[0].center)
    regions = [kind for triangle, kind in zip(mesh.triangles, mesh.face_surface_regions)
               if Polygon([mesh.points[i][:2] for i in triangle]).covers(center)]
    assert "road" in regions
    path = tmp_path / "with_lines.usda"
    write_ground_mesh_usda(mesh, str(path), scene=scene)
    assert 'def Mesh "ParkingLines"' in path.read_text()
    from pxr import Usd, UsdGeom
    stage = Usd.Stage.Open(str(path))
    line_mesh = UsdGeom.Mesh.Get(stage, "/World/ParkingLines")
    points = np.asarray(line_mesh.GetPointsAttr().Get())
    assert len(points) == 4 * len(scene.depot.parking_spots)
    basis_inv = np.linalg.inv(np.column_stack([scene.depot.row_direction,
                                               scene.depot.cross_row_direction]))
    for index in range(0, len(points), 4):
        back = (points[index, :2] + points[index + 1, :2]) / 2
        tip = (points[index + 2, :2] + points[index + 3, :2]) / 2
        assert np.linalg.norm(back - tip) == pytest.approx(scene.depot.parking_spots[index // 4].length_m, abs=1e-4)
        assert (basis_inv @ (back - np.asarray(scene.depot.origin)))[1] == pytest.approx(
            scene.depot.cross_row_width_m, abs=1e-4)
    no_lines = replace(scene, depot=replace(scene.depot, draw_parking_spaces=False))
    path = tmp_path / "without_lines.usda"
    write_ground_mesh_usda(mesh, str(path), scene=no_lines)
    assert 'def Mesh "ParkingLines"' not in path.read_text()
