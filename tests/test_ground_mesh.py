import math

from shapely.geometry import LineString, Point

from export.usd import (
    ChannelUndulationConfig,
    build_channel_faces,
    build_ground_mesh,
    build_hydrology_pslg,
    build_road_surface_patches,
    build_undulated_hydrology_edges,
    write_ground_mesh_usda,
)
from generation.orchestrator import FarmGenerationConfig, generate_farm


def test_ground_mesh_covers_bounds_and_contains_channels(tmp_path):
    bounds = (0.0, 0.0, 50.0, 50.0)
    scene = generate_farm(FarmGenerationConfig(
        bounds=bounds,
        seed=1,
        max_faces=4,
        standoff=3.5,
        headland_width=7.5,
        sideland_width=6.5,
    ))
    pslg = build_hydrology_pslg(scene, bounds, max_segment_length=1.0)
    mesh = build_ground_mesh(scene, bounds, flat_resolution=1.0)

    assert pslg.vertices
    assert pslg.segments
    lengths = [math.dist(pslg.vertices[a], pslg.vertices[b]) for a, b in pslg.segments]
    assert max(lengths) <= 1.0 + 1e-9
    road_area = next(
        patch.polygon
        for patch in build_road_surface_patches(scene, bounds)
        if not patch.is_crossing
    )
    for a, b in pslg.segments:
        midpoint = LineString([pslg.vertices[a], pslg.vertices[b]]).interpolate(0.5, normalized=True)
        assert not road_area.contains(midpoint) or road_area.boundary.distance(midpoint) < 1e-6

    assert mesh.triangles
    assert min(p[0] for p in mesh.points) == bounds[0]
    assert max(p[0] for p in mesh.points) == bounds[2]
    assert min(p[1] for p in mesh.points) == bounds[1]
    assert max(p[1] for p in mesh.points) == bounds[3]
    assert min(p[2] for p in mesh.points) == -0.8
    assert max(p[2] for p in mesh.points) == 0.0
    assert all(len(set(t)) == 3 for t in mesh.triangles)
    triangle_edges = {
        tuple(sorted((tri[i], tri[(i + 1) % 3])))
        for tri in mesh.triangles
        for i in range(3)
    }
    # Every shoulder, channel-bottom, and domain constraint is an actual
    # triangle edge: the terrain cannot interpolate across a breakline.
    assert mesh.breakline_edges
    assert mesh.road_breakline_edges
    assert {tuple(sorted(edge)) for edge in mesh.breakline_edges} <= triangle_edges
    assert {tuple(sorted(edge)) for edge in mesh.road_breakline_edges} <= triangle_edges
    road_boundary_tolerance = road_area.boundary.buffer(2e-6)
    assert all(
        road_boundary_tolerance.covers(LineString([
            mesh.points[a][:2], mesh.points[b][:2]
        ]))
        for a, b in mesh.road_breakline_edges
    )
    cross_slope_edges = [
        edge for edge in mesh.breakline_edges
        if {round(mesh.points[i][2], 9) for i in edge} == {-0.8, 0.0}
    ]
    assert len(cross_slope_edges) >= sum(len(parcel.polygon) for parcel in scene.parcels.values())
    xy_area = sum(
        abs(
            (mesh.points[b][0] - mesh.points[a][0]) * (mesh.points[c][1] - mesh.points[a][1])
            - (mesh.points[c][0] - mesh.points[a][0]) * (mesh.points[b][1] - mesh.points[a][1])
        ) / 2
        for (a, b, c), kind in zip(mesh.triangles, mesh.face_classes)
        if kind != "crossing_wall"
    )
    assert abs(xy_area - 2500.0) < 1e-7
    assert scene.crossings
    assert "road" in mesh.face_classes
    assert "crossing_wall" in mesh.face_classes
    assert mesh.crossing_breakline_edges
    triangle_edges = {
        tuple(sorted((tri[i], tri[(i + 1) % 3])))
        for tri in mesh.triangles for i in range(3)
    }
    assert {
        tuple(sorted(edge)) for edge in mesh.crossing_breakline_edges
    } <= triangle_edges
    assert all(
        mesh.points[a][:2] != mesh.points[b][:2]
        and abs(mesh.points[a][2] - mesh.points[b][2]) > 1e-9
        for a, b in mesh.crossing_breakline_edges
    )
    assert all(
        all(abs(mesh.points[i][2]) < 1e-12 for i in triangle)
        for triangle, kind in zip(mesh.triangles, mesh.face_classes)
        if kind == "road"
    )
    road_mesh_area = sum(
        abs(
            (mesh.points[b][0] - mesh.points[a][0]) * (mesh.points[c][1] - mesh.points[a][1])
            - (mesh.points[c][0] - mesh.points[a][0]) * (mesh.points[b][1] - mesh.points[a][1])
        ) / 2
        for (a, b, c), kind in zip(mesh.triangles, mesh.face_classes)
        if kind == "road"
    )
    assert abs(road_mesh_area - road_area.area) < 1e-4

    output = tmp_path / "ground.usda"
    write_ground_mesh_usda(mesh, str(output))
    text = output.read_text()
    assert text.startswith("#usda 1.0")
    assert 'def Mesh "Ground"' in text
    assert 'normal3f[] normals' in text
    assert 'interpolation = "faceVarying"' in text


def test_channel_undulation_is_bounded_repeatable_and_non_mutating():
    bounds = (0.0, 0.0, 16.0, 16.0)
    scene = generate_farm(FarmGenerationConfig(bounds=bounds, seed=4, max_faces=4))
    original = {edge.id: list(edge.polyline) for edge in scene.hydrology.edges.values()}
    config = ChannelUndulationConfig(max_amplitude=0.10, min_wavelength=1.0)

    first = build_undulated_hydrology_edges(scene, config)
    second = build_undulated_hydrology_edges(scene, config)
    changed_seed = build_undulated_hydrology_edges(
        scene, ChannelUndulationConfig(max_amplitude=0.10, min_wavelength=1.0, seed_offset=1)
    )
    assert first == second
    assert any(first[key].polyline != changed_seed[key].polyline for key in first)
    assert original == {edge.id: list(edge.polyline) for edge in scene.hydrology.edges.values()}

    for key, perturbed in first.items():
        source_edge = next(
            edge for edge in scene.hydrology.edges.values()
            if tuple(sorted((edge.node_a, edge.node_b))) == key
        )
        source = LineString(source_edge.polyline)
        assert source.distance(Point(perturbed.polyline[0])) < 1e-12
        assert source.distance(Point(perturbed.polyline[-1])) < 1e-12
        assert max(source.distance(Point(point)) for point in perturbed.polyline) <= 0.10 + 1e-9

    # The perturbed PSLG still survives as mandatory final mesh edges.
    mesh = build_ground_mesh(scene, bounds, undulation=config)
    triangle_edges = {
        tuple(sorted((tri[i], tri[(i + 1) % 3])))
        for tri in mesh.triangles for i in range(3)
    }
    assert {tuple(sorted(edge)) for edge in mesh.breakline_edges} <= triangle_edges


def test_channel_faces_have_exact_shared_corner_ribs():
    bounds = (0.0, 0.0, 16.0, 16.0)
    scene = generate_farm(FarmGenerationConfig(bounds=bounds, seed=2, max_faces=4))
    faces = build_channel_faces(scene, ChannelUndulationConfig())
    assert len(faces) == sum(len(parcel.boundary_refs) for parcel in scene.parcels.values())

    by_parcel = {}
    for face in faces:
        by_parcel.setdefault(face.parcel_id, []).append(face)
        assert face.shoulder[0] != face.bottom[0]
        assert face.shoulder[-1] != face.bottom[-1]
        edge_index = int(face.id.rsplit("_", 1)[1])
        corner = scene.parcels[face.parcel_id].polygon[edge_index]
        upper = (face.shoulder[0][0] - corner[0], face.shoulder[0][1] - corner[1])
        lower = (face.bottom[0][0] - corner[0], face.bottom[0][1] - corner[1])
        assert abs(upper[0] * lower[1] - upper[1] * lower[0]) < 1e-10
        assert abs(math.hypot(*upper) / math.hypot(*lower) - 2.0) < 1e-10
    for parcel_faces in by_parcel.values():
        for i, face in enumerate(parcel_faces):
            following = parcel_faces[(i + 1) % len(parcel_faces)]
            assert math.dist(face.shoulder[-1], following.shoulder[0]) < 1e-9
            assert math.dist(face.bottom[-1], following.bottom[0]) < 1e-9


def test_seed_3_crossing_has_both_channel_slope_intersections():
    """Regression: a collinear CDT T-junction used to omit the upper wall."""
    bounds = (0.0, 0.0, 50.0, 50.0)
    scene = generate_farm(FarmGenerationConfig(
        bounds=bounds,
        seed=3,
        max_faces=4,
        standoff=3.5,
        headland_width=7.5,
        sideland_width=6.5,
    ))
    mesh = build_ground_mesh(scene, bounds, undulation=ChannelUndulationConfig())
    crossing = scene.crossings["crossing_0001"]
    nearby = []
    for edge in mesh.crossing_breakline_edges:
        midpoint = tuple(
            (mesh.points[edge[0]][i] + mesh.points[edge[1]][i]) * 0.5
            for i in range(2)
        )
        if (
            math.dist(midpoint, crossing.location) < 2.5
            and midpoint[0] < crossing.location[0] - 1.0
        ):
            nearby.append(edge)
    assert len(nearby) == 2
    assert any(
        sum(mesh.points[i][1] for i in edge) * 0.5 < crossing.location[1]
        for edge in nearby
    )
    assert any(
        sum(mesh.points[i][1] for i in edge) * 0.5 > crossing.location[1]
        for edge in nearby
    )

    # The crossing right of center previously skipped a shoulder vertex that
    # was 0.3 micrometres off the equivalent road constraint, pulling the wall
    # diagonally from the channel bottom to a later road vertex.
    right_crossing = scene.crossings["crossing_0003"]
    right_edges = []
    for edge in mesh.crossing_breakline_edges:
        midpoint = tuple(
            (mesh.points[edge[0]][i] + mesh.points[edge[1]][i]) * 0.5
            for i in range(2)
        )
        if math.dist(midpoint, right_crossing.location) < 2.4:
            right_edges.append(edge)
    assert len(right_edges) >= 4
    quadrants = {
        (
            -1 if sum(mesh.points[i][0] for i in edge) * 0.5 < right_crossing.location[0] else 1,
            -1 if sum(mesh.points[i][1] for i in edge) * 0.5 < right_crossing.location[1] else 1,
        )
        for edge in right_edges
    }
    assert quadrants == {(-1, -1), (-1, 1), (1, -1), (1, 1)}

    # Independently snapped road/channel points once produced nearly
    # collinear triangles whose tiny XY footprint folded through 0.8 m in Z.
    for triangle, face_class in zip(mesh.triangles, mesh.face_classes):
        if face_class == "crossing_wall":
            continue
        a, b, c = (mesh.points[i] for i in triangle)
        area = abs(
            (b[0] - a[0]) * (c[1] - a[1])
            - (c[0] - a[0]) * (b[1] - a[1])
        ) * 0.5
        longest = max(
            math.dist(a[:2], b[:2]),
            math.dist(b[:2], c[:2]),
            math.dist(c[:2], a[:2]),
        )
        assert area / longest**2 >= 1e-5
