import math

from shapely.geometry import LineString, Point

from export.usd import (
    ChannelUndulationConfig,
    build_channel_faces,
    build_ground_mesh,
    build_hydrology_pslg,
    build_undulated_hydrology_edges,
    write_ground_mesh_usda,
)
from generation.orchestrator import FarmGenerationConfig, generate_farm


def test_ground_mesh_covers_bounds_and_contains_channels(tmp_path):
    bounds = (0.0, 0.0, 16.0, 16.0)
    scene = generate_farm(FarmGenerationConfig(bounds=bounds, seed=1, max_faces=4))
    pslg = build_hydrology_pslg(scene, bounds, max_segment_length=1.0)
    mesh = build_ground_mesh(scene, bounds, flat_resolution=1.0)

    assert pslg.vertices
    assert pslg.segments
    lengths = [math.dist(pslg.vertices[a], pslg.vertices[b]) for a, b in pslg.segments]
    assert max(lengths) <= 1.0 + 1e-9

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
    assert {tuple(sorted(edge)) for edge in mesh.breakline_edges} <= triangle_edges
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
        for a, b, c in mesh.triangles
    )
    assert abs(xy_area - 256.0) < 1e-9

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
