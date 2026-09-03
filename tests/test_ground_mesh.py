import math

from export.usd import build_ground_mesh, build_hydrology_pslg, write_ground_mesh_usda
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
