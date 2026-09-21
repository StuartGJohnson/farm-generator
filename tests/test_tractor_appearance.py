from dataclasses import replace
import math

import pytest

from tractor_generation.appearance import rounded_box_data
from tractor_generation.config import TractorConfig
from tractor_generation.wheel_appearance import wheel_mesh_data


@pytest.mark.parametrize("size", [(3.5, 1.5, 1), (0.01, 0.2, 4), (8, 3, 0.1)])
def test_panels_stay_inside_collision_envelope_with_outward_faces(size):
    points, normals, uvs, indices = rounded_box_data(size, 0.12)
    assert len(points) == len(normals) == len(uvs) == len(indices)
    for point, normal in zip(points, normals):
        assert all(abs(point[a]) <= size[a]/2 + 1e-12 for a in range(3))
        assert sum(n*n for n in normal) == pytest.approx(1)
    for offset in range(0, len(indices), 4):
        a, b, c = points[offset:offset+3]
        ab, ac = [b[i]-a[i] for i in range(3)], [c[i]-a[i] for i in range(3)]
        cross = [ab[1]*ac[2]-ab[2]*ac[1], ab[2]*ac[0]-ab[0]*ac[2], ab[0]*ac[1]-ab[1]*ac[0]]
        assert sum(cross[i]*normals[offset][i] for i in range(3)) > 0
    assert all(math.isfinite(v) for uv in uvs for v in uv)


@pytest.mark.parametrize("livery", ["tiger_stripes", "cheetah_spots"])
@pytest.mark.parametrize("scale", [0.5, 1.0, 2.0])
def test_usd_livery_assets_and_collision_invariants(tmp_path, livery, scale):
    pytest.importorskip("pxr.PhysxSchema")
    from pxr import Usd, UsdGeom, UsdPhysics, UsdShade
    from tractor_generation.usd import write_tractor_usda

    base = TractorConfig()
    scaled = {name: getattr(base, name) * scale for name in (
        "wheelbase", "tractor_body_length", "tractor_body_width", "tractor_body_height",
        "tractor_body_ground_clearance", "tractor_body_front_axle", "front_tire_dia",
        "rear_tire_dia", "front_tire_width", "rear_tire_width", "front_track_width",
        "rear_track_width", "wheel_well_clearance", "sensor_pod_length", "sensor_pod_width",
        "sensor_pod_height", "sensor_pod_height_above_ground")}
    config = replace(base, livery=livery, **scaled)
    output = write_tractor_usda(config, tmp_path / "tractor.usda")
    stage = Usd.Stage.Open(str(output))
    colliders = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)]
    assert len(colliders) == len(config.body_segments) + 4
    vehicle = stage.GetPrimAtPath("/Tractor/base_link")
    pod = stage.GetPrimAtPath("/Tractor/base_link/sensor_pod_link")
    pod_origin = UsdGeom.Xformable(pod).ComputeLocalToWorldTransform(0).ExtractTranslation()
    assert tuple(pod_origin) == pytest.approx((0, 0, config.sensor_pod_height_above_ground))
    expected_mass = 1000 * (sum(length*width*config.tractor_body_height for _, length, width in config.body_segments)
                            + config.sensor_pod_length*config.sensor_pod_width*config.sensor_pod_height)
    assert UsdPhysics.MassAPI(vehicle).GetMassAttr().Get() == pytest.approx(expected_mass)
    paint = stage.GetPrimAtPath("/Tractor/Looks/Livery/Texture")
    texture = UsdShade.Shader(paint).GetInput("file").Get()
    assert texture.path == f"textures/{livery}.png"
    assert (tmp_path / texture.path).is_file()
    assert texture.resolvedPath
    paths = [f"/Tractor/base_link/Coachwork/panel_{i}" for i in range(len(config.body_segments))]
    paths.append("/Tractor/base_link/sensor_pod_link")
    for path in paths:
        prim = stage.GetPrimAtPath(path)
        assert prim.IsA(UsdGeom.Mesh)
        assert not prim.HasAPI(UsdPhysics.CollisionAPI)
        assert UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0].GetPath() == "/Tractor/Looks/Livery"
        mesh = UsdGeom.Mesh(prim)
        assert len(UsdGeom.PrimvarsAPI(prim).GetPrimvar("st").Get()) == len(mesh.GetFaceVertexIndicesAttr().Get())


@pytest.mark.parametrize("radius,width,depth", [(0.5, 0.2, 0.2), (0.8, 0.3, 0.2), (0.1, 0.05, 0.2)])
def test_tread_and_hubs_stay_inside_original_wheel_envelope(radius, width, depth):
    meshes = wheel_mesh_data(radius, width, depth)
    for points, faces, normals, uvs in meshes.values():
        for x, y, z in points:
            assert abs(x) <= width/2 + 1e-12
            assert math.hypot(y, z) <= radius + 1e-12
        assert all(0 <= i < len(points) for face in faces for i in face)
        if uvs:
            assert len(uvs) == len(points)
            assert all(0 <= v <= 1 for uv in uvs for v in uv)
    tread = meshes["Tread"][0]
    radii = [math.hypot(y, z) for _, y, z in tread]
    assert max(radii) == pytest.approx(radius)
    assert min(radii) < radius


def test_wheel_cosmetics_do_not_change_physics(tmp_path):
    pytest.importorskip("pxr.PhysxSchema")
    from pxr import Usd, UsdGeom, UsdPhysics
    from tractor_generation.names import WHEELS
    from tractor_generation.usd import write_tractor_usda

    def physics_snapshot(stage):
        return {
            (str(prim.GetPath()), attr.GetName()): str(attr.Get())
            for prim in stage.Traverse() for attr in prim.GetAttributes()
            if attr.GetName().startswith(("physics:", "physx"))
        }

    old = replace(TractorConfig(), front_tire_depth=0.1, rear_tire_depth=0.15)
    old_stage = Usd.Stage.Open(str(write_tractor_usda(old, tmp_path / "old.usda")))
    new_stage = Usd.Stage.Open(str(write_tractor_usda(TractorConfig(), tmp_path / "new.usda")))
    assert physics_snapshot(old_stage) == physics_snapshot(new_stage)
    for wheel in WHEELS:
        for name in ("Tire", "Tread", "Rim"):
            prim = new_stage.GetPrimAtPath(f"/Tractor/base_link/{wheel}/{name}")
            assert prim.IsA(UsdGeom.Mesh)
            assert not prim.HasAPI(UsdPhysics.CollisionAPI)
            assert not prim.HasAPI(UsdPhysics.MassAPI)
            assert not prim.HasAPI(UsdPhysics.RigidBodyAPI)
    texture = new_stage.GetPrimAtPath("/Tractor/Looks/WheelHub/Texture").GetAttribute("inputs:file").Get()
    assert texture.resolvedPath
