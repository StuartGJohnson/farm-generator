"""Build reusable chain-link fence USD assets and a short assembly scene.

Coordinates are metres in a Z-up frame. A panel runs from X=0 to X=2,
stands on Z=0, and faces both +/-Y. Its diamond lattice is real tube geometry;
the invisible planar collider is separate from the visible wires.
"""

from __future__ import annotations

import math
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade


ROOT = Path(__file__).resolve().parent
PANEL_LENGTH_M = 2.0
FENCE_HEIGHT_M = 2.0
WIRE_DIAMETER_M = 0.0032
DIAMOND_PITCH_M = 0.10  # 50 mm spacing at the crossings in X and Z.


def _stage(path: Path, root_name: str):
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        path.unlink()
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, f"/{root_name}")
    stage.SetDefaultPrim(root.GetPrim())
    return stage, root


def _metal(stage, path: str, color=(0.66, 0.71, 0.73)):
    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/PreviewSurface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.82)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.37)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def _solid_color(stage, path: str, color):
    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/PreviewSurface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.8)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def _bind(prim, material):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)


def _cylinder(stage, path, radius, height, center, material,
              axis=UsdGeom.Tokens.z):
    cylinder = UsdGeom.Cylinder.Define(stage, path)
    cylinder.CreateAxisAttr(axis)
    cylinder.CreateRadiusAttr(radius)
    cylinder.CreateHeightAttr(height)
    UsdGeom.Xformable(cylinder).AddTranslateOp().Set(Gf.Vec3d(*center))
    _bind(cylinder.GetPrim(), material)
    return cylinder


def build_post(path: Path):
    stage, _ = _stage(path, "FencePost")
    metal = _metal(stage, "/FencePost/GalvanizedSteel")
    # The shaft touches the ground; the rounded cap finishes at exactly 2 m.
    _cylinder(stage, "/FencePost/Shaft", 0.030, 1.940,
              (0, 0, 0.970), metal)
    _cylinder(stage, "/FencePost/CapRim", 0.038, 0.014,
              (0, 0, 1.940), metal)
    dome = UsdGeom.Sphere.Define(stage, "/FencePost/RoundedCap")
    dome.CreateRadiusAttr(0.038)
    UsdGeom.Xformable(dome).AddTranslateOp().Set(Gf.Vec3d(0, 0, 1.962))
    _bind(dome.GetPrim(), metal)
    stage.GetRootLayer().Save()


def _clipped_diagonal(slope: int, intercept: float, xmin: float, xmax: float,
                      zmin: float, zmax: float):
    """Clip z = slope*x + intercept to the panel's open wire rectangle."""
    candidates = []
    for x in (xmin, xmax):
        z = slope * x + intercept
        if zmin - 1e-9 <= z <= zmax + 1e-9:
            candidates.append((x, max(zmin, min(zmax, z))))
    for z in (zmin, zmax):
        x = (z - intercept) / slope
        if xmin - 1e-9 <= x <= xmax + 1e-9:
            candidates.append((max(xmin, min(xmax, x)), z))
    candidates = sorted(set((round(x, 9), round(z, 9)) for x, z in candidates))
    return (candidates[0], candidates[-1]) if len(candidates) >= 2 else None


def _tube(points, counts, indices, start, end, radius, sides=8):
    """Append one open-ended cylindrical tube with outward-facing quads."""
    direction = (end - start).GetNormalized()
    u = Gf.Vec3d(0, 1, 0)
    v = Gf.Cross(direction, u).GetNormalized()
    base = len(points)
    for center in (start, end):
        for side in range(sides):
            angle = 2 * math.pi * side / sides
            p = center + radius * (math.cos(angle) * u + math.sin(angle) * v)
            points.append(Gf.Vec3f(*p))
    for side in range(sides):
        next_side = (side + 1) % sides
        indices.extend((base + side, base + next_side,
                        base + sides + next_side, base + sides + side))
        counts.append(4)


def build_panel(path: Path):
    stage, _ = _stage(path, "FencePanel")
    metal = _metal(stage, "/FencePanel/GalvanizedSteel")

    # Narrow vertical termination pipes fit between 60 mm posts at X=0, 2.
    for name, x in (("LeftTerminationPipe", 0.065),
                    ("RightTerminationPipe", PANEL_LENGTH_M - 0.065)):
        _cylinder(stage, f"/FencePanel/{name}", 0.012, FENCE_HEIGHT_M,
                  (x, 0, FENCE_HEIGHT_M / 2), metal)

    # Rails meet the vertical pipes at their centerlines. The slight overlap
    # hides the wire ends without requiring separate corner fittings.
    xmin, xmax = 0.065, PANEL_LENGTH_M - 0.065
    zmin, zmax = 0.05, FENCE_HEIGHT_M - 0.05
    for name, z in (("BottomRail", zmin), ("TopRail", zmax)):
        _cylinder(stage, f"/FencePanel/{name}", 0.012, xmax - xmin,
                  ((xmin + xmax) / 2, 0, z), metal, UsdGeom.Tokens.x)

    points, counts, indices = [], [], []
    radius = WIRE_DIAMETER_M / 2
    for slope, y in ((1, -0.0018), (-1, 0.0018)):
        intercept_min = zmin - slope * (xmax if slope > 0 else xmin)
        intercept_max = zmax - slope * (xmin if slope > 0 else xmax)
        first = math.floor(intercept_min / DIAMOND_PITCH_M)
        last = math.ceil(intercept_max / DIAMOND_PITCH_M)
        for step in range(first, last + 1):
            ends = _clipped_diagonal(slope, step * DIAMOND_PITCH_M,
                                     xmin, xmax, zmin, zmax)
            if ends is None:
                continue
            (x0, z0), (x1, z1) = ends
            if math.hypot(x1 - x0, z1 - z0) < 0.005:
                continue
            _tube(points, counts, indices, Gf.Vec3d(x0, y, z0),
                  Gf.Vec3d(x1, y, z1), radius)

    wires = UsdGeom.Mesh.Define(stage, "/FencePanel/WireLattice")
    wires.CreatePointsAttr(points)
    wires.CreateFaceVertexCountsAttr(counts)
    wires.CreateFaceVertexIndicesAttr(indices)
    wires.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    wires.CreateDoubleSidedAttr(True)
    _bind(wires.GetPrim(), metal)

    # Static, two-sided triangle collision surface. Kept invisible so the
    # visible gaps between the wires remain transparent in the viewport.
    collider = UsdGeom.Mesh.Define(stage, "/FencePanel/CollisionPlane")
    collider.CreatePointsAttr([(0, 0, 0), (2, 0, 0), (2, 0, 2), (0, 0, 2)])
    collider.CreateFaceVertexCountsAttr([3, 3])
    collider.CreateFaceVertexIndicesAttr([0, 1, 2, 0, 2, 3])
    collider.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    collider.CreateDoubleSidedAttr(True)
    collider.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
    UsdPhysics.CollisionAPI.Apply(collider.GetPrim()).CreateCollisionEnabledAttr(True)
    UsdPhysics.MeshCollisionAPI.Apply(collider.GetPrim()).CreateApproximationAttr("none")
    stage.GetRootLayer().Save()


def build_test_scene(path: Path):
    stage, _ = _stage(path, "World")
    ground_mat = _solid_color(stage, "/World/Materials/Ground", (0.27, 0.34, 0.27))
    marker_mats = [
        _solid_color(stage, "/World/Materials/Blue", (0.07, 0.35, 0.8)),
        _solid_color(stage, "/World/Materials/Orange", (0.9, 0.35, 0.06)),
    ]
    ground = UsdGeom.Cube.Define(stage, "/World/Ground")
    ground.CreateSizeAttr(1.0)
    UsdGeom.Xformable(ground).AddTranslateOp().Set(Gf.Vec3d(3, 4, -0.02))
    UsdGeom.Xformable(ground).AddScaleOp().Set(Gf.Vec3f(9, 11, 0.04))
    _bind(ground.GetPrim(), ground_mat)

    # Three panels along +X, then four along +Y. The corner belongs to both
    # runs, so it appears once in post_positions.
    first_count, second_count = 3, 4
    corner_x = first_count * PANEL_LENGTH_M
    post_positions = [
        (index * PANEL_LENGTH_M, 0) for index in range(first_count + 1)
    ] + [
        (corner_x, index * PANEL_LENGTH_M)
        for index in range(1, second_count + 1)
    ]
    for index, (x, y) in enumerate(post_positions):
        post = UsdGeom.Xform.Define(stage, f"/World/Fence/Post_{index:02d}")
        post.GetPrim().GetReferences().AddReference("./assets/chain_link_post.usda")
        UsdGeom.Xformable(post).AddTranslateOp().Set(Gf.Vec3d(x, y, 0))
    panel_positions = [
        (index * PANEL_LENGTH_M, 0, 0) for index in range(first_count)
    ] + [
        (corner_x, index * PANEL_LENGTH_M, 90)
        for index in range(second_count)
    ]
    for index, (x, y, angle) in enumerate(panel_positions):
        panel = UsdGeom.Xform.Define(stage, f"/World/Fence/Panel_{index:02d}")
        panel.GetPrim().GetReferences().AddReference("./assets/chain_link_panel.usda")
        UsdGeom.Xformable(panel).AddTranslateOp().Set(Gf.Vec3d(x, y, 0))
        UsdGeom.Xformable(panel).AddRotateZOp().Set(angle)

    # Color behind the open lattice makes the see-through geometry easy to
    # inspect from the authored camera without changing the fence assets.
    marker_positions = [(x, 0.75) for x in (1.0, 3.0, 5.0)] + [
        (corner_x - 0.75, y) for y in (1.0, 3.0, 5.0, 7.0)
    ]
    for index, (x, y) in enumerate(marker_positions):
        marker = UsdGeom.Cube.Define(stage, f"/World/BehindFence/Marker_{index:02d}")
        marker.CreateSizeAttr(1.0)
        UsdGeom.Xformable(marker).AddTranslateOp().Set(Gf.Vec3d(x, y, 0.375))
        UsdGeom.Xformable(marker).AddScaleOp().Set(Gf.Vec3f(0.42, 0.35, 0.75))
        _bind(marker.GetPrim(), marker_mats[index % 2])

    sky = UsdLux.DomeLight.Define(stage, "/World/Lights/Sky")
    sky.CreateIntensityAttr(500.0)
    sun = UsdLux.DistantLight.Define(stage, "/World/Lights/Sun")
    sun.CreateIntensityAttr(1200.0)
    UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(35, 15, 25))
    camera = UsdGeom.Camera.Define(stage, "/World/InspectionCamera")
    camera.CreateProjectionAttr(UsdGeom.Tokens.perspective)
    camera.CreateFocalLengthAttr(20.0)
    look_at = Gf.Matrix4d().SetLookAt(Gf.Vec3d(13.5, -6.5, 7.0),
                                     Gf.Vec3d(3.5, 3.5, 1.0), Gf.Vec3d(0, 0, 1))
    UsdGeom.Xformable(camera).AddTransformOp().Set(look_at.GetInverse())
    stage.GetRootLayer().Save()


def main():
    assets = ROOT / "assets"
    build_post(assets / "chain_link_post.usda")
    build_panel(assets / "chain_link_panel.usda")
    build_test_scene(ROOT / "fence_test_scene.usda")
    print(f"Wrote fence assets and test scene to {ROOT}")


if __name__ == "__main__":
    main()
