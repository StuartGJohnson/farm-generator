"""Build reusable chain-link fence USD assets for farm generation.

Coordinates are metres in a Z-up frame. A panel runs from X=0 to X=2,
stands on Z=0, and faces both +/-Y. Its diamond lattice is real tube geometry;
the invisible planar collider is separate from the visible wires.
"""

from __future__ import annotations

import math
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade


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



def generate_fence_assets(output_dir: Path) -> tuple[Path, Path]:
    """Build the reusable post and panel assets for a farm bundle."""
    output_dir = Path(output_dir)
    post_path = output_dir / "chain_link_post.usda"
    panel_path = output_dir / "chain_link_panel.usda"
    build_post(post_path)
    build_panel(panel_path)
    return post_path, panel_path
