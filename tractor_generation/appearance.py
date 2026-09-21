"""Portable paint materials and low-poly, collision-free coachwork."""

import math
from pathlib import Path
import shutil


def paint_material(stage, output, livery):
    source = Path(__file__).parent / "assets" / "livery" / f"{livery}.png"
    return textured_material(stage, output, source, "Livery", 0.12, 0.34)


def textured_material(stage, output, source, name, metallic=0.0, roughness=0.5):
    """Copy an albedo map into the export and bind it through explicit UVs."""
    from pxr import Sdf, UsdShade

    destination = output.parent / "textures" / source.name
    destination.parent.mkdir(parents=True, exist_ok=True)
    if source.resolve() != destination.resolve():
        shutil.copyfile(source, destination)
    path = f"/Tractor/Looks/{name}"
    material = UsdShade.Material.Define(stage, path)
    surface = UsdShade.Shader.Define(stage, path + "/Surface")
    surface.CreateIdAttr("UsdPreviewSurface")
    surface.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    surface.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    reader = UsdShade.Shader.Define(stage, path + "/UV")
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)
    texture = UsdShade.Shader.Define(stage, path + "/Texture")
    texture.CreateIdAttr("UsdUVTexture")
    texture.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(f"textures/{source.name}"))
    texture.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
    for axis in ("S", "T"):
        texture.CreateInput("wrap" + axis, Sdf.ValueTypeNames.Token).Set("repeat")
    texture.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
    texture.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    surface.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(texture.ConnectableAPI(), "rgb")
    material.CreateSurfaceOutput().ConnectToSource(surface.ConnectableAPI(), "surface")
    return material


def rounded_box_data(size, radius, center=(0, 0, 0), tile_size=2.0):
    """Six gridded faces projected onto a rounded box, bounded by size.

    Face-varying planar UVs use physical meters, including the center offset,
    so adjacent body segments retain the same motif scale and phase.
    """
    half = [v / 2 for v in size]
    radius = min(radius, min(half) * 0.45)
    inner = [h - radius for h in half]
    points, normals, uvs, indices = [], [], [], []
    for axis in range(3):
        u, v = (axis + 1) % 3, (axis + 2) % 3
        # For side faces use X horizontally and Z vertically in the paint map.
        uv_axes = (0, 1) if axis == 2 else ((0, 2) if axis == 1 else (1, 2))
        for sign in (-1, 1):
            grids = [[-half[a], -inner[a], inner[a], half[a]] if radius else [-half[a], half[a]] for a in (u, v)]
            for i in range(len(grids[0]) - 1):
                for j in range(len(grids[1]) - 1):
                    corners = [(i, j), (i+1, j), (i+1, j+1), (i, j+1)]
                    if sign < 0:
                        corners.reverse()
                    for ci, cj in corners:
                        p = [0.0] * 3
                        p[axis], p[u], p[v] = sign * half[axis], grids[0][ci], grids[1][cj]
                        q = [max(-inner[a], min(inner[a], p[a])) for a in range(3)]
                        delta = [p[a] - q[a] for a in range(3)]
                        norm = math.sqrt(sum(d*d for d in delta))
                        n = [d/norm for d in delta] if radius else [float(a == axis)*sign for a in range(3)]
                        p = [q[a] + radius*n[a] + center[a] for a in range(3)]
                        indices.append(len(points))
                        points.append(tuple(p))
                        normals.append(tuple(n))
                        uvs.append(tuple(p[a] / tile_size + 0.5 for a in uv_axes))
    return points, normals, uvs, indices


def rounded_box(stage, path, size, center, material, radius=0.06):
    from pxr import Gf, Sdf, UsdGeom, UsdShade

    points, normals, uvs, indices = rounded_box_data(size, radius, center)
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(points)
    mesh.CreateFaceVertexCountsAttr([4] * (len(indices)//4))
    mesh.CreateFaceVertexIndicesAttr(indices)
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateNormalsAttr(normals)
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    mesh.CreateExtentAttr([Gf.Vec3f(*(min(p[a] for p in points) for a in range(3))),
                           Gf.Vec3f(*(max(p[a] for p in points) for a in range(3)))])
    UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,
                                         UsdGeom.Tokens.faceVarying).Set(uvs)
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)
    return mesh
