"""Procedural generation of ground cover weeds (dry grass, fennel, ashweed) and textures in USD."""

from __future__ import annotations

import math
import os
import random
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw, ImageFilter
from pxr import Usd, UsdGeom, UsdShade, Sdf, Gf

def get_orthonormal_basis(direction: Gf.Vec3d) -> tuple[Gf.Vec3d, Gf.Vec3d]:
    """Find two unit vectors orthogonal to the given direction and each other."""
    dir_norm = direction.GetNormalized()
    if abs(dir_norm[0]) < 0.9:
        u = Gf.Cross(Gf.Vec3d(1, 0, 0), dir_norm).GetNormalized()
    else:
        u = Gf.Cross(Gf.Vec3d(0, 1, 0), dir_norm).GetNormalized()
    v = Gf.Cross(dir_norm, u).GetNormalized()
    return u, v

# ==============================================================================
# 1. DRY GRASS GENERATOR
# ==============================================================================

def generate_dry_grass_texture(output_path: str, seed: int = 42) -> None:
    """Generate a straw-yellow/dry brown gradient texture for grass blades."""
    width, height = 256, 512
    rng_np = np.random.default_rng(seed)
    
    # Generate vertically-stretched noise bands to simulate grass fibers
    oct1 = rng_np.random((512, 8))
    img_oct1 = Image.fromarray((oct1 * 255).astype(np.uint8)).resize((width, height), Image.Resampling.BILINEAR)
    arr_oct1 = np.array(img_oct1) / 255.0
    
    rgb = np.zeros((height, width, 3), dtype=np.uint8)
    for y in range(height):
        t = y / height  # 0 at top (tip), 1 at bottom (base)
        
        # Color gradient: dry brown at tip, golden straw in middle, slightly green at the base
        if t < 0.15:
            # Tip: faded dry brown
            factor = t / 0.15
            c = np.array([135, 110, 75]) * (1.0 - factor) + np.array([200, 170, 110]) * factor
        elif t > 0.85:
            # Base: slightly green
            factor = (t - 0.85) / 0.15
            c = np.array([200, 170, 110]) * (1.0 - factor) + np.array([120, 135, 70]) * factor
        else:
            c = np.array([200, 170, 110])
            
        noise_val = arr_oct1[y, :]
        for x in range(width):
            nv = noise_val[x]
            # Add fiber shading variations
            c_perturbed = c * (0.85 + 0.25 * nv)
            rgb[y, x, 0] = min(255, max(0, int(c_perturbed[0])))
            rgb[y, x, 1] = min(255, max(0, int(c_perturbed[1])))
            rgb[y, x, 2] = min(255, max(0, int(c_perturbed[2])))
            
    img = Image.fromarray(rgb)
    img = img.filter(ImageFilter.GaussianBlur(0.4))
    img.save(output_path)

def generate_dry_grass_usd(output_path: Path, seed: int = 42) -> Path:
    """Generate a procedurally modeled dry grass clump USDA asset."""
    output_path = Path(output_path).resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    textures_dir = output_path.parent / "textures"
    textures_dir.mkdir(parents=True, exist_ok=True)
    texture_filename = f"{output_path.stem}_diffuse.png"
    texture_path = textures_dir / texture_filename
    generate_dry_grass_texture(str(texture_path), seed=seed)
    
    rng = random.Random(seed)
    
    vertices: list[Gf.Vec3f] = []
    indices: list[int] = []
    face_counts: list[int] = []
    uvs: list[Gf.Vec2f] = []
    
    # Spawn grass blades in a clump
    num_blades = rng.randint(50, 70)
    for _ in range(num_blades):
        # Base offset from center
        bx = rng.uniform(-0.04, 0.04)
        by = rng.uniform(-0.04, 0.04)
        pos = Gf.Vec3d(bx, by, 0.0)
        
        # Initial direction leaning outward
        theta = rng.uniform(0, 2 * math.pi)
        phi = math.radians(rng.uniform(15, 45))  # Angle from vertical
        direction = Gf.Vec3d(
            math.cos(theta) * math.sin(phi),
            math.sin(theta) * math.sin(phi),
            math.cos(phi)
        ).GetNormalized()
        
        blade_len = rng.uniform(0.20, 0.35)
        num_segs = 6
        seg_len = blade_len / num_segs
        
        base_idx = len(vertices)
        curr_pos = pos
        curr_dir = direction
        max_w = rng.uniform(0.007, 0.012)
        
        # Forces: gravity + outward push to create fountain shape
        gravity = Gf.Vec3d(0, 0, -0.16)
        out_push = Gf.Vec3d(math.cos(theta), math.sin(theta), 0) * 0.14
        
        for j in range(num_segs + 1):
            t = j / num_segs
            # Blade tapers to 0 at tip (t=1)
            w = max_w * math.sin(math.pi * (1.0 - t * 0.98))
            
            # Align ribbon width perpendicular to direction and Z
            right = Gf.Cross(curr_dir, Gf.Vec3d(0, 0, 1))
            if right.GetLength() < 0.001:
                right = Gf.Vec3d(1, 0, 0)
            else:
                right.Normalize()
                
            v0 = curr_pos - (w / 2.0) * right
            v1 = curr_pos + (w / 2.0) * right
            
            vertices.append(Gf.Vec3f(v0[0], v0[1], v0[2]))
            vertices.append(Gf.Vec3f(v1[0], v1[1], v1[2]))
            
            # Map texture (v=0 is base, v=1 is tip)
            uvs.append(Gf.Vec2f(0.0, t))
            uvs.append(Gf.Vec2f(1.0, t))
            
            if j < num_segs:
                noise = Gf.Vec3d(rng.uniform(-0.02, 0.02), rng.uniform(-0.02, 0.02), rng.uniform(-0.01, 0.01))
                curr_dir = (curr_dir + gravity + out_push * t + noise).GetNormalized()
                curr_pos = curr_pos + curr_dir * seg_len
                
        # Define quad faces for the blade
        for j in range(num_segs):
            idx0 = base_idx + 2 * j
            idx1 = base_idx + 2 * j + 1
            idx2 = base_idx + 2 * (j + 1) + 1
            idx3 = base_idx + 2 * (j + 1)
            
            indices.extend([idx0, idx1, idx2, idx3])
            face_counts.append(4)
            
    # Add a few taller seed stalks (culms)
    num_stalks = rng.randint(4, 8)
    for _ in range(num_stalks):
        bx = rng.uniform(-0.02, 0.02)
        by = rng.uniform(-0.02, 0.02)
        pos = Gf.Vec3d(bx, by, 0.0)
        
        theta = rng.uniform(0, 2 * math.pi)
        phi = math.radians(rng.uniform(3, 12))  # Stays mostly upright
        direction = Gf.Vec3d(
            math.cos(theta) * math.sin(phi),
            math.sin(theta) * math.sin(phi),
            math.cos(phi)
        ).GetNormalized()
        
        stalk_len = rng.uniform(0.40, 0.55)
        num_segs = 8
        seg_len = stalk_len / num_segs
        
        base_idx = len(vertices)
        curr_pos = pos
        curr_dir = direction
        max_w = rng.uniform(0.003, 0.005)
        
        for j in range(num_segs + 1):
            t = j / num_segs
            w = max_w * (1.0 - t * 0.5)
            
            right = Gf.Cross(curr_dir, Gf.Vec3d(0, 0, 1))
            if right.GetLength() < 0.001:
                right = Gf.Vec3d(1, 0, 0)
            else:
                right.Normalize()
                
            v0 = curr_pos - (w / 2.0) * right
            v1 = curr_pos + (w / 2.0) * right
            
            vertices.append(Gf.Vec3f(v0[0], v0[1], v0[2]))
            vertices.append(Gf.Vec3f(v1[0], v1[1], v1[2]))
            
            # Map to center region of texture
            uvs.append(Gf.Vec2f(0.45, t))
            uvs.append(Gf.Vec2f(0.55, t))
            
            if j < num_segs:
                gravity = Gf.Vec3d(0, 0, -0.05)
                curr_dir = (curr_dir + gravity).GetNormalized()
                curr_pos = curr_pos + curr_dir * seg_len
                
        for j in range(num_segs):
            idx0 = base_idx + 2 * j
            idx1 = base_idx + 2 * j + 1
            idx2 = base_idx + 2 * (j + 1) + 1
            idx3 = base_idx + 2 * (j + 1)
            
            indices.extend([idx0, idx1, idx2, idx3])
            face_counts.append(4)
            
        # Spawn branched seed heads at the tip
        tip_pos = curr_pos
        for k in range(3):
            br_angle = (k * 120.0 + rng.uniform(-15, 15)) * math.pi / 180.0
            br_dir = (curr_dir + Gf.Vec3d(math.cos(br_angle) * 0.35, math.sin(br_angle) * 0.35, 0.25)).GetNormalized()
            br_len = rng.uniform(0.04, 0.07)
            br_end = tip_pos + br_dir * br_len
            
            # Tiny flat card for seed glumes
            seed_idx = len(vertices)
            su, sv = get_orthonormal_basis(br_dir)
            q_w = 0.012
            q_h = 0.025
            
            c0 = br_end - (q_w / 2.0) * su
            c1 = br_end + (q_w / 2.0) * su
            c2 = br_end + (q_w / 2.0) * su + q_h * sv
            c3 = br_end - (q_w / 2.0) * su + q_h * sv
            
            vertices.extend([
                Gf.Vec3f(c0[0], c0[1], c0[2]),
                Gf.Vec3f(c1[0], c1[1], c1[2]),
                Gf.Vec3f(c2[0], c2[1], c2[2]),
                Gf.Vec3f(c3[0], c3[1], c3[2])
            ])
            
            # Map seed card to top center of the texture
            uvs.extend([
                Gf.Vec2f(0.4, 0.85),
                Gf.Vec2f(0.6, 0.85),
                Gf.Vec2f(0.6, 1.0),
                Gf.Vec2f(0.4, 1.0)
            ])
            
            indices.extend([seed_idx, seed_idx + 1, seed_idx + 2, seed_idx + 3])
            face_counts.append(4)
            
    # Setup USD Stage
    if output_path.exists():
        try:
            os.remove(output_path)
        except OSError:
            pass
            
    stage = Usd.Stage.CreateNew(str(output_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    
    root_xform = UsdGeom.Xform.Define(stage, "/DryGrass")
    stage.SetDefaultPrim(root_xform.GetPrim())
    
    # Define mesh prim
    mesh = UsdGeom.Mesh.Define(stage, "/DryGrass/Blades")
    mesh.CreatePointsAttr(vertices)
    mesh.CreateFaceVertexIndicesAttr(indices)
    mesh.CreateFaceVertexCountsAttr(face_counts)
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateDoubleSidedAttr(True)
    
    texCoords = UsdGeom.PrimvarsAPI(mesh.GetPrim()).CreatePrimvar(
        "st",
        Sdf.ValueTypeNames.TexCoord2fArray,
        UsdGeom.Tokens.varying
    )
    texCoords.Set(uvs)
    
    # Material
    materials_scope = UsdGeom.Scope.Define(stage, "/DryGrass/Materials")
    mat_path = f"/DryGrass/Materials/GrassMat_{output_path.stem}"
    mat = UsdShade.Material.Define(stage, mat_path)
    shader = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.8)
    shader.CreateInput("specularColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.02, 0.02, 0.02))
    mat.CreateOutput("surface", Sdf.ValueTypeNames.Token).ConnectToSource(shader.ConnectableAPI(), "surface")
    
    tex_reader = UsdShade.Shader.Define(stage, f"{mat_path}/Texture")
    tex_reader.CreateIdAttr("UsdUVTexture")
    tex_reader.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(f"textures/{texture_filename}")
    tex_reader.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
    tex_reader.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
    
    primvar_reader = UsdShade.Shader.Define(stage, f"{mat_path}/PrimvarReader")
    primvar_reader.CreateIdAttr("UsdPrimvarReader_float2")
    primvar_reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    primvar_reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)
    
    tex_reader.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(primvar_reader.ConnectableAPI(), "result")
    tex_reader.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex_reader.ConnectableAPI(), "rgb")
    
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
    
    stage.GetRootLayer().Save()
    return output_path

# ==============================================================================
# 2. FENNEL GENERATOR
# ==============================================================================

def draw_fennel_feather(draw: ImageDraw.ImageDraw, x0: float, y0: float, length: float, angle_deg: float, rng: random.Random, depth: int = 0) -> None:
    """Draw a feathery fennel leaflet structure recursively."""
    if depth > 3 or length < 4:
        return
        
    angle_rad = math.radians(angle_deg)
    x1 = x0 + length * math.cos(angle_rad)
    y1 = y0 + length * math.sin(angle_rad)
    
    # Green shading: darker for stems, brighter/lighter green for fine fibers
    g_val = int(90 + depth * 35)
    color = (40, min(230, g_val), 50, 255)
    w = max(1, 6 - depth * 2)
    
    draw.line([(x0, y0), (x1, y1)], fill=color, width=w)
    
    num_branches = 4
    for i in range(num_branches):
        t = 0.25 + 0.65 * (i / num_branches)
        bx = x0 + t * (x1 - x0)
        by = y0 + t * (y1 - y0)
        
        # Alternating branch directions
        side = 1.0 if i % 2 == 0 else -1.0
        b_angle = angle_deg + side * rng.uniform(35, 50)
        b_len = length * 0.42 * (1.0 - t * 0.3)
        draw_fennel_feather(draw, bx, by, b_len, b_angle, rng, depth + 1)

def generate_fennel_texture(output_path: str, seed: int = 42) -> None:
    """Generate a fennel atlas texture sheet with leaf, flower, and stem regions."""
    # RGBA image: 1024x1024
    img = Image.new("RGBA", (1024, 1024), (0, 0, 0, 0))
    draw = ImageDraw.Draw(img)
    rng = random.Random(seed)
    
    # 1. Left half (0 to 512): Feathery Leaf Card
    # Base: (256, 1000), Tip: (256, 100). Growing upwards (-Y)
    draw_fennel_feather(draw, 256, 1000, 850, -90, rng, depth=0)
    
    # 2. Right top (512 to 1024, Y: 0 to 512): Flower Umbel Card
    # Center of base: (768, 500), Center of top dome: (768, 150)
    cx, cy = 768.0, 480.0
    num_rays = 18
    for i in range(num_rays):
        t = i / (num_rays - 1)
        theta_deg = 180.0 + 20.0 + t * 140.0 # From 200 to 340 degrees
        theta = math.radians(theta_deg)
        r = 280.0
        
        x_end = cx + r * math.cos(theta)
        y_end = cy + r * math.sin(theta)
        
        # Draw green-yellow ray
        draw.line([(cx, cy), (x_end, y_end)], fill=(125, 175, 65, 255), width=3)
        
        # Mini-umbellet at the end of the ray
        num_mini = 6
        for m in range(num_mini):
            m_angle = rng.uniform(0, 2 * math.pi)
            m_r = rng.uniform(10, 22)
            mx = x_end + m_r * math.cos(m_angle)
            my = y_end + m_r * math.sin(m_angle)
            
            draw.line([(x_end, y_end), (mx, my)], fill=(190, 210, 50, 255), width=1)
            # Draw tiny yellow flower dot
            f_rad = rng.uniform(3, 5)
            draw.ellipse(
                [(mx - f_rad, my - f_rad), (mx + f_rad, my + f_rad)],
                fill=(250, 225, 35, 255)
            )
            
    # 3. Right bottom (512 to 1024, Y: 512 to 1024): Stem texture
    # Create smooth green vertical striped texture
    for x in range(512, 1024):
        # Stripe variation based on sine waves
        stripe = math.sin((x - 512) * 0.15) * 0.1 + math.sin((x - 512) * 0.05) * 0.05
        factor = 0.85 + stripe
        r_col = min(255, max(0, int(65 * factor)))
        g_col = min(255, max(0, int(135 * factor)))
        b_col = min(255, max(0, int(55 * factor)))
        
        draw.line([(x, 512), (x, 1024)], fill=(r_col, g_col, b_col, 255), width=1)
        
    img.save(output_path)

def generate_fennel_usd(output_path: Path, seed: int = 42) -> Path:
    """Generate a procedurally modeled Fennel plant USDA asset."""
    output_path = Path(output_path).resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    textures_dir = output_path.parent / "textures"
    textures_dir.mkdir(parents=True, exist_ok=True)
    texture_filename = f"{output_path.stem}_diffuse.png"
    texture_path = textures_dir / texture_filename
    generate_fennel_texture(str(texture_path), seed=seed)
    
    rng = random.Random(seed)
    
    stem_segments: list[dict] = []
    leaf_cards: list[dict] = []
    flower_cards: list[dict] = []
    
    def grow_fennel_skeleton(
        pos: Gf.Vec3d,
        direction: Gf.Vec3d,
        radius: float,
        seg_len: float,
        step: int,
        max_steps: int,
        depth: int,
        max_depth: int,
        cum_len: float
    ) -> None:
        if step >= max_steps:
            if depth < max_depth:
                # Splitting terminal branching
                num_br = rng.randint(2, 3)
                for i in range(num_br):
                    angle_deg = (i * 360.0 / num_br) + rng.uniform(-15, 15)
                    angle_rad = math.radians(angle_deg)
                    phi = math.radians(rng.uniform(20, 35))
                    
                    u, v = get_orthonormal_basis(direction)
                    b_dir = (direction * math.cos(phi) + (u * math.cos(angle_rad) + v * math.sin(angle_rad)) * math.sin(phi)).GetNormalized()
                    
                    # Taper branching properties
                    b_radius = max(0.002, radius * 0.65)
                    b_seg_len = seg_len * 0.75
                    b_max_steps = rng.randint(3, 5)
                    grow_fennel_skeleton(pos, b_dir, b_radius, b_seg_len, 0, b_max_steps, depth + 1, max_depth, 0.0)
            else:
                # Add terminal flower umbel card
                flower_cards.append({
                    'pos': pos,
                    'dir': direction,
                    'size': rng.uniform(0.08, 0.12)
                })
            return
            
        r_start = radius
        r_end = max(0.002, radius * 0.9)
        next_pos = pos + direction * seg_len
        
        stem_segments.append({
            'start': pos,
            'end': next_pos,
            'r_start': r_start,
            'r_end': r_end,
            'cum_len': cum_len
        })
        
        # Add random directional noise
        noise = Gf.Vec3d(rng.uniform(-0.04, 0.04), rng.uniform(-0.04, 0.04), rng.uniform(-0.01, 0.01))
        next_dir = (direction + noise).GetNormalized()
        
        # Spawn leaf cards along the stalks
        if depth > 0 or step >= 2:
            if rng.random() < 0.35:
                u, v = get_orthonormal_basis(direction)
                side = rng.choice([-1.0, 1.0])
                leaf_dir = (direction * 0.4 + u * side + Gf.Vec3d(0, 0, -0.15)).GetNormalized()
                leaf_cards.append({
                    'pos': pos,
                    'dir': leaf_dir,
                    'size': rng.uniform(0.12, 0.22)
                })
                
        grow_fennel_skeleton(next_pos, next_dir, r_end, seg_len, step + 1, max_steps, depth, max_depth, cum_len + seg_len)

    # Start growth: 0.7m tall main stalk branching at the top
    grow_fennel_skeleton(
        pos=Gf.Vec3d(0, 0, 0),
        direction=Gf.Vec3d(0, 0, 1),
        radius=0.012,
        seg_len=0.08,
        step=0,
        max_steps=6,
        depth=0,
        max_depth=2,
        cum_len=0.0
    )
    
    # Create USD Stage
    if output_path.exists():
        try:
            os.remove(output_path)
        except OSError:
            pass
            
    stage = Usd.Stage.CreateNew(str(output_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    
    root_xform = UsdGeom.Xform.Define(stage, "/Fennel")
    stage.SetDefaultPrim(root_xform.GetPrim())
    
    # 1. Stem Mesh (Cylinders)
    stem_vertices: list[Gf.Vec3f] = []
    stem_indices: list[int] = []
    stem_face_counts: list[int] = []
    stem_uvs: list[Gf.Vec2f] = []
    
    N = 5 # Pentagonal cylinders
    for seg in stem_segments:
        A = seg['start']
        B = seg['end']
        r_start = seg['r_start']
        r_end = seg['r_end']
        
        dir_vec = (B - A).GetNormalized()
        u, v = get_orthonormal_basis(dir_vec)
        base_idx = len(stem_vertices)
        
        # Start ring (N+1 vertices for UV wrap)
        for i in range(N + 1):
            theta = i * 2 * math.pi / N
            pt = A + r_start * (math.cos(theta) * u + math.sin(theta) * v)
            stem_vertices.append(Gf.Vec3f(pt[0], pt[1], pt[2]))
            # Map to stem region of texture: U in [0.5, 1.0], V in [0.0, 0.5]
            # Scale V by 0.4 to prevent wrapping into transparent/flower regions
            stem_uvs.append(Gf.Vec2f(0.5 + 0.5 * (i / N), seg['cum_len'] * 0.4))
            
        # End ring
        for i in range(N + 1):
            theta = i * 2 * math.pi / N
            pt = B + r_end * (math.cos(theta) * u + math.sin(theta) * v)
            stem_vertices.append(Gf.Vec3f(pt[0], pt[1], pt[2]))
            stem_uvs.append(Gf.Vec2f(0.5 + 0.5 * (i / N), (seg['cum_len'] + (B - A).GetLength()) * 0.4))
            
        for i in range(N):
            idx0 = base_idx + i
            idx1 = base_idx + i + 1
            idx2 = base_idx + N + 1 + i + 1
            idx3 = base_idx + N + 1 + i
            
            stem_indices.extend([idx0, idx1, idx2, idx3])
            stem_face_counts.append(4)
            
    stems_mesh = UsdGeom.Mesh.Define(stage, "/Fennel/Stems")
    stems_mesh.CreatePointsAttr(stem_vertices)
    stems_mesh.CreateFaceVertexIndicesAttr(stem_indices)
    stems_mesh.CreateFaceVertexCountsAttr(stem_face_counts)
    stems_mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    
    stem_texCoords = UsdGeom.PrimvarsAPI(stems_mesh.GetPrim()).CreatePrimvar(
        "st",
        Sdf.ValueTypeNames.TexCoord2fArray,
        UsdGeom.Tokens.varying
    )
    stem_texCoords.Set(stem_uvs)
    
    # 2. Leaf Mesh (Curved leaf cards, mapped to Left Half)
    leaf_vertices: list[Gf.Vec3f] = []
    leaf_indices: list[int] = []
    leaf_face_counts: list[int] = []
    leaf_uvs: list[Gf.Vec2f] = []
    
    for leaf in leaf_cards:
        P = leaf['pos']
        L_dir = leaf['dir']
        size = leaf['size']
        w = size * 0.45
        
        base_idx = len(leaf_vertices)
        
        # 3 rings (2 quads along length) to bend leaf
        num_segs = 2
        curr_pos = P
        curr_dir = L_dir
        c_pts = [curr_pos]
        seg_step = size / num_segs
        
        for j in range(num_segs):
            curr_dir = (curr_dir + Gf.Vec3d(0, 0, -0.18)).GetNormalized() # sag
            curr_pos = curr_pos + curr_dir * seg_step
            c_pts.append(curr_pos)
            
        for j in range(num_segs + 1):
            t = j / num_segs
            c_pt = c_pts[j]
            right = Gf.Cross(curr_dir, Gf.Vec3d(0, 0, 1)).GetNormalized()
            
            v0 = c_pt - (w / 2.0) * right
            v1 = c_pt + (w / 2.0) * right
            
            leaf_vertices.append(Gf.Vec3f(v0[0], v0[1], v0[2]))
            leaf_vertices.append(Gf.Vec3f(v1[0], v1[1], v1[2]))
            
            # Left half of texture: U in [0.0, 0.5], V in [0.0, 1.0] (base to tip)
            leaf_uvs.append(Gf.Vec2f(0.0, t))
            leaf_uvs.append(Gf.Vec2f(0.5, t))
            
        for j in range(num_segs):
            idx0 = base_idx + 2 * j
            idx1 = base_idx + 2 * j + 1
            idx2 = base_idx + 2 * (j + 1) + 1
            idx3 = base_idx + 2 * (j + 1)
            
            leaf_indices.extend([idx0, idx1, idx2, idx3])
            leaf_face_counts.append(4)
            
    leaves_mesh = UsdGeom.Mesh.Define(stage, "/Fennel/Leaves")
    leaves_mesh.CreatePointsAttr(leaf_vertices)
    leaves_mesh.CreateFaceVertexIndicesAttr(leaf_indices)
    leaves_mesh.CreateFaceVertexCountsAttr(leaf_face_counts)
    leaves_mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    leaves_mesh.CreateDoubleSidedAttr(True)
    
    leaf_texCoords = UsdGeom.PrimvarsAPI(leaves_mesh.GetPrim()).CreatePrimvar(
        "st",
        Sdf.ValueTypeNames.TexCoord2fArray,
        UsdGeom.Tokens.varying
    )
    leaf_texCoords.Set(leaf_uvs)
    
    # 3. Flower Mesh (Billboard Clouds: 3 intersecting quads, mapped to Right Top)
    flw_vertices: list[Gf.Vec3f] = []
    flw_indices: list[int] = []
    flw_face_counts: list[int] = []
    flw_uvs: list[Gf.Vec2f] = []
    
    for flw in flower_cards:
        P = flw['pos']
        rad = flw['size']
        
        # Center of the flower cluster offset slightly above branch tip
        center = P + flw['dir'] * 0.02
        
        # 3 intersecting vertical cards rotated at 0, 60, 120 deg
        for rot in [0.0, 60.0, 120.0]:
            angle = math.radians(rot)
            right = Gf.Vec3d(math.cos(angle), math.sin(angle), 0)
            up = Gf.Vec3d(0, 0, 1)
            
            base_idx = len(flw_vertices)
            
            # Quad corners
            c0 = center - rad * right - rad * 0.4 * up
            c1 = center + rad * right - rad * 0.4 * up
            c2 = center + rad * right + rad * 0.6 * up
            c3 = center - rad * right + rad * 0.6 * up
            
            flw_vertices.extend([
                Gf.Vec3f(c0[0], c0[1], c0[2]),
                Gf.Vec3f(c1[0], c1[1], c1[2]),
                Gf.Vec3f(c2[0], c2[1], c2[2]),
                Gf.Vec3f(c3[0], c3[1], c3[2])
            ])
            
            # Right top of texture: U in [0.5, 1.0], V in [0.5, 1.0]
            flw_uvs.extend([
                Gf.Vec2f(0.5, 0.5),
                Gf.Vec2f(1.0, 0.5),
                Gf.Vec2f(1.0, 1.0),
                Gf.Vec2f(0.5, 1.0)
            ])
            
            flw_indices.extend([base_idx, base_idx + 1, base_idx + 2, base_idx + 3])
            flw_face_counts.append(4)
            
    flowers_mesh = UsdGeom.Mesh.Define(stage, "/Fennel/Flowers")
    flowers_mesh.CreatePointsAttr(flw_vertices)
    flowers_mesh.CreateFaceVertexIndicesAttr(flw_indices)
    flowers_mesh.CreateFaceVertexCountsAttr(flw_face_counts)
    flowers_mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    flowers_mesh.CreateDoubleSidedAttr(True)
    
    flw_texCoords = UsdGeom.PrimvarsAPI(flowers_mesh.GetPrim()).CreatePrimvar(
        "st",
        Sdf.ValueTypeNames.TexCoord2fArray,
        UsdGeom.Tokens.varying
    )
    flw_texCoords.Set(flw_uvs)
    
    # 4. Materials and Binding Setup (Single shared Material)
    materials_scope = UsdGeom.Scope.Define(stage, "/Fennel/Materials")
    mat_path = f"/Fennel/Materials/FennelMat_{output_path.stem}"
    mat = UsdShade.Material.Define(stage, mat_path)
    shader = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.4)
    shader.CreateInput("specularColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.04, 0.04, 0.04))
    shader.CreateInput("opacityThreshold", Sdf.ValueTypeNames.Float).Set(0.45)
    mat.CreateOutput("surface", Sdf.ValueTypeNames.Token).ConnectToSource(shader.ConnectableAPI(), "surface")
    
    tex_reader = UsdShade.Shader.Define(stage, f"{mat_path}/Texture")
    tex_reader.CreateIdAttr("UsdUVTexture")
    tex_reader.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(f"textures/{texture_filename}")
    tex_reader.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
    tex_reader.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
    
    primvar_reader = UsdShade.Shader.Define(stage, f"{mat_path}/PrimvarReader")
    primvar_reader.CreateIdAttr("UsdPrimvarReader_float2")
    primvar_reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    primvar_reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)
    
    tex_reader.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(primvar_reader.ConnectableAPI(), "result")
    tex_reader.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    tex_reader.CreateOutput("a", Sdf.ValueTypeNames.Float)
    
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex_reader.ConnectableAPI(), "rgb")
    shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).ConnectToSource(tex_reader.ConnectableAPI(), "a")
    
    # Bind material to all Fennel meshes
    UsdShade.MaterialBindingAPI.Apply(stems_mesh.GetPrim()).Bind(mat)
    UsdShade.MaterialBindingAPI.Apply(leaves_mesh.GetPrim()).Bind(mat)
    UsdShade.MaterialBindingAPI.Apply(flowers_mesh.GetPrim()).Bind(mat)
    
    stage.GetRootLayer().Save()
    return output_path

# ==============================================================================
# 3. ASHWEED GENERATOR
# ==============================================================================

def generate_ashweed_texture(output_path: str, seed: int = 42) -> None:
    """Generate a serrated ashweed leaflet texture with distinct green vein layers."""
    img = Image.new("RGBA", (512, 512), (0, 0, 0, 0))
    draw = ImageDraw.Draw(img)
    
    # Draw the leaflet starting from the very bottom of the canvas (y=512) to the top (y=32)
    # to ensure that the leaflet geometry base maps to an opaque green region of the texture.
    y_base = 512.0
    y_tip = 32.0
    length = y_base - y_tip
    
    left_pts = []
    right_pts = []
    
    num_steps = 100
    for i in range(num_steps + 1):
        t = i / num_steps
        y = y_base - t * length
        
        # Base elliptical leaflet envelope
        w_envelope = 135.0 * math.sin(math.pi * t) * (1.0 - 0.28 * t)
        
        # Add sawtooth teeth serrations
        if 0.12 < t < 0.90:
            phase = (t * 11.0) % 1.0
            w = w_envelope * (1.0 - 0.12 * phase)
        else:
            w = w_envelope
            
        left_pts.append((256.0 - w, y))
        right_pts.append((256.0 + w, y))
        
    polygon_pts = left_pts + right_pts[::-1]
    
    # Fill leaflet body with dark green
    draw.polygon(polygon_pts, fill=(50, 115, 55, 255))
    
    # Draw central petiole/vein (thick light green)
    draw.line([(256, y_base), (256, y_tip)], fill=(125, 185, 95, 255), width=4)
    
    # Draw lateral veins branching off central vein
    num_veins = 6
    for j in range(1, num_veins):
        t_v = j / num_veins
        y_v = y_base - t_v * length
        w_v = 135.0 * math.sin(math.pi * t_v) * (1.0 - 0.28 * t_v)
        
        # Left vein curving up
        draw.line([(256, y_v), (256 - w_v * 0.85, y_v - 25)], fill=(100, 160, 75, 255), width=2)
        # Right vein curving up
        draw.line([(256, y_v), (256 + w_v * 0.85, y_v - 25)], fill=(100, 160, 75, 255), width=2)
        
    img.save(output_path)

def generate_ashweed_usd(output_path: Path, seed: int = 42) -> Path:
    """Generate a procedurally modeled low-lying Ashweed (ground elder) clump."""
    output_path = Path(output_path).resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    textures_dir = output_path.parent / "textures"
    textures_dir.mkdir(parents=True, exist_ok=True)
    texture_filename = f"{output_path.stem}_diffuse.png"
    texture_path = textures_dir / texture_filename
    generate_ashweed_texture(str(texture_path), seed=seed)
    
    rng = random.Random(seed)
    
    stalk_segments: list[dict] = []
    leaves: list[dict] = []
    
    num_stalks = rng.randint(12, 18)
    for i in range(num_stalks):
        # Base offset from center
        bx = rng.uniform(-0.04, 0.04)
        by = rng.uniform(-0.04, 0.04)
        pos = Gf.Vec3d(bx, by, 0.0)
        
        # Up and out angle
        theta = (i * 2.0 * math.pi / num_stalks) + rng.uniform(-0.2, 0.2)
        phi = math.radians(rng.uniform(25, 45))
        
        direction = Gf.Vec3d(
            math.cos(theta) * math.sin(phi),
            math.sin(theta) * math.sin(phi),
            math.cos(phi)
        ).GetNormalized()
        
        stalk_len = rng.uniform(0.16, 0.25)
        num_segs = 5
        seg_len = stalk_len / num_segs
        
        curr_pos = pos
        curr_dir = direction
        radius = 0.0018  # Thinner, more realistic ashweed leaf stems
        
        # Pull down (gravitropism) + outward bias
        gravity = Gf.Vec3d(0, 0, -0.22)
        out_bias = Gf.Vec3d(math.cos(theta), math.sin(theta), 0) * 0.08
        
        cum_len = 0.0
        for j in range(num_segs):
            r_start = radius
            radius = max(0.0010, radius * 0.85)
            
            next_pos = curr_pos + curr_dir * seg_len
            stalk_segments.append({
                'start': curr_pos,
                'end': next_pos,
                'r_start': r_start,
                'r_end': radius,
                'cum_len': cum_len
            })
            
            cum_len += seg_len
            curr_dir = (curr_dir + gravity + out_bias).GetNormalized()
            curr_pos = next_pos
            
        # At the tip of the petiole, split into a ternate (3 leaflets) structure
        right = Gf.Cross(curr_dir, Gf.Vec3d(0, 0, 1)).GetNormalized()
        
        # Ternate leaf directions
        dir_c = (curr_dir * 0.85 - Gf.Vec3d(0, 0, 0.15)).GetNormalized()
        dir_l = (curr_dir * 0.65 - right * 0.5 - Gf.Vec3d(0, 0, 0.1)).GetNormalized()
        dir_r = (curr_dir * 0.65 + right * 0.5 - Gf.Vec3d(0, 0, 0.1)).GetNormalized()
        
        # Center petiole and leaflet
        pos_c = curr_pos + dir_c * 0.04
        stalk_segments.append({
            'start': curr_pos,
            'end': pos_c,
            'r_start': radius,
            'r_end': radius * 0.8,
            'cum_len': cum_len
        })
        leaves.append({
            'pos': pos_c,
            'dir': dir_c,
            'len': rng.uniform(0.055, 0.075),
            'wid': rng.uniform(0.04, 0.052)
        })
        
        # Left petiole and leaflet
        pos_l = curr_pos + dir_l * 0.03
        stalk_segments.append({
            'start': curr_pos,
            'end': pos_l,
            'r_start': radius,
            'r_end': radius * 0.8,
            'cum_len': cum_len
        })
        leaves.append({
            'pos': pos_l,
            'dir': dir_l,
            'len': rng.uniform(0.045, 0.06),
            'wid': rng.uniform(0.035, 0.045)
        })
        
        # Right petiole and leaflet
        pos_r = curr_pos + dir_r * 0.03
        stalk_segments.append({
            'start': curr_pos,
            'end': pos_r,
            'r_start': radius,
            'r_end': radius * 0.8,
            'cum_len': cum_len
        })
        leaves.append({
            'pos': pos_r,
            'dir': dir_r,
            'len': rng.uniform(0.045, 0.06),
            'wid': rng.uniform(0.035, 0.045)
        })

    # Setup USD Stage
    if output_path.exists():
        try:
            os.remove(output_path)
        except OSError:
            pass
            
    stage = Usd.Stage.CreateNew(str(output_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    
    root_xform = UsdGeom.Xform.Define(stage, "/Ashweed")
    stage.SetDefaultPrim(root_xform.GetPrim())
    
    # 1. Stems Mesh
    stem_vertices: list[Gf.Vec3f] = []
    stem_indices: list[int] = []
    stem_face_counts: list[int] = []
    stem_uvs: list[Gf.Vec2f] = []
    
    N = 4 # Quadrilateral cylinders
    for seg in stalk_segments:
        A = seg['start']
        B = seg['end']
        r_start = seg['r_start']
        r_end = seg['r_end']
        
        dir_vec = (B - A).GetNormalized()
        u, v = get_orthonormal_basis(dir_vec)
        base_idx = len(stem_vertices)
        
        for i in range(N + 1):
            theta = i * 2 * math.pi / N
            pt = A + r_start * (math.cos(theta) * u + math.sin(theta) * v)
            stem_vertices.append(Gf.Vec3f(pt[0], pt[1], pt[2]))
            # Map stems to the solid green center vein of the leaflet (U=0.5, V=0.5) to keep it fully opaque green
            stem_uvs.append(Gf.Vec2f(0.5, 0.5))
            
        for i in range(N + 1):
            theta = i * 2 * math.pi / N
            pt = B + r_end * (math.cos(theta) * u + math.sin(theta) * v)
            stem_vertices.append(Gf.Vec3f(pt[0], pt[1], pt[2]))
            stem_uvs.append(Gf.Vec2f(0.5, 0.5))
            
        for i in range(N):
            idx0 = base_idx + i
            idx1 = base_idx + i + 1
            idx2 = base_idx + N + 1 + i + 1
            idx3 = base_idx + N + 1 + i
            
            stem_indices.extend([idx0, idx1, idx2, idx3])
            stem_face_counts.append(4)
            
    stems_mesh = UsdGeom.Mesh.Define(stage, "/Ashweed/Stems")
    stems_mesh.CreatePointsAttr(stem_vertices)
    stems_mesh.CreateFaceVertexIndicesAttr(stem_indices)
    stems_mesh.CreateFaceVertexCountsAttr(stem_face_counts)
    stems_mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    
    stem_texCoords = UsdGeom.PrimvarsAPI(stems_mesh.GetPrim()).CreatePrimvar(
        "st",
        Sdf.ValueTypeNames.TexCoord2fArray,
        UsdGeom.Tokens.varying
    )
    stem_texCoords.Set(stem_uvs)
    
    # 2. Leaflet Mesh (Smoothly bent 3D grids: 3x3 quads)
    leaf_vertices: list[Gf.Vec3f] = []
    leaf_indices: list[int] = []
    leaf_face_counts: list[int] = []
    leaf_uvs: list[Gf.Vec2f] = []
    
    for leaf in leaves:
        P = leaf['pos']
        L_dir = leaf['dir']
        L_len = leaf['len']
        L_wid = leaf['wid']
        
        L_right = Gf.Cross(L_dir, Gf.Vec3d(0, 0, 1)).GetNormalized()
        base_idx = len(leaf_vertices)
        
        num_segments = 3
        num_cols = 3
        
        # Build bent center points
        curr_pos = P
        curr_dir = L_dir
        c_pts = [curr_pos]
        seg_step = L_len / num_segments
        
        for step_idx in range(num_segments):
            curr_dir = (curr_dir + Gf.Vec3d(0, 0, -0.22)).GetNormalized() # Curve down
            curr_pos = curr_pos + curr_dir * seg_step
            c_pts.append(curr_pos)
            
        for idx_t in range(num_segments + 1):
            t = idx_t / num_segments
            c_pt = c_pts[idx_t]
            
            # Side sag increases towards tip
            max_side_sag = L_wid * 0.3 * t
            
            for i in range(num_cols + 1):
                u = i / num_cols
                x = u - 0.5
                sag_factor = 4.0 * x * x
                side_sag = max_side_sag * sag_factor
                
                # Leaflet vertex: parabolic side curve
                pt = c_pt + L_right * (x * L_wid) - Gf.Vec3d(0, 0, 1) * side_sag
                leaf_vertices.append(Gf.Vec3f(pt[0], pt[1], pt[2]))
                
                # Leaf UV: mapped directly to full texture (since leaflet is centered)
                leaf_uvs.append(Gf.Vec2f(u, t))
                
        for j in range(num_segments):
            for col in range(num_cols):
                idx0 = base_idx + 4 * j + col
                idx1 = base_idx + 4 * j + col + 1
                idx2 = base_idx + 4 * j + col + 4 + 1
                idx3 = base_idx + 4 * j + col + 4
                
                leaf_indices.extend([idx0, idx1, idx2, idx3])
                leaf_face_counts.append(4)
                
    leaves_mesh = UsdGeom.Mesh.Define(stage, "/Ashweed/Leaves")
    leaves_mesh.CreatePointsAttr(leaf_vertices)
    leaves_mesh.CreateFaceVertexIndicesAttr(leaf_indices)
    leaves_mesh.CreateFaceVertexCountsAttr(leaf_face_counts)
    leaves_mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    leaves_mesh.CreateDoubleSidedAttr(True)
    
    leaf_texCoords = UsdGeom.PrimvarsAPI(leaves_mesh.GetPrim()).CreatePrimvar(
        "st",
        Sdf.ValueTypeNames.TexCoord2fArray,
        UsdGeom.Tokens.varying
    )
    leaf_texCoords.Set(leaf_uvs)
    
    # 3. Materials and Binding Setup
    materials_scope = UsdGeom.Scope.Define(stage, "/Ashweed/Materials")
    mat_path = f"/Ashweed/Materials/AshweedMat_{output_path.stem}"
    mat = UsdShade.Material.Define(stage, mat_path)
    shader = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.38)
    shader.CreateInput("specularColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.03, 0.03, 0.03))
    shader.CreateInput("opacityThreshold", Sdf.ValueTypeNames.Float).Set(0.4)
    mat.CreateOutput("surface", Sdf.ValueTypeNames.Token).ConnectToSource(shader.ConnectableAPI(), "surface")
    
    tex_reader = UsdShade.Shader.Define(stage, f"{mat_path}/Texture")
    tex_reader.CreateIdAttr("UsdUVTexture")
    tex_reader.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(f"textures/{texture_filename}")
    tex_reader.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
    tex_reader.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
    
    primvar_reader = UsdShade.Shader.Define(stage, f"{mat_path}/PrimvarReader")
    primvar_reader.CreateIdAttr("UsdPrimvarReader_float2")
    primvar_reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    primvar_reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)
    
    tex_reader.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(primvar_reader.ConnectableAPI(), "result")
    tex_reader.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    tex_reader.CreateOutput("a", Sdf.ValueTypeNames.Float)
    
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex_reader.ConnectableAPI(), "rgb")
    shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).ConnectToSource(tex_reader.ConnectableAPI(), "a")
    
    # Bind materials
    UsdShade.MaterialBindingAPI.Apply(stems_mesh.GetPrim()).Bind(mat)
    UsdShade.MaterialBindingAPI.Apply(leaves_mesh.GetPrim()).Bind(mat)
    
    stage.GetRootLayer().Save()
    return output_path
