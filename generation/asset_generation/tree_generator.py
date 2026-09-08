"""Procedural generation of a young pecan tree and textures in USD."""

from __future__ import annotations

import math
import os
import random
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw, ImageFilter
from pxr import Usd, UsdGeom, UsdShade, Sdf, Gf

def generate_bark_texture(output_path: str, seed: int = 42) -> None:
    """Generate a gray-brown seamless-looking bark texture using stretched noise."""
    width, height = 512, 512
    # Seed numpy random number generator for repeatability
    rng_np = np.random.default_rng(seed)
    noise = np.zeros((height, width))
    
    # Octave 1: coarse vertical bands
    oct1 = rng_np.random((512, 16))
    img_oct1 = Image.fromarray((oct1 * 255).astype(np.uint8)).resize((width, height), Image.Resampling.BILINEAR)
    arr_oct1 = np.array(img_oct1) / 255.0
    noise += arr_oct1 * 0.5
    
    # Octave 2: medium vertical bands
    oct2 = rng_np.random((512, 32))
    img_oct2 = Image.fromarray((oct2 * 255).astype(np.uint8)).resize((width, height), Image.Resampling.BILINEAR)
    arr_oct2 = np.array(img_oct2) / 255.0
    noise += arr_oct2 * 0.3
    
    # Octave 3: fine vertical bands
    oct3 = rng_np.random((512, 64))
    img_oct3 = Image.fromarray((oct3 * 255).astype(np.uint8)).resize((width, height), Image.Resampling.BILINEAR)
    arr_oct3 = np.array(img_oct3) / 255.0
    noise += arr_oct3 * 0.2
    
    # Add some high-frequency pixel noise
    pixel_noise = rng_np.random((height, width)) * 0.1
    noise += pixel_noise
    
    # Normalize noise to [0, 1]
    noise = (noise - noise.min()) / (noise.max() - noise.min())
    
    # Map noise to gray-brown bark palette
    c_dark = np.array([80, 65, 55])       # Dark gray-brown
    c_light = np.array([135, 120, 105])   # Light gray-brown
    
    rgb = np.zeros((height, width, 3), dtype=np.uint8)
    for c in range(3):
        rgb[:, :, c] = (c_dark[c] + (c_light[c] - c_dark[c]) * noise).astype(np.uint8)
        
    img = Image.fromarray(rgb)
    img = img.filter(ImageFilter.GaussianBlur(0.8)) # Light blur for smooth transition
    img.save(output_path)

def generate_leaf_texture(output_path: str) -> None:
    """Generate a highly detailed leaf cluster atlas texture (spray of compound leaves) with alpha mask."""
    img = Image.new("RGBA", (1024, 1024), (0, 0, 0, 0))
    draw = ImageDraw.Draw(img)
    
    def draw_leaflet(bx: float, by: float, length: float, width: float, angle_deg: float) -> None:
        angle = math.radians(angle_deg)
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        
        D = np.array([cos_a, sin_a])
        P_vec = np.array([-sin_a, cos_a])
        
        num_steps = 25
        left_pts = []
        right_pts = []
        tip_pt = np.array([bx, by]) + length * D
        
        for i in range(num_steps + 1):
            t = i / num_steps
            c_pt = np.array([bx, by]) + t * length * D
            w = width * math.sin(math.pi * t) * (1.0 - 0.15 * t)
            if t == 0 or t == 1:
                w = 0.0
                
            pt_l = c_pt + w * P_vec
            pt_r = c_pt - w * P_vec
            
            left_pts.append((pt_l[0], pt_l[1]))
            right_pts.append((pt_r[0], pt_r[1]))
            
        polygon_pts = left_pts + right_pts[::-1]
        leaf_color = (40, 110, 45, 255)
        draw.polygon(polygon_pts, fill=leaf_color)
        
        # Veins
        vein_color = (95, 150, 60, 255)
        num_veins = 6
        for j in range(1, num_veins):
            t_vein = j / num_veins
            v_pt = np.array([bx, by]) + t_vein * length * D
            w_vein = width * math.sin(math.pi * t_vein) * (1.0 - 0.15 * t_vein)
            
            left_vein_angle = angle + math.radians(45)
            left_vein_dir = np.array([math.cos(left_vein_angle), math.sin(left_vein_angle)])
            left_vein_end = v_pt + (w_vein * 0.95) * left_vein_dir
            draw.line([(v_pt[0], v_pt[1]), (left_vein_end[0], left_vein_end[1])], fill=vein_color, width=1)
            
            right_vein_angle = angle - math.radians(45)
            right_vein_dir = np.array([math.cos(right_vein_angle), math.sin(right_vein_angle)])
            right_vein_end = v_pt + (w_vein * 0.95) * right_vein_dir
            draw.line([(v_pt[0], v_pt[1]), (right_vein_end[0], right_vein_end[1])], fill=vein_color, width=1)
            
        central_vein_color = (120, 175, 80, 255)
        draw.line([(bx, by), (tip_pt[0], tip_pt[1])], fill=central_vein_color, width=2)
        
    def draw_compound_leaf(bx: float, by: float, length: float, angle_deg: float) -> None:
        angle = math.radians(angle_deg)
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        
        ex = bx + length * cos_a
        ey = by + length * sin_a
        
        # Draw rachis
        draw.line([(bx, by), (ex, ey)], fill=(105, 125, 75, 255), width=4)
        
        # 5 pairs of lateral leaflets along the rachis
        num_pairs = 5
        for i in range(num_pairs):
            t = 0.25 + 0.55 * (i / (num_pairs - 1)) # distribute leaflets along outer part
            ax = bx + t * length * cos_a
            ay = by + t * length * sin_a
            
            l_len = length * 0.22 * math.sin(math.pi * t)
            l_wid = l_len * 0.23
            
            draw_leaflet(ax, ay, l_len, l_wid, angle_deg + 75)
            draw_leaflet(ax, ay, l_len, l_wid, angle_deg - 75)
            
        # Terminal leaflet
        draw_leaflet(ex, ey, length * 0.20, length * 0.20 * 0.23, angle_deg)

    # Draw a fan cluster of 3 compound leaves starting at the very bottom center to ensure direct connection to branches without a gap
    draw_compound_leaf(512, 1020, 620, 240)
    draw_compound_leaf(512, 1020, 620, 300)
    draw_compound_leaf(512, 1020, 750, 270)
    
    img.save(output_path)

def get_orthonormal_basis(direction: Gf.Vec3d) -> tuple[Gf.Vec3d, Gf.Vec3d]:
    """Find two unit vectors orthogonal to the given direction and each other."""
    dir_norm = direction.GetNormalized()
    if abs(dir_norm[0]) < 0.9:
        u = Gf.Cross(Gf.Vec3d(1, 0, 0), dir_norm).GetNormalized()
    else:
        u = Gf.Cross(Gf.Vec3d(0, 1, 0), dir_norm).GetNormalized()
    v = Gf.Cross(dir_norm, u).GetNormalized()
    return u, v

def generate_pecan_tree_usd(output_path: Path, seed: int = 42) -> Path:
    """Generate a procedurally-modeled 3-year pecan tree as a USDA stage."""
    output_path = Path(output_path).resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    # 1. Setup texture directories and generate the textures
    textures_dir = output_path.parent / "textures"
    textures_dir.mkdir(parents=True, exist_ok=True)
    
    bark_texture_path = textures_dir / "bark_diffuse.png"
    leaf_texture_path = textures_dir / "leaf_diffuse.png"
    
    generate_bark_texture(str(bark_texture_path), seed=seed)
    generate_leaf_texture(str(leaf_texture_path))
    
    # 2. Build the tree skeleton (L-system style branching)
    all_segments: list[dict] = []
    all_leaves: list[dict] = []
    
    rng = random.Random(seed)
    
    def add_leaves_along_branch(pos: Gf.Vec3d, direction: Gf.Vec3d, cumulative_length: float) -> None:
        u, v = get_orthonormal_basis(direction)
        # Alternate sides of the branch to attach leaves
        side = rng.choice([-1.0, 1.0])
        
        # Primary leaf cluster: pointing outward and sagging more
        leaf_dir = (direction * 0.4 + u * side + Gf.Vec3d(0, 0, -0.3)).GetNormalized()
        all_leaves.append({
            'pos': pos,
            'dir': leaf_dir,
            'length': rng.uniform(0.7, 0.95),  # Broader and shorter
            'width': rng.uniform(1.3, 1.7)
        })

    def add_leaf_cluster(pos: Gf.Vec3d, direction: Gf.Vec3d) -> None:
        # Just spawn a single leaf card representing the cluster at the tip pointing more downwards
        leaf_dir_term = (direction + Gf.Vec3d(0, 0, -0.25)).GetNormalized()
        all_leaves.append({
            'pos': pos,
            'dir': leaf_dir_term,
            'length': rng.uniform(0.75, 1.0),
            'width': rng.uniform(1.3, 1.7)
        })

    def grow(
        pos: Gf.Vec3d,
        direction: Gf.Vec3d,
        radius: float,
        segment_length: float,
        step: int,
        max_steps: int,
        depth: int,
        max_depth: int,
        cumulative_length: float
    ) -> None:
        if step >= max_steps:
            if depth < max_depth:
                # Force terminal branching at the end of the branch instead of terminating
                num_branches = rng.randint(2, 3)
                for i in range(num_branches):
                    # Distribute terminal branches spirally/symmetrically
                    angle_deg = (i * 360.0 / num_branches) + rng.uniform(-15, 15)
                    angle_rad = math.radians(angle_deg)
                    phi = math.radians(rng.uniform(25, 40)) # angle relative to direction
                    
                    u, v = get_orthonormal_basis(direction)
                    b_dir = (direction * math.cos(phi) + (u * math.cos(angle_rad) + v * math.sin(angle_rad)) * math.sin(phi)).GetNormalized()
                    
                    b_radius = max(0.005, radius * 0.75)
                    b_segment_length = segment_length * 0.75
                    b_max_steps = rng.randint(4, 7)
                    
                    grow(pos, b_dir, b_radius, b_segment_length, 0, b_max_steps, depth + 1, max_depth, 0.0)
            else:
                # We are at max depth, add a terminal leaf cluster
                add_leaf_cluster(pos, direction)
            return
            
        r_start = radius
        # Tapering: branches taper slightly slower than trunk, with a hard minimum radius clamp to prevent invisibility
        r_end = max(0.005, radius * 0.92 if depth > 0 else radius * 0.96)
        
        next_pos = pos + direction * segment_length
        
        all_segments.append({
            'start': pos,
            'end': next_pos,
            'radius_start': r_start,
            'radius_end': r_end,
            'cumulative_length': cumulative_length,
            'depth': depth
        })
        
        next_cumulative = cumulative_length + segment_length
        
        # Calculate next grow direction
        if depth > 0:
            # Gravitropism (upwards pull) and outward bias from trunk center
            out_bias = Gf.Vec3d(pos[0], pos[1], 0)
            if out_bias.GetLength() > 0.001:
                out_bias.Normalize()
            else:
                out_bias = Gf.Vec3d(0, 0, 0)
            next_dir = (direction * 0.8 + Gf.Vec3d(0, 0, 0.15) + out_bias * 0.05).GetNormalized()
        else:
            next_dir = direction
            
        # Add random directional noise for natural look
        noise = Gf.Vec3d(
            rng.uniform(-0.05, 0.05),
            rng.uniform(-0.05, 0.05),
            rng.uniform(-0.02, 0.02)
        )
        next_dir = (next_dir + noise).GetNormalized()
        
        # Spawn leaves along secondary/tertiary branches with 9% probability to reduce leaf count by ~30%
        if depth > 0 and step >= 2:
            if rng.random() < 0.09:
                add_leaves_along_branch(pos, direction, next_cumulative)
            
        # Grow the next segment
        grow(next_pos, next_dir, r_end, segment_length, step + 1, max_steps, depth, max_depth, next_cumulative)
        
        # Branching behavior
        if depth == 0:
            # Trunk lateral branching starts above 0.6m (step index >= 3)
            if step >= 3:
                # Lower density of lateral branches since we also branch terminally at the top
                if rng.random() < 0.4:
                    # Distribute branches spirally around trunk using golden angle spacing
                    angle_deg = (step * 137.5) + rng.uniform(-20, 20)
                    angle_rad = math.radians(angle_deg)
                    phi = math.radians(rng.uniform(40, 60)) # angle relative to Z-axis
                    
                    b_dir = Gf.Vec3d(
                        math.cos(angle_rad) * math.sin(phi),
                        math.sin(angle_rad) * math.sin(phi),
                        math.cos(phi)
                    ).GetNormalized()
                    
                    b_radius = max(0.005, r_start * 0.7)
                    b_segment_length = 0.15
                    b_max_steps = rng.randint(6, 9)
                    
                    grow(pos, b_dir, b_radius, b_segment_length, 0, b_max_steps, depth + 1, max_depth, 0.0)
                    
        elif depth == 1:
            # Primary branch branching: spawn secondary branches
            if step >= 2 and step < max_steps - 1:
                if rng.random() < 0.25:
                    right = Gf.Cross(direction, Gf.Vec3d(0, 0, 1))
                    if right.GetLength() < 0.001:
                        right = Gf.Vec3d(1, 0, 0)
                    else:
                        right.Normalize()
                        
                    side = rng.choice([-1.0, 1.0])
                    # Branch off to one side and slightly up
                    b_dir = (direction * 0.7 + right * (0.5 * side) + Gf.Vec3d(0, 0, 0.15)).GetNormalized()
                    
                    b_radius = max(0.005, r_start * 0.7)
                    b_segment_length = 0.12
                    b_max_steps = rng.randint(4, 6)
                    
                    grow(pos, b_dir, b_radius, b_segment_length, 0, b_max_steps, depth + 1, max_depth, 0.0)
                    
        elif depth == 2:
            # Secondary branch branching: spawn tertiary branches
            if step >= 1 and step < max_steps - 1:
                if rng.random() < 0.2:
                    right = Gf.Cross(direction, Gf.Vec3d(0, 0, 1))
                    if right.GetLength() < 0.001:
                        right = Gf.Vec3d(1, 0, 0)
                    else:
                        right.Normalize()
                        
                    side = rng.choice([-1.0, 1.0])
                    b_dir = (direction * 0.75 + right * (0.45 * side) + Gf.Vec3d(0, 0, 0.1)).GetNormalized()
                    
                    b_radius = max(0.005, r_start * 0.7)
                    b_segment_length = 0.08
                    b_max_steps = rng.randint(2, 4)
                    
                    grow(pos, b_dir, b_radius, b_segment_length, 0, b_max_steps, depth + 1, max_depth, 0.0)

    # Grow the pecan tree skeleton
    # Height of trunk: 10 segments of 0.2m = 2.0m. Base radius = 4.5cm (9cm diameter trunk).
    grow(
        pos=Gf.Vec3d(0, 0, 0),
        direction=Gf.Vec3d(0, 0, 1),
        radius=0.045,
        segment_length=0.2,
        step=0,
        max_steps=10,
        depth=0,
        max_depth=3, # Maximum depth of 3!
        cumulative_length=0.0
    )
    
    # 3. Create the USD Stage
    if output_path.exists():
        try:
            os.remove(output_path)
        except OSError:
            pass
            
    stage = Usd.Stage.CreateNew(str(output_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    
    # Root Xform
    root_xform = UsdGeom.Xform.Define(stage, "/PecanTree")
    stage.SetDefaultPrim(root_xform.GetPrim())
    
    # 4. Generate mesh data
    # --- Trunk and branches mesh ---
    branch_vertices: list[Gf.Vec3f] = []
    branch_indices: list[int] = []
    branch_face_counts: list[int] = []
    branch_uvs: list[Gf.Vec2f] = []
    
    N = 6 # Hexagonal cylinder segments
    
    for seg in all_segments:
        A = seg['start']
        B = seg['end']
        r_start = seg['radius_start']
        r_end = seg['radius_end']
        
        dir_vec = (B - A).GetNormalized()
        u, v = get_orthonormal_basis(dir_vec)
        
        base_idx = len(branch_vertices)
        
        # Start ring (N+1 vertices to handle UV seam wrapping)
        for i in range(N + 1):
            theta = i * 2 * math.pi / N
            cos_t = math.cos(theta)
            sin_t = math.sin(theta)
            pt = A + r_start * (cos_t * u + sin_t * v)
            branch_vertices.append(Gf.Vec3f(pt[0], pt[1], pt[2]))
            branch_uvs.append(Gf.Vec2f(i / N, seg['cumulative_length'] * 3.0))
            
        # End ring (N+1 vertices)
        for i in range(N + 1):
            theta = i * 2 * math.pi / N
            cos_t = math.cos(theta)
            sin_t = math.sin(theta)
            pt = B + r_end * (cos_t * u + sin_t * v)
            branch_vertices.append(Gf.Vec3f(pt[0], pt[1], pt[2]))
            branch_uvs.append(Gf.Vec2f(i / N, (seg['cumulative_length'] + (B - A).GetLength()) * 3.0))
            
        # Faces connecting start and end rings
        for i in range(N):
            idx0 = base_idx + i
            idx1 = base_idx + i + 1
            idx2 = base_idx + N + 1 + i + 1
            idx3 = base_idx + N + 1 + i
            
            branch_indices.extend([idx0, idx1, idx2, idx3])
            branch_face_counts.append(4)
            
    # Define Branch Mesh
    branch_mesh = UsdGeom.Mesh.Define(stage, "/PecanTree/Trunk")
    branch_mesh.CreatePointsAttr(branch_vertices)
    branch_mesh.CreateFaceVertexIndicesAttr(branch_indices)
    branch_mesh.CreateFaceVertexCountsAttr(branch_face_counts)
    branch_mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    
    # Attach UVs
    texCoords = UsdGeom.PrimvarsAPI(branch_mesh.GetPrim()).CreatePrimvar(
        "st",
        Sdf.ValueTypeNames.TexCoord2fArray,
        UsdGeom.Tokens.varying
    )
    texCoords.Set(branch_uvs)
    
    # --- Leaves mesh ---
    leaf_vertices: list[Gf.Vec3f] = []
    leaf_indices: list[int] = []
    leaf_face_counts: list[int] = []
    leaf_uvs: list[Gf.Vec2f] = []
    
    for leaf in all_leaves:
        P = leaf['pos']
        L_dir = leaf['dir']
        L_len = leaf['length']
        L_wid = leaf['width']
        
        # Leaf card coordinate frame
        if abs(L_dir[2]) < 0.9:
            L_right = Gf.Cross(L_dir, Gf.Vec3d(0, 0, 1)).GetNormalized()
        else:
            L_right = Gf.Cross(L_dir, Gf.Vec3d(1, 0, 0)).GetNormalized()
            
        base_idx = len(leaf_vertices)
        
        # We define 10 segments along the leaf length (11 rings of vertices) to form a smooth, continuous 3D curve
        num_segments = 10
        t_values = [i / num_segments for i in range(num_segments + 1)]
        
        # Build the center line points sequentially, pulling the direction down due to gravity
        curr_pos = P
        curr_dir = L_dir
        c_pts = [curr_pos]
        
        segment_len_step = L_len / num_segments
        # Scale down gravity pull per step since we have 10 steps instead of 3
        gravity_pull = Gf.Vec3d(0, 0, -0.28)
        
        for step_idx in range(num_segments):
            curr_dir = (curr_dir + gravity_pull).GetNormalized()
            curr_pos = curr_pos + curr_dir * segment_len_step
            c_pts.append(curr_pos)
            
        # Add vertices for the 11 rings (5 vertices per ring to form 4 transverse sections for a smooth curve)
        num_cols = 4
        for idx_t, t in enumerate(t_values):
            c_pt = c_pts[idx_t]
            
            # Side sag increases towards the leaf tip and is scaled to be more droopy (factor 0.38)
            max_side_sag = L_wid * 0.38 * t
            
            for i in range(num_cols + 1):
                u = i / num_cols
                x = u - 0.5
                # Parabolic sag factor across the width (0 at center, 1 at edges)
                sag_factor = 4.0 * x * x
                side_sag = max_side_sag * sag_factor
                
                pt = c_pt + L_right * (x * L_wid) - Gf.Vec3d(0, 0, 1) * side_sag
                leaf_vertices.append(Gf.Vec3f(pt[0], pt[1], pt[2]))
                leaf_uvs.append(Gf.Vec2f(u, t))
            
        # Add faces: 4 quad columns connecting the 11 rings
        for j in range(num_segments):
            for col in range(num_cols):
                idx0 = base_idx + 5 * j + col
                idx1 = base_idx + 5 * j + col + 1
                idx2 = base_idx + 5 * j + col + 5 + 1
                idx3 = base_idx + 5 * j + col + 5
                
                leaf_indices.extend([idx0, idx1, idx2, idx3])
                leaf_face_counts.append(4)
        
    # Define Leaves Mesh
    leaf_mesh = UsdGeom.Mesh.Define(stage, "/PecanTree/Leaves")
    leaf_mesh.CreatePointsAttr(leaf_vertices)
    leaf_mesh.CreateFaceVertexIndicesAttr(leaf_indices)
    leaf_mesh.CreateFaceVertexCountsAttr(leaf_face_counts)
    leaf_mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    leaf_mesh.CreateDoubleSidedAttr(True)
    
    # Attach UVs
    leafTexCoords = UsdGeom.PrimvarsAPI(leaf_mesh.GetPrim()).CreatePrimvar(
        "st",
        Sdf.ValueTypeNames.TexCoord2fArray,
        UsdGeom.Tokens.varying
    )
    leafTexCoords.Set(leaf_uvs)
    
    # 5. Materials and Shading setup
    materials_scope = UsdGeom.Scope.Define(stage, "/PecanTree/Materials")
    
    # Texture relative paths inside USDA
    rel_bark_texture = "textures/bark_diffuse.png"
    rel_leaf_texture = "textures/leaf_diffuse.png"
    
    # Bark Material
    bark_mat = UsdShade.Material.Define(stage, "/PecanTree/Materials/BarkMat")
    bark_shader = UsdShade.Shader.Define(stage, "/PecanTree/Materials/BarkMat/Shader")
    bark_shader.CreateIdAttr("UsdPreviewSurface")
    bark_shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.85)
    bark_shader.CreateInput("specularColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.03, 0.03, 0.03))
    bark_mat.CreateOutput("surface", Sdf.ValueTypeNames.Token).ConnectToSource(bark_shader.ConnectableAPI(), "surface")
    
    bark_tex_reader = UsdShade.Shader.Define(stage, "/PecanTree/Materials/BarkMat/Texture")
    bark_tex_reader.CreateIdAttr("UsdUVTexture")
    bark_tex_reader.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(rel_bark_texture)
    bark_tex_reader.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
    bark_tex_reader.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
    
    bark_primvar_reader = UsdShade.Shader.Define(stage, "/PecanTree/Materials/BarkMat/PrimvarReader")
    bark_primvar_reader.CreateIdAttr("UsdPrimvarReader_float2")
    bark_primvar_reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    bark_primvar_reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)
    
    bark_tex_reader.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(bark_primvar_reader.ConnectableAPI(), "result")
    bark_tex_reader.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    bark_shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(bark_tex_reader.ConnectableAPI(), "rgb")
    
    # Bind Bark Material to trunk/branches
    UsdShade.MaterialBindingAPI.Apply(branch_mesh.GetPrim()).Bind(bark_mat)
    
    # Leaf Material
    leaf_mat = UsdShade.Material.Define(stage, "/PecanTree/Materials/LeafMat")
    leaf_shader = UsdShade.Shader.Define(stage, "/PecanTree/Materials/LeafMat/Shader")
    leaf_shader.CreateIdAttr("UsdPreviewSurface")
    leaf_shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.35)
    leaf_shader.CreateInput("specularColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.05, 0.05, 0.05))
    leaf_shader.CreateInput("opacityThreshold", Sdf.ValueTypeNames.Float).Set(0.4) # Cutout opacity mask
    leaf_mat.CreateOutput("surface", Sdf.ValueTypeNames.Token).ConnectToSource(leaf_shader.ConnectableAPI(), "surface")
    
    leaf_tex_reader = UsdShade.Shader.Define(stage, "/PecanTree/Materials/LeafMat/Texture")
    leaf_tex_reader.CreateIdAttr("UsdUVTexture")
    leaf_tex_reader.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(rel_leaf_texture)
    leaf_tex_reader.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
    leaf_tex_reader.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
    
    leaf_primvar_reader = UsdShade.Shader.Define(stage, "/PecanTree/Materials/LeafMat/PrimvarReader")
    leaf_primvar_reader.CreateIdAttr("UsdPrimvarReader_float2")
    leaf_primvar_reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    leaf_primvar_reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)
    
    leaf_tex_reader.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(leaf_primvar_reader.ConnectableAPI(), "result")
    leaf_tex_reader.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    leaf_tex_reader.CreateOutput("a", Sdf.ValueTypeNames.Float)
    
    leaf_shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(leaf_tex_reader.ConnectableAPI(), "rgb")
    leaf_shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).ConnectToSource(leaf_tex_reader.ConnectableAPI(), "a")
    
    # Bind Leaf Material to leaves
    UsdShade.MaterialBindingAPI.Apply(leaf_mesh.GetPrim()).Bind(leaf_mat)
    
    # Save the USDA file
    stage.GetRootLayer().Save()
    return output_path
