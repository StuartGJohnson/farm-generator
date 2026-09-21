"""Render-only agricultural tires; local X is the axle, matching Vehicle 2 visuals.

Tire depth is radial rubber thickness, not a change to the rolling radius.
Chevron lugs occupy the outer portion of that thickness. Every point stays
inside the original radius/width envelope; no physics APIs are authored here.
"""

import math


def wheel_mesh_data(radius, width, depth, segments=64):
    """Return tire, chevron tread, and two-sided textured dish mesh data."""
    rim = max(radius * 0.1, radius - depth)
    rubber = radius - rim
    lug_height = min(rubber * 0.35, width * 0.3)
    crown = radius - lug_height
    half = width / 2
    profile = [(-half, rim), (-half, rim + rubber*0.22),
               (-half*0.97, crown-rubber*0.12), (-half*0.78, crown),
               (half*0.78, crown), (half*0.97, crown-rubber*0.12),
               (half, rim + rubber*0.22), (half, rim)]
    tire_points, tire_normals, tire_faces = [], [], []
    for j, (x, r) in enumerate(profile):
        prev, nxt = profile[max(0, j-1)], profile[min(len(profile)-1, j+1)]
        dx, dr = nxt[0]-prev[0], nxt[1]-prev[1]
        length = math.hypot(dx, dr)
        for i in range(segments):
            angle = math.tau * i / segments
            c, s = math.cos(angle), math.sin(angle)
            tire_points.append((x, r*c, r*s))
            tire_normals.append((-dr/length, dx*c/length, dx*s/length))
    for j in range(len(profile)-1):
        for i in range(segments):
            k = (i+1) % segments
            tire_faces.append((j*segments+i, j*segments+k, (j+1)*segments+k, (j+1)*segments+i))

    tread_points, tread_faces = [], []
    lug_count = 24 if radius < 0.65 else 32
    pitch = math.tau / lug_count
    # Separate left/right swept bars form each V, leaving a narrow center gap.
    for row in range(lug_count):
        for side in (-1, 1):
            for section in range(4):
                axial = [side*half*(0.04 + 0.92*(section+k)/4) for k in (0, 1)]
                axial.sort()
                angles = [row*pitch + abs(x)*1.25/radius for x in axial]
                start = len(tread_points)
                for level in (0, 1):
                    spread = pitch * (0.19 if level == 0 else 0.135)
                    for end, edge in ((0, -1), (1, -1), (1, 1), (0, 1)):
                        x = axial[end]
                        # Taper the outer shoulder to meet the rounded sidewall.
                        shoulder = max(0, (abs(x)/half - 0.78)/0.22)
                        r = (radius if level else crown-rubber*0.12) - shoulder*lug_height*0.28
                        angle = angles[end] + edge*spread
                        tread_points.append((x, r*math.cos(angle), r*math.sin(angle)))
                for face in ((0, 1, 2, 3), (4, 7, 6, 5), (0, 4, 5, 1),
                             (1, 5, 6, 2), (2, 6, 7, 3), (3, 7, 4, 0)):
                    tread_faces.append(tuple(start+i for i in face))

    hub_points, hub_faces, hub_uvs = [], [], []
    for side in (-1, 1):
        start = len(hub_points)
        hub_points.append((side*half*0.99, 0, 0))
        hub_uvs.append((0.5, 0.5))
        # Shallow dish with raised axle cap and bead lip; texture supplies bolts.
        rings = [(0.28, 0.99), (0.38, 0.88), (0.72, 0.84), (0.92, 0.94), (1.0, 1.0)]
        for fraction, axial in rings:
            for i in range(segments):
                angle = math.tau*i/segments
                y, z = fraction*math.cos(angle), fraction*math.sin(angle)
                hub_points.append((side*half*axial, rim*y, rim*z))
                hub_uvs.append((0.5 + side*0.485*y, 0.5 + 0.485*z))
        for i in range(segments):
            k = (i+1) % segments
            face = (start, start+1+i, start+1+k)
            hub_faces.append(face if side > 0 else tuple(reversed(face)))
        for ring in range(len(rings)-1):
            a, b = start+1+ring*segments, start+1+(ring+1)*segments
            for i in range(segments):
                k = (i+1) % segments
                face = (a+i, b+i, b+k, a+k)
                hub_faces.append(face if side > 0 else tuple(reversed(face)))
    return {
        "Tire": (tire_points, tire_faces, tire_normals, None),
        "Tread": (tread_points, tread_faces, None, None),
        "Rim": (hub_points, hub_faces, None, hub_uvs),
    }


def add_wheel_appearance(stage, path, radius, width, depth, rubber_material, hub_material):
    from pxr import Gf, Sdf, UsdGeom, UsdShade

    for name, (points, faces, normals, uvs) in wheel_mesh_data(radius, width, depth).items():
        mesh = UsdGeom.Mesh.Define(stage, f"{path}/{name}")
        mesh.CreatePointsAttr(points)
        mesh.CreateFaceVertexCountsAttr([len(face) for face in faces])
        mesh.CreateFaceVertexIndicesAttr([index for face in faces for index in face])
        mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        mesh.CreateExtentAttr([Gf.Vec3f(*(min(p[a] for p in points) for a in range(3))),
                               Gf.Vec3f(*(max(p[a] for p in points) for a in range(3)))])
        if normals:
            mesh.CreateNormalsAttr(normals)
            mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
        if uvs:
            UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
                "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex).Set(uvs)
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(hub_material if name == "Rim" else rubber_material)
