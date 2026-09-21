"""Report managed Vehicle 2 wheel axes from an open, running Isaac Sim stage.

Run from Isaac Sim's Script Editor after pressing Play (and optionally after a
short throttle input)::

    exec(open("/home/sjohnson/farm-generator/examples/inspect_tractor_runtime_frames.py").read())

This script only reads the stage.  It does not modify or save it.
"""

from pathlib import Path

import omni.usd
from pxr import Gf, Usd, UsdGeom


VEHICLE_PATH_CANDIDATES = (
    "/World/Tractor/base_link",
    "/World/tractor/base_link",
)
WHEEL_NAMES = (
    "front_left_wheel",
    "front_right_wheel",
    "rear_left_wheel",
    "rear_right_wheel",
)
REPORT_PATH = Path(
    "/home/sjohnson/farm-generator/debug_out/tractor/runtime_frame_report.txt"
)


def _normalized(vector):
    vector = Gf.Vec3d(vector)
    return vector.GetNormalized() if vector.GetLength() else vector


def _world_direction(matrix, local_direction):
    return _normalized(matrix.TransformDir(Gf.Vec3d(local_direction)))


stage = omni.usd.get_context().get_stage()
if stage is None:
    raise RuntimeError("No USD stage is open")

vehicle_path = next(
    (path for path in VEHICLE_PATH_CANDIDATES if stage.GetPrimAtPath(path).IsValid()),
    None,
)
if vehicle_path is None:
    raise RuntimeError("Could not find the tractor base_link")

time = Usd.TimeCode.Default()
vehicle_prim = stage.GetPrimAtPath(vehicle_path)
vehicle_matrix = UsdGeom.Xformable(vehicle_prim).ComputeLocalToWorldTransform(time)
tractor_forward = _world_direction(vehicle_matrix, (1, 0, 0))
tractor_lateral = _world_direction(vehicle_matrix, (0, 1, 0))
tractor_up = _world_direction(vehicle_matrix, (0, 0, 1))

lines = [
    f"vehicle = {vehicle_path}",
    f"tractor world forward (+X) = {tractor_forward}",
    f"tractor world lateral (+Y) = {tractor_lateral}",
    f"tractor world up (+Z) = {tractor_up}",
]

axis_vectors = {
    "X": Gf.Vec3d(1, 0, 0),
    "Y": Gf.Vec3d(0, 1, 0),
    "Z": Gf.Vec3d(0, 0, 1),
}

for wheel_name in WHEEL_NAMES:
    wheel_path = f"{vehicle_path}/{wheel_name}"
    wheel_prim = stage.GetPrimAtPath(wheel_path)
    render_prim = stage.GetPrimAtPath(f"{wheel_path}/Render")
    if not wheel_prim.IsValid() or not render_prim.IsValid():
        lines.extend(["", f"{wheel_path}: MISSING"])
        continue

    wheel_matrix = UsdGeom.Xformable(wheel_prim).ComputeLocalToWorldTransform(time)
    render_matrix = UsdGeom.Xformable(render_prim).ComputeLocalToWorldTransform(time)
    axis_token = str(UsdGeom.Cylinder(render_prim).GetAxisAttr().Get()).upper()
    axle_world = _world_direction(render_matrix, axis_vectors[axis_token])
    dots = (
        axle_world.GetDot(tractor_forward),
        axle_world.GetDot(tractor_lateral),
        axle_world.GetDot(tractor_up),
    )
    lines.extend(
        [
            "",
            wheel_path,
            f"  managed local transform = {UsdGeom.Xformable(wheel_prim).GetLocalTransformation()}",
            f"  managed world transform = {wheel_matrix}",
            f"  cylinder authored axis = {axis_token}",
            f"  rendered axle world direction = {axle_world}",
            "  rendered axle in tractor (forward, lateral, up) = "
            f"({dots[0]:.9f}, {dots[1]:.9f}, {dots[2]:.9f})",
        ]
    )

REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
REPORT_PATH.write_text("\n".join(lines) + "\n", encoding="utf-8")
print(f"Runtime wheel-frame report written to {REPORT_PATH}")
