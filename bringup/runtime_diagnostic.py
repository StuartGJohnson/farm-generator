"""Small runtime diagnostics for an active Isaac Sim tractor stage."""

from pathlib import Path


def write_wheel_frame_report(stage, vehicle_path: str, output_path: str | Path) -> Path:
    """Write rendered wheel axle directions in the moving tractor frame."""
    from pxr import Gf, Usd, UsdGeom

    def normalized(vector):
        vector = Gf.Vec3d(vector)
        return vector.GetNormalized() if vector.GetLength() else vector

    def world_direction(matrix, local_direction):
        return normalized(matrix.TransformDir(Gf.Vec3d(local_direction)))

    time = Usd.TimeCode.Default()
    vehicle_prim = stage.GetPrimAtPath(vehicle_path)
    if not vehicle_prim.IsValid():
        raise RuntimeError(f"Vehicle not found at {vehicle_path}")
    vehicle_matrix = UsdGeom.Xformable(vehicle_prim).ComputeLocalToWorldTransform(time)
    tractor_axes = (
        world_direction(vehicle_matrix, (1, 0, 0)),
        world_direction(vehicle_matrix, (0, 1, 0)),
        world_direction(vehicle_matrix, (0, 0, 1)),
    )
    lines = [
        f"vehicle = {vehicle_path}",
        f"tractor world forward (+X) = {tractor_axes[0]}",
        f"tractor world lateral (+Y) = {tractor_axes[1]}",
        f"tractor world up (+Z) = {tractor_axes[2]}",
    ]
    axis_vectors = {
        "X": Gf.Vec3d(1, 0, 0),
        "Y": Gf.Vec3d(0, 1, 0),
        "Z": Gf.Vec3d(0, 0, 1),
    }
    wheel_names = (
        "front_left_wheel",
        "front_right_wheel",
        "rear_left_wheel",
        "rear_right_wheel",
    )
    for wheel_name in wheel_names:
        wheel_path = f"{vehicle_path}/{wheel_name}"
        wheel_prim = stage.GetPrimAtPath(wheel_path)
        render_prim = stage.GetPrimAtPath(f"{wheel_path}/Render")
        axis_token = str(UsdGeom.Cylinder(render_prim).GetAxisAttr().Get()).upper()
        render_matrix = UsdGeom.Xformable(render_prim).ComputeLocalToWorldTransform(time)
        axle_world = world_direction(render_matrix, axis_vectors[axis_token])
        dots = tuple(axle_world.GetDot(axis) for axis in tractor_axes)
        lines.extend(
            [
                "",
                wheel_path,
                f"  managed local transform = {UsdGeom.Xformable(wheel_prim).GetLocalTransformation()}",
                f"  cylinder authored axis = {axis_token}",
                f"  rendered axle world direction = {axle_world}",
                "  rendered axle in tractor (forward, lateral, up) = "
                f"({dots[0]:.9f}, {dots[1]:.9f}, {dots[2]:.9f})",
            ]
        )

    output = Path(output_path).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return output
