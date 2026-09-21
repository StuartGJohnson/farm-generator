"""Ask PhysX Vehicle to derive the tractor suspension frames in an open stage.

Run this file from Isaac Sim's Script Editor while the farm and tractor are
loaded and the timeline is stopped::

    exec(open("/home/sjohnson/farm-generator/examples/autocompute_tractor_frames.py").read())

The changes are authored as overrides in the open stage only.  The script does
not save the stage or start the timeline.  It also writes an exact report to
``debug_out/tractor/vehicle_frame_report.txt`` so the values do not need to be
copied from the Script Editor output.
"""

from pathlib import Path

import omni.physxvehicle
import omni.timeline
import omni.usd
from pxr import UsdGeom


VEHICLE_PATH_CANDIDATES = (
    "/World/tractor/base_link",
    "/World/Tractor/base_link",
)
WHEEL_NAMES = (
    "front_left_wheel",
    "front_right_wheel",
    "rear_left_wheel",
    "rear_right_wheel",
)
REPORT_PATH = Path(
    "/home/sjohnson/farm-generator/debug_out/tractor/vehicle_frame_report.txt"
)


def _attribute_value(prim, name):
    attribute = prim.GetAttribute(name)
    return attribute.Get() if attribute else "<attribute absent>"


def _describe_wheel(stage, wheel_path):
    prim = stage.GetPrimAtPath(wheel_path)
    if not prim.IsValid():
        return [f"{wheel_path}: MISSING"]

    xformable = UsdGeom.Xformable(prim)
    transform_result = xformable.GetLocalTransformation()
    # USD releases differ here: older Python bindings returned
    # (matrix, resetsXformStack), while USD 25.08 returns only the matrix.
    if isinstance(transform_result, tuple):
        local_transform = transform_result[0]
        resets_stack = transform_result[1] if len(transform_result) > 1 else None
    else:
        local_transform = transform_result
        resets_stack = xformable.GetResetXformStack()
    return [
        wheel_path,
        f"  xformOpOrder = {_attribute_value(prim, 'xformOpOrder')}",
        f"  xformOp:translate = {_attribute_value(prim, 'xformOp:translate')}",
        f"  xformOp:orient = {_attribute_value(prim, 'xformOp:orient')}",
        f"  local transform = {local_transform}",
        f"  resets xform stack = {resets_stack}",
        "  suspension frame position = "
        f"{_attribute_value(prim, 'physxVehicleWheelAttachment:suspensionFramePosition')}",
        "  suspension frame orientation = "
        f"{_attribute_value(prim, 'physxVehicleWheelAttachment:suspensionFrameOrientation')}",
    ]


timeline = omni.timeline.get_timeline_interface()
if timeline.is_playing():
    raise RuntimeError("Stop the Isaac Sim timeline before running this diagnostic")

stage = omni.usd.get_context().get_stage()
if stage is None:
    raise RuntimeError("No USD stage is open")
vehicle_path = next(
    (path for path in VEHICLE_PATH_CANDIDATES if stage.GetPrimAtPath(path).IsValid()),
    None,
)
if vehicle_path is None:
    raise RuntimeError(
        "Vehicle not found. Expected /World/tractor/base_link "
        "or /World/Tractor/base_link."
    )

lines = ["BEFORE PHYSX AUTOCOMPUTE"]
for wheel_name in WHEEL_NAMES:
    lines.extend(_describe_wheel(stage, f"{vehicle_path}/{wheel_name}"))

vehicle_interface = omni.physxvehicle.get_physx_vehicle_interface()
result = vehicle_interface.compute_suspension_frame_transforms(vehicle_path)
lines.extend(["", f"AUTOCOMPUTE RESULT = {result}", "", "AFTER PHYSX AUTOCOMPUTE"])

for wheel_name in WHEEL_NAMES:
    lines.extend(_describe_wheel(stage, f"{vehicle_path}/{wheel_name}"))

REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
REPORT_PATH.write_text("\n".join(lines) + "\n", encoding="utf-8")

print(f"PhysX suspension-frame autocompute returned {result}")
print(f"Full report written to {REPORT_PATH}")
print("The stage was not saved. Press Play now to test whether the wheels remain fixed.")
