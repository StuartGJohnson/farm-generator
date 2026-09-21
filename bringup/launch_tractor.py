"""Compose a farm and tractor in Isaac Sim, with optional ROS 2 cameras."""

from __future__ import annotations

import argparse
from pathlib import Path

from isaacsim import SimulationApp


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--world", default="debug_out/mesh/farm_seed_1_120m2_ground.usda"
    )
    parser.add_argument("--tractor", default="debug_out/tractor/tractor.usda")
    parser.add_argument("--config", default="configs/tractor_default.yaml")
    parser.add_argument("--position", nargs=3, type=float, default=(60.0, 60.0, 2.0))
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--ros2", action="store_true")
    parser.add_argument(
        "--frame-report",
        action="store_true",
        help="write managed wheel axes after the simulation has initialized",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    app = SimulationApp({"headless": args.headless})
    teleop = None
    try:
        import omni.timeline
        import omni.usd
        from pxr import Gf, UsdGeom

        from tractor_generation.config import load_tractor_config

        context = omni.usd.get_context()
        context.open_stage(str(Path(args.world).resolve()))
        app.update()
        stage = context.get_stage()
        tractor = stage.DefinePrim("/World/Tractor", "Xform")
        tractor.GetReferences().AddReference(str(Path(args.tractor).resolve()))
        UsdGeom.Xformable(tractor).AddTranslateOp().Set(Gf.Vec3d(*args.position))
        app.update()

        # The tractor asset contains a scene for convenient manual inspection;
        # the composed farm's /World/PhysicsScene remains authoritative.
        nested_scene = stage.GetPrimAtPath("/World/Tractor/PhysicsScene")
        if nested_scene:
            nested_scene.SetActive(False)

        if args.ros2:
            from isaacsim.core.utils.extensions import enable_extension

            enable_extension("isaacsim.ros2.bridge")
            app.update()
            from bringup.action_graphs import create_ros2_camera_graphs

            create_ros2_camera_graphs(load_tractor_config(args.config))

        if not args.headless:
            from bringup.teleop import TractorKeyboardTeleop

            vehicle = stage.GetPrimAtPath("/World/Tractor/base_link")
            teleop = TractorKeyboardTeleop(vehicle)
            print("W/S throttle and reverse, A/D steer, Space brake")

        omni.timeline.get_timeline_interface().play()
        update_count = 0
        while app.is_running():
            if teleop is not None:
                teleop.update()
            app.update()
            update_count += 1
            if args.frame_report and update_count == 30:
                from bringup.runtime_diagnostic import write_wheel_frame_report

                report = write_wheel_frame_report(
                    stage,
                    "/World/Tractor/base_link",
                    "debug_out/tractor/runtime_frame_report.txt",
                )
                print(f"Runtime wheel-frame report written to {report}")
    finally:
        if teleop is not None:
            teleop.close()
        app.close()


if __name__ == "__main__":
    main()
