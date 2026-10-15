"""Compose a farm and tractor in Isaac Sim, with optional ROS 2 cameras."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from isaacsim import SimulationApp


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--world", default="debug_out/farms/farm_seed_1_120m2/world/farm.usda"
    )
    parser.add_argument("--tractor", default="debug_out/tractor/tractor.usda")
    parser.add_argument("--config", default="configs/tractor_default.yaml")
    parser.add_argument("--sensor-config", default="configs/sim_sensors.yaml")
    parser.add_argument("--farm-config", default="debug_out/farms/farm_seed_1_120m2/farm.yaml")
    parser.add_argument("--position", nargs=3, type=float, default=(60.0, 60.0, 2.0))
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--ros2", action="store_true")
    parser.add_argument("--max-updates", type=int, default=None, help="stop after this many app updates")
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
    state_streams = None
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

        # The tractor asset contains a scene for convenient manual inspection;
        # the composed farm's /World/PhysicsScene remains authoritative.
        nested_scene = stage.GetPrimAtPath("/World/Tractor/PhysicsScene")
        if nested_scene:
            nested_scene.SetActive(False)

        from bringup.surface_friction import configure_vehicle_surface_friction

        tire_count = configure_vehicle_surface_friction(stage)
        if tire_count:
            print(f"Connected {tire_count} tires to /World/TireFrictionTable")

        if args.ros2:
            from isaacsim.core.utils.extensions import enable_extension

            enable_extension("isaacsim.ros2.bridge")
            app.update()
            from bringup.action_graphs import create_ros2_camera_graphs, create_ros2_clock_graph

            create_ros2_camera_graphs(load_tractor_config(args.config))
            create_ros2_clock_graph()
            from bringup.sim_sensors import TractorStateStreams, load_sensor_config

            sensor_config, reference = load_sensor_config(args.sensor_config, args.farm_config)
            state_streams = TractorStateStreams(stage, sensor_config, reference)

        if not args.headless:
            from bringup.teleop import TractorKeyboardTeleop

            vehicle = stage.GetPrimAtPath("/World/Tractor/base_link")
            teleop = TractorKeyboardTeleop(vehicle)
            print("W/S throttle and reverse, A/D steer, Space brake")

        # Generated assets have no animation range. Give this interactive
        # simulation a usable range so Kit does not loop a single frame.
        stage.SetStartTimeCode(0)
        stage.SetEndTimeCode(stage.GetTimeCodesPerSecond() * 86400)
        timeline = omni.timeline.get_timeline_interface()
        timeline.set_looping(False)
        timeline.play()
        update_count = 0
        while app.is_running():
            if teleop is not None:
                teleop.update()
            app.update()
            update_count += 1
            if args.max_updates is not None and update_count >= args.max_updates:
                break
            if args.frame_report and update_count == 30:
                from bringup.runtime_diagnostic import write_wheel_frame_report

                report = write_wheel_frame_report(
                    stage,
                    "/World/Tractor/base_link",
                    "debug_out/tractor/runtime_frame_report.txt",
                )
                print(f"Runtime wheel-frame report written to {report}")
    except Exception:
        import traceback

        traceback.print_exc()
        raise
    finally:
        if args.max_updates is not None and state_streams is not None:
            truth_count = state_streams.truth.samples if state_streams.truth else 0
            gnss_count = state_streams.gnss.samples if state_streams.gnss else 0
            print(f"State stream samples: ground_truth={truth_count}, gnss={gnss_count}")
        if state_streams is not None:
            state_streams.close()
        if teleop is not None:
            teleop.close()
        app.close()


if __name__ == "__main__":
    main()
