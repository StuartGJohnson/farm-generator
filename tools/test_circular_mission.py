"""Inject one sampled field arc into mission_executor and measure Nav2 tracking."""

from __future__ import annotations

import argparse
import csv
import json
import math
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from PIL import Image
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from std_msgs.msg import Float32MultiArray

from farm_mission_interfaces.action import ExecuteMission
from route_planning.diagnostic import load_route_json, save_route_json
from route_planning.model import FarmTask, ImplementSide, Pose, Route, RouteSegment


FARM = Path("debug_out/farms/open_seed_1_field")
START = (60.0, 60.0)
DEFAULT_RADIUS_M = 2.0
SWEEP_RAD = 1.5 * math.pi
SPACING_M = 0.1


def make_route(radius_m: float) -> Route:
    count = math.ceil(radius_m * SWEEP_RAD / SPACING_M)
    poses = tuple(Pose(START[0] + radius_m * math.sin(SWEEP_RAD * i / count),
                       START[1] + radius_m * (1 - math.cos(SWEEP_RAD * i / count)),
                       SWEEP_RAD * i / count)
                  for i in range(count + 1))
    distance = sum(math.hypot(b.x - a.x, b.y - a.y)
                   for a, b in zip(poses, poses[1:]))
    task = FarmTask((), ImplementSide.BOTH, poses[0], poses[-1], None)
    segment = RouteSegment("circle_270deg", 0, "headland", None, poses, distance,
                           frozenset(), (1,) * len(poses))
    return Route(task, [segment], distance, round(distance * 1000), "SYNTHETIC",
                 {segment.arc_id: 1}, [])


def prepare_route(path: Path, radius_m: float) -> Route:
    route = make_route(radius_m)
    frame = json.loads((FARM / "gis/map_frame.json").read_text())
    farm = SimpleNamespace(origin_easting_m=frame["origin_easting_m"],
                           origin_northing_m=frame["origin_northing_m"])
    save_route_json(route, farm, path)
    return route


def plots(output: Path, route: Route, rows: list[dict], radius_m: float):
    poses = route.continuous_path()
    fig, ax = plt.subplots(figsize=(7, 7))
    with Image.open(FARM / "gis/orthophoto.tif") as source:
        ax.imshow(source.convert("RGB"), extent=(0, 120, 0, 120), origin="upper")
    ax.plot([p.x for p in poses], [p.y for p in poses], color="#1045cb",
            linewidth=2.5, label="planned rear-axle path")
    ax.plot([r["x_m"] for r in rows], [r["y_m"] for r in rows], color="#11a956",
            linewidth=2, label="measured rear axle")
    ax.scatter([poses[0].x], [poses[0].y], color="#00c46a", edgecolor="black",
               s=55, zorder=5, label="start")
    ax.scatter([poses[-1].x], [poses[-1].y], color="#f24049", edgecolor="black",
               marker="s", s=55, zorder=5, label="goal")
    xs = [p.x for p in poses] + [r["x_m"] for r in rows]
    ys = [p.y for p in poses] + [r["y_m"] for r in rows]
    ax.set_xlim(min(xs) - 2, max(xs) + 2)
    ax.set_ylim(min(ys) - 2, max(ys) + 2)
    ax.set(xlabel="Map x (m)", ylabel="Map y (m)",
           title=f"Nav2 270° field arc, R={radius_m:g} m")
    ax.set_aspect("equal")
    ax.grid(alpha=0.2)
    ax.legend()
    fig.savefig(output / "plan_view.png", dpi=180, bbox_inches="tight")
    plt.close(fig)

    for filename, cmd, actual, label in (
        ("velocity.png", "cmd_speed_mps", "actual_speed_mps", "Forward speed (m/s)"),
        ("yaw_rate.png", "cmd_yaw_rate_rps", "actual_yaw_rate_rps", "Yaw rate (rad/s)"),
    ):
        for xkey, suffix, xlabel in (("sim_s", "", "Simulation time (s)"),
                                    ("station_m", "_vs_distance", "Route station (m)")):
            fig, ax = plt.subplots(figsize=(9, 4))
            ax.plot([r[xkey] for r in rows], [r[cmd] for r in rows], label="Nav2 commanded")
            ax.plot([r[xkey] for r in rows], [r[actual] for r in rows], label="measured")
            ax.set(xlabel=xlabel, ylabel=label)
            ax.grid(alpha=0.3)
            ax.legend()
            fig.savefig(output / filename.replace(".png", f"{suffix}.png"),
                        dpi=150, bbox_inches="tight")
            plt.close(fig)

    moving = [r for r in rows if r["cmd_speed_mps"] > 0.1
              and r["actual_speed_mps"] > 0.1]
    fig, ax = plt.subplots(figsize=(9, 4))
    requested = [r["cmd_yaw_rate_rps"] / r["cmd_speed_mps"] for r in moving]
    ax.plot([r["station_m"] for r in moving], requested,
            label="RPP requested")
    ax.plot([r["station_m"] for r in moving],
            [min(0.5, max(-0.5, value)) for value in requested],
            label="adapter steering limit")
    ax.plot([r["station_m"] for r in moving],
            [r["actual_yaw_rate_rps"] / r["actual_speed_mps"] for r in moving],
            label="measured")
    ax.set(xlabel="Route station (m)", ylabel="Curvature (1/m)")
    ax.grid(alpha=0.3)
    ax.legend()
    fig.savefig(output / "curvature_vs_distance.png", dpi=150, bbox_inches="tight")
    plt.close(fig)

    if "actuator_throttle" in rows[0]:
        peak = max(rows, key=lambda row: row["actual_speed_mps"])
        fig, axes = plt.subplots(5, 1, figsize=(10, 11), sharex=True,
                                 constrained_layout=True)
        times = [r["sim_s"] for r in rows]
        axes[0].plot(times, [r["cmd_speed_mps"] for r in rows], label="Nav2 target")
        axes[0].plot(times, [r["ramped_target_mps"] for r in rows], label="adapter setpoint")
        axes[0].plot(times, [r["actual_speed_mps"] for r in rows], label="actual")
        axes[0].set_ylabel("Speed (m/s)")
        axes[0].legend(loc="best")
        axes[1].plot(times, [r["actuator_throttle"] for r in rows])
        axes[1].set_ylabel("Throttle fraction")
        axes[2].plot(times, [r["actuator_brake"] for r in rows], color="#d97800")
        axes[2].set_ylabel("Brake fraction")
        axes[3].plot(times, [r["desired_accel_mps2"] for r in rows])
        axes[3].set_ylabel("Requested accel. (m/s²)")
        axes[4].plot(times, [r["pi_integral"] for r in rows])
        axes[4].set(xlabel="Simulation time (s)", ylabel="PI integral")
        for ax in axes:
            ax.axvline(peak["sim_s"], color="#d52e2e", linestyle="--", linewidth=1,
                       label=f"speed peak, {peak['sim_s']:.1f} s")
            ax.grid(alpha=0.2)
        driving_times = [r["sim_s"] for r in rows if r["cmd_speed_mps"] > 0.1]
        if driving_times[-1] - driving_times[0] > 4:
            left, right = driving_times[0] + 2, driving_times[-1] - 1
            axes[-1].set_xlim(left, right)
            brake_values = [r["actuator_brake"] for r in rows
                            if left <= r["sim_s"] <= right
                            and math.isfinite(r["actuator_brake"])]
            if brake_values and max(brake_values) > 0:
                axes[2].set_ylim(0, max(brake_values) * 1.15)
        fig.savefig(output / "control_effort.png", dpi=170)
        plt.close(fig)


def tracking_metrics(rows: list[dict], radius_m: float) -> dict:
    import statistics

    drive = [r for r in rows if 0.5 <= r["station_m"] <= radius_m * SWEEP_RAD - 0.5
             and r["cmd_speed_mps"] > 0.1 and r["actual_speed_mps"] > 0.1]
    if not drive:
        return {}
    radial_errors = sorted(abs(math.hypot(r["x_m"] - START[0],
                                           r["y_m"] - (START[1] + radius_m)) - radius_m)
                           for r in drive)
    requested = [r["cmd_yaw_rate_rps"] / r["cmd_speed_mps"] for r in drive]
    return {
        "tracking_samples": len(drive),
        "median_abs_radial_error_m": statistics.median(radial_errors),
        "p95_abs_radial_error_m": radial_errors[int(0.95 * (len(radial_errors) - 1))],
        "max_abs_radial_error_m": radial_errors[-1],
        "median_speed_mps": statistics.median(r["actual_speed_mps"] for r in drive),
        "min_speed_mps": min(r["actual_speed_mps"] for r in drive),
        "max_speed_mps": max(r["actual_speed_mps"] for r in drive),
        "median_requested_curvature_inv_m": statistics.median(requested),
        "fraction_requested_above_steering_limit": sum(abs(v) > 0.5 for v in requested) / len(requested),
        "median_measured_curvature_inv_m": statistics.median(
            r["actual_yaw_rate_rps"] / r["actual_speed_mps"] for r in drive),
    }


def surface_context_plots(output: Path, route: Route, rows: list[dict], tractor_config_path: str):
    """Compare the driven arc with generated friction and terrain height."""
    import numpy as np
    from generation.orchestrator import load_farm
    from generation.surface_friction import sample_field
    from generation.surface_height import sample_height
    from tractor_generation.config import load_tractor_config
    import matplotlib.patheffects as path_effects

    scene, _ = load_farm(str(FARM / "farm.yaml"))
    friction = scene.surface_friction.crop
    height = scene.surface_height
    tractor = load_tractor_config(tractor_config_path)
    planned = route.continuous_path()
    peak = max(rows, key=lambda row: row["actual_speed_mps"])
    at_eight = min(rows, key=lambda row: abs(row["sim_s"] - 8.0))
    xs = [p.x for p in planned] + [r["x_m"] for r in rows]
    ys = [p.y for p in planned] + [r["y_m"] for r in rows]
    xmin, xmax = min(xs) - 2.0, max(xs) + 2.0
    ymin, ymax = min(ys) - 2.0, max(ys) + 2.0
    grid_x = np.linspace(xmin, xmax, 180)
    grid_y = np.linspace(ymin, ymax, 180)
    xx, yy = np.meshgrid(grid_x, grid_y)
    grid_xy = np.column_stack((xx.ravel(), yy.ravel()))
    friction_grid = sample_field(friction, grid_xy).reshape(xx.shape)
    height_grid = sample_height(height, grid_xy).reshape(xx.shape)

    fig, axes = plt.subplots(1, 2, figsize=(13, 6), constrained_layout=True)
    outline = [path_effects.Stroke(linewidth=4, foreground="white"), path_effects.Normal()]
    for ax, data, cmap, limits, title, colorbar in (
        (axes[0], friction_grid, "viridis", (friction.friction_min, friction.friction_max),
         "Generated field friction", "Friction coefficient μ"),
        (axes[1], height_grid, "terrain", (height.height_min, height.height_max),
         "Generated ground height", "Height offset (m)"),
    ):
        colors = ax.pcolormesh(xx, yy, data, shading="auto", cmap=cmap,
                               vmin=limits[0], vmax=limits[1], rasterized=True)
        fig.colorbar(colors, ax=ax, label=colorbar, shrink=0.82)
        ax.plot([p.x for p in planned], [p.y for p in planned], color="#1552dc",
                linewidth=2.1, label="planned rear axle", path_effects=outline)
        ax.plot([r["x_m"] for r in rows], [r["y_m"] for r in rows], color="#14a34a",
                linewidth=1.6, label="measured rear axle", path_effects=outline)
        ax.scatter(at_eight["x_m"], at_eight["y_m"], marker="D", s=70,
                   color="#eebd26", edgecolor="black", zorder=6, label="t = 8 s")
        ax.scatter(peak["x_m"], peak["y_m"], marker="*", s=145,
                   color="#fa4545", edgecolor="black", zorder=6,
                   label=f"speed peak, t = {peak['sim_s']:.1f} s")
        ax.set(xlim=(xmin, xmax), ylim=(ymin, ymax), xlabel="Map x (m)",
               ylabel="Map y (m)", title=title)
        ax.set_aspect("equal")
        ax.grid(alpha=0.15)
    axes[0].legend(loc="lower left", fontsize=8)
    fig.suptitle("Circular mission: generated surface fields and measured trajectory")
    fig.savefig(output / "surface_plan_view.png", dpi=180)
    plt.close(fig)

    local_wheels = np.array(((0, tractor.rear_track_width / 2),
                             (0, -tractor.rear_track_width / 2),
                             (tractor.wheelbase, tractor.front_track_width / 2),
                             (tractor.wheelbase, -tractor.front_track_width / 2)))
    positions = np.array([(r["x_m"], r["y_m"]) for r in rows])
    yaw = np.array([r["yaw_rad"] for r in rows])
    forward = np.column_stack((np.cos(yaw), np.sin(yaw)))
    left = np.column_stack((-np.sin(yaw), np.cos(yaw)))
    wheel_xy = (positions[:, None, :] + forward[:, None, :] * local_wheels[None, :, 0, None]
                + left[:, None, :] * local_wheels[None, :, 1, None])
    wheel_mu = sample_field(friction, wheel_xy.reshape(-1, 2)).reshape(-1, 4)
    grade = (sample_height(height, positions + 0.5 * forward)
             - sample_height(height, positions - 0.5 * forward))
    times = np.array([r["sim_s"] for r in rows])
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True, constrained_layout=True)
    axes[0].plot(times, [r["cmd_speed_mps"] for r in rows], label="Nav2 command")
    axes[0].plot(times, [r["actual_speed_mps"] for r in rows], label="actual")
    axes[0].set_ylabel("Speed (m/s)")
    axes[0].legend(loc="best")
    axes[1].fill_between(times, wheel_mu.min(axis=1), wheel_mu.max(axis=1),
                         color="#6b61a8", alpha=0.25, label="four wheel locations")
    axes[1].plot(times, wheel_mu.mean(axis=1), color="#5a358b", label="wheel-location mean")
    axes[1].set_ylabel("Sampled friction μ")
    axes[1].legend(loc="best")
    axes[2].plot(times, grade * 100, color="#a3592e")
    axes[2].axhline(0, color="gray", linewidth=0.8)
    axes[2].set(xlabel="Simulation time (s)", ylabel="Grade along heading (%)")
    for ax in axes:
        ax.axvline(at_eight["sim_s"], color="#b08915", linestyle=":", linewidth=1.2)
        ax.axvline(peak["sim_s"], color="#d52e2e", linestyle="--", linewidth=1.2)
        ax.grid(alpha=0.2)
    fig.suptitle("Field surface samples (no wheel-slip measurement)")
    fig.savefig(output / "surface_vs_time.png", dpi=170)
    plt.close(fig)

    def marker(row):
        index = rows.index(row)
        return {"sim_s": row["sim_s"], "station_m": row["station_m"],
                "speed_mps": row["actual_speed_mps"], "command_mps": row["cmd_speed_mps"],
                "rear_axle_xy_m": [row["x_m"], row["y_m"]],
                "wheel_friction_samples": wheel_mu[index].tolist(),
                "grade_along_heading_pct": float(grade[index] * 100)}
    (output / "surface_context.json").write_text(json.dumps({
        "field_friction_range": [friction.friction_min, friction.friction_max],
        "field_height_range_m": [height.height_min, height.height_max],
        "wheel_order": ["rear_left", "rear_right", "front_left", "front_right"],
        "time_8s": marker(at_eight), "speed_peak": marker(peak),
        "note": "Bilinear samples of generated fields at approximate wheel centers; "
                "USD mesh friction is face-quantized and wheel slip was not measured.",
    }, indent=2) + "\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--radius", type=float, default=DEFAULT_RADIUS_M)
    parser.add_argument("--output-dir", type=Path, default=Path("debug_out/circular_mission_2m"))
    parser.add_argument("--prepare-only", action="store_true")
    parser.add_argument("--plot-only", action="store_true")
    parser.add_argument("--max-wall-seconds", type=float, default=180)
    parser.add_argument("--tractor-config", default="configs/tractor_route_2m.yaml")
    args = parser.parse_args()
    if not math.isfinite(args.radius) or args.radius <= 0:
        parser.error("radius must be positive and finite")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    result_path = args.output_dir / "result.json"
    if args.plot_only:
        result = json.loads(result_path.read_text())
        args.radius = result["radius_m"]
        route = load_route_json(args.output_dir / "route.json")
    else:
        route = prepare_route(args.output_dir / "route.json", args.radius)
    if args.prepare_only:
        print(f"Prepared {len(route.continuous_path())} poses over {route.total_distance_m:.3f} m")
        return
    if args.plot_only:
        with (args.output_dir / "samples.csv").open(newline="") as stream:
            rows = [{key: float(value) for key, value in row.items()}
                    for row in csv.DictReader(stream)]
        result.update(tracking_metrics(rows, args.radius))
        plots(args.output_dir, route, rows, args.radius)
        surface_context_plots(args.output_dir, route, rows, args.tractor_config)
        result_path.write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result, indent=2))
        return

    rclpy.init()
    node = rclpy.create_node("circular_mission_trial", parameter_overrides=[
        Parameter("use_sim_time", value=True)])
    state = {"cmd_speed_mps": 0.0, "cmd_yaw_rate_rps": 0.0,
             "station_m": 0.0, "section": 0,
             "carrot_x_m": math.nan, "carrot_y_m": math.nan,
             "actuator_throttle": math.nan, "actuator_brake": math.nan,
             "ramped_target_mps": math.nan, "desired_accel_mps2": math.nan,
             "pi_integral": math.nan}
    rows = []

    def on_cmd(msg):
        state["cmd_speed_mps"] = msg.linear.x
        state["cmd_yaw_rate_rps"] = msg.angular.z

    def on_actuation(msg):
        if len(msg.data) == 5:
            state["actuator_throttle"] = msg.data[0]
            state["actuator_brake"] = msg.data[1]

    def on_control_debug(msg):
        if len(msg.data) == 8:
            state["ramped_target_mps"] = msg.data[1]
            state["desired_accel_mps2"] = msg.data[3]
            state["pi_integral"] = msg.data[6]

    def on_odom(msg):
        q = msg.pose.pose.orientation
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z),
                          1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        rows.append({"sim_s": stamp, "station_m": state["station_m"],
                     "section": state["section"],
                     "cmd_speed_mps": state["cmd_speed_mps"],
                     "cmd_yaw_rate_rps": state["cmd_yaw_rate_rps"],
                     "actuator_throttle": state["actuator_throttle"],
                     "actuator_brake": state["actuator_brake"],
                     "ramped_target_mps": state["ramped_target_mps"],
                     "desired_accel_mps2": state["desired_accel_mps2"],
                     "pi_integral": state["pi_integral"],
                     "actual_speed_mps": msg.twist.twist.linear.x,
                     "actual_yaw_rate_rps": msg.twist.twist.angular.z,
                     "lateral_speed_mps": msg.twist.twist.linear.y,
                     "x_m": msg.pose.pose.position.x, "y_m": msg.pose.pose.position.y,
                     "z_m": msg.pose.pose.position.z,
                     "qx": q.x, "qy": q.y, "qz": q.z, "qw": q.w,
                     "roll_rad": roll, "pitch_rad": pitch, "yaw_rad": yaw,
                     "carrot_x_m": state["carrot_x_m"],
                     "carrot_y_m": state["carrot_y_m"]})

    def on_carrot(msg):
        state["carrot_x_m"] = msg.point.x
        state["carrot_y_m"] = msg.point.y

    def on_feedback(msg):
        state["station_m"] = msg.feedback.distance_traveled_m
        state["section"] = msg.feedback.section_index

    node.create_subscription(Twist, "/cmd_vel", on_cmd, 10)
    node.create_subscription(Float32MultiArray, "/vehicle/actuation", on_actuation, 10)
    node.create_subscription(Float32MultiArray, "/vehicle/control_debug", on_control_debug, 10)
    node.create_subscription(PointStamped, "/lookahead_point", on_carrot, 10)
    node.create_subscription(Odometry, "/odom", on_odom, 10)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    client = ActionClient(node, ExecuteMission, "mission_executor")

    def wait(future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.1)
        if not future.done():
            raise TimeoutError("Mission action timed out")
        return future.result()

    result = {"route_length_m": route.total_distance_m, "pose_count": len(route.continuous_path()),
              "radius_m": args.radius, "sweep_deg": math.degrees(SWEEP_RAD),
              "path_spacing_m": route.total_distance_m / (len(route.continuous_path()) - 1),
              "success": False}
    handle = None
    try:
        if not client.wait_for_server(timeout_sec=60):
            raise RuntimeError("mission_executor unavailable")
        ready_by = time.monotonic() + 90
        while len(rows) < 10 and time.monotonic() < ready_by:
            time.sleep(0.2)
        if len(rows) < 10:
            raise RuntimeError("No tractor odometry received")
        time.sleep(2)
        handle = wait(client.send_goal_async(ExecuteMission.Goal(mission_id="injected"),
                                             feedback_callback=on_feedback), 10)
        if not handle.accepted:
            raise RuntimeError("Injected mission rejected")
        outcome = wait(handle.get_result_async(), args.max_wall_seconds)
        result["action_status"] = outcome.status
        result["message"] = outcome.result.message
        result["success"] = outcome.status == GoalStatus.STATUS_SUCCEEDED and outcome.result.success
    except Exception as exc:
        result["error"] = str(exc)
        if handle is not None:
            wait(handle.cancel_goal_async(), 10)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=5)
        if rows:
            t0 = rows[0]["sim_s"]
            for row in rows:
                row["sim_s"] -= t0
            with (args.output_dir / "samples.csv").open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
                writer.writeheader()
                writer.writerows(rows)
            plots(args.output_dir, route, rows, args.radius)
            surface_context_plots(args.output_dir, route, rows, args.tractor_config)
            result.update(tracking_metrics(rows, args.radius))
            result["final_station_m"] = rows[-1]["station_m"]
            result["final_goal_error_m"] = math.hypot(rows[-1]["x_m"] - route.task.end.x,
                                                       rows[-1]["y_m"] - route.task.end.y)
        result["samples"] = len(rows)
        (args.output_dir / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    if not result["success"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
