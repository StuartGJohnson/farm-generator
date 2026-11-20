"""Measure short constant-curvature turns on the open field world.

Run one case per freshly spawned tractor after starting tractor_control_node.
The plotted reference is an ideal rear-axle arc at the requested speed and
radius; this deliberately exposes acceleration lag and terrain effects.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from PIL import Image


FIELD_IMAGE = Path("debug_out/farms/open_seed_1_field/gis/orthophoto.tif")
FIELD_WORLD = "debug_out/farms/open_seed_1_field/world/farm.usda"
SETTLE_S = 1.0
STOP_S = 2.0


def angles(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z),
                      1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))
    return roll, pitch, yaw


def ideal_arc(start, radius, speed, duration, count=200):
    x0, y0, yaw0 = start
    points = []
    for i in range(count + 1):
        turn = speed * duration * i / (count * radius)
        points.append((x0 + radius * (math.sin(yaw0 + turn) - math.sin(yaw0)),
                       y0 - radius * (math.cos(yaw0 + turn) - math.cos(yaw0))))
    return points


def summarize(rows, speed, radius, duration):
    drive = [row for row in rows if 0 <= row["drive_s"] < duration]
    steady = [row for row in drive if 3 <= row["drive_s"] < duration - 1
              and math.isfinite(row["actual_speed_mps"])
              and abs(row["actual_speed_mps"]) > 0.15]
    if not steady:
        raise RuntimeError("No steady driving samples")
    actual_curvatures = [row["actual_yaw_rate_rps"] / row["actual_speed_mps"]
                         for row in steady]
    median_curvature = statistics.median(actual_curvatures)
    return {
        "world": FIELD_WORLD,
        "tractor": "debug_out/tractor_route_2m/tractor.usda",
        "commanded_speed_mps": speed,
        "commanded_radius_m": radius,
        "commanded_yaw_rate_rps": speed / radius,
        "turn_duration_s": duration,
        "samples": len(rows),
        "steady_samples": len(steady),
        "steady_speed_mps": statistics.median(row["actual_speed_mps"] for row in steady),
        "steady_yaw_rate_rps": statistics.median(row["actual_yaw_rate_rps"] for row in steady),
        "steady_curvature_inv_m": median_curvature,
        "steady_radius_m": 1 / abs(median_curvature) if median_curvature else math.inf,
        "steady_lateral_speed_mps": statistics.median(row["lateral_speed_mps"] for row in steady),
        "max_abs_roll_deg": max(abs(math.degrees(row["roll_rad"])) for row in drive),
        "max_abs_pitch_deg": max(abs(math.degrees(row["pitch_rad"])) for row in drive),
        "max_speed_mps": max(row["actual_speed_mps"] for row in drive),
    }


def save_plots(output, rows, metrics, start):
    speed = metrics["commanded_speed_mps"]
    radius = metrics["commanded_radius_m"]
    duration = metrics["turn_duration_s"]
    drive = [row for row in rows if row["drive_s"] >= 0]
    planned = ideal_arc(start, radius, speed, duration)
    with Image.open(FIELD_IMAGE) as source:
        photo = source.convert("RGB")
        fig, ax = plt.subplots(figsize=(7, 7))
        ax.imshow(photo, extent=(0, 120, 0, 120), origin="upper")
    ax.plot([p[0] for p in planned], [p[1] for p in planned], color="#1045cb",
            linewidth=2.5, label="ideal commanded arc")
    ax.plot([row["x_m"] for row in drive], [row["y_m"] for row in drive],
            color="#11a956", linewidth=2, label="measured rear axle")
    ax.scatter([start[0]], [start[1]], color="#00c46a", edgecolor="black",
               s=55, zorder=5, label="start")
    ax.scatter([planned[-1][0]], [planned[-1][1]], color="#f24049",
               edgecolor="black", marker="s", s=55, zorder=5, label="ideal end")
    xs = [p[0] for p in planned] + [row["x_m"] for row in drive]
    ys = [p[1] for p in planned] + [row["y_m"] for row in drive]
    ax.set_xlim(min(xs) - 2, max(xs) + 2)
    ax.set_ylim(min(ys) - 2, max(ys) + 2)
    ax.set(xlabel="Map x (m)", ylabel="Map y (m)",
           title=f"Field turn: R={radius:g} m, v={speed:g} m/s")
    ax.set_aspect("equal")
    ax.grid(alpha=0.2)
    ax.legend()
    fig.savefig(output / "plan_view.png", dpi=180, bbox_inches="tight")
    plt.close(fig)

    for stem, x_key, xlabel in (("", "sim_s", "Elapsed simulation time (s)"),
                                ("_vs_distance", "distance_m", "Measured distance (m)")):
        fig, ax = plt.subplots(figsize=(9, 4))
        ax.plot([row[x_key] for row in rows], [row["cmd_speed_mps"] for row in rows],
                label="commanded")
        ax.plot([row[x_key] for row in rows], [row["actual_speed_mps"] for row in rows],
                label="measured")
        ax.set(xlabel=xlabel, ylabel="Rear-axle forward speed (m/s)")
        ax.grid(alpha=0.3)
        ax.legend()
        fig.savefig(output / f"velocity{stem}.png", dpi=150, bbox_inches="tight")
        plt.close(fig)

        fig, ax = plt.subplots(figsize=(9, 4))
        ax.plot([row[x_key] for row in rows], [row["cmd_yaw_rate_rps"] for row in rows],
                label="commanded")
        ax.plot([row[x_key] for row in rows], [row["actual_yaw_rate_rps"] for row in rows],
                label="measured")
        ax.set(xlabel=xlabel, ylabel="Yaw rate (rad/s)")
        ax.grid(alpha=0.3)
        ax.legend()
        fig.savefig(output / f"yaw_rate{stem}.png", dpi=150, bbox_inches="tight")
        plt.close(fig)


def main():
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import Float32MultiArray

    parser = argparse.ArgumentParser()
    parser.add_argument("--speed", type=float, required=True)
    parser.add_argument("--radius", type=float, required=True)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--max-wall-seconds", type=float, default=180.0)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    if min(args.speed, args.radius, args.duration, args.rate) <= 0:
        parser.error("speed, radius, duration, and rate must be positive")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node("field_turn_trial")
    pub = node.create_publisher(Twist, "/cmd_vel", 10)
    state = {"clock": None, "odom": None, "act": [math.nan] * 5,
             "debug": [math.nan] * 8}
    node.create_subscription(Clock, "/clock", lambda msg: state.update(
        clock=msg.clock.sec + msg.clock.nanosec * 1e-9), 10)
    node.create_subscription(Odometry, "/ground_truth/odom",
                             lambda msg: state.update(odom=msg), 10)
    node.create_subscription(Float32MultiArray, "/vehicle/actuation",
                             lambda msg: state.update(act=list(msg.data)), 10)
    node.create_subscription(Float32MultiArray, "/vehicle/control_debug",
                             lambda msg: state.update(debug=list(msg.data)), 10)
    rows = []
    start_sim = None
    last_sim = None
    last_publish_wall = 0.0
    start_pose = None
    distance = 0.0
    started_wall = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - started_wall < args.max_wall_seconds:
            rclpy.spin_once(node, timeout_sec=1 / args.rate)
            wall_now = time.monotonic()
            if wall_now - last_publish_wall < 1 / args.rate:
                continue
            last_publish_wall = wall_now
            now_sim, odom = state["clock"], state["odom"]
            if now_sim is None or odom is None or odom.child_frame_id != "base_link":
                continue
            if start_sim is None:
                start_sim = now_sim
            elapsed = now_sim - start_sim
            if elapsed < 0:
                raise RuntimeError("Simulation clock reset during trial")
            driving = SETTLE_S <= elapsed < SETTLE_S + args.duration
            cmd_speed = args.speed if driving else 0.0
            cmd_yaw = cmd_speed / args.radius
            command = Twist()
            command.linear.x, command.angular.z = cmd_speed, cmd_yaw
            pub.publish(command)
            if last_sim is not None and now_sim <= last_sim:
                continue
            q = odom.pose.pose.orientation
            roll, pitch, yaw = angles(q)
            actual_speed = odom.twist.twist.linear.x
            if last_sim is not None and elapsed >= SETTLE_S:
                distance += max(0.0, actual_speed) * (now_sim - last_sim)
            if start_pose is None and elapsed >= SETTLE_S:
                start_pose = (odom.pose.pose.position.x, odom.pose.pose.position.y, yaw)
            act, debug = state["act"], state["debug"]
            rows.append({"sim_s": elapsed, "drive_s": elapsed - SETTLE_S,
                         "distance_m": distance, "cmd_speed_mps": cmd_speed,
                         "cmd_yaw_rate_rps": cmd_yaw,
                         "actual_speed_mps": actual_speed,
                         "lateral_speed_mps": odom.twist.twist.linear.y,
                         "actual_yaw_rate_rps": odom.twist.twist.angular.z,
                         "x_m": odom.pose.pose.position.x,
                         "y_m": odom.pose.pose.position.y,
                         "yaw_rad": yaw, "roll_rad": roll, "pitch_rad": pitch,
                         "steer_norm": act[3] if len(act) == 5 else math.nan,
                         "accelerator": act[0] if len(act) == 5 else math.nan,
                         "brake": act[1] if len(act) == 5 else math.nan,
                         "ramped_speed_mps": debug[1] if len(debug) == 8 else math.nan})
            last_sim = now_sim
            if elapsed >= SETTLE_S + args.duration + STOP_S:
                break
    finally:
        for _ in range(10):
            pub.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.shutdown()
    if not rows or start_pose is None or rows[-1]["sim_s"] < SETTLE_S + args.duration:
        raise RuntimeError("Field turn did not complete the commanded interval")
    with (args.output_dir / "samples.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    metrics = summarize(rows, args.speed, args.radius, args.duration)
    metrics["start_pose"] = start_pose
    (args.output_dir / "metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    save_plots(args.output_dir, rows, metrics, start_pose)
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
