"""Reproducible, bounded open-loop drivetrain trials for one surface.

Run with system ROS 2 Humble, while the C++ controller is stopped and Isaac's
ROS actuator adapter is selected. Saves raw samples and a trial manifest.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
from pathlib import Path


def run(args):
    import rclpy
    from rclpy.parameter import Parameter
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import Float32MultiArray

    args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node("tractor_open_loop_calibration", parameter_overrides=[
        Parameter("use_sim_time", value=True)])
    pub = node.create_publisher(Float32MultiArray, "/vehicle/actuation", 10)
    state = {"sim": None, "speed": None, "odom_wall": None, "x": None, "y": None}
    def clock_cb(msg):
        state["sim"] = msg.clock.sec + msg.clock.nanosec * 1e-9
    def odom_cb(msg):
        if msg.child_frame_id == "base_link" and math.isfinite(msg.twist.twist.linear.x):
            state.update(speed=msg.twist.twist.linear.x, odom_wall=time.monotonic(),
                         x=msg.pose.pose.position.x, y=msg.pose.pose.position.y)
    clock_sub = node.create_subscription(Clock, "/clock", clock_cb, 10)
    odom_sub = node.create_subscription(Odometry, "/ground_truth/odom", odom_cb, 10)
    _ = (clock_sub, odom_sub)
    def send(a, b, gear):
        msg = Float32MultiArray()
        msg.data = [float(a), float(b), 0.0, 0.0, float(gear)]
        pub.publish(msg)

    # Each segment ends on time or speed, whichever arrives first. Holding
    # brake precedes every gear transition; the Isaac adapter guards it too.
    phases = [("initial_hold", 1, 0, 0.35, 1.0, None)]
    for gear in (1, -1):
        phases.append(("shift_hold", gear, 0, 0.35, 1.0, None))
        for level in (0.06, 0.12, 0.20):
            tag = f"throttle_{level:.2f}"
            phases.extend(((tag, gear, level, 0, 3.5, ("above", 1.4)),
                           (tag + "_coast", gear, 0, 0, 1.2, None),
                           (tag + "_brake", gear, 0, 0.08, 3.0, ("below", 0.12)),
                           (tag + "_hold", gear, 0, 0.35, 0.8, None)))
        for level in (0.03, 0.06, 0.10, 0.16):
            tag = f"brake_{level:.2f}"
            phases.extend(((tag + "_approach", gear, 0.16, 0, 4.0, ("above", 1.3)),
                           (tag + "_coast", gear, 0, 0, 0.5, None),
                           (tag, gear, 0, level, 4.0, ("below", 0.12)),
                           (tag + "_hold", gear, 0, 0.35, 0.8, None)))
    phases.append(("final_hold", -1, 0, 0.35, 2.0, None))
    rows = []
    phase_index = 0
    phase_start = None
    first_sim = None
    last_sim = None
    start_xy = None
    start_wall = time.monotonic()
    last_publish_sim = None
    last_publisher_check = 0.0
    outcome = "completed"
    try:
        while rclpy.ok() and phase_index < len(phases):
            rclpy.spin_once(node, timeout_sec=0.01)
            wall = time.monotonic()
            sim = state["sim"]
            if sim is None or state["odom_wall"] is None:
                if wall - start_wall > 120:
                    raise RuntimeError("No /clock and body-frame /ground_truth/odom")
                continue
            if wall - state["odom_wall"] > 0.5 or wall - start_wall > args.max_wall_seconds:
                raise RuntimeError("Stale odometry or trial wall-time limit")
            if last_sim is not None and sim < last_sim:
                raise RuntimeError("Simulation time reset during calibration")
            last_sim = sim
            if last_publish_sim is not None and sim - last_publish_sim < 1 / args.rate - 1e-9:
                continue
            last_publish_sim = sim
            if first_sim is None:
                first_sim = sim
                start_xy = state["x"], state["y"]
            if wall - last_publisher_check > 1.0:
                last_publisher_check = wall
                if len(node.get_publishers_info_by_topic("/vehicle/actuation")) > 1:
                    raise RuntimeError("Another /vehicle/actuation publisher is active")
            if abs(state["speed"]) > args.max_speed:
                raise RuntimeError("Speed safety limit reached")
            if math.hypot(state["x"] - start_xy[0], state["y"] - start_xy[1]) > args.max_distance:
                raise RuntimeError("Distance safety limit reached")
            name, gear, accel, brake, max_time, end_speed = phases[phase_index]
            if phase_start is None:
                phase_start = sim
            elapsed = sim - phase_start
            reached = (end_speed is not None and
                       ((end_speed[0] == "above" and abs(state["speed"]) >= end_speed[1]) or
                        (end_speed[0] == "below" and abs(state["speed"]) <= end_speed[1])))
            if elapsed >= max_time or reached:
                if name == "shift_hold" and abs(state["speed"]) > 0.08:
                    raise RuntimeError("Vehicle did not stop before reverse shift")
                phase_index += 1
                phase_start = None
                continue
            # Prevent a direction change until speed is nearly zero. The
            # adapter additionally demands a braked stationary shift request.
            if name == "shift_hold" and abs(state["speed"]) > 0.08:
                gear = 1
                brake = 0.35
            send(accel, brake, gear)
            rows.append(dict(sim_s=sim - first_sim, phase=name, phase_s=elapsed,
                             gear=gear, accelerator=accel, brake0=brake,
                             speed_mps=state["speed"], x=state["x"], y=state["y"]))
    except Exception as exc:
        outcome = str(exc)
        raise
    finally:
        gear = phases[min(phase_index, len(phases) - 1)][1]
        for _ in range(12):
            send(0, 0.35, gear)
            rclpy.spin_once(node, timeout_sec=0.05)
        with (args.output / "samples.csv").open("w", newline="") as f:
            fields = ["sim_s", "phase", "phase_s", "gear", "accelerator", "brake0", "speed_mps", "x", "y"]
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)
        (args.output / "manifest.json").write_text(json.dumps({
            "surface": args.surface, "world": args.world, "rate_hz": args.rate,
            "max_speed_mps": args.max_speed, "max_distance_m": args.max_distance,
            "samples": len(rows), "outcome": outcome, "phases": [p[0] for p in phases]
        }, indent=2) + "\n")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--surface", required=True)
    parser.add_argument("--world", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--max-speed", type=float, default=2.0)
    parser.add_argument("--max-distance", type=float, default=35.0)
    parser.add_argument("--max-wall-seconds", type=float, default=300.0)
    args = parser.parse_args()
    if args.rate <= 0 or args.max_speed <= 0 or args.max_distance <= 0:
        parser.error("rate and safety limits must be positive")
    run(args)
