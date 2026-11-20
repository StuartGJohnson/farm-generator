"""Deterministic straight-line cmd_vel trial with CSV, plots, and metrics.

Run using the system Humble Python after sourcing activate_venv.sh.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
from pathlib import Path


def profile(t, direction=1, high_speed=1.0, low_speed=0.5):
    if t < 2:
        return 0.0
    if t < 10:
        return direction * high_speed
    if t < 18:
        return direction * low_speed
    return 0.0


def metrics(rows, direction=1, high_speed=1.0, low_speed=0.5):
    valid = [r for r in rows if math.isfinite(r["measured_mps"])]
    if not valid:
        return {"error": "No valid body-frame odometry samples"}
    def hold(lo, hi, target):
        values = [r["measured_mps"] for r in valid if lo <= r["sim_s"] < hi]
        return (sum(direction * v - target for v in values) / len(values)) if values else None
    errors = [r["cmd_mps"] - r["measured_mps"] for r in valid]
    rise = next((r["sim_s"] - 2 for r in valid if r["sim_s"] >= 2 and direction * r["measured_mps"] >= 0.9 * high_speed), None)
    return {
        "rmse_mps": math.sqrt(sum(e * e for e in errors) / len(errors)),
        "steady_error_high_mps": hold(7, 10, high_speed),
        "steady_error_low_mps": hold(15, 18, low_speed),
        "overshoot_high_mps": max((direction * r["measured_mps"] - high_speed for r in valid if 2 <= r["sim_s"] < 10), default=None),
        "rise_to_90pct_s": rise,
        "settling_high_s": next((r["sim_s"] - 2 for r in valid if 2 <= r["sim_s"] < 10 and
            all(abs(direction * s["measured_mps"] - high_speed) <= 0.1 * high_speed for s in valid if r["sim_s"] <= s["sim_s"] < 10)), None),
    }


def main():
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import Float32MultiArray

    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=Path("debug_out/vehicle_speed_trial"))
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--max-wall-seconds", type=float, default=180.0)
    parser.add_argument("--reverse", action="store_true")
    parser.add_argument("--model", default="src/tractor_control_adapter/config/road.csv")
    parser.add_argument("--kp", type=float, default=1.1)
    parser.add_argument("--ki", type=float, default=0.15)
    parser.add_argument("--high-speed", type=float, default=1.0)
    parser.add_argument("--low-speed", type=float, default=0.5)
    args = parser.parse_args()
    if args.rate <= 0 or args.high_speed <= 0 or args.low_speed <= 0 or args.low_speed >= args.high_speed:
        parser.error("rate and speeds must be positive, with low speed below high speed")
    args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    from rclpy.parameter import Parameter
    node = rclpy.create_node("vehicle_speed_profile", parameter_overrides=[
        Parameter("use_sim_time", value=True)])
    pub = node.create_publisher(Twist, "/cmd_vel", 10)
    state = {"clock": None, "speed": math.nan, "act": [math.nan] * 5, "debug": [math.nan] * 8}
    def on_clock(msg):
        state["clock"] = msg.clock.sec + msg.clock.nanosec * 1e-9
    def on_odom(msg):
        state["speed"] = msg.twist.twist.linear.x if msg.child_frame_id == "base_link" else math.nan
    def on_act(msg):
        if len(msg.data) == 5:
            state["act"] = list(msg.data)
    def on_debug(msg):
        if len(msg.data) == 8:
            state["debug"] = list(msg.data)
    node.create_subscription(Clock, "/clock", on_clock, 10)
    node.create_subscription(Odometry, "/ground_truth/odom", on_odom, 10)
    node.create_subscription(Float32MultiArray, "/vehicle/actuation", on_act, 10)
    node.create_subscription(Float32MultiArray, "/vehicle/control_debug", on_debug, 10)
    rows = []
    start_wall = time.monotonic()
    start_sim = None
    last_sim = None
    last_publish_sim = None
    stop_since = None
    try:
        while rclpy.ok() and time.monotonic() - start_wall < args.max_wall_seconds:
            rclpy.spin_once(node, timeout_sec=1 / args.rate)
            now_sim = state["clock"]
            if now_sim is None:
                continue
            if start_sim is None or now_sim < last_sim:
                start_sim = now_sim
                last_publish_sim = None
                rows.clear()
            last_sim = now_sim
            if last_publish_sim is not None and now_sim - last_publish_sim < 1 / args.rate - 1e-9:
                continue
            last_publish_sim = now_sim
            elapsed = now_sim - start_sim
            direction = -1 if args.reverse else 1
            target = profile(elapsed, direction, args.high_speed, args.low_speed)
            msg = Twist()
            msg.linear.x = target
            pub.publish(msg)
            a = state["act"]
            d = state["debug"]
            rows.append(dict(sim_s=elapsed, cmd_mps=target, measured_mps=state["speed"],
                             accelerator=a[0], brake0=a[1], brake1=a[2], steer=a[3], gear=a[4],
                             ramped_mps=d[1], desired_accel_mps2=d[3], integral=d[6]))
            if elapsed >= 20 and math.isfinite(state["speed"]) and abs(state["speed"]) < 0.05:
                stop_since = now_sim if stop_since is None else stop_since
                if now_sim - stop_since >= 2:
                    break
            else:
                stop_since = None
    finally:
        zero = Twist()
        for _ in range(10):
            pub.publish(zero)
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.shutdown()
    with (args.output / "samples.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]) if rows else ["sim_s", "cmd_mps", "measured_mps", "accelerator", "brake0", "brake1", "steer", "gear", "ramped_mps", "desired_accel_mps2", "integral"], lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    result = metrics(rows, -1 if args.reverse else 1, args.high_speed, args.low_speed)
    result.update({"profile": f"0-2s:0, 2-10s:{args.high_speed}, 10-18s:{args.low_speed}, 18+s:0 m/s", "direction": "reverse" if args.reverse else "forward", "rate_hz": args.rate,
                   "high_speed_mps": args.high_speed, "low_speed_mps": args.low_speed,
                   "world": "debug_out/farms/open_seed_1_road", "model": args.model,
                   "controller_gains": {"kp": args.kp, "ki": args.ki}})
    (args.output / "metrics.json").write_text(json.dumps(result, indent=2) + "\n")
    try:
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(2, 1, sharex=True, figsize=(9, 6))
        axes[0].plot([r["sim_s"] for r in rows], [r["cmd_mps"] for r in rows], label="command")
        axes[0].plot([r["sim_s"] for r in rows], [r["ramped_mps"] for r in rows], label="ramped")
        axes[0].plot([r["sim_s"] for r in rows], [r["measured_mps"] for r in rows], label="body speed")
        axes[0].set_ylabel("m/s")
        axes[0].legend()
        axes[1].plot([r["sim_s"] for r in rows], [r["accelerator"] for r in rows], label="accelerator")
        axes[1].plot([r["sim_s"] for r in rows], [r["brake0"] for r in rows], label="brake0")
        axes[1].set(xlabel="simulation elapsed time (s)", ylabel="normalized input")
        axes[1].legend()
        fig.tight_layout()
        fig.savefig(args.output / "tracking.png", dpi=150)
        plt.close(fig)
    except ImportError:
        print("matplotlib unavailable; CSV and metrics were saved")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
