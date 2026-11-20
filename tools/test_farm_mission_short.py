"""Run a short mission and save commanded/actual velocity and route plots."""

from __future__ import annotations

import csv
import argparse
import json
import math
import threading
import time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patheffects as path_effects
from PIL import Image
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from farm_mission_interfaces.action import ExecuteMission, PlanMission
from route_planning import load_farm_map
from route_planning.diagnostic import load_route_json
from tractor_generation.config import load_tractor_config


def route_reversals(route):
    """Return route stations and poses where driving direction changes."""
    reversals = []
    station = 0.0
    previous_direction = None
    for segment in route.segments:
        lengths = [math.hypot(b.x - a.x, b.y - a.y)
                   for a, b in zip(segment.poses, segment.poses[1:])]
        scale = segment.distance_m / sum(lengths) if sum(lengths) else 0.0
        for index, (length, direction) in enumerate(zip(lengths, segment.driving_directions[1:])):
            if previous_direction is not None and direction != previous_direction:
                reversals.append((station, segment.poses[index]))
            station += length * scale
            previous_direction = direction
    return reversals


def save_plan_view(output, route, rows, farm, rear_axle_x=None,
                   base_link_reference="rear_axle"):
    """Overlay measured rear-axle positions on the georeferenced farm image."""
    fig, ax = plt.subplots(figsize=(8, 8))
    if farm.orthophoto_path and farm.orthophoto_extent:
        with Image.open(farm.orthophoto_path) as source:
            image = source.convert("RGB")
            x0, y0, x1, y1 = farm.orthophoto_extent
            ax.imshow(image, extent=(x0, x1, y0, y1), origin="upper", zorder=0)
    planned = route.continuous_path()
    outline = [path_effects.Stroke(linewidth=4.5, foreground="white"),
               path_effects.Normal()]
    ax.plot([p.x for p in planned], [p.y for p in planned],
            color="#1045cb", linewidth=2.4, label="planned", zorder=2,
            path_effects=outline)
    base_is_rear = base_link_reference == "rear_axle"
    ax.plot([r["x"] for r in rows], [r["y"] for r in rows],
            color="#11a956" if base_is_rear else "#f04418", linewidth=2.0,
            label="rear axle / ROS base_link" if base_is_rear else "body center / old base_link",
            zorder=3,
            path_effects=outline)
    if rear_axle_x is not None and rows and "yaw" in rows[0]:
        offset = -rear_axle_x if base_is_rear else rear_axle_x
        ax.plot([r["x"] + offset * math.cos(r["yaw"]) for r in rows],
                [r["y"] + offset * math.sin(r["yaw"]) for r in rows],
                color="#f04418" if base_is_rear else "#11a956", linewidth=2.0,
                label="body center" if base_is_rear else "rear axle center", zorder=3,
                path_effects=outline)
    ax.scatter([planned[0].x], [planned[0].y], color="#00c46a", edgecolor="black",
               s=55, label="start", zorder=4)
    ax.scatter([planned[-1].x], [planned[-1].y], color="#f24049", edgecolor="black",
               marker="s", s=55, label="goal", zorder=4)
    reversals = route_reversals(route)
    if reversals:
        ax.scatter([pose.x for _, pose in reversals], [pose.y for _, pose in reversals],
                   color="#ffd43b", edgecolor="black", marker="D", s=65,
                   label="direction change", zorder=5)
    xs = [p.x for p in planned]
    ys = [p.y for p in planned]
    ax.set_xlim(min(xs) - 2, max(xs) + 2)
    ax.set_ylim(min(ys) - 2, max(ys) + 2)
    ax.set(xlabel="Map x (m)", ylabel="Map y (m)")
    ax.set_aspect("equal")
    ax.grid(alpha=0.15)
    ax.legend(loc="best")
    fig.savefig(output / "plan_view.png", dpi=180, bbox_inches="tight")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--route-number", type=int, choices=(1, 2, 3), default=1)
    parser.add_argument("--max-wall-seconds", type=float, default=600.0)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--plot-only", action="store_true")
    parser.add_argument("--base-link-reference", choices=("rear_axle", "rigid_origin"),
                        default="rear_axle")
    args = parser.parse_args()
    output = args.output_dir or Path("debug_out/farm_mission_short" if args.route_number == 1 else
                                     f"debug_out/farm_mission_short_route_{args.route_number}")
    output.mkdir(parents=True, exist_ok=True)
    route = load_route_json(f"debug_out/routes/short/short_route_{args.route_number}.json")
    farm = load_farm_map("debug_out/farms/farm_seed_1_120m2_depot/gis")
    tractor_config = load_tractor_config(
        "configs/tractor_route_2m.yaml" if args.route_number == 2
        else "configs/tractor_default.yaml")
    if args.plot_only:
        with (output / "velocity.csv").open(newline="") as stream:
            rows = [{key: float(value) for key, value in row.items()}
                    for row in csv.DictReader(stream)]
        save_plan_view(output, route, rows, farm, tractor_config.rear_axle_x,
                       args.base_link_reference)
        return
    start, end = route.task.start, route.task.end
    rclpy.init()
    node = rclpy.create_node(f"farm_mission_short_{args.route_number}_trial")
    state = {"cmd": 0.0, "cmd_yaw_rate": 0.0, "speed": math.nan,
             "lateral_speed": math.nan, "yaw_rate": math.nan,
             "steer_norm": math.nan, "x": math.nan, "y": math.nan,
             "yaw": math.nan, "station": 0.0, "section": 0, "stamp": None}
    rows = []

    def cmd(msg):
        state["cmd"] = msg.linear.x
        state["cmd_yaw_rate"] = msg.angular.z

    def actuation(msg):
        if len(msg.data) == 5:
            state["steer_norm"] = msg.data[3]

    def odom(msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        state.update(speed=msg.twist.twist.linear.x,
                     lateral_speed=msg.twist.twist.linear.y,
                     yaw_rate=msg.twist.twist.angular.z,
                     yaw=yaw,
                     x=msg.pose.pose.position.x, y=msg.pose.pose.position.y,
                     stamp=stamp)
        rows.append(dict(state))

    def feedback(msg):
        state["station"] = msg.feedback.distance_traveled_m
        state["section"] = msg.feedback.section_index

    node.create_subscription(Twist, "/cmd_vel", cmd, 10)
    node.create_subscription(Float32MultiArray, "/vehicle/actuation", actuation, 10)
    node.create_subscription(Odometry, "/odom", odom, 10)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    planner = ActionClient(node, PlanMission, "mission_planner")
    runner = ActionClient(node, ExecuteMission, "mission_executor")

    def wait(future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.1)
        if not future.done():
            raise TimeoutError("Mission action timed out")
        return future.result()

    status = {"success": False, "base_link_reference": args.base_link_reference,
              "rear_axle_x_m": tractor_config.rear_axle_x,
              "reversal_stations_m": [station for station, _ in route_reversals(route)]}
    handle = None
    try:
        if not planner.wait_for_server(timeout_sec=60) or not runner.wait_for_server(timeout_sec=60):
            raise RuntimeError("Mission action servers unavailable")
        goal = PlanMission.Goal()
        goal.implement_side = "both"
        if args.route_number == 3:
            goal.start_type = goal.end_type = "depot"
            goal.start_depot_spot, goal.end_depot_spot = 1, 2
            goal.start_depot_direction = goal.end_depot_direction = "in"
        else:
            goal.start_x, goal.start_y, goal.start_yaw = start.x, start.y, start.yaw
            goal.end_x, goal.end_y, goal.end_yaw = end.x, end.y, end.yaw
        planned = wait(planner.send_goal_async(goal), 10)
        if not planned.accepted:
            raise RuntimeError("Planning goal rejected")
        plan_result = wait(planned.get_result_async(), 120)
        if plan_result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"Planning failed: {plan_result.result.message}")
        status["mission_id"] = plan_result.result.mission_id
        status["planned_distance_m"] = plan_result.result.total_distance_m
        if args.route_number == 3 and abs(plan_result.result.total_distance_m - route.total_distance_m) > 0.1:
            raise RuntimeError("Planner did not reproduce saved short_route_3")
        ready_by = time.monotonic() + 90
        while len(rows) < 10 and time.monotonic() < ready_by:
            time.sleep(0.2)
        if len(rows) < 10:
            raise RuntimeError("No tractor odometry received")
        time.sleep(2.0)
        handle = wait(runner.send_goal_async(
            ExecuteMission.Goal(mission_id=plan_result.result.mission_id),
            feedback_callback=feedback), 10)
        if not handle.accepted:
            raise RuntimeError("Execution goal rejected")
        outcome = wait(handle.get_result_async(), args.max_wall_seconds)
        status["action_status"] = outcome.status
        status["message"] = outcome.result.message
        status["success"] = outcome.status == GoalStatus.STATUS_SUCCEEDED and outcome.result.success
    except Exception as exc:
        status["error"] = str(exc)
        if handle is not None:
            wait(handle.cancel_goal_async(), 10)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=5)
        if rows:
            with (output / "velocity.csv").open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
                writer.writeheader()
                writer.writerows(rows)
            t0 = rows[0]["stamp"]
            fig, ax = plt.subplots(figsize=(9, 4))
            ax.plot([r["stamp"] - t0 for r in rows], [r["cmd"] for r in rows], label="commanded")
            ax.plot([r["stamp"] - t0 for r in rows], [r["speed"] for r in rows], label="actual")
            ax.set(xlabel="Simulation time (s)", ylabel="Forward speed (m/s)")
            ax.grid(alpha=0.3)
            ax.legend()
            fig.savefig(output / "velocity.png", dpi=150, bbox_inches="tight")
            plt.close(fig)
            fig, ax = plt.subplots(figsize=(9, 4))
            ax.plot([r["station"] for r in rows], [r["cmd"] for r in rows], label="commanded")
            ax.plot([r["station"] for r in rows], [r["speed"] for r in rows], label="actual")
            for boundary in range(5, math.ceil(route.total_distance_m), 5):
                ax.axvline(boundary, color="gray", linestyle="--", linewidth=1,
                           label="section boundary" if boundary == 5 else None)
            for index, (station, _) in enumerate(route_reversals(route)):
                ax.axvline(station, color="#bd36c5", linestyle=":", linewidth=1.5,
                           label="direction change" if index == 0 else None)
            ax.set(xlabel="Distance along route (m)", ylabel="Forward speed (m/s)")
            ax.grid(alpha=0.3)
            ax.legend()
            fig.savefig(output / "velocity_vs_distance.png", dpi=150, bbox_inches="tight")
            plt.close(fig)
            save_plan_view(output, route, rows, farm, tractor_config.rear_axle_x,
                           args.base_link_reference)
            fig, ax = plt.subplots(figsize=(9, 4))
            ax.plot([r["station"] for r in rows], [r["cmd_yaw_rate"] for r in rows],
                    label="commanded yaw rate")
            ax.plot([r["station"] for r in rows], [r["yaw_rate"] for r in rows],
                    label="actual yaw rate")
            for index, (station, _) in enumerate(route_reversals(route)):
                ax.axvline(station, color="#bd36c5", linestyle=":", linewidth=1.5,
                           label="direction change" if index == 0 else None)
            ax.set(xlabel="Distance along route (m)", ylabel="Yaw rate (rad/s)")
            ax.grid(alpha=0.3)
            ax.legend()
            fig.savefig(output / "yaw_rate_vs_distance.png", dpi=150, bbox_inches="tight")
            plt.close(fig)
        status["samples"] = len(rows)
        status["sections_observed"] = sorted(set(r["section"] for r in rows))
        status["handoffs"] = []
        for i in range(1, len(rows)):
            if rows[i]["section"] == rows[i - 1]["section"]:
                continue
            station = rows[i]["station"]
            following = [row for row in rows[i:] if row["station"] < station + 1.0]
            status["handoffs"].append({
                "section": rows[i]["section"], "station_m": station,
                "cmd_before_mps": rows[i - 1]["cmd"],
                "cmd_at_mps": rows[i]["cmd"],
                "min_cmd_next_meter_mps": min(row["cmd"] for row in following),
            })
        (output / "result.json").write_text(json.dumps(status, indent=2) + "\n")
    print(json.dumps(status, indent=2))
    if not status["success"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
