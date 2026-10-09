"""Route plots and machine-readable diagnostics, independent of the solver."""

from __future__ import annotations

import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from PIL import Image

from collections import Counter

from .model import FarmMap, FarmTask, ImplementSide, Pose, Route, RouteSegment
from .path_feasibility import tree_convex_hulls


def optimality_gap_percent(route: Route) -> float | None:
    """Upper bound on improvement, relative to the found objective."""
    if (route.optimality_status != "FEASIBLE" or route.best_bound_cost is None
            or route.objective_cost <= 0):
        return None
    return 100 * max(0.0, route.objective_cost - route.best_bound_cost) / route.objective_cost


def save_route_json(route: Route, farm: FarmMap, path: str | Path) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    doc = {
        "frame": "local_ENU_m",
        "pose_reference": "rear_axle_center",
        "rs_tree_block_rule": "rear_axle_outside_tree_convex_hull",
        "gis_crs": "EPSG:32610",
        "origin_easting_m": farm.origin_easting_m,
        "origin_northing_m": farm.origin_northing_m,
        "task": {
            "field_ids": route.task.field_ids,
            "implement_side": route.task.implement_side.value,
            "implement_range_m": route.task.implement_range_m,
            "start_pose": vars(route.task.start),
            "end_pose": vars(route.task.end),
        },
        "status": route.optimality_status,
        "total_distance_m": route.total_distance_m,
        "reverse_distance_m": sum(s.reverse_distance_m for s in route.segments),
        "gear_changes": sum(s.gear_changes for s in route.segments),
        "reverse_cost_multiplier": route.reverse_cost_multiplier,
        "gear_change_penalty_m": route.gear_change_penalty_m,
        "objective_cost": route.objective_cost,
        "best_bound_cost": route.best_bound_cost,
        "solver_wall_time_s": route.solver_wall_time_s,
        "saturated_arc_ids": route.saturated_arc_ids,
        "segments": [{
            "arc_id": s.arc_id, "occurrence_index": s.occurrence_index,
            "arc_type": s.kind, "field_id": s.field_id,
            "distance_m": s.distance_m, "reverse_distance_m": s.reverse_distance_m,
            "gear_changes": s.gear_changes, "service_ids": sorted(s.service_ids),
            "poses": [[p.x, p.y, p.yaw] for p in s.poses],
            "driving_directions": list(s.driving_directions),
        } for s in route.segments],
    }
    path.write_text(json.dumps(doc, indent=2) + "\n")


def load_route_json(path: str | Path) -> Route:
    """Reload a saved route for plotting without another optimization run."""
    doc = json.loads(Path(path).read_text())
    task_doc = doc["task"]
    task = FarmTask(tuple(task_doc["field_ids"]), ImplementSide(task_doc["implement_side"]),
                    Pose(**task_doc["start_pose"]), Pose(**task_doc["end_pose"]),
                    task_doc["implement_range_m"])
    segments = [RouteSegment(s["arc_id"], s["occurrence_index"], s["arc_type"],
                             s["field_id"], tuple(Pose(*p) for p in s["poses"]),
                             s["distance_m"], frozenset(s["service_ids"]),
                             tuple(s.get("driving_directions", [1] * len(s["poses"]))),
                             s.get("reverse_distance_m", 0.0), s.get("gear_changes", 0))
                for s in doc["segments"]]
    return Route(task, segments, doc["total_distance_m"], doc["objective_cost"],
                 doc["status"], dict(Counter(s.arc_id for s in segments)),
                 doc["saturated_arc_ids"], doc.get("reverse_cost_multiplier", 1.0),
                 doc.get("gear_change_penalty_m", 0.0),
                 doc.get("best_bound_cost"), doc.get("solver_wall_time_s"))


def save_route_plot(route: Route, farm: FarmMap, path: str | Path) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(12, 12), constrained_layout=True)
    if farm.orthophoto_path and farm.orthophoto_extent:
        image = Image.open(farm.orthophoto_path).convert("RGB")
        x0, y0, x1, y1 = farm.orthophoto_extent
        ax.imshow(image, extent=(x0, x1, y0, y1), origin="upper", alpha=0.83, zorder=0)
    for field_id, polygon in farm.fields.items():
        x, y = polygon.exterior.xy
        selected = field_id in route.task.field_ids
        ax.fill(x, y, color="#24b9ad", alpha=0.09 if selected else 0.0, zorder=1)
        ax.plot(x, y, color="#18bbaa" if selected else "#58616c",
                lw=2.2 if selected else 0.8, zorder=2)
        center = polygon.representative_point()
        ax.text(center.x, center.y, farm.field_names.get(field_id, field_id),
                color="white", fontsize=9, weight="bold", ha="center", va="center",
                bbox={"facecolor": "#12343d", "alpha": 0.83, "edgecolor": "none", "pad": 2}, zorder=12)
    for road in farm.roads.values():
        x, y = road.line.xy
        ax.plot(x, y, color="#e9eef2", lw=1.1, alpha=0.7, zorder=3)
    for channel in farm.irrigation_channels.values():
        x, y = channel.exterior.xy
        ax.fill(x, y, color="#3b98dd", alpha=0.11, zorder=3)
        ax.plot(x, y, color="#46b7ee", lw=0.7, alpha=0.65, zorder=3)
    for tree in farm.trees.values():
        ax.plot(tree.x, tree.y, ".", color="#184e1d", markersize=1.8, alpha=0.65, zorder=4)
    for hull in tree_convex_hulls(farm, route.task.field_ids).values():
        if hull.geom_type == "Polygon":
            x, y = hull.exterior.xy
            ax.plot(x, y, color="#efc348", linestyle="--", lw=1.2,
                    alpha=0.9, zorder=5)
    done = set()
    colors = {"road": "#1fb9ed", "rs": "#ff9c2d", "row": "#e72d97", "deadhead": "#b969e2"}
    for index, segment in enumerate(route.segments, 1):
        service = segment.kind == "row" and bool(segment.service_ids - done)
        category = "deadhead" if segment.kind == "row" and not service else segment.kind
        points = segment.poses
        x, y = [p.x for p in points], [p.y for p in points]
        ax.plot(x, y, color="#141819", lw=5.0, alpha=0.7, zorder=6)
        ax.plot(x, y, color=colors[category], lw=2.9, zorder=7)
        if segment.kind == "rs":
            for i in range(1, len(points)):
                if segment.driving_directions[i] < 0:
                    ax.plot(x[i - 1:i + 1], y[i - 1:i + 1], color="#ff374c",
                            lw=3.5, linestyle="--", zorder=8)
        if len(points) >= 2:
            middle = max(0, (len(points) - 2) // 2)
            a, b = points[middle], points[middle + 1]
            ax.annotate("", xy=(b.x, b.y), xytext=(a.x, a.y),
                        arrowprops={"arrowstyle": "-|>", "color": "#111827",
                                    "lw": 1.2, "mutation_scale": 8}, zorder=8)
        done.update(segment.service_ids)
    start, end = route.task.start, route.task.end
    ax.scatter([start.x], [start.y], s=160, marker="o", facecolor="#00d67c",
               edgecolor="black", linewidth=1.2, zorder=10)
    ax.scatter([end.x], [end.y], s=125, marker="s", facecolor="#ff4f50",
               edgecolor="black", linewidth=1.2, zorder=11)
    ax.annotate("START", (start.x, start.y), xytext=(7, 8), textcoords="offset points",
                color="white", fontsize=9, weight="bold",
                bbox={"facecolor": "#095b36", "alpha": 0.9, "pad": 2}, zorder=12)
    ax.annotate("END", (end.x, end.y), xytext=(7, -17), textcoords="offset points",
                color="white", fontsize=9, weight="bold",
                bbox={"facecolor": "#932c32", "alpha": 0.9, "pad": 2}, zorder=12)
    handles = [Line2D([0], [0], color=colors[k], lw=3, label=label) for k, label in (
        ("road", "Road travel"), ("row", "Row service"),
        ("deadhead", "Row deadhead"), ("rs", "Reeds–Shepp rear axle"))]
    handles.append(Line2D([0], [0], color="#ff374c", lw=3.5,
                          linestyle="--", label="Reverse travel"))
    handles.append(Line2D([0], [0], color="#efc348", lw=1.2,
                          linestyle="--", label="RS exclusion: tree hull"))
    ax.legend(handles=handles, loc="lower center", bbox_to_anchor=(0.5, -0.09),
              ncol=2, framealpha=0.96, fontsize=9)
    field_names = ", ".join(farm.field_names.get(fid, fid) for fid in route.task.field_ids)
    reverse_m = sum(s.reverse_distance_m for s in route.segments)
    gap = optimality_gap_percent(route)
    status = route.optimality_status
    if gap is not None:
        status += f" • optimality gap ≤ {math.ceil(gap * 10) / 10:.1f}%"
    ax.set_title(f"Farm route: {field_names}\n{route.total_distance_m:.1f} m total • "
                 f"{reverse_m:.1f} m reverse • {status} • "
                 f"{len(route.segments)} traversals",
                 fontsize=14, weight="bold")
    ax.set_xlabel("Local east / ROS map X (m)")
    ax.set_ylabel("Local north / ROS map Y (m)")
    ax.set_aspect("equal")
    ax.set_xlim(min(p.bounds[0] for p in farm.fields.values()) - 3,
                max(p.bounds[2] for p in farm.fields.values()) + 3)
    ax.set_ylim(min(p.bounds[1] for p in farm.fields.values()) - 3,
                max(p.bounds[3] for p in farm.fields.values()) + 3)
    ax.grid(alpha=0.12)
    fig.savefig(path, dpi=180, bbox_inches="tight")
    plt.close(fig)
