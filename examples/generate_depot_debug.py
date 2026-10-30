"""Rebuild the depot debugging artifacts in debug_out/depot/seed_4."""

from pathlib import Path
from dataclasses import replace
import argparse
import json

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon
import numpy as np
from shapely.geometry import Point, Polygon

from export.usd import build_ground_mesh, save_ground_mesh_plan, write_ground_mesh_usda
from generation.orchestrator import FarmGenerationConfig, generate_farm, save_farm
from generation.crops import _depot_exclusion_polygon
from gis.export import export_gis
from visualization.debug_view import plot_scene, save_scene_png
from visualization.surface_friction import save_surface_friction_plot
from visualization.surface_height import save_surface_height_plot
from visualization.depot_comparison import save_depot_tree_comparison


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plots-only", action="store_true",
                        help="write a geometry preview without replacing GIS or USD exports")
    args = parser.parse_args()
    output = Path("debug_out/depot/seed_4") / ("preview" if args.plots_only else "")
    output.mkdir(parents=True, exist_ok=True)
    config = FarmGenerationConfig(bounds=(0, 0, 120, 120), seed=4, max_faces=3,
        row_spacing=5, tree_spacing=4, headland_width=6, sideland_width=5,
        standoff=3, depot_enabled=True, depot_cross_row_size=4)
    scene = generate_farm(config)
    if not args.plots_only:
        save_farm(scene, config, str(output / "farm.json"))
    save_scene_png(scene, str(output / "farm_layout.png"), dpi=180)
    fig, ax = plot_scene(scene, title="Tractor depot and adjacent crop rows")
    x0, y0, x1, y1 = Polygon(scene.depot.polygon).bounds
    ax.set_xlim(x0 - 8, x1 + 8)
    ax.set_ylim(y0 - 8, y1 + 8)
    fig.savefig(output / "depot_detail.png", dpi=200, bbox_inches="tight")
    plt.close(fig)
    original = generate_farm(replace(config, depot_enabled=False))
    save_depot_tree_comparison(scene, original, output / "depot_tree_comparison.png")
    field_id = scene.depot.field_id
    original_zone = next(zone for zone in original.weed_zones.values()
                         if zone.tags.get("parcel_id") == field_id)
    basis_inv = np.linalg.inv(np.column_stack([
        scene.depot.row_direction, scene.depot.cross_row_direction]))
    origin = np.asarray(scene.depot.origin)
    exclusion = _depot_exclusion_polygon(
        Polygon(scene.depot.polygon), np.asarray(scene.depot.row_direction),
        np.asarray(scene.depot.cross_row_direction),
        config.depot_headland_m, config.depot_sideland_m)
    row_number = 1 + min(range(len(original_zone.row_centerlines)),
                         key=lambda index: (basis_inv @ (
                             np.asarray(original_zone.row_centerlines[index][0]) - origin))[1])
    original_row = [tree for tree in original.trees.values()
                    if tree.tags.get("parcel_id") == field_id and
                    tree.tags.get("row_number") == str(row_number)]
    retained = {tuple(round(value, 5) for value in tree.position)
                for tree in scene.trees.values() if tree.tags.get("parcel_id") == field_id}
    removed_row = [tree for tree in original_row
                   if tuple(round(value, 5) for value in tree.position) not in retained]
    kept_row = [tree for tree in original_row
                if tuple(round(value, 5) for value in tree.position) in retained]
    fig, ax = plot_scene(scene, title="Road-facing row: retained trees and depot removal")
    field_x0 = Polygon(scene.parcels[field_id].polygon).bounds[0]
    ax.set_xlim(field_x0 - 3, x1 + 5)
    ax.set_ylim(y0 - 6, y0 + 11)
    ax.scatter([tree.position[0] for tree in removed_row],
               [tree.position[1] for tree in removed_row],
               s=40, marker="x", color="crimson", label="removed near depot", zorder=9)
    ax.add_patch(MplPolygon(list(exclusion.exterior.coords), closed=True, fill=False,
                            edgecolor="#e33333", linestyle="--", linewidth=1.8,
                            label="depot exclusion", zorder=8))
    ax.legend(loc="upper right", fontsize=9)
    fig.savefig(output / "roadside_row_detail.png", dpi=200, bbox_inches="tight")
    plt.close(fig)
    report = {"field_id": field_id, "roadside_row_number": row_number,
              "original_row_trees": len(original_row),
              "retained_row_trees": len(kept_row),
              "removed_row_trees": len(removed_row),
              "nearest_retained_tree_to_depot_m": min(
                  (Polygon(scene.depot.polygon).distance(Point(tree.position)) for tree in kept_row),
                  default=None),
              "depot_headland_m": config.depot_headland_m,
              "depot_sideland_m": config.depot_sideland_m,
              "exclusion_polygon_xy": list(exclusion.exterior.coords[:-1]),
              "all_removed_trees_inside_exclusion": all(
                  exclusion.covers(Point(tree.position)) for tree in original.trees.values()
                  if tree.tags.get("parcel_id") == field_id and
                  tuple(round(value, 5) for value in tree.position) not in retained),
              "all_retained_trees_outside_exclusion": all(
                  not exclusion.covers(Point(tree.position)) for tree in scene.trees.values()
                  if tree.tags.get("parcel_id") == field_id),
              "roads_unchanged": scene.roads == original.roads,
              "irrigation_unchanged": scene.hydrology == original.hydrology}
    (output / "roadside_row_report.json").write_text(json.dumps(report, indent=2) + "\n")
    if args.plots_only:
        print(output.resolve())
        return
    mesh = build_ground_mesh(scene, config.domain_bounds)
    save_surface_height_plot(mesh, output / "height_fields.png")
    save_surface_friction_plot(mesh, output / "friction_fields.png")
    save_ground_mesh_plan(mesh, str(output / "ground_plan.png"))
    write_ground_mesh_usda(mesh, str(output / "ground.usda"), scene=scene)
    export_gis(scene, config.domain_bounds, output, tiles=False)
    print(output.resolve())


if __name__ == "__main__":
    main()
