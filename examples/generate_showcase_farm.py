"""Generate a portable farm with USD, GIS, orthophoto, and diagnostics."""

import argparse
from pathlib import Path
import shutil
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from export.usd import (
    ChannelUndulationConfig,
    export_scene_ground,
    save_ground_mesh_plan,
    save_ground_mesh_wireframe,
)
from generation.orchestrator import FarmGenerationConfig, SurfaceFrictionConfig, generate_farm, generate_validated, save_farm
from visualization.debug_view import save_scene_png
from visualization.surface_friction import save_surface_friction_plot
from visualization.surface_height import save_surface_height_plot
from export.usd.friction import quantize_friction
from gis import export_gis


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--gsd", type=float, default=0.05, help="orthophoto pixel size in metres")
    parser.add_argument("--output-root", type=Path, default=Path("debug_out/farms"))
    parser.add_argument("--vector-only", action="store_true", help="skip Isaac rendering for GIS development")
    parser.add_argument("--depot", action="store_true", help="include a tractor depot and suffix the farm name")
    parser.add_argument("--seed4-debug-profile", action="store_true",
                        help="use the seed 4 depot regression geometry in a full farm bundle")
    args = parser.parse_args()
    seed = args.seed
    if args.seed4_debug_profile and (seed != 4 or not args.depot):
        parser.error("--seed4-debug-profile requires --seed 4 --depot")
    bounds = (0.0, 0.0, 120.0, 120.0)
    stem = f"farm_seed_{seed}_120m2" + ("_depot" if args.depot else "")
    scene_dir = Path("debug_out") / "farm_scenes"
    mesh_dir = Path("debug_out") / "mesh"
    asset_dir = Path("debug_out") / "procedural_assets"
    farm_dir = args.output_root / stem
    world_dir = farm_dir / "world"
    world_dir.mkdir(parents=True, exist_ok=True)
    scene_dir.mkdir(parents=True, exist_ok=True)
    mesh_dir.mkdir(parents=True, exist_ok=True)

    tree_assets = tuple(sorted(asset_dir.glob("trees/*/*.usda")))
    weed_assets = tuple(sorted(asset_dir.glob("weeds/*/*.usda")))
    if len(tree_assets) != 4 or not weed_assets:
        raise FileNotFoundError(
            "the four-tree procedural library is missing; run "
            "`python examples/generate_procedural_assets.py` first"
        )

    config = FarmGenerationConfig(
        bounds=bounds,
        seed=seed,
        # Increase variation_range for stronger contrasts; decrease scale_m
        # for finer spatial variation. The shortest wavelength is 2 * resolution.
        farm_resolution_m=1.0,
        crop_surface=SurfaceFrictionConfig(
            friction_mean=0.5,
            friction_variation_range=0.90,
            friction_scale_m=10.0,
            friction_spectral_slope=4.0,
            friction_seed=1001,
        ),
        road_surface=SurfaceFrictionConfig(
            friction_mean=1.05,
            friction_variation_range=0.20,
            friction_scale_m=10.0,
            friction_spectral_slope=4.0,
            friction_seed=2001,
        ),
        channel_friction=0.30,  # Channels currently use a constant coefficient.
        # Height uses the same grid resolution and shares the crop seed, so
        # their patterns are correlated despite the different spatial scales.
        # The water surface remains at its original level.
        height_mean=0.0,
        height_variation_range_m=0.5,
        height_scale_m=10.0,
        height_spectral_slope=4.0,
        height_seed=1001,
        road_height_variation_range_m=0.125,
        road_height_scale_m=10.0,
        shoreline_taper_m=1.0,
        max_faces=4,
        standoff=3.5,
        headland_width=7.5,
        sideland_width=6.5,
        tree_spacing=4.0,
        row_spacing=5.0,
        optimize_tree_layout=True,
        weed_row_density_per_m=4.5,
        weed_row_falloff_m=0.35,
        hydrology_add_water=True,
        hydrology_water_depth_fraction=0.5,
        depot_enabled=args.depot,
        depot_cross_row_size=4.0,
    )
    if args.seed4_debug_profile:
        config = FarmGenerationConfig(
            bounds=bounds, seed=4, max_faces=3, row_spacing=5.0,
            tree_spacing=4.0, headland_width=6.0, sideland_width=5.0,
            standoff=3.0, depot_enabled=True, depot_cross_row_size=4.0,
        )
    scene, issues, used_seed = generate_validated(config)
    if issues:
        raise RuntimeError(f"showcase generation failed validation: {issues}")

    save_scene_png(
        scene, str(scene_dir / f"{stem}.png"),
        title=f"farm-gen showcase — seed {used_seed}, 120 m × 120 m",
    )
    save_farm(scene, config, str(scene_dir / f"{stem}.yaml"))
    save_farm(scene, config, str(farm_dir / "farm.yaml"))
    repository_assets = Path(__file__).resolve().parents[1] / "assets"
    bundled_assets = farm_dir / "assets"
    bundled_procedural = farm_dir / "procedural_assets"
    shutil.copytree(repository_assets, bundled_assets, dirs_exist_ok=True)
    shutil.copytree(asset_dir, bundled_procedural, dirs_exist_ok=True)
    tree_assets = tuple(sorted(bundled_procedural.glob("trees/*/*.usda")))
    weed_assets = tuple(sorted(bundled_procedural.glob("weeds/*/*.usda")))
    mesh = export_scene_ground(
        scene,
        bounds,
        str(world_dir / "farm.usda"),
        undulation=ChannelUndulationConfig(
            max_amplitude=0.10,
            min_wavelength=1.0,
            sample_spacing=1.0,
        ),
        tree_assets=tree_assets,
        weed_assets=weed_assets,
        asset_root=bundled_assets,
    )
    rgb_path = farm_dir / "gis" / "_orthophoto_rgb.png"
    if not args.vector_only:
        subprocess.run([
            sys.executable, "-m", "gis.render_orthophoto", str(world_dir / "farm.usda"),
            str(rgb_path), "--bounds", *(str(v) for v in bounds), "--gsd", str(args.gsd),
        ], check=True)
    export_gis(scene, bounds, farm_dir, rgb_path=rgb_path if not args.vector_only else None,
               gsd_m=args.gsd, tiles=not args.vector_only)
    rgb_path.unlink(missing_ok=True)
    bins, assignments = quantize_friction(mesh)
    save_surface_friction_plot(mesh, mesh_dir / f"{stem}_friction.png",
                               [bins[i][2] for i in assignments])
    save_surface_height_plot(mesh, mesh_dir / f"{stem}_height.png")
    if args.depot:
        from dataclasses import replace
        import json
        import numpy as np
        from shutil import copy2
        import matplotlib.pyplot as plt
        from shapely.geometry import LineString, Point, Polygon
        from generation.crops import _depot_exclusion_polygon
        from visualization.debug_view import plot_scene
        from visualization.depot_comparison import save_depot_tree_comparison
        diagnostics = farm_dir / "debug"
        diagnostics.mkdir(parents=True, exist_ok=True)
        copy2(scene_dir / f"{stem}.png", diagnostics / "farm_layout.png")
        copy2(mesh_dir / f"{stem}_friction.png", diagnostics / "friction_fields.png")
        copy2(mesh_dir / f"{stem}_height.png", diagnostics / "height_fields.png")
        fig, ax = plot_scene(scene, title="Tractor depot and parking spots")
        x0, y0, x1, y1 = Polygon(scene.depot.polygon).bounds
        ax.set_xlim(x0 - 8, x1 + 8)
        ax.set_ylim(y0 - 8, y1 + 8)
        fig.savefig(diagnostics / "depot_detail.png", dpi=200, bbox_inches="tight")
        plt.close(fig)
        plain = generate_farm(replace(config, seed=used_seed, depot_enabled=False))
        save_depot_tree_comparison(scene, plain, diagnostics / "depot_tree_comparison.png")
        field_id = scene.depot.field_id
        depot_poly = Polygon(scene.depot.polygon)
        exclusion = _depot_exclusion_polygon(
            depot_poly, np.asarray(scene.depot.row_direction),
            np.asarray(scene.depot.cross_row_direction),
            config.depot_headland_m, config.depot_sideland_m)
        retained = {tuple(round(v, 5) for v in tree.position)
                    for tree in scene.trees.values() if tree.tags.get("parcel_id") == field_id}
        original = [tree for tree in plain.trees.values()
                    if tree.tags.get("parcel_id") == field_id]
        lost = [tree for tree in original
                if tuple(round(v, 5) for v in tree.position) not in retained]
        original_zone = next(zone for zone in plain.weed_zones.values()
                             if zone.tags.get("parcel_id") == field_id)
        unaffected_rows = [str(index) for index, row in enumerate(original_zone.row_centerlines, 1)
                           if not LineString(row).intersects(exclusion)]
        unaffected_trees_preserved = all(
            tuple(round(v, 5) for v in tree.position) in retained
            for tree in original if tree.tags.get("row_number") in unaffected_rows)
        report = {
            "depot_field_id": field_id,
            "depot_area_m2": depot_poly.area,
            "parking_spots": len(scene.depot.parking_spots),
            "depot_headland_m": config.depot_headland_m,
            "depot_sideland_m": config.depot_sideland_m,
            "exclusion_polygon_xy": list(exclusion.exterior.coords[:-1]),
            "original_field_rows": sum(len(zone.row_centerlines) for zone in plain.weed_zones.values()
                                       if zone.tags.get("parcel_id") == field_id),
            "depot_field_rows": sum(len(zone.row_centerlines) for zone in scene.weed_zones.values()
                                    if zone.tags.get("parcel_id") == field_id),
            "original_field_trees": len(original),
            "depot_field_trees": len(retained),
            "lost_trees": len(lost),
            "tree_set_matches_exclusion": retained == {
                tuple(round(v, 5) for v in tree.position) for tree in original
                if not exclusion.covers(Point(tree.position))},
            "farthest_lost_tree_from_depot_m": max((depot_poly.distance(Point(t.position))
                                                     for t in lost), default=0.0),
            "unaffected_row_count": len(unaffected_rows),
            "unaffected_trees_preserved": unaffected_trees_preserved,
            "roads_unchanged": scene.roads == plain.roads,
            "irrigation_unchanged": scene.hydrology == plain.hydrology,
            "crossings_unchanged": scene.crossings == plain.crossings,
        }
        (diagnostics / "depot_geometry_report.json").write_text(json.dumps(report, indent=2) + "\n")
        if not args.vector_only:
            from PIL import Image
            margin = 2.0
            image = Image.open(farm_dir / "gis" / "orthophoto.tif").convert("RGB")
            left = max(0, round((x0 - margin - bounds[0]) / args.gsd))
            top = max(0, round((bounds[3] - y1 - margin) / args.gsd))
            right = min(image.width, round((x1 + margin - bounds[0]) / args.gsd))
            bottom = min(image.height, round((bounds[3] - y0 + margin) / args.gsd))
            image.crop((left, top, right, bottom)).save(diagnostics / "depot_orthophoto.png")
    save_ground_mesh_wireframe(
        mesh,
        str(mesh_dir / f"{stem}_ground_wireframe.png"),
        title=f"hydrology ground mesh — showcase seed {used_seed}, 120 m × 120 m",
    )
    save_ground_mesh_plan(
        mesh,
        str(mesh_dir / f"{stem}_ground_zoom.png"),
        title="showcase mesh detail — x=60–100 m, y=60–100 m",
        bounds=(60.0, 60.0, 100.0, 100.0),
    )
    print(
        f"{stem}: {len(scene.parcels)} parcels, {len(scene.trees)} trees, "
        f"{len(mesh.points)} ground vertices, {len(mesh.triangles)} triangles"
    )


if __name__ == "__main__":
    main()
