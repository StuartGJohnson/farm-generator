"""Refresh showcase ground meshes and height diagnostics without rewriting GIS."""

import argparse
import json
from pathlib import Path
from shutil import copy2

import numpy as np

from export.usd import (ChannelUndulationConfig, export_scene_ground,
                        save_ground_mesh_plan, save_ground_mesh_wireframe)
from export.usd.friction import quantize_friction
from generation.orchestrator import generate_validated, load_farm
from visualization.surface_friction import save_surface_friction_plot
from visualization.surface_height import save_surface_height_plot


STEMS = ("farm_seed_1_120m2", "farm_seed_1_120m2_depot")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stem", choices=STEMS,
                        help="refresh only this bundle (default: both)")
    args = parser.parse_args()
    mesh_dir = Path("debug_out/mesh")
    mesh_dir.mkdir(parents=True, exist_ok=True)

    for stem in (args.stem,) if args.stem else STEMS:
        farm_dir = Path("debug_out/farms") / stem
        _, config = load_farm(str(farm_dir / "farm.yaml"))
        scene, issues, used_seed = generate_validated(config)
        if issues or used_seed != config.seed:
            raise RuntimeError(f"{stem}: farm generation changed seed or failed validation")
        tree_assets = tuple(sorted((farm_dir / "procedural_assets").glob("trees/*/*.usda")))
        weed_assets = tuple(sorted((farm_dir / "procedural_assets").glob("weeds/*/*.usda")))
        mesh = export_scene_ground(
            scene, config.bounds, str(mesh_dir / f"{stem}_ground.usda"),
            undulation=ChannelUndulationConfig(max_amplitude=0.10,
                                               min_wavelength=1.0,
                                               sample_spacing=1.0),
            tree_assets=tree_assets, weed_assets=weed_assets,
            asset_root=farm_dir / "assets",
        )
        height_path = mesh_dir / f"{stem}_height.png"
        friction_path = mesh_dir / f"{stem}_friction.png"
        save_surface_height_plot(mesh, height_path)
        bins, assignments = quantize_friction(mesh)
        save_surface_friction_plot(mesh, friction_path,
                                   [bins[index][2] for index in assignments])
        save_ground_mesh_wireframe(mesh, str(mesh_dir / f"{stem}_ground_wireframe.png"))
        save_ground_mesh_plan(mesh, str(mesh_dir / f"{stem}_ground_zoom.png"),
                              bounds=(60, 60, 100, 100))
        crop = np.asarray(scene.surface_height.values)
        road = np.asarray(scene.surface_height.road_values)
        stats = {
            "crop_height_std_m": float(crop.std()),
            "road_height_std_m": float(road.std()),
            "crop_road_correlation": float(np.corrcoef(crop.ravel(), road.ravel())[0, 1]),
            "crop_height_range_m": config.height_variation_range_m,
            "road_height_range_m": config.road_height_variation_range_m,
            "road_field_blend_m": config.road_field_blend_m,
        }
        (mesh_dir / f"{stem}_height_stats.json").write_text(json.dumps(stats, indent=2) + "\n")
        if scene.depot is not None:
            debug_dir = farm_dir / "debug"
            debug_dir.mkdir(parents=True, exist_ok=True)
            copy2(height_path, debug_dir / "height_fields.png")
            copy2(friction_path, debug_dir / "friction_fields.png")
        print(f"{stem}: {len(mesh.points)} vertices, "
              f"crop σ={stats['crop_height_std_m']:.3f} m, "
              f"road/depot σ={stats['road_height_std_m']:.3f} m, "
              f"correlation={stats['crop_road_correlation']:.3f}", flush=True)


if __name__ == "__main__":
    main()
