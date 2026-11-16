"""Write open-world meshes and diagnostics in debug_out/mesh."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from export.usd import export_scene_ground, save_ground_mesh_plan, save_ground_mesh_wireframe
from export.usd.friction import quantize_friction
from generation.open_world import load_open_world
from visualization.surface_friction import save_surface_friction_plot
from visualization.surface_height import save_surface_height_plot


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stem", action="append",
                        help="open-world directory name; default: both seed 1 worlds")
    parser.add_argument("--output-dir", type=Path, default=Path("debug_out/mesh"))
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    stems = args.stem or ("open_seed_1_field", "open_seed_1_road")
    for stem in stems:
        world_dir = Path("debug_out/farms") / stem
        scene, config = load_open_world(str(world_dir / "farm.yaml"))
        mesh = export_scene_ground(
            scene, config.bounds, str(args.output_dir / f"{stem}_ground.usda"),
            surface_region=config.surface_region,
            asset_root=world_dir / "assets",
        )
        bins, assignments = quantize_friction(mesh)
        save_surface_friction_plot(
            mesh, args.output_dir / f"{stem}_friction.png",
            [bins[index][2] for index in assignments], open_surface=config.surface)
        save_surface_height_plot(
            mesh, args.output_dir / f"{stem}_height.png", open_surface=config.surface)
        save_ground_mesh_wireframe(
            mesh, str(args.output_dir / f"{stem}_ground_wireframe.png"),
            title=f"Open {config.surface} ground mesh — {config.bounds[2] - config.bounds[0]:g} m")
        x0, y0, x1, y1 = config.bounds
        center_x, center_y = (x0 + x1) / 2, (y0 + y1) / 2
        half = min(20.0, (x1 - x0) / 3, (y1 - y0) / 3)
        save_ground_mesh_plan(
            mesh, str(args.output_dir / f"{stem}_ground_zoom.png"),
            title=f"Open {config.surface} mesh — central {2 * half:g} m",
            bounds=(center_x - half, center_y - half,
                    center_x + half, center_y + half))
        elevations = np.asarray(mesh.points)[:, 2]
        friction = np.asarray(mesh.face_friction)
        stats = {
            "surface": config.surface,
            "bounds_m": config.bounds,
            "vertices": len(mesh.points),
            "triangles": len(mesh.triangles),
            "height_min_m": float(elevations.min()),
            "height_max_m": float(elevations.max()),
            "height_std_m": float(elevations.std()),
            "friction_min": float(friction.min()),
            "friction_max": float(friction.max()),
            "friction_mean": float(friction.mean()),
            "visual_material": ("GrassMaterial" if config.surface == "field"
                                else "GravelRoadMaterial"),
        }
        (args.output_dir / f"{stem}_height_stats.json").write_text(
            json.dumps(stats, indent=2) + "\n")
        print(f"{stem}: {stats['triangles']} faces, "
              f"height [{stats['height_min_m']:.3f}, {stats['height_max_m']:.3f}] m, "
              f"friction [{stats['friction_min']:.3f}, {stats['friction_max']:.3f}]",
              flush=True)


if __name__ == "__main__":
    main()
