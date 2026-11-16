"""Generate a single-surface Isaac Sim driving world with GIS and diagnostics."""

from __future__ import annotations

import argparse
from dataclasses import replace
from pathlib import Path
import shutil
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from export.usd import export_scene_ground, save_ground_mesh_plan, save_ground_mesh_wireframe
from export.usd.friction import quantize_friction
from generation.open_world import (OpenWorldConfig, generate_open_world,
                                   load_open_world, save_open_world)
from generation.orchestrator import SurfaceFrictionConfig
from gis import export_gis
from visualization.surface_friction import save_surface_friction_plot
from visualization.surface_height import save_surface_height_plot


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--surface", choices=("field", "road"), required=True)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--size-m", type=float, default=120.0)
    parser.add_argument("--resolution-m", type=float, default=1.0)
    parser.add_argument("--height-range-m", type=float,
                        help="variation range for the selected surface")
    parser.add_argument("--friction-range", type=float,
                        help="variation range for the selected surface")
    parser.add_argument("--gsd", type=float, default=0.10)
    parser.add_argument("--vector-only", action="store_true",
                        help="skip Isaac Sim render and orthophoto")
    parser.add_argument("--render-only", action="store_true",
                        help="render an existing farm.usda and finish its GIS orthophoto")
    parser.add_argument("--output-root", type=Path, default=Path("debug_out/farms"))
    args = parser.parse_args()
    if args.size_m <= 0 or args.gsd <= 0:
        parser.error("size-m and gsd must be positive")
    stem = f"open_seed_{args.seed}_{args.surface}"
    output = args.output_root / stem
    world = output / "world"
    diagnostics = output / "debug"
    if args.render_only:
        if args.vector_only:
            parser.error("--render-only and --vector-only cannot be combined")
        scene, config = load_open_world(str(output / "farm.yaml"))
        rgb = output / "gis" / "_orthophoto_rgb.png"
        subprocess.run([sys.executable, "-m", "gis.render_orthophoto",
                        str(world / "farm.usda"), str(rgb), "--bounds",
                        *(str(value) for value in config.bounds), "--gsd", str(args.gsd)],
                       check=True)
        export_gis(scene, config.bounds, output, rgb_path=rgb,
                   gsd_m=args.gsd, tiles=True, open_surface=config.surface)
        rgb.unlink(missing_ok=True)
        print(f"{stem}: Isaac Sim orthophoto and GIS complete")
        return
    world.mkdir(parents=True, exist_ok=True)
    diagnostics.mkdir(parents=True, exist_ok=True)
    config = OpenWorldConfig(
        bounds=(0.0, 0.0, args.size_m, args.size_m), seed=args.seed,
        surface=args.surface, farm_resolution_m=args.resolution_m,
        crop_surface=SurfaceFrictionConfig(0.5, 0.90, 10.0, 4.0, 1000 + args.seed),
        road_surface=SurfaceFrictionConfig(1.05, 0.20, 10.0, 4.0, 2000 + args.seed),
        height_seed=1000 + args.seed, road_height_seed=2000 + args.seed,
    )
    if args.height_range_m is not None:
        config = replace(config, **({"height_variation_range_m": args.height_range_m}
                                    if args.surface == "field" else
                                    {"road_height_variation_range_m": args.height_range_m}))
    if args.friction_range is not None:
        selected = config.crop_surface if args.surface == "field" else config.road_surface
        config = replace(config, **({"crop_surface": replace(selected, friction_variation_range=args.friction_range)}
                                    if args.surface == "field" else
                                    {"road_surface": replace(selected, friction_variation_range=args.friction_range)}))
    scene = generate_open_world(config)
    save_open_world(scene, config, str(output / "farm.yaml"))
    shutil.copytree(Path(__file__).resolve().parents[1] / "assets", output / "assets",
                    dirs_exist_ok=True)
    mesh = export_scene_ground(scene, config.bounds, str(world / "farm.usda"),
                               surface_region=config.surface_region,
                               asset_root=output / "assets")
    bins, assignments = quantize_friction(mesh)
    save_surface_friction_plot(mesh, diagnostics / "friction.png",
                               [bins[index][2] for index in assignments],
                               open_surface=args.surface)
    save_surface_height_plot(mesh, diagnostics / "height.png", open_surface=args.surface)
    save_ground_mesh_wireframe(mesh, str(diagnostics / "ground_wireframe.png"),
                               title=f"Open {args.surface} ground mesh — {args.size_m:g} m")
    middle = args.size_m / 2
    half = min(20.0, args.size_m / 3)
    save_ground_mesh_plan(mesh, str(diagnostics / "ground_center.png"),
                          title=f"Open {args.surface} mesh — central {2 * half:g} m",
                          bounds=(middle - half, middle - half,
                                  middle + half, middle + half))
    rgb = output / "gis" / "_orthophoto_rgb.png"
    if not args.vector_only:
        subprocess.run([sys.executable, "-m", "gis.render_orthophoto",
                        str(world / "farm.usda"), str(rgb), "--bounds",
                        *(str(value) for value in config.bounds), "--gsd", str(args.gsd)],
                       check=True)
    export_gis(scene, config.bounds, output, rgb_path=rgb if not args.vector_only else None,
               gsd_m=args.gsd, tiles=not args.vector_only, open_surface=args.surface)
    rgb.unlink(missing_ok=True)
    print(f"{stem}: {len(mesh.points)} vertices, {len(mesh.triangles)} triangles, "
          f"{args.surface} visual surface, {'vector GIS' if args.vector_only else 'orthophoto'}")


if __name__ == "__main__":
    main()
