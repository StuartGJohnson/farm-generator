"""Generate one large, presentation-oriented farm scene."""

from pathlib import Path

from export.usd import (
    ChannelUndulationConfig,
    export_scene_ground,
    save_ground_mesh_plan,
    save_ground_mesh_wireframe,
)
from generation.orchestrator import FarmGenerationConfig, SurfaceFrictionConfig, generate_validated, save_farm
from visualization.debug_view import save_scene_png
from visualization.surface_friction import save_surface_friction_plot
from visualization.surface_height import save_surface_height_plot
from export.usd.friction import quantize_friction


def main() -> None:
    seed = 1
    bounds = (0.0, 0.0, 120.0, 120.0)
    stem = f"farm_seed_{seed}_120m2"
    scene_dir = Path("debug_out") / "farm_scenes"
    mesh_dir = Path("debug_out") / "mesh"
    asset_dir = Path("debug_out") / "procedural_assets"
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
    )
    scene, issues, used_seed = generate_validated(config)
    if issues:
        raise RuntimeError(f"showcase generation failed validation: {issues}")

    save_scene_png(
        scene, str(scene_dir / f"{stem}.png"),
        title=f"farm-gen showcase — seed {used_seed}, 120 m × 120 m",
    )
    save_farm(scene, config, str(scene_dir / f"{stem}.yaml"))
    mesh = export_scene_ground(
        scene,
        bounds,
        str(mesh_dir / f"{stem}_ground.usda"),
        undulation=ChannelUndulationConfig(
            max_amplitude=0.10,
            min_wavelength=1.0,
            sample_spacing=1.0,
        ),
        tree_assets=tree_assets,
        weed_assets=weed_assets,
    )
    bins, assignments = quantize_friction(mesh)
    save_surface_friction_plot(mesh, mesh_dir / f"{stem}_friction.png",
                               [bins[i][2] for i in assignments])
    save_surface_height_plot(mesh, mesh_dir / f"{stem}_height.png")
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
