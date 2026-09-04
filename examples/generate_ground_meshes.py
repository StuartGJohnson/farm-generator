"""Generate five small farm IRs and their hydrology-derived USD meshes."""

import os

from export.usd import ChannelUndulationConfig, export_scene_ground, save_ground_mesh_wireframe
from generation.orchestrator import FarmGenerationConfig, generate_validated, save_farm
from visualization.debug_view import save_scene_png


def main() -> None:
    scene_dir = os.path.join("debug_out", "farm_scenes")
    mesh_dir = os.path.join("debug_out", "mesh")
    os.makedirs(scene_dir, exist_ok=True)
    os.makedirs(mesh_dir, exist_ok=True)
    bounds = (0.0, 0.0, 50.0, 50.0)

    for seed in [1, 2, 3, 4, 5]:
        config = FarmGenerationConfig(
            bounds=bounds,
            seed=seed,
            max_faces=4,
            standoff=3.5,
            headland_width=7.5,
            sideland_width=6.5,
            hydrology_add_water=True,
            hydrology_water_depth_fraction=0.5,
        )
        scene, issues, used_seed = generate_validated(config)
        status = "OK" if not issues else f"{len(issues)} ISSUE(S)"
        retry = f" (used seed {used_seed} after retry)" if used_seed != seed else ""
        print(f"seed={seed}{retry}: {len(scene.parcels)} parcels -- {status}")
        save_scene_png(scene, os.path.join(scene_dir, f"farm_seed_{seed}.png"), title=f"farm-gen debug view\nseed={seed}")
        save_farm(scene, config, os.path.join(scene_dir, f"farm_seed_{seed}.yaml"))
        mesh = export_scene_ground(
            scene,
            bounds,
            os.path.join(mesh_dir, f"farm_seed_{seed}_ground.usda"),
            undulation=ChannelUndulationConfig(
                max_amplitude=0.10,
                min_wavelength=1.0,
                sample_spacing=1.0,
            ),
        )
        save_ground_mesh_wireframe(
            mesh,
            os.path.join(mesh_dir, f"farm_seed_{seed}_ground_wireframe.png"),
            title=f"hydrology ground mesh — seed {seed}",
        )
        water_vertices = sum(len(surface.points) for surface in mesh.water_surfaces)
        water_triangles = sum(len(surface.triangles) for surface in mesh.water_surfaces)
        print(
            f"    ground: {len(mesh.points)} vertices, {len(mesh.triangles)} triangles; "
            f"water: {len(mesh.water_surfaces)} surfaces, {water_vertices} vertices, "
            f"{water_triangles} triangles"
        )


if __name__ == "__main__":
    main()
