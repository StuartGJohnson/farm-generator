"""USD export and exporter-side terrain baking."""

from .friction import UsdFrictionConfig

from .ground_mesh import (
    ChannelUndulationConfig,
    ChannelFace,
    GroundMesh,
    PlanarStraightLineGraph,
    RoadSurfacePatch,
    WaterSurfaceMesh,
    build_channel_faces,
    build_ground_mesh,
    build_hydrology_pslg,
    build_road_surface_patches,
    build_undulated_hydrology_edges,
    build_water_surface_meshes,
    export_scene_ground,
    save_ground_mesh_plan,
    save_ground_mesh_wireframe,
    write_ground_mesh_usda,
)

__all__ = [
    "UsdFrictionConfig",
    "ChannelUndulationConfig",
    "ChannelFace",
    "GroundMesh",
    "PlanarStraightLineGraph",
    "RoadSurfacePatch",
    "WaterSurfaceMesh",
    "build_channel_faces",
    "build_ground_mesh",
    "build_hydrology_pslg",
    "build_road_surface_patches",
    "build_undulated_hydrology_edges",
    "build_water_surface_meshes",
    "export_scene_ground",
    "save_ground_mesh_plan",
    "save_ground_mesh_wireframe",
    "write_ground_mesh_usda",
]
