"""Depot fence layout and portable USD assembly."""

from dataclasses import replace
import math

import pytest
from pxr import Gf, Usd, UsdGeom, UsdPhysics
from shapely.geometry import LineString, Polygon

from export.usd import build_ground_mesh, write_ground_mesh_usda
from generation.asset_generation.fence_generator import generate_fence_assets
from generation.depot_fence import FENCE_SECTION_LENGTH_M, fence_placements
from generation.orchestrator import FarmGenerationConfig, generate_farm, load_farm, save_farm


def test_depot_fence_uses_whole_panels_and_one_corner_post(tmp_path):
    config = FarmGenerationConfig(
        bounds=(0, 0, 55, 55), seed=1, max_faces=1,
        row_spacing=5, tree_spacing=4, headland_width=6, sideland_width=5,
        standoff=3, depot_enabled=True, include_depot_fence=True,
        depot_cross_row_size=3,
    )
    scene = generate_farm(config)
    plain = generate_farm(replace(config, include_depot_fence=False))
    depot = scene.depot
    assert len(depot.fence_lines) == 2
    assert not plain.depot.fence_lines
    assert scene.roads == plain.roads
    assert scene.crossings == plain.crossings

    polygon = Polygon(depot.polygon)
    for line, span in zip(depot.fence_lines,
                          (depot.along_row_length_m, depot.cross_row_width_m)):
        segment = LineString(line)
        assert segment.length == pytest.approx(
            math.floor((span + 1e-8) / FENCE_SECTION_LENGTH_M) * FENCE_SECTION_LENGTH_M)
        assert polygon.boundary.distance(segment) < 1e-6
    posts, panels = fence_placements(depot.fence_lines)
    assert len(posts) == len(panels) + 1
    assert posts[0] == pytest.approx(depot.fence_lines[0][0])
    assert sum(1 for point in posts if point == posts[0]) == 1

    farm_path = tmp_path / "farm.yaml"
    save_farm(scene, config, str(farm_path))
    loaded, loaded_config = load_farm(str(farm_path))
    assert loaded.depot.fence_lines == depot.fence_lines
    assert loaded_config.include_depot_fence

    fence_assets = generate_fence_assets(tmp_path / "assets" / "fences")
    mesh = build_ground_mesh(scene, config.bounds)
    world_path = tmp_path / "world" / "farm.usda"
    write_ground_mesh_usda(mesh, str(world_path), scene=scene,
                           fence_assets=fence_assets)
    stage = Usd.Stage.Open(str(world_path))
    assert stage is not None
    root = stage.GetPrimAtPath("/World/DepotFence")
    assert len([p for p in root.GetChildren() if p.GetName().startswith("Post_")]) == len(posts)
    assert len([p for p in root.GetChildren() if p.GetName().startswith("Panel_")]) == len(panels)
    xforms = UsdGeom.XformCache()
    for index in range(len(posts)):
        prim = stage.GetPrimAtPath(f"/World/DepotFence/Post_{index:03d}")
        shaft = prim.GetChild("Shaft")
        assert shaft.IsValid()
        assert tuple(xforms.GetLocalToWorldTransform(prim).Transform(Gf.Vec3d(0, 0, 0)))[2] == 0
        assert UsdGeom.Cylinder(shaft).GetAxisAttr().Get() == "Z"
    for index in range(len(panels)):
        panel = stage.GetPrimAtPath(f"/World/DepotFence/Panel_{index:03d}")
        transform = xforms.GetLocalToWorldTransform(panel)
        start = transform.Transform(Gf.Vec3d(0, 0, 0))
        end = transform.Transform(Gf.Vec3d(FENCE_SECTION_LENGTH_M, 0, 0))
        assert tuple(start) == pytest.approx((*panels[index][:2], 0))
        assert any(tuple(end) == pytest.approx((*xy, 0)) for xy in posts)
        collider = panel.GetChild("CollisionPlane")
        assert collider.HasAPI(UsdPhysics.CollisionAPI)
        assert collider.GetAttribute("visibility").Get() == "invisible"
