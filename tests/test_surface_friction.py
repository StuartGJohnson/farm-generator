from dataclasses import replace

import numpy as np
import pytest

from generation.orchestrator import SurfaceFrictionConfig
from farm_ir.serialize import to_plain
from generation.orchestrator import FarmGenerationConfig, generate_farm, load_farm, save_farm
from generation.surface_friction import generate_field, sample_field, face_surface_regions


def test_seeded_bounded_field_and_continuous_sampling():
    params = SurfaceFrictionConfig()
    bounds = (12, -7, 76, 57)
    field = generate_field(params, bounds, 1)
    values = np.asarray(field.values)
    assert field == generate_field(params, bounds, 1)
    assert field != generate_field(replace(params, friction_seed=2), bounds, 1)
    assert values.mean() == pytest.approx(0.65)
    assert values.min() >= 0.5 and values.max() <= 0.8
    xy = np.random.default_rng(9).uniform((12, -7), (76, 57), (1000, 2))
    samples = sample_field(field, xy)
    assert samples.min() >= 0.5 and samples.max() <= 0.8
    assert np.max(np.abs(sample_field(field, xy+1e-8)-samples)) < 1e-7
    assert np.allclose(samples, sample_field(field, xy + (64, 64)))
    assert np.all(sample_field(generate_field(replace(params, friction_variation_range=0), bounds, 1), xy) == 0.65)


def test_spectrum_transfer_and_radial_nyquist():
    params = SurfaceFrictionConfig(friction_spectral_slope=4)
    size = 128
    values = np.asarray(generate_field(params, (0, 0, size, size), 1).values)
    white = np.random.default_rng(params.friction_seed).standard_normal(values.shape)
    freq = np.fft.fftfreq(size, d=1)
    f = np.hypot(freq[:, None], freq[None, :])
    observed = np.abs(np.fft.fft2(values-values.mean()))**2
    expected = np.abs(np.fft.fft2(white))**2 * (1+(f*params.friction_scale_m)**2)**(-params.friction_spectral_slope/2)
    valid = (f > 0) & (f <= 0.5) & (expected > 1e-8)
    ratios = observed[valid]/expected[valid]
    assert np.std(ratios)/np.mean(ratios) < 1e-9
    assert observed[f > 0.5].max() < 1e-20
    smoother = np.asarray(generate_field(replace(params, friction_spectral_slope=8), (0,0,size,size), 1).values)
    power = np.abs(np.fft.fft2(smoother-smoother.mean()))**2
    assert power[f > .3].sum()/power.sum() < observed[f > .3].sum()/observed.sum()


@pytest.mark.parametrize("kwargs", [{"friction_mean": .1}, {"friction_scale_m": 0},
                                   {"friction_variation_range": -1}, {"friction_spectral_slope": -1},
                                   {"friction_seed": -1}, {"friction_mean": float('nan')}])
def test_invalid_field_parameters(kwargs):
    with pytest.raises(ValueError):
        SurfaceFrictionConfig(**kwargs)


def test_config_round_trip_legacy_and_independent_seed(tmp_path):
    config = FarmGenerationConfig(bounds=(0,0,16,16), seed=4, max_faces=4)
    scene = generate_farm(config)
    path = str(tmp_path / "farm.json")
    save_farm(scene, config, path)
    restored, restored_config = load_farm(path)
    assert to_plain(restored) == to_plain(scene)
    assert to_plain(restored_config) == to_plain(config)
    changed = generate_farm(replace(config, crop_surface=SurfaceFrictionConfig(friction_seed=3)))
    assert changed.parcels == scene.parcels and changed.trees == scene.trees
    assert changed.surface_friction.road == scene.surface_friction.road
    import json
    doc = json.loads((tmp_path / "farm.json").read_text())
    doc['config']['road_surface'] = 'gravel'
    doc['config'].pop('road_surface_type')
    doc['scene'].pop('surface_friction')
    (tmp_path / 'farm.json').write_text(json.dumps(doc))
    legacy, legacy_config = load_farm(path)
    assert legacy.surface_friction == scene.surface_friction
    assert legacy_config.road_surface_type.value == 'gravel'


def test_usd_material_partition_and_vehicle_binding(tmp_path):
    pytest.importorskip("pxr.PhysxSchema")
    from pxr import Usd, UsdGeom, UsdShade, UsdPhysics, PhysxSchema
    from export.usd import build_ground_mesh, write_ground_mesh_usda, UsdFrictionConfig
    from export.usd.friction import quantize_friction
    from tractor_generation.config import TractorConfig
    from tractor_generation.usd import write_tractor_usda
    from bringup.surface_friction import configure_vehicle_surface_friction
    config = FarmGenerationConfig(bounds=(0,0,16,16), seed=4, max_faces=4, hydrology_add_water=True)
    mesh = build_ground_mesh(generate_farm(config), config.bounds)
    regions = mesh.face_surface_regions
    for kind, mu in zip(regions, mesh.face_friction):
        low, high = {'crop':(.5,.8), 'road':(.95,1.15), 'channel':(.3,.3)}[kind]
        assert low-1e-12 <= mu <= high+1e-12
    bins, assignments = quantize_friction(mesh)
    assert len(bins) == 21
    for i, b in enumerate(assignments):
        assert abs(mesh.face_friction[i]-bins[b][2]) <= {'crop':.015,'road':.01,'channel':0}[regions[i]]+1e-12
    path = tmp_path/'world.usda'
    write_ground_mesh_usda(mesh, str(path))
    stage = Usd.Stage.Open(str(path))
    ground = UsdGeom.Mesh.Get(stage, '/World/Ground')
    subsets = UsdShade.MaterialBindingAPI(ground).GetMaterialBindSubsets()
    faces = [i for sub in subsets for i in sub.GetIndicesAttr().Get()]
    assert sorted(faces) == list(range(len(mesh.triangles)))
    for subset in subsets:
        api = UsdShade.MaterialBindingAPI(subset)
        material = api.ComputeBoundMaterial('physics')[0]
        assert material.GetPrim().HasAPI(UsdPhysics.MaterialAPI)
        assert api.ComputeBoundMaterial()[0].GetPath().HasPrefix('/World/Looks')
    table = PhysxSchema.PhysxVehicleTireFrictionTable.Get(stage, '/World/TireFrictionTable')
    assert len(table.GetGroundMaterialsRel().GetTargets()) == len(table.GetFrictionValuesAttr().Get()) == 21
    tractor = write_tractor_usda(TractorConfig(), tmp_path/'tractor.usda')
    stage.DefinePrim('/World/Tractor').GetReferences().AddReference(str(tractor))
    assert configure_vehicle_surface_friction(stage) == 4
    for prim in stage.Traverse():
        if prim.HasAPI(PhysxSchema.PhysxVehicleTireAPI):
            assert PhysxSchema.PhysxVehicleTireAPI(prim).GetFrictionTableRel().GetTargets() == [table.GetPath()]
    constant_config = replace(config, crop_surface=SurfaceFrictionConfig(friction_variation_range=0),
                              road_surface=SurfaceFrictionConfig(1.05,0,5,4,2001))
    constant_mesh = build_ground_mesh(generate_farm(constant_config), config.bounds)
    assert len(quantize_friction(constant_mesh)[0]) == 3
    with pytest.raises(ValueError):
        UsdFrictionConfig(0)
