"""Headless Vehicle 2 material/contact verification and equal-input traction A/B."""

import argparse
import json
from pathlib import Path
import sys
import traceback

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", default="debug_out/mesh/farm_seed_1_120m2_ground.usda")
    parser.add_argument("--output-dir", default="debug_out/friction_check")
    args = parser.parse_args()
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True, "multi_gpu": False})
    try:
        import omni.usd
        from omni.physx import get_physx_interface
        from omni.physx.bindings import _physx
        from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema
        from bringup.surface_friction import configure_vehicle_surface_friction
        from export.usd import GroundMesh, write_ground_mesh_usda
        from generation.orchestrator import SurfaceFrictionConfig
        from generation.orchestrator import FarmGenerationConfig
        from generation.surface_friction import generate_surface_friction, sample_mesh_friction
        from tractor_generation.config import TractorConfig
        from tractor_generation.names import WHEELS
        from tractor_generation.usd import write_tractor_usda

        output = Path(args.output_dir).resolve()
        output.mkdir(parents=True, exist_ok=True)
        tractor_asset = write_tractor_usda(TractorConfig(), output / "tractor.usda")
        physics = get_physx_interface()

        def run(path, position, throttle, steps):
            omni.usd.get_context().open_stage(str(Path(path).resolve()))
            for _ in range(3):
                app.update()
            stage = omni.usd.get_context().get_stage()
            tractor = stage.DefinePrim('/World/Tractor', 'Xform')
            tractor.GetReferences().AddReference(str(tractor_asset))
            UsdGeom.Xformable(tractor).AddTranslateOp().Set(Gf.Vec3d(*position))
            nested = stage.GetPrimAtPath('/World/Tractor/PhysicsScene')
            if nested:
                nested.SetActive(False)
            assert configure_vehicle_surface_friction(stage) == 4
            assert len([p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]) == 1
            # Scene vegetation is irrelevant to this physics-only test.
            vegetation = stage.GetPrimAtPath('/World/Vegetation')
            if vegetation:
                vegetation.SetActive(False)
            table = PhysxSchema.PhysxVehicleTireFrictionTable.Get(stage, '/World/TireFrictionTable')
            coefficients = dict(zip(map(str, table.GetGroundMaterialsRel().GetTargets()), table.GetFrictionValuesAttr().Get()))
            vehicle = stage.GetPrimAtPath('/World/Tractor/base_link')
            controller = PhysxSchema.PhysxVehicleControllerAPI(vehicle)
            controller.GetAcceleratorAttr().Set(0)
            physics.start_simulation()
            observations = []
            for step in range(steps + 60):
                if step == 60:
                    controller.GetAcceleratorAttr().Set(throttle)
                physics.update_simulation(1/60, step/60)
                physics.update_transformations(False, True, True)
                if step >= 60 and step % 10 == 0:
                    for wheel in WHEELS:
                        state = physics.get_wheel_state('/World/Tractor/base_link/' + wheel)
                        if state and state[_physx.VEHICLE_WHEEL_STATE_IS_ON_GROUND]:
                            material = state[_physx.VEHICLE_WHEEL_STATE_GROUND_MATERIAL]
                            mu = state[_physx.VEHICLE_WHEEL_STATE_TIRE_FRICTION]
                            assert material in coefficients, f"Unknown material at wheel: {material}"
                            nominal = coefficients[material]
                            # Existing tire slip graph ranges from 1 down to .85.
                            assert nominal*.85-1e-4 <= mu <= nominal+1e-4, (material, nominal, mu)
                            observations.append({"step": step, "wheel": wheel, "material": material,
                                                 "surface_mu": nominal, "effective_mu": mu,
                                                 "hit": list(state[_physx.VEHICLE_WHEEL_STATE_GROUND_HIT_POSITION]),
                                                 "suspension_force": list(state[_physx.VEHICLE_WHEEL_STATE_SUSPENSION_FORCE]),
                                                 "tire_force": list(state[_physx.VEHICLE_WHEEL_STATE_TIRE_FORCE]),
                                                 "longitudinal_slip": state[_physx.VEHICLE_WHEEL_STATE_TIRE_LONGITUDINAL_SLIP],
                                                 "lateral_slip": state[_physx.VEHICLE_WHEEL_STATE_TIRE_LATERAL_SLIP]})
            final_position = UsdGeom.Xformable(vehicle).ComputeLocalToWorldTransform(0).ExtractTranslation()
            velocity = UsdPhysics.RigidBodyAPI(vehicle).GetVelocityAttr().Get()
            report = {"position": list(final_position), "velocity": list(velocity),
                      "materials": sorted({o['material'] for o in observations}),
                      "observations": observations}
            assert len(observations) >= 4, "No sustained wheel contacts"
            physics.reset_simulation()
            print(f"CONTACT_PASS {path}: {len(observations)} samples, {len(report['materials'])} materials", flush=True)
            return report

        reports = {}
        for label, mu in (("low_grip", 0.15), ("high_grip", 1.05)):
            fields = generate_surface_friction(FarmGenerationConfig(
                bounds=(-100,-100,100,100), seed=1,
                road_surface=SurfaceFrictionConfig(mu,0,5,4,2001)))
            points = [(-100,-100,0),(100,-100,0),(100,100,0),(-100,100,0)]
            triangles, classes = [(0,1,2),(0,2,3)], ['road','road']
            mesh = GroundMesh(points, triangles, [], [], [], classes, [], fields,
                              sample_mesh_friction(fields, points, triangles, classes))
            path = output / f'{label}.usda'
            write_ground_mesh_usda(mesh, str(path))
            reports[label] = run(path, (0,0,0.05), 1.0, 240)
        low, high = reports['low_grip']['velocity'][0], reports['high_grip']['velocity'][0]
        assert high > low + 0.5, f"Expected stronger acceleration on high grip: {low=}, {high=}"
        reports['farm'] = run(args.world, (20,3.5,0.05), 0.5, 300)
        assert len(reports['farm']['materials']) >= 2, "Farm run did not encounter varying materials"
        hit_heights = [sample['hit'][2] for sample in reports['farm']['observations']]
        assert max(hit_heights) - min(hit_heights) > 0.03, "Vehicle 2 did not query varied terrain heights"
        destination = output / 'report.json'
        destination.write_text(json.dumps(reports, indent=2) + '\n')
        print(f"FRICTION_RUNTIME_PASS low={low:.3f} high={high:.3f} m/s; {destination}", flush=True)
    except BaseException:
        traceback.print_exc()
        app.close(exit_code=1)
        raise
    else:
        app.close()


if __name__ == '__main__':
    main()
