"""Headless regression: actual Carb key dispatch, braking, and resumed motion."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', default='debug_out/mesh/farm_seed_1_120m2_ground.usda')
    args = parser.parse_args()
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True})
    teleop = None
    try:
        import carb.input
        import omni.timeline
        import omni.usd
        from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema
        from bringup.teleop import TractorKeyboardTeleop
        from bringup.surface_friction import configure_vehicle_surface_friction
        context = omni.usd.get_context()
        context.open_stage(str(Path(args.world).resolve()))
        app.update()
        stage = context.get_stage()
        tractor = stage.DefinePrim('/World/Tractor', 'Xform')
        tractor.GetReferences().AddReference(str(Path('debug_out/tractor/tractor.usda').resolve()))
        UsdGeom.Xformable(tractor).AddTranslateOp().Set(Gf.Vec3d(20, 3.5, .05))
        stage.GetPrimAtPath('/World/Tractor/PhysicsScene').SetActive(False)
        assert configure_vehicle_surface_friction(stage) == 4
        stage.SetStartTimeCode(0)
        stage.SetEndTimeCode(86400 * stage.GetTimeCodesPerSecond())
        timeline = omni.timeline.get_timeline_interface()
        timeline.set_looping(False)
        vehicle = stage.GetPrimAtPath('/World/Tractor/base_link')
        teleop = TractorKeyboardTeleop(vehicle)
        controller = PhysxSchema.PhysxVehicleControllerAPI(vehicle)
        provider = carb.input.acquire_input_provider()
        input_interface = carb.input.acquire_input_interface()
        leaked = []
        # Same priority as Kit's hotkey dispatcher: driving keys must not arrive.
        downstream = input_interface.subscribe_to_input_events(lambda e: leaked.append(e) or True, order=0)
        key = carb.input.KeyboardInput
        def event(button, down):
            provider.buffer_keyboard_key_event(teleop._keyboard,
                carb.input.KeyboardEventType.KEY_PRESS if down else carb.input.KeyboardEventType.KEY_RELEASE,
                button, 0)
            input_interface.distribute_buffered_events()
            teleop.update()
        def advance(seconds):
            end = timeline.get_current_time() + seconds
            for _ in range(10000):
                teleop.update()
                app.update()
                assert timeline.is_playing(), 'Driving key paused simulation'
                if timeline.get_current_time() >= end:
                    return
            raise AssertionError('Timeline failed to advance')
        def speed():
            return UsdPhysics.RigidBodyAPI(vehicle).GetVelocityAttr().Get()[0]
        timeline.play()
        advance(1)
        event(key.W, True)
        advance(3)
        moving = speed()
        event(key.SPACE, True)
        assert controller.GetAcceleratorAttr().Get() == 0
        assert controller.GetBrake0Attr().Get() == 1
        advance(3)
        stopped = speed()
        event(key.SPACE, False)
        assert controller.GetBrake0Attr().Get() == 0
        assert controller.GetAcceleratorAttr().Get() == 1
        advance(3)
        resumed = speed()
        event(key.W, False)
        event(key.SPACE, True)
        advance(3)
        event(key.SPACE, False)
        event(key.S, True)
        advance(3)
        reverse = speed()
        assert moving > 1 and abs(stopped) < .2 and resumed > 1 and reverse < -1, (moving, stopped, resumed, reverse)
        assert not leaked, 'Driving keys reached downstream hotkeys'
        for name in ('front_left_wheel', 'front_right_wheel', 'rear_left_wheel', 'rear_right_wheel'):
            tire = PhysxSchema.PhysxVehicleTireAPI(stage.GetPrimAtPath('/World/Tractor/base_link/' + name))
            assert list(map(str, tire.GetFrictionTableRel().GetTargets())) == ['/World/TireFrictionTable']
        input_interface.unsubscribe_to_input_events(downstream)
        print(f'TELEOP_PASS forward={moving:.3f}, braked={stopped:.3f}, resumed={resumed:.3f}, reverse={reverse:.3f} m/s', flush=True)
        timeline.stop()
    except BaseException:
        import traceback
        traceback.print_exc()
        if teleop is not None:
            teleop.close()
            teleop = None
        app.close(exit_code=1)
        raise
    finally:
        if teleop is not None:
            teleop.close()
    app.close()


if __name__ == '__main__':
    main()
