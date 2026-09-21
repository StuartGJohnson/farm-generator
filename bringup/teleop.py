"""Non-ROS keyboard controls for a PhysX Vehicle 2 tractor."""

from __future__ import annotations


class TractorKeyboardTeleop:
    """Map WASD and Space onto a PhysxVehicleControllerAPI."""

    def __init__(self, vehicle_prim) -> None:
        import carb.input
        import omni.appwindow
        from pxr import PhysxSchema

        self._carb_input = carb.input
        self._pressed: set[object] = set()
        self._controller = PhysxSchema.PhysxVehicleControllerAPI(vehicle_prim)
        if not self._controller:
            raise ValueError(f"{vehicle_prim.GetPath()} has no PhysxVehicleControllerAPI")
        self._input = carb.input.acquire_input_interface()
        self._keyboard = omni.appwindow.get_default_app_window().get_keyboard()
        self._subscription = self._input.subscribe_to_keyboard_events(
            self._keyboard, self._on_keyboard
        )

    def _on_keyboard(self, event, *_args) -> bool:
        event_type = event.type
        if event_type in (
            self._carb_input.KeyboardEventType.KEY_PRESS,
            self._carb_input.KeyboardEventType.KEY_REPEAT,
        ):
            self._pressed.add(event.input)
        elif event_type == self._carb_input.KeyboardEventType.KEY_RELEASE:
            self._pressed.discard(event.input)
        return True

    def update(self) -> None:
        key = self._carb_input.KeyboardInput
        forward = key.W in self._pressed
        reverse = key.S in self._pressed
        left = key.A in self._pressed
        right = key.D in self._pressed
        conflicting_throttle = forward and reverse

        self._controller.GetAcceleratorAttr().Set(
            0.0 if conflicting_throttle else float(forward or reverse)
        )
        self._controller.GetTargetGearAttr().Set(-1 if reverse and not forward else 1)
        self._controller.GetSteerAttr().Set(float(left) - float(right))
        self._controller.GetBrake0Attr().Set(
            float(conflicting_throttle or key.SPACE in self._pressed)
        )

    def close(self) -> None:
        if self._subscription is not None:
            self._input.unsubscribe_to_keyboard_events(self._keyboard, self._subscription)
            self._subscription = None

