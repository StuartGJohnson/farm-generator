"""Receive ROS actuator vectors and write the proven teleop PhysX attributes."""

from __future__ import annotations

import math

ACCELERATOR, BRAKE0, BRAKE1, STEER, GEAR = range(5)
FORWARD_GEAR = 1  # bringup.teleop uses +1 forward and -1 reverse.
REVERSE_GEAR = -1


class VehicleActuationAdapter:
    def __init__(self, vehicle_prim, topic="/vehicle/actuation", timeout=0.5):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("actuation timeout must be positive and finite")
        import rclpy
        from rclpy.parameter import Parameter
        from isaacsim.core.simulation_manager import SimulationManager
        from pxr import PhysxSchema
        from std_msgs.msg import Float32MultiArray
        from nav_msgs.msg import Odometry

        self.rclpy = rclpy
        self.owns_rclpy = not rclpy.ok()
        if self.owns_rclpy:
            rclpy.init()
        self.node = rclpy.create_node("tractor_physx_actuation", parameter_overrides=[
            Parameter("use_sim_time", value=True)])
        self.simulation_manager = SimulationManager
        self.controller = PhysxSchema.PhysxVehicleControllerAPI(vehicle_prim)
        if not self.controller:
            raise ValueError(f"{vehicle_prim.GetPath()} has no PhysxVehicleControllerAPI")
        self.timeout = timeout
        self.latest = None
        self.received = None
        self.speed = None
        self.speed_received = None
        self.applied_gear = FORWARD_GEAR
        self.subscription = self.node.create_subscription(
            Float32MultiArray, topic, self._on_message, 10
        )
        self.odom_subscription = self.node.create_subscription(
            Odometry, "/ground_truth/odom", self._on_odom, 10
        )
        self.stop()

    def _on_odom(self, msg):
        value = msg.twist.twist.linear.x
        if msg.child_frame_id == "base_link" and math.isfinite(value):
            self.speed = value
            self.speed_received = self.simulation_manager.get_simulation_time()

    def _on_message(self, msg):
        values = list(msg.data)
        if (len(values) != 5 or not all(math.isfinite(x) for x in values)
                or any(not 0 <= values[i] <= 1 for i in (ACCELERATOR, BRAKE0, BRAKE1))
                or not -1 <= values[STEER] <= 1
                or values[GEAR] not in (FORWARD_GEAR, REVERSE_GEAR)):
            self.latest = None
            self.received = None
            self.node.get_logger().warn("Invalid actuation vector; braking")
            return
        self.latest = values
        self.received = self.simulation_manager.get_simulation_time()

    def stop(self):
        self.controller.GetAcceleratorAttr().Set(0.0)
        self.controller.GetBrake0Attr().Set(1.0)
        self.controller.GetBrake1Attr().Set(0.0)
        self.controller.GetSteerAttr().Set(0.0)
        # Keep the current gear on timeout; do not request repeated shifts.

    def update(self):
        for _ in range(4):
            self.rclpy.spin_once(self.node, timeout_sec=0.0)
        sim_time = self.simulation_manager.get_simulation_time()
        if self.received is None or not 0 <= sim_time - self.received <= self.timeout:
            self.stop()
            return
        a, b0, b1, steer, gear = self.latest
        if gear != self.applied_gear:
            speed_fresh = (self.speed_received is not None and
                           0 <= sim_time - self.speed_received <= self.timeout)
            if (not speed_fresh or abs(self.speed) > 0.08 or a > 0 or b0 < 0.2):
                self.stop()
                return
            self.controller.GetTargetGearAttr().Set(int(gear))
            self.applied_gear = int(gear)
        # Keep throttle and service brake mutually exclusive even for rogue senders.
        if b0 > 0:
            a = 0.0
        self.controller.GetAcceleratorAttr().Set(float(a))
        self.controller.GetBrake0Attr().Set(float(b0))
        self.controller.GetBrake1Attr().Set(float(b1))
        self.controller.GetSteerAttr().Set(float(steer))
        # Gear is written only after the stationary, braked transition above.

    def close(self):
        self.stop()
        self.node.destroy_node()
        if self.owns_rclpy:
            self.rclpy.shutdown()
