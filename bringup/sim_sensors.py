"""Physics-step ground truth and GNSS publishers for the Isaac ROS 2 bridge."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from bringup.geo_reference import GeoReference
from tractor_generation.names import BASE_LINK, GPS_LINK, SENSOR_POD_LINK


def _stamp(seconds: float):
    from builtin_interfaces.msg import Time

    whole = math.floor(seconds)
    return Time(sec=whole, nanosec=round((seconds - whole) * 1e9))


def _rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate a vector by a unit quaternion in wxyz order."""
    w, x, y, z = q
    u = np.array((x, y, z))
    return v + 2 * (w * np.cross(u, v) + np.cross(u, np.cross(u, v)))


def _velocity_at_origin(v_com, omega_world, orientation, com_local) -> np.ndarray:
    """Shift PhysX's world-space COM velocity to the base_link origin."""
    return np.asarray(v_com, dtype=float) - np.cross(omega_world, _rotate(orientation, com_local))


@dataclass(frozen=True)
class RigidState:
    position: np.ndarray  # world position of the base_link rigid-body origin
    orientation: np.ndarray  # wxyz, base_link to world
    linear_velocity: np.ndarray  # world-space velocity at the base_link origin
    angular_velocity: np.ndarray  # PhysX world-space radians/second


class GroundTruthPublisher:
    def __init__(self, node, config: dict):
        from nav_msgs.msg import Odometry

        self.config = config
        self.publisher = node.create_publisher(Odometry, config["topic"], 10)
        self.next_time: float | None = None
        self.samples = 0

    def publish(self, state: RigidState, sim_time: float) -> None:
        from nav_msgs.msg import Odometry

        rate = self.config.get("rate_hz")
        if rate is not None:
            if self.next_time is not None and sim_time + 1e-9 < self.next_time:
                return
            self.next_time = sim_time + 1.0 / rate
        msg = Odometry()
        msg.header.stamp = _stamp(sim_time)
        msg.header.frame_id = self.config["world_frame"]
        msg.child_frame_id = self.config["base_frame"]
        p, q = state.position, state.orientation
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = map(float, p)
        msg.pose.pose.orientation.w = float(q[0])
        msg.pose.pose.orientation.x = float(q[1])
        msg.pose.pose.orientation.y = float(q[2])
        msg.pose.pose.orientation.z = float(q[3])
        # PhysX reports both velocities in world axes. Odometry twist is in
        # child_frame_id, so rotate by the inverse base_link orientation.
        inverse = np.array((q[0], -q[1], -q[2], -q[3]))
        linear = _rotate(inverse, state.linear_velocity)
        angular = _rotate(inverse, state.angular_velocity)
        msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = map(float, linear)
        msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z = map(float, angular)
        self.publisher.publish(msg)
        self.samples += 1


class GnssSimulator:
    def __init__(self, node, config: dict, georeference: GeoReference, antenna_base_xyz):
        from sensor_msgs.msg import NavSatFix

        self.config = config
        self.georeference = georeference
        self.antenna_base_xyz = np.asarray(antenna_base_xyz, dtype=float)
        self.publisher = node.create_publisher(NavSatFix, config["topic"], 10)
        self.rng = np.random.default_rng(config["random_seed"])
        self.next_time: float | None = None
        self.samples = 0

    def publish_if_due(self, state: RigidState, sim_time: float) -> None:
        from sensor_msgs.msg import NavSatFix, NavSatStatus

        if self.next_time is not None and sim_time + 1e-9 < self.next_time:
            return
        interval = 1.0 / self.config["update_rate_hz"]
        self.next_time = sim_time + interval
        antenna = state.position + _rotate(state.orientation, self.antenna_base_xyz)
        horizontal = self.config["horizontal_noise_std_m"]
        vertical = self.config["vertical_noise_std_m"]
        if horizontal:
            antenna[:2] += self.rng.normal(0.0, horizontal, 2)
        if vertical:
            antenna[2] += self.rng.normal(0.0, vertical)
        latitude, longitude, altitude = self.georeference.to_wgs84(*antenna)
        msg = NavSatFix()
        msg.header.stamp = _stamp(sim_time)
        msg.header.frame_id = self.config["antenna_frame"]
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude, msg.longitude, msg.altitude = latitude, longitude, altitude
        msg.position_covariance = [horizontal**2, 0.0, 0.0,
                                   0.0, horizontal**2, 0.0,
                                   0.0, 0.0, vertical**2]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.publisher.publish(msg)
        self.samples += 1


def load_sensor_config(path: str | Path, farm_path: str | Path) -> tuple[dict, GeoReference]:
    config = yaml.safe_load(Path(path).read_text())
    geo = config["georeference"]
    if "origin_easting" in geo and "origin_northing" in geo:
        reference = GeoReference(
            geo["projected_crs"], geo["origin_easting"], geo["origin_northing"],
            geo["origin_altitude"], geo["world_to_grid_heading_deg"],
        )
    else:
        farm = yaml.safe_load(Path(farm_path).read_text())
        origin = farm["scene"]["origin"]
        reference = GeoReference.from_wgs84_origin(
            origin["lat0"], origin["lon0"], origin["alt0"], geo["projected_crs"],
            geo["world_to_grid_heading_deg"],
        )
    for section, key in (("ground_truth", "rate_hz"), ("gnss", "update_rate_hz")):
        rate = config[section].get(key)
        if rate is not None and rate <= 0:
            raise ValueError(f"{section}.{key} must be positive")
    for key in ("horizontal_noise_std_m", "vertical_noise_std_m"):
        if config["gnss"][key] < 0:
            raise ValueError(f"gnss.{key} must be non-negative")
    return config, reference


class TractorStateStreams:
    """Owns one ROS node and one post-physics subscription for this launcher."""

    def __init__(self, stage, config: dict, reference: GeoReference, tractor_path="/World/Tractor"):
        import rclpy
        from isaacsim.core.experimental.prims import RigidPrim
        from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager
        from pxr import UsdGeom, UsdPhysics

        self._rclpy = rclpy
        self._simulation_manager = SimulationManager
        self._callback_id = None
        self._owns_rclpy = not rclpy.ok()
        if self._owns_rclpy:
            rclpy.init()
        self.node = rclpy.create_node("tractor_sim_state")
        self.rigid = RigidPrim(f"{tractor_path}/{BASE_LINK}")
        base = stage.GetPrimAtPath(f"{tractor_path}/{BASE_LINK}")
        center_of_mass = UsdPhysics.MassAPI(base).GetCenterOfMassAttr().Get()
        self._com_local = np.asarray(center_of_mass if center_of_mass is not None else (0, 0, 0), dtype=float)
        self.truth = GroundTruthPublisher(self.node, config["ground_truth"]) if config["ground_truth"]["enabled"] else None
        self.gnss = None
        gps_path = f"{tractor_path}/{BASE_LINK}/{SENSOR_POD_LINK}/{GPS_LINK}"
        if config["gnss"]["enabled"] and stage.GetPrimAtPath(gps_path).IsValid():
            gps = stage.GetPrimAtPath(gps_path)
            relative = (UsdGeom.Xformable(gps).ComputeLocalToWorldTransform(0) *
                        UsdGeom.Xformable(base).ComputeLocalToWorldTransform(0).GetInverse())
            antenna_base_xyz = relative.ExtractTranslation()
            self.gnss = GnssSimulator(self.node, config["gnss"], reference, antenna_base_xyz)
        elif config["gnss"]["enabled"]:
            print(f"GNSS disabled: no {gps_path} in tractor asset")
        self._last_time = None
        self._callback_id = SimulationManager.register_callback(
            self._on_physics_step, event=SimulationEvent.PHYSICS_POST_STEP
        )

    def _on_physics_step(self, _step_dt, _context) -> None:
        sim_time = self._simulation_manager.get_simulation_time()
        if self._last_time is not None and sim_time < self._last_time:
            if self.truth:
                self.truth.next_time = None
            if self.gnss:
                self.gnss.next_time = None
                self.gnss.rng = np.random.default_rng(self.gnss.config["random_seed"])
        self._last_time = sim_time
        positions, orientations = self.rigid.get_world_poses()
        linear, angular = self.rigid.get_velocities()
        orientation = np.asarray(orientations.numpy()[0], dtype=float)
        angular_world = np.asarray(angular.numpy()[0], dtype=float)
        # omni.physics.tensors RigidBodyView.get_velocities documents global
        # axes and linear velocity at the COM. Shift it to base_link's origin:
        # v_base = v_com - omega x R_world_base * com_local.
        base_velocity = _velocity_at_origin(linear.numpy()[0], angular_world, orientation, self._com_local)
        state = RigidState(
            np.asarray(positions.numpy()[0], dtype=float),
            orientation,
            base_velocity,
            angular_world,
        )
        if self.truth:
            self.truth.publish(state, sim_time)
        if self.gnss:
            self.gnss.publish_if_due(state, sim_time)

    def close(self) -> None:
        if self._callback_id is not None:
            self._simulation_manager.deregister_callback(self._callback_id)
            self._callback_id = None
        self.node.destroy_node()
        if self._owns_rclpy:
            self._rclpy.shutdown()
