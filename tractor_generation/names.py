"""Canonical names shared by USDA, URDF, ROS, and bringup code."""

BASE_LINK = "base_link"
SENSOR_POD_LINK = "sensor_pod_link"
WHEELS = ("front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel")
WHEEL_JOINTS = tuple(f"{name}_joint" for name in WHEELS)
STEERING_LINKS = ("front_left_steering_link", "front_right_steering_link")
STEERING_JOINTS = ("front_left_steering_joint", "front_right_steering_joint")


def camera_link(name: str) -> str:
    return f"{name}_camera_link"


def camera_joint(name: str) -> str:
    return f"{name}_camera_joint"
