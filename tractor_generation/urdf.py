"""Generate a ROS-compatible URDF with names matching the Isaac USD asset."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

from .config import TractorConfig
from .names import BASE_LINK, SENSOR_POD_LINK, STEERING_JOINTS, STEERING_LINKS, WHEELS, WHEEL_JOINTS, camera_joint, camera_link

DENSITY = 1000.0


def _inertial_box(link, mass: float, size: tuple[float, float, float], origin=(0, 0, 0)):
    x, y, z = size
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "origin", xyz=" ".join(map(str, origin)), rpy="0 0 0")
    ET.SubElement(inertial, "mass", value=f"{mass:.9g}")
    ET.SubElement(inertial, "inertia", ixx=f"{mass*(y*y+z*z)/12:.9g}", ixy="0", ixz="0",
                  iyy=f"{mass*(x*x+z*z)/12:.9g}", iyz="0", izz=f"{mass*(x*x+y*y)/12:.9g}")


def _box(link, size, color, collision=True):
    for kind in (("visual", "collision") if collision else ("visual",)):
        node = ET.SubElement(link, kind)
        geometry = ET.SubElement(node, "geometry")
        ET.SubElement(geometry, "box", size=" ".join(map(str, size)))
        if kind == "visual":
            material = ET.SubElement(node, "material", name=color[0])
            ET.SubElement(material, "color", rgba=color[1])


def _wheel_link(robot, name, radius, width):
    link = ET.SubElement(robot, "link", name=name)
    volume = math.pi * radius * radius * width
    mass = DENSITY * volume
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "mass", value=f"{mass:.9g}")
    ET.SubElement(inertial, "inertia", ixx=f"{mass*(3*radius*radius+width*width)/12:.9g}", ixy="0", ixz="0",
                  iyy=f"{mass*radius*radius/2:.9g}", iyz="0", izz=f"{mass*(3*radius*radius+width*width)/12:.9g}")
    for kind in ("visual", "collision"):
        node = ET.SubElement(link, kind)
        geometry = ET.SubElement(node, "geometry")
        ET.SubElement(geometry, "cylinder", radius=str(radius), length=str(width))
        ET.SubElement(node, "origin", xyz="0 0 0", rpy=f"{math.pi/2} 0 0")
    return link


def write_tractor_urdf(config: TractorConfig, path: str | Path) -> Path:
    robot = ET.Element("robot", name="farm_tractor")
    base = ET.SubElement(robot, "link", name=BASE_LINK)
    body_parts = [
        (
            DENSITY * length * width * config.tractor_body_height,
            (length, width, config.tractor_body_height),
            center_x,
        )
        for center_x, length, width in config.body_segments
    ]
    body_mass = sum(mass for mass, _, _ in body_parts)
    body_com_x = sum(mass * center_x for mass, _, center_x in body_parts) / body_mass
    inertia = [0.0, 0.0, 0.0]
    for mass, (length, width, height), center_x in body_parts:
        inertia[0] += mass * (width * width + height * height) / 12
        inertia[1] += mass * (length * length + height * height) / 12 + mass * (center_x-body_com_x)**2
        inertia[2] += mass * (length * length + width * width) / 12 + mass * (center_x-body_com_x)**2
    inertial = ET.SubElement(base, "inertial")
    ET.SubElement(inertial, "origin", xyz=f"{body_com_x} 0 0", rpy="0 0 0")
    ET.SubElement(inertial, "mass", value=f"{body_mass:.9g}")
    ET.SubElement(
        inertial, "inertia", ixx=f"{inertia[0]:.9g}", ixy="0", ixz="0",
        iyy=f"{inertia[1]:.9g}", iyz="0", izz=f"{inertia[2]:.9g}",
    )
    # base_link is the body-center frame in both URDF and USD. Compound boxes
    # leave actual empty wheel wells instead of overlapping tire geometry.
    for center_x, length, width in config.body_segments:
        before = len(base)
        _box(base, (length, width, config.tractor_body_height), ("tractor_green", "0.08 0.32 0.10 1"))
        for node in list(base)[before:]:
            ET.SubElement(node, "origin", xyz=f"{center_x} 0 0", rpy="0 0 0")

    pod = ET.SubElement(robot, "link", name=SENSOR_POD_LINK)
    pod_size = (config.sensor_pod_length, config.sensor_pod_width, config.sensor_pod_height)
    _inertial_box(pod, DENSITY * math.prod(pod_size), pod_size)
    _box(pod, pod_size, ("pod_black", "0.04 0.05 0.04 1"))
    joint = ET.SubElement(robot, "joint", name="sensor_pod_joint", type="fixed")
    ET.SubElement(joint, "parent", link=BASE_LINK); ET.SubElement(joint, "child", link=SENSOR_POD_LINK)
    pod_local_z = config.sensor_pod_height_above_ground - config.body_center_z
    ET.SubElement(joint, "origin", xyz=f"0 0 {pod_local_z}", rpy="0 0 0")

    specs = (
        (0, config.front_axle_x, config.front_track_width/2, config.front_tire_dia/2, config.front_tire_width),
        (1, config.front_axle_x, -config.front_track_width/2, config.front_tire_dia/2, config.front_tire_width),
        (2, config.rear_axle_x, config.rear_track_width/2, config.rear_tire_dia/2, config.rear_tire_width),
        (3, config.rear_axle_x, -config.rear_track_width/2, config.rear_tire_dia/2, config.rear_tire_width),
    )
    for index, x, y, radius, width in specs:
        parent = BASE_LINK
        if index < 2:
            steer_link = ET.SubElement(robot, "link", name=STEERING_LINKS[index])
            steer_joint = ET.SubElement(robot, "joint", name=STEERING_JOINTS[index], type="revolute")
            ET.SubElement(steer_joint, "parent", link=BASE_LINK); ET.SubElement(steer_joint, "child", link=STEERING_LINKS[index])
            ET.SubElement(steer_joint, "origin", xyz=f"{x} {y} {radius-config.body_center_z}", rpy="0 0 0")
            ET.SubElement(steer_joint, "axis", xyz="0 0 1")
            ET.SubElement(steer_joint, "limit", lower=f"{-config.max_steer_angle_rad}", upper=f"{config.max_steer_angle_rad}", effort="10000", velocity="1.5")
            parent = STEERING_LINKS[index]
            wheel_origin = "0 0 0"
        else:
            wheel_origin = f"{x} {y} {radius-config.body_center_z}"
        _wheel_link(robot, WHEELS[index], radius, width)
        wheel_joint = ET.SubElement(robot, "joint", name=WHEEL_JOINTS[index], type="continuous")
        ET.SubElement(wheel_joint, "parent", link=parent); ET.SubElement(wheel_joint, "child", link=WHEELS[index])
        ET.SubElement(wheel_joint, "origin", xyz=wheel_origin, rpy="0 0 0")
        ET.SubElement(wheel_joint, "axis", xyz="0 1 0")

    camera_positions = {
        "front": (config.sensor_pod_length/2, 0, 0), "rear": (-config.sensor_pod_length/2, 0, 0),
        "left": (0, config.sensor_pod_width/2, 0), "right": (0, -config.sensor_pod_width/2, 0),
    }
    for name in config.enabled_cameras:
        ET.SubElement(robot, "link", name=camera_link(name))
        joint = ET.SubElement(robot, "joint", name=camera_joint(name), type="fixed")
        ET.SubElement(joint, "parent", link=SENSOR_POD_LINK); ET.SubElement(joint, "child", link=camera_link(name))
        ET.SubElement(joint, "origin", xyz=" ".join(map(str, camera_positions[name])), rpy="0 0 0")

    ET.indent(robot, space="  ")
    output = Path(path); output.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(robot).write(output, encoding="unicode", xml_declaration=True)
    output.write_text(output.read_text() + "\n")
    return output
