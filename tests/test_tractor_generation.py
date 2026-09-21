import math
import xml.etree.ElementTree as ET

import pytest

from tractor_generation.config import TractorConfig, load_tractor_config
from tractor_generation.names import BASE_LINK, SENSOR_POD_LINK, STEERING_JOINTS, WHEELS, WHEEL_JOINTS, camera_link
from tractor_generation.urdf import write_tractor_urdf


def test_default_tractor_geometry_and_config_file():
    config = load_tractor_config("configs/tractor_default.yaml")
    assert config.front_axle_x == pytest.approx(1.45)
    assert config.rear_axle_x == pytest.approx(-1.25)
    assert config.front_axle_x - config.rear_axle_x == pytest.approx(config.wheelbase)
    assert config.body_center_z == pytest.approx(0.8)
    assert config.max_steer_angle_rad == pytest.approx(math.pi / 4)
    assert config.enabled_cameras == ("front",)


def test_config_rejects_unknown_and_impossible_values(tmp_path):
    path = tmp_path / "bad.yaml"
    path.write_text("unknown_parameter: 2\n")
    with pytest.raises(ValueError, match="unknown tractor configuration"):
        load_tractor_config(path)
    with pytest.raises(ValueError, match="rear axle"):
        TractorConfig(wheelbase=4.0)
    with pytest.raises(ValueError, match="sensor pod"):
        TractorConfig(sensor_pod_height_above_ground=1.0)


def test_urdf_uses_canonical_names_and_water_density(tmp_path):
    config = TractorConfig(front_camera=True, rear_camera=True)
    path = write_tractor_urdf(config, tmp_path / "tractor.urdf")
    root = ET.parse(path).getroot()
    links = {node.attrib["name"] for node in root.findall("link")}
    joints = {node.attrib["name"] for node in root.findall("joint")}
    assert {BASE_LINK, SENSOR_POD_LINK, *WHEELS, camera_link("front"), camera_link("rear")} <= links
    assert {*WHEEL_JOINTS, *STEERING_JOINTS, "sensor_pod_joint"} <= joints
    base = next(node for node in root.findall("link") if node.attrib["name"] == BASE_LINK)
    expected_mass = 1000 * sum(
        length * width * config.tractor_body_height
        for _, length, width in config.body_segments
    )
    assert float(base.find("inertial/mass").attrib["value"]) == pytest.approx(expected_mass)


def test_body_segments_clear_tire_inner_sidewalls():
    config = TractorConfig()
    for center_x, length, width in config.body_segments:
        for axle_x, diameter, track, tire_width in (
            (config.front_axle_x, config.front_tire_dia, config.front_track_width, config.front_tire_width),
            (config.rear_axle_x, config.rear_tire_dia, config.rear_track_width, config.rear_tire_width),
        ):
            wheel_zone = abs(center_x - axle_x) < (length + diameter) / 2
            if wheel_zone:
                assert width / 2 + config.wheel_well_clearance <= (track - tire_width) / 2 + 1e-12


def test_usd_source_authors_non_com_vehicle_reference_frame():
    source = open("tractor_generation/usd.py").read()
    assert "referenceFrameIsCenterOfMass" in source
    assert "collision.CreateAxisAttr(UsdGeom.Tokens.y)" in source
    assert "render.CreateAxisAttr(UsdGeom.Tokens.x)" in source
    assert "wheel_xform.AddOrientOp().Set(Gf.Quatf(half_sqrt, 0, 0, half_sqrt))" in source
    assert "attachment.CreateSuspensionFrameOrientationAttr(Gf.Quatf(1, 0, 0, 0))" in source
    assert "CreateWheelFrameOrientationAttr" not in source


def test_usd_camera_constrains_optical_axis_and_image_up():
    source = open("tractor_generation/usd.py").read()
    assert "aim = Gf.Rotation(Gf.Vec3d(0, 0, -1), look)" in source
    assert "current_up = aim.TransformDir(Gf.Vec3d(0, 1, 0))" in source
    assert "orient = (aim * roll).GetQuat()" in source
