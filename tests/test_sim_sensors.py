import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from bringup.geo_reference import GeoReference
from bringup.sim_sensors import _rotate, _velocity_at_origin, load_sensor_config
from tractor_generation.config import TractorConfig
from tractor_generation.urdf import write_tractor_urdf


def test_gps_frame_is_fixed_to_pod_and_optional(tmp_path):
    enabled = ET.parse(write_tractor_urdf(TractorConfig(center_gps=True), tmp_path / "gps.urdf"))
    joint = enabled.find(".//joint[@name='gps_joint']")
    assert joint.find("parent").attrib["link"] == "sensor_pod_link"
    assert joint.find("child").attrib["link"] == "gps_link"
    assert float(joint.find("origin").attrib["xyz"].split()[2]) == pytest.approx(0.15)
    gps = enabled.find(".//link[@name='gps_link']")
    assert float(gps.find("visual/geometry/sphere").attrib["radius"]) == pytest.approx(0.10)
    assert float(gps.find("visual/origin").attrib["xyz"].split()[2]) == pytest.approx(-0.05)
    disabled = ET.parse(write_tractor_urdf(TractorConfig(center_gps=False), tmp_path / "plain.urdf"))
    assert disabled.find(".//link[@name='gps_link']") is None


def test_georeference_axes_heading_and_altitude():
    pytest.importorskip("pyproj")
    reference = GeoReference.from_wgs84_origin(38.2, -121.5, 12.0)
    assert reference.to_wgs84(0, 0, 0) == pytest.approx((38.2, -121.5, 12.0))
    lat_east, lon_east, alt = reference.to_wgs84(10, 0, 2)
    lat_north, lon_north, _ = reference.to_wgs84(0, 10, 0)
    assert lon_east > -121.5
    assert lat_north > 38.2
    assert alt == 14.0
    rotated = GeoReference(reference.projected_crs, reference.origin_easting,
                           reference.origin_northing, 12, 90)
    assert rotated.project_local(10, 0) == pytest.approx(reference.project_local(0, 10))


def test_farm_origin_and_twist_rotation():
    pytest.importorskip("pyproj")
    config, reference = load_sensor_config(
        "configs/sim_sensors.yaml", "debug_out/farms/farm_seed_1_120m2/farm.yaml"
    )
    assert config["gnss"]["update_rate_hz"] == 10
    assert reference.to_wgs84(0, 0, 0) == pytest.approx((38.2, -121.5, 0))
    q = np.array((math.sqrt(0.5), 0, 0, math.sqrt(0.5)))
    assert _rotate(np.array((q[0], -q[1], -q[2], -q[3])), np.array((0, 1, 0))) == pytest.approx((1, 0, 0))
    assert _velocity_at_origin((0, 1, 0), (0, 0, 1), (1, 0, 0, 0), (1, 0, 0)) == pytest.approx((0, 0, 0))
