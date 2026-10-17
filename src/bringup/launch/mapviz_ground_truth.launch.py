"""Launch system ROS 2 localization and Mapviz for an Isaac Sim farm."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from bringup_ros.mapviz_config import farm_map_origin, write_mapviz_config
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_farm(context):
    farm = Path(LaunchConfiguration("farm").perform(context)).expanduser().resolve()
    template = Path(get_package_share_directory("bringup")) / "config" / "tractor.mviz"
    config = write_mapviz_config(farm, template)
    latitude, longitude, altitude, heading = farm_map_origin(farm)
    return [
        LogInfo(msg=f"Mapviz farm view: {config}"),
        Node(
            package="ground_truth_localization",
            executable="ground_truth_localization_node",
            name="ground_truth_localization_node",
            parameters=[{"use_sim_time": True}],
            output="screen",
        ),
        Node(
            package="swri_transform_util",
            executable="initialize_origin.py",
            name="initialize_origin",
            parameters=[{
                "local_xy_frame": "map",
                "local_xy_origin": "farm",
                "local_xy_origins": [latitude, longitude, altitude, heading],
            }],
            output="screen",
        ),
        Node(
            package="mapviz",
            executable="mapviz",
            name="mapviz",
            parameters=[{"config": str(config), "use_sim_time": True}],
            output="screen",
            on_exit=Shutdown(),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "farm",
            default_value="debug_out/farms/farm_seed_1_120m2",
            description="Generated farm directory containing farm.yaml and gis/.",
        ),
        OpaqueFunction(function=_launch_farm),
    ])
