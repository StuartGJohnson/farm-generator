"""Launch the system ROS nodes; start Isaac Sim separately in its conda environment."""

from pathlib import Path
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from tractor_generation.config import load_tractor_config


def tractor_control_node(context):
    config = load_tractor_config(LaunchConfiguration("tractor_config_path").perform(context))
    curvature_override = LaunchConfiguration("controller_max_curvature").perform(context)
    steer_override = LaunchConfiguration("controller_max_steer_angle_rad").perform(context)
    return [Node(package="tractor_control_adapter", executable="tractor_control_node",
                 parameters=[{"use_sim_time": True,
                              "wheelbase": config.wheelbase,
                              "front_track_width": config.front_track_width,
                              "max_curvature": float(curvature_override) if curvature_override else 1.0 / config.turning_radius,
                              "max_steer_angle_rad": float(steer_override) if steer_override else config.max_steer_angle_rad}],
                 output="screen")]


def generate_launch_description():
    params = str(Path(get_package_share_directory("bringup")) / "config" / "nav2_farm.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("farm", default_value="debug_out/farms/farm_seed_1_120m2_depot"),
        DeclareLaunchArgument("mission_segmentation_strategy", default_value="by_length"),
        DeclareLaunchArgument("by_length_m", default_value="5.0"),
        DeclareLaunchArgument("overlap_distance_m", default_value="1.0"),
        DeclareLaunchArgument("max_nav_pose_spacing_m", default_value="0.2"),
        DeclareLaunchArgument("desired_linear_vel_mps", default_value="0.5"),
        DeclareLaunchArgument("tractor_config_path", default_value="configs/tractor_default.yaml"),
        DeclareLaunchArgument("injected_route_path", default_value=""),
        DeclareLaunchArgument("controller_max_curvature", default_value=""),
        DeclareLaunchArgument("controller_max_steer_angle_rad", default_value=""),
        Node(package="ground_truth_localization", executable="ground_truth_localization_node",
             parameters=[{"use_sim_time": True}], output="screen"),
        OpaqueFunction(function=tractor_control_node),
        Node(package="nav2_controller", executable="controller_server",
             parameters=[params, {
                 "FollowPath.desired_linear_vel": ParameterValue(
                     LaunchConfiguration("desired_linear_vel_mps"), value_type=float),
                 "FollowPathReverse.desired_linear_vel": ParameterValue(
                     LaunchConfiguration("desired_linear_vel_mps"), value_type=float),
             }], output="screen"),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation", output="screen",
             parameters=[{"use_sim_time": True, "autostart": True,
                          "node_names": ["controller_server"]}]),
        Node(executable=str(Path(os.environ["VIRTUAL_ENV"]) / "bin" / "python"),
             arguments=["-m", "farm_mission_processor.node"], name="farm_mission_processor",
             parameters=[{"use_sim_time": True, "farm": LaunchConfiguration("farm"),
                          "mission_segmentation_strategy": LaunchConfiguration("mission_segmentation_strategy"),
                          "tractor_config_path": LaunchConfiguration("tractor_config_path"),
                          "injected_route_path": LaunchConfiguration("injected_route_path"),
                          "by_length_m": ParameterValue(LaunchConfiguration("by_length_m"), value_type=float),
                          "overlap_distance_m": ParameterValue(LaunchConfiguration("overlap_distance_m"), value_type=float),
                          "max_nav_pose_spacing_m": ParameterValue(LaunchConfiguration("max_nav_pose_spacing_m"), value_type=float)}],
             output="screen"),
    ])
