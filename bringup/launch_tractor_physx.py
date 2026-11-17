"""Farm launcher with exclusive ROS PhysX actuation selected by default.

Use --ros2 so the existing sensor Action Graphs and physics-step odometry
publisher remain active. This delegates stage setup to launch_tractor.
"""

from bringup.launch_tractor import main


if __name__ == "__main__":
    main(default_control_source="ros")
