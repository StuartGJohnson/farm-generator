#!/bin/bash

# Prevent ~/.local Python packages from leaking into the venv
export PYTHONNOUSERSITE=1

# Activate Python environment
source "$(dirname "${BASH_SOURCE[0]}")/venv/bin/activate"

# Activate ROS2 Humble
source /opt/ros/humble/setup.bash

# Activate local ROS2 workspace, if built
if [ -f "$(dirname "${BASH_SOURCE[0]}")/install/setup.bash" ]; then
    source "$(dirname "${BASH_SOURCE[0]}")/install/setup.bash"
fi

