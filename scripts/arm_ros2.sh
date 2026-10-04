#!/usr/bin/env bash
# ROS 2 CLI with this workspace and discovery domain loaded.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source "$HOME/can_bezier_ws/install/setup.bash"
export ROS_DOMAIN_ID=73 ROS_LOCALHOST_ONLY=1
exec ros2 "$@"
