#!/usr/bin/env bash
# Run in the Ubuntu VM after building ~/can_bezier_ws.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source "$HOME/can_bezier_ws/install/setup.bash"
export ROS_DOMAIN_ID=73
export ROS_LOCALHOST_ONLY=1
exec ros2 launch arm_control bringup.launch.py \
  config:="$HOME/can_bezier_ws/src/arm_control/config/robot_mit.yaml" \
  backend:=can_mock rviz:="${ARM_RVIZ:-false}"
