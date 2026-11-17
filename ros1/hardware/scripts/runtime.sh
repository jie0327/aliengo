#!/usr/bin/env bash
set -e
mkdir -p "$HOME" "${ROS_HOME:-$HOME/.ros}"
source /opt/ros/noetic/setup.bash
source /opt/gbp_ws/devel/setup.bash --extend
source /gbp/devel/setup.bash --extend
source /work/devel/setup.bash --extend
source /lio/devel/setup.bash --extend
export ROS_PACKAGE_PATH="/course/ros1:$ROS_PACKAGE_PATH"
unset ROS_HOSTNAME
exec "$@"
