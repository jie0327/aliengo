#!/usr/bin/env bash
# Boundary double: inspect the actual launch environment without opening a camera.
set -euo pipefail
printf 'ROS_MASTER_URI=%s\nROS_IP=%s\nROS_HOSTNAME=%s\n' "$ROS_MASTER_URI" "$ROS_IP" "${ROS_HOSTNAME:-<unset>}"
[[ "$ROS_MASTER_URI" == http://192.168.123.100:11311 ]]
[[ "$ROS_IP" == 192.168.123.12 ]]
[[ -z "${ROS_HOSTNAME:-}" ]]
