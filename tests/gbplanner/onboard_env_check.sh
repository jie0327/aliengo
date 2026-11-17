#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
docker run --rm --network none --user "$(id -u):$(id -g)" \
    -e HOME=/tmp/onboard-test -e ROS_HOME=/tmp/onboard-test/.ros -e ROS_LOG_DIR=/tmp/onboard-test/log \
    --mount "type=bind,source=$root/ros1/hardware,target=/source,readonly" \
    --mount "type=bind,source=$root/tests/fixtures/camera_roslaunch_capture.sh,target=/opt/ros/noetic/bin/roslaunch,readonly" \
    --entrypoint bash aliengo-learning:gbplanner-real -c '
mkdir -p "$HOME"
exec env -u ROS_MASTER_URI -u ROS_IP ROS_HOSTNAME=localhost bash /source/onboard_camera.sh
'
