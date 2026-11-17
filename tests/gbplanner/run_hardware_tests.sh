#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
mkdir -p "$root/.hardware/test-output"
docker run --rm --init --network none --name aliengo-hardware-tests --cpus=2 --memory=4g \
    --user "$(id -u):$(id -g)" -e HOME=/tmp/hardware-tests -e ROS_HOME=/tmp/hardware-tests/.ros \
    -e ROS_LOG_DIR=/tmp/hardware-tests/log \
    -e ROS_MASTER_URI=http://127.0.0.1:11325 -e ROS_IP=127.0.0.1 \
    --mount "type=bind,source=$root,target=/course,readonly" \
    --mount "type=bind,source=$root/.gbplanner,target=/gbp,readonly" \
    --mount "type=bind,source=$root/.super_lio,target=/lio,readonly" \
    --mount "type=bind,source=$root/.mid360,target=/work,readonly" \
    --mount "type=bind,source=$root/.hardware,target=/hardware" \
    --mount "type=bind,source=$root/unitree_legged_sdk_v3.8.4,target=/sdk,readonly" \
    --entrypoint bash aliengo-learning:gbplanner-real \
    /course/ros1/hardware/scripts/runtime.sh bash -c '
set -e
g++ -std=c++14 -I/course/ros1/hardware/include /course/tests/gbplanner/hardware_gate.cpp -o /tmp/hardware_gate
/tmp/hardware_gate
python3 /course/tests/hardware_ros_check.py
'
