#!/usr/bin/env bash
# 只有传感器与时钟回放，不启动或连接 Gazebo。
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
if [[ $# != 1 ]]; then
    echo '用法：./scripts/verification/super_lio_replay.sh <sensors.bag>' >&2; exit 2
fi
bag="$(realpath -- "$1")"
source "$root/scripts/compute_guard.sh"
acquire_compute_guard "$(compute_guard_path)"
output="$(mktemp -d "$root/logs/super-lio-replay-XXXXXX")"
echo "Super-LIO 离线回放记录：$output"
docker run --rm --init --network none --name aliengo-super-lio-replay \
    --memory=2g --memory-swap=2g --cpus=2 --pids-limit=256 \
    --user "$(id -u):$(id -g)" \
    -e HOME=/tmp/aliengo-lio-replay -e ROS_HOME=/tmp/aliengo-lio-replay/.ros \
    -e ROS_LOG_DIR=/tmp/aliengo-lio-replay/.ros/log \
    -e ROS_HOSTNAME=localhost -e ROS_IP=127.0.0.1 \
    -e OMP_NUM_THREADS=1 -e OPENBLAS_NUM_THREADS=1 \
    --mount "type=bind,source=$root/.super_lio,target=/lio" \
    --mount "type=bind,source=$root,target=/course,readonly" \
    --mount "type=bind,source=$bag,target=/input/sensors.bag,readonly" \
    --mount "type=bind,source=$output,target=/result" \
    --entrypoint bash aliengo-learning:noetic-lio -c '
set -e
mkdir -p "$ROS_LOG_DIR"
source /opt/ros/noetic/setup.bash
source /lio/devel/setup.bash --extend
export ROS_PACKAGE_PATH="/course/ros1:$ROS_PACKAGE_PATH"
exec python3 /course/scripts/verification/super_lio_replay.py --bag /input/sensors.bag --output /result'
