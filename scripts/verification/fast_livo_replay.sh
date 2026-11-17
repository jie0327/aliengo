#!/usr/bin/env bash
# 不启动 Gazebo；只重放 RGB、CameraInfo、点云、IMU 和静态安装 TF。
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
if [[ $# != 1 || ! -f "$1" ]]; then
    echo '用法：./scripts/verification/fast_livo_replay.sh <sensors.bag>' >&2; exit 2
fi
bag="$(realpath -- "$1")"
source "$root/scripts/compute_guard.sh"
acquire_compute_guard "$(compute_guard_path)"
output="$(mktemp -d "$root/logs/fast-livo-replay-XXXXXX")"
echo "FAST-LIVO2 回放记录：$output"
docker run --rm --init --network none --name aliengo-fast-livo-replay \
    --memory=3g --memory-swap=3g --cpus=2 --pids-limit=256 \
    --user "$(id -u):$(id -g)" \
    -e HOME=/tmp/aliengo-fast-livo-replay -e ROS_HOME=/tmp/aliengo-fast-livo-replay/.ros \
    -e ROS_LOG_DIR=/tmp/aliengo-fast-livo-replay/.ros/log \
    -e ROS_HOSTNAME=localhost -e ROS_IP=127.0.0.1 \
    -e OMP_NUM_THREADS=2 -e OPENBLAS_NUM_THREADS=1 \
    --mount "type=bind,source=$root/.fast_livo,target=/fast_livo" \
    --mount "type=bind,source=$root,target=/course,readonly" \
    --mount "type=bind,source=$bag,target=/input/sensors.bag,readonly" \
    --mount "type=bind,source=$output,target=/result" \
    --entrypoint bash aliengo-learning:noetic-fast-livo -c '
set -e
mkdir -p "$ROS_LOG_DIR"
source /opt/ros/noetic/setup.bash
source /fast_livo/devel/setup.bash --extend
export LD_LIBRARY_PATH="/fast_livo/sophus-build:${LD_LIBRARY_PATH:-}"
export ROS_PACKAGE_PATH="/course/ros1:$ROS_PACKAGE_PATH"
exec python3 /course/scripts/verification/fast_livo_trial.py --bag /input/sensors.bag --output /result'
