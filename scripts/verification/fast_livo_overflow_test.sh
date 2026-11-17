#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$root/scripts/compute_guard.sh"
acquire_compute_guard "$(compute_guard_path)"
docker run --rm --init --network none --memory=2g --memory-swap=2g --cpus=2 \
    --pids-limit=256 --user "$(id -u):$(id -g)" \
    --mount "type=bind,source=$root/.fast_livo,target=/fast_livo" \
    --mount "type=bind,source=$root,target=/course,readonly" \
    --entrypoint bash aliengo-learning:noetic-fast-livo -c '
set -eo pipefail
source /opt/ros/noetic/setup.bash
source /fast_livo/devel/setup.bash --extend
set -u
export LD_LIBRARY_PATH="/fast_livo/sophus-build:${LD_LIBRARY_PATH:-}"
export ROS_HOME=/tmp/fast_livo_overflow
exec python3 /course/scripts/verification/fast_livo_overflow_test.py'
