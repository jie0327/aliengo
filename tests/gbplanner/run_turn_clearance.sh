#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$root/scripts/compute_guard.sh"
acquire_compute_guard "$(compute_guard_path)"
docker run --rm --init --network none --cpus=2 --memory=2g --memory-swap=2g \
  --user "$(id -u):$(id -g)" -e HOME=/tmp/gbp-turn-tests \
  -e ROS_HOME=/tmp/gbp-turn-tests/.ros -e ROS_LOG_DIR=/tmp/gbp-turn-tests/log \
  -e ROS_MASTER_URI=http://127.0.0.1:11323 -e ROS_IP=127.0.0.1 -e ROS_HOSTNAME=localhost \
  --mount "type=bind,source=$root,target=/course,readonly" \
  --mount "type=bind,source=$root/.gbplanner,target=/gbp,readonly" \
  --entrypoint bash aliengo-learning:exploration -c '
set -e
mkdir -p "$HOME"
source /opt/gbp_ws/devel/setup.bash
source /gbp/devel/setup.bash --extend
exec bash /course/tests/gbplanner/turn_clearance_ros.sh
'
