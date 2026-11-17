#!/usr/bin/env bash
set -euo pipefail
container=aliengo-ros1-sim
if [[ "$(docker inspect -f '{{.State.Running}}' "$container" 2>/dev/null || true)" != true ]]; then
    echo 'ROS 1 仿真未运行，请先运行 ./scripts/ros1_sim.sh。' >&2; exit 1
fi
exec docker exec -it "$container" bash -c '
set -e
source /opt/ros/noetic/setup.bash
source /work/devel/setup.bash
exec /work/devel/lib/unitree_guide/junior_ctrl'
