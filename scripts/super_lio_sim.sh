#!/usr/bin/env bash
# 先启动仿真并站稳，再单独启动 LIO 完成静止标定。
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export ALIENGO_ROS1_IMAGE=aliengo-learning:noetic-lio
export ALIENGO_LIO_WORKSPACE="$root/.super_lio"
sim_args=()
for arg in "$@"; do
    if [[ "$arg" == --rviz ]]; then
        echo 'Super LIO 的 RViz 由 ./scripts/super_lio_start.sh 开启；请先启动仿真并让机器人站稳。'
    else
        sim_args+=("$arg")
    fi
done
exec "$root/scripts/ros1_sim.sh" "${sim_args[@]}" sim_ground_truth_tf:=false
