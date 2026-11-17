#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export ALIENGO_ROS1_IMAGE=aliengo-learning:exploration
export ALIENGO_LIO_WORKSPACE="$root/.super_lio"
export ALIENGO_GBP_WORKSPACE="$root/.gbplanner"
# A supported lower-resolution profile reduces rendering/transport overhead;
# extrinsics and sensor field of view remain unchanged.
defaults=(d435_color_width:=640 d435_color_height:=360 d435_depth_width:=424 d435_depth_height:=240 d435_rate:=15)
args=()
for default in "${defaults[@]}"; do
    supplied=false
    for arg in "$@"; do
        if [[ "$arg" == "${default%%:=*}:="* ]]; then supplied=true; break; fi
    done
    if [[ "$supplied" == false ]]; then args+=("$default"); fi
done
for arg in "$@"; do
    if [[ "$arg" == --rviz ]]; then
        echo '探索 RViz 随 ./scripts/gbplanner_start.sh 开启。'
    elif [[ "$arg" == d435_enabled:=false ]]; then
        echo '当前探索配置需要下排 D435，请保持启用。' >&2; exit 2
    else
        args+=("$arg")
    fi
done
exec "$root/scripts/ros1_sim.sh" "${args[@]}" sim_ground_truth_tf:=false
