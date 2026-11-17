#!/usr/bin/env bash
# 三传感器仿真；站稳后使用 fast_livo_start.sh 启动估计器。
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export ALIENGO_ROS1_IMAGE=aliengo-learning:noetic-fast-livo
export ALIENGO_FAST_LIVO_WORKSPACE="$root/.fast_livo"
unset ALIENGO_LIO_WORKSPACE ALIENGO_GBP_WORKSPACE
sim_args=()
for arg in "$@"; do
    case "$arg" in
        --rviz) echo 'FAST-LIVO2 的 RViz 由 ./scripts/fast_livo_start.sh 启动。' ;;
        d435_enabled:=false) echo 'FAST-LIVO2 需要 D435 RGB 图像，请保留相机启用。' >&2; exit 2 ;;
        *) sim_args+=("$arg") ;;
    esac
done
# 减少仿真渲染和视觉计算负担；用户可在后续参数中覆盖尺寸和频率。
exec "$root/scripts/ros1_sim.sh" d435_color_width:=640 d435_color_height:=360 \
    d435_depth_width:=424 d435_depth_height:=240 d435_rate:=15 \
    "${sim_args[@]}" sim_ground_truth_tf:=false
