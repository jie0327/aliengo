#!/usr/bin/env bash
set -euo pipefail
container=aliengo-ros1-sim
rviz=true
for arg in "$@"; do
    case "$arg" in
        --headless) rviz=false ;;
        *) echo '用法：./scripts/gbplanner_start.sh [--headless]' >&2; exit 2 ;;
    esac
done
if [[ "$(docker inspect -f '{{.State.Running}}' "$container" 2>/dev/null || true)" != true ]]; then
    echo '请先运行 ./scripts/gbplanner_sim.sh，再启动键盘并按 2 站稳。' >&2; exit 1
fi
map_dir="../../maps/exploration-$(date +%Y%m%d-%H%M%S)-$$"
echo '探索建图启动；保持站稳等待 Map init done，再遥控前进约 1.5–2 米采集初始地面。'
echo '采图后运行 ./scripts/gbplanner_enable.sh；停止用 ./scripts/gbplanner_stop.sh 或键盘 2。'
exec docker exec -i "$container" bash -c '
set -e
if [[ "${ALIENGO_GBP_ENABLED:-}" != 1 || "${ALIENGO_LIO_ENABLED:-}" != 1 ]]; then
    echo "此容器没有规划器/LIO工作区；请使用 ./scripts/gbplanner_sim.sh 启动。" >&2; exit 1
fi
source /opt/ros/noetic/setup.bash
source /opt/gbp_ws/devel/setup.bash --extend
source /gbp/devel/setup.bash --extend
source /work/devel/setup.bash --extend
source /lio/devel/setup.bash --extend
export LD_LIBRARY_PATH="${ALIENGO_NVIDIA_LIBRARY_PATH:+$ALIENGO_NVIDIA_LIBRARY_PATH:}${LD_LIBRARY_PATH:-}"
export ROS_PACKAGE_PATH="/course/ros1:$ROS_PACKAGE_PATH"
if rosnode list 2>/dev/null | grep -qx "/super_lio_node"; then
    echo "已有 Super-LIO 节点运行。请先关闭之前的 super_lio_start.sh，再启动探索建图。" >&2; exit 1
fi
exec roslaunch /course/ros1/exploration/exploration.launch "rviz:=$1" "map_dir:=$2"' bash "$rviz" "$map_dir"
