#!/usr/bin/env bash
# 在已有 LIO 仿真容器中启动建图；每次启动使用独立地图目录。
set -euo pipefail
container=aliengo-ros1-sim
rviz=true
for arg in "$@"; do
    case "$arg" in
        --headless) rviz=false ;;
        *) echo '用法：./scripts/super_lio_start.sh [--headless]' >&2; exit 2 ;;
    esac
done
if [[ "$(docker inspect -f '{{.State.Running}}' "$container" 2>/dev/null || true)" != true ]]; then
    echo 'Super LIO 仿真未运行，请先运行 ./scripts/super_lio_sim.sh 并让机器人站稳。' >&2
    exit 1
fi
lio_ready="$(docker inspect -f '{{range .Mounts}}{{if eq .Destination "/lio"}}mounted{{end}}{{end}} {{range .Config.Env}}{{if eq . "ALIENGO_LIO_ENABLED=1"}}enabled{{end}}{{end}}' "$container")"
if [[ "$lio_ready" != 'mounted enabled' ]]; then
    echo '当前容器未挂载 Super LIO 工作区；请使用 ./scripts/super_lio_sim.sh 启动仿真。' >&2
    exit 1
fi
map_dir="../../maps/$(date +%Y%m%d-%H%M%S)-$$"
tty_args=(-i)
if [[ -t 0 && -t 1 ]]; then tty_args+=(-t); fi
echo "请保持机器人站稳，等待 Super LIO 完成静止标定。地图目录：$map_dir"
exec docker exec "${tty_args[@]}" "$container" bash -c '
set -e
source /opt/ros/noetic/setup.bash
source /work/devel/setup.bash
source /lio/devel/setup.bash --extend
export LD_LIBRARY_PATH="${ALIENGO_NVIDIA_LIBRARY_PATH:+$ALIENGO_NVIDIA_LIBRARY_PATH:}${LD_LIBRARY_PATH:-}"
export ROS_PACKAGE_PATH="/course/ros1:$ROS_PACKAGE_PATH"
exec roslaunch /course/ros1/super_lio/mapping.launch "rviz:=$1" "map_dir:=$2"' bash "$rviz" "$map_dir"
