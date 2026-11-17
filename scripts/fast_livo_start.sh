#!/usr/bin/env bash
set -euo pipefail
container=aliengo-ros1-sim
rviz=true
for arg in "$@"; do
    case "$arg" in
        --headless) rviz=false ;;
        *) echo '用法：./scripts/fast_livo_start.sh [--headless]' >&2; exit 2 ;;
    esac
done
if [[ "$(docker inspect -f '{{.State.Running}}' "$container" 2>/dev/null || true)" != true ]]; then
    echo '请先运行 ./scripts/fast_livo_sim.sh，并用键盘 2 让机器人站稳。' >&2; exit 1
fi
ready="$(docker inspect -f '{{range .Mounts}}{{if eq .Destination "/fast_livo"}}mounted{{end}}{{end}} {{range .Config.Env}}{{if eq . "ALIENGO_FAST_LIVO_ENABLED=1"}}enabled{{end}}{{end}}' "$container")"
if [[ "$ready" != 'mounted enabled' ]]; then
    echo '当前容器未挂载 FAST-LIVO2；请使用 ./scripts/fast_livo_sim.sh 启动。' >&2; exit 1
fi
map_dir="/fast_livo/maps/$(TZ=Asia/Shanghai date +%Y%m%d-%H%M%S)-$$"
log_dir="/fast_livo/logs/${map_dir##*/}"
tty_args=(-i)
if [[ -t 0 && -t 1 ]]; then tty_args+=(-t); fi
echo "保持机器人站稳，等待 IMU 初始化及视觉跟踪。地图目录：$map_dir"
echo "运行日志：$log_dir"
exec docker exec "${tty_args[@]}" "$container" bash -c '
set -e
source /opt/ros/noetic/setup.bash
source /work/devel/setup.bash
source /fast_livo/devel/setup.bash --extend
export LD_LIBRARY_PATH="${ALIENGO_NVIDIA_LIBRARY_PATH:+$ALIENGO_NVIDIA_LIBRARY_PATH:}/fast_livo/sophus-build:${LD_LIBRARY_PATH:-}"
export ROS_PACKAGE_PATH="/course/ros1:$ROS_PACKAGE_PATH"
if rosnode list | grep -Eq "^/(fast_livo/laserMapping|super_lio_node)$"; then
    echo "已有定位节点运行，请先结束现有估计器。" >&2; exit 1
fi
mkdir -p "$2" "$3"
export ROS_LOG_DIR="$3"
exec roslaunch /course/ros1/fast_livo/mapping.launch "rviz:=$1" "map_dir:=$2"' bash "$rviz" "$map_dir" "$log_dir"
