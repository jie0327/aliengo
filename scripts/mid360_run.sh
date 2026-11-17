#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if (( $# < 1 )); then
    echo '用法：./scripts/mid360_run.sh <网络配置 JSON> [roslaunch 参数...]' >&2; exit 2
fi
config="$(realpath -- "$1")"
shift
if [[ ! -f "$config" || ! -r "$config" ]]; then echo '无法读取网络配置。' >&2; exit 1; fi
if [[ ! -f "$root/.mid360/devel/setup.bash" ]]; then echo '请先运行 ./scripts/mid360_build.sh。' >&2; exit 1; fi
exec docker run --rm --init --network host --name aliengo-mid360-driver --user "$(id -u):$(id -g)" \
    -e HOME=/tmp/aliengo-mid360 -e ROS_HOME=/tmp/aliengo-mid360/.ros \
    -e ROS_MASTER_URI -e ROS_IP -e ROS_HOSTNAME \
    --mount "type=bind,source=$root/.mid360,target=/work,readonly" \
    --mount "type=bind,source=$root/ros1/mid360,target=/mid360,readonly" \
    --mount "type=bind,source=$config,target=/mid360-config.json,readonly" \
    --entrypoint bash aliengo-learning:mid360 -c '
set -e
mkdir -p "$HOME"
cd "$HOME"
source /opt/ros/noetic/setup.bash
source /work/devel/setup.bash
exec roslaunch /mid360/driver.launch config:=/mid360-config.json "$@"' bash "$@"
