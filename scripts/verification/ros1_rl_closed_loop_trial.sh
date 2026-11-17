#!/usr/bin/env bash
# Explicit experimental trial: creates one isolated ROS/Gazebo container.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
if [[ $# != 1 || ( "${1:-}" != --calibrate && "${1:-}" != --baseline ) ]]; then
    echo '仅供隔离研究：用法 ./scripts/verification/ros1_rl_closed_loop_trial.sh --baseline|--calibrate' >&2
    exit 2
fi
if [[ -n "$(docker ps -q)" ]]; then
    echo '已有容器运行；停止占用资源的容器后再由统一调度执行，不启动试验。' >&2
    exit 1
fi
if [[ ! -f "$root/.ros1/devel/setup.bash" || ! -f "$root/.ros1_rl/site/.verified-runtime" ]]; then
    echo '需要先准备 ROS1 构建和策略推理依赖。' >&2; exit 1
fi
mkdir -p "$root/logs"
output="$(mktemp -d "$root/logs/rl-closed-loop-XXXXXX")"
echo "实验记录：$output"
timeout --signal=TERM --kill-after=10s 300s docker run --rm --init --network none \
    --memory=4g --memory-swap=4g --cpus=4 --shm-size=512m --user "$(id -u):$(id -g)" \
    -e HOME=/tmp/rl-trial -e ROS_HOME=/tmp/rl-trial/.ros -e ROS_LOG_DIR=/tmp/rl-trial/.ros/log \
    -e ROS_MASTER_URI=http://127.0.0.1:11311 -e ROS_HOSTNAME=127.0.0.1 -e ROS_IP=127.0.0.1 \
    -e GAZEBO_MODEL_DATABASE_URI= -e GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models \
    -e GAZEBO_PLUGIN_PATH=/work/devel/lib \
    --mount "type=bind,source=$root/.ros1,target=/work,readonly" \
    --mount "type=bind,source=$root,target=/course,readonly" \
    --mount "type=bind,source=$output,target=/result" \
    --entrypoint bash aliengo-learning:noetic -c '
set -e
mkdir -p "$ROS_LOG_DIR"
source /opt/ros/noetic/setup.bash
source /work/devel/setup.bash
export PYTHONPATH=/course/.ros1_rl/site:/course/ros1:${PYTHONPATH:-}
exec python3 /course/scripts/verification/ros1_rl_closed_loop_trial.py "$1"' bash "$1"
