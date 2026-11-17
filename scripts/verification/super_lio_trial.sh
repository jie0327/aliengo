#!/usr/bin/env bash
# 一次只启动一个有界容器，独立记录真实 LIO 估计、传感器包和地图。
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$root/scripts/compute_guard.sh"
acquire_compute_guard "$(compute_guard_path)"
output="$(mktemp -d "$root/logs/super-lio-trial-XXXXXX")"
echo "Super-LIO 验证记录：$output"
display_args=()
gpu_args=()
for arg in "$@"; do
    if [[ "$arg" == --gui || "$arg" == --rviz ]]; then
        source "$root/scripts/ros1_graphics.sh"
        configure_ros1_gpu
        display_args+=( -e "DISPLAY=$DISPLAY" -e QT_X11_NO_MITSHM=1
            --mount type=bind,source=/tmp/.X11-unix,target=/tmp/.X11-unix,readonly )
        auth_file="${XAUTHORITY:-$HOME/.Xauthority}"
        if [[ -r "$auth_file" ]]; then
            display_args+=( -e XAUTHORITY=/tmp/aliengo.xauthority
                --mount "type=bind,source=$auth_file,target=/tmp/aliengo.xauthority,readonly" )
        fi
        if [[ -d /usr/share/fonts/opentype/noto ]]; then
            display_args+=( --mount type=bind,source=/usr/share/fonts/opentype/noto,target=/usr/share/fonts/opentype/noto,readonly )
        fi
        break
    fi
done
docker run --rm --init --network none --name aliengo-super-lio-trial \
    --memory=4g --memory-swap=4g --cpus=4 --shm-size=512m --pids-limit=512 \
    --user "$(id -u):$(id -g)" \
    -e HOME=/tmp/aliengo-lio-trial -e ROS_HOME=/tmp/aliengo-lio-trial/.ros \
    -e ROS_LOG_DIR=/tmp/aliengo-lio-trial/.ros/log \
    -e ROS_HOSTNAME=localhost -e ROS_IP=127.0.0.1 \
    -e OMP_NUM_THREADS=1 -e OPENBLAS_NUM_THREADS=1 \
    -e GAZEBO_MODEL_DATABASE_URI= -e GAZEBO_PLUGIN_PATH=/work/devel/lib \
    -e GAZEBO_MODEL_PATH=/course/ros1/worlds/models:/usr/share/gazebo-11/models \
    --mount "type=bind,source=$root/.ros1,target=/work,readonly" \
    --mount "type=bind,source=$root/.super_lio,target=/lio" \
    --mount "type=bind,source=$root,target=/course,readonly" \
    --mount "type=bind,source=$output,target=/result" "${display_args[@]}" "${gpu_args[@]}" \
    --entrypoint bash aliengo-learning:noetic-lio -c '
set -e
mkdir -p "$ROS_LOG_DIR"
source /opt/ros/noetic/setup.bash
source /work/devel/setup.bash
source /lio/devel/setup.bash --extend
export LD_LIBRARY_PATH="${ALIENGO_NVIDIA_LIBRARY_PATH:+$ALIENGO_NVIDIA_LIBRARY_PATH:}${LD_LIBRARY_PATH:-}"
export ROS_PACKAGE_PATH="/course/ros1:$ROS_PACKAGE_PATH"
exec python3 /course/scripts/verification/super_lio_trial.py "$@" --output /result' bash "$@"
