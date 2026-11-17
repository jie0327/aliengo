#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$root/scripts/compute_guard.sh"
acquire_compute_guard "$(compute_guard_path)"
output="$(mktemp -d "$root/logs/fast-livo-trial-XXXXXX")"
echo "FAST-LIVO2 验证记录：$output"
display_args=()
gpu_args=()
if [[ -n "${DISPLAY:-}" && -d /tmp/.X11-unix ]]; then
    source "$root/scripts/ros1_graphics.sh"
    configure_ros1_gpu
    display_args+=( -e "DISPLAY=$DISPLAY" -e QT_X11_NO_MITSHM=1
        --mount type=bind,source=/tmp/.X11-unix,target=/tmp/.X11-unix,readonly )
    auth_file="${XAUTHORITY:-$HOME/.Xauthority}"
    if [[ -r "$auth_file" ]]; then
        display_args+=( -e XAUTHORITY=/tmp/aliengo.xauthority
            --mount "type=bind,source=$auth_file,target=/tmp/aliengo.xauthority,readonly" )
    fi
fi
docker run --rm --init --network none --name aliengo-fast-livo-trial \
    --memory=5g --memory-swap=5g --cpus=4 --shm-size=512m --pids-limit=512 \
    --user "$(id -u):$(id -g)" \
    -e HOME=/tmp/aliengo-fast-livo-trial -e ROS_HOME=/tmp/aliengo-fast-livo-trial/.ros \
    -e ROS_LOG_DIR=/tmp/aliengo-fast-livo-trial/.ros/log \
    -e ROS_HOSTNAME=localhost -e ROS_IP=127.0.0.1 \
    -e OMP_NUM_THREADS=2 -e OPENBLAS_NUM_THREADS=1 \
    -e GAZEBO_MODEL_DATABASE_URI= -e GAZEBO_PLUGIN_PATH=/work/devel/lib \
    -e GAZEBO_MODEL_PATH=/course/ros1/worlds/models:/usr/share/gazebo-11/models \
    --mount "type=bind,source=$root/.ros1,target=/work,readonly" \
    --mount "type=bind,source=$root/.fast_livo,target=/fast_livo" \
    --mount "type=bind,source=$root,target=/course,readonly" \
    --mount "type=bind,source=$output,target=/result" "${display_args[@]}" "${gpu_args[@]}" \
    --entrypoint bash aliengo-learning:noetic-fast-livo -c '
set -e
mkdir -p "$ROS_LOG_DIR"
source /opt/ros/noetic/setup.bash
source /work/devel/setup.bash
source /fast_livo/devel/setup.bash --extend
export LD_LIBRARY_PATH="${ALIENGO_NVIDIA_LIBRARY_PATH:+$ALIENGO_NVIDIA_LIBRARY_PATH:}/fast_livo/sophus-build:${LD_LIBRARY_PATH:-}"
export ROS_PACKAGE_PATH="/course/ros1:$ROS_PACKAGE_PATH"
if [[ -z "${DISPLAY:-}" ]]; then
    export LIBGL_ALWAYS_SOFTWARE=1 LP_NUM_THREADS=2
    exec xvfb-run -a -s "-screen 0 1280x720x24 -nolisten tcp" python3 /course/scripts/verification/fast_livo_trial.py "$@" --output /result
fi
exec python3 /course/scripts/verification/fast_livo_trial.py "$@" --output /result' bash "$@"
