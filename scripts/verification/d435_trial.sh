#!/usr/bin/env bash
# Bounded real RGB-D and MID360 regression, rendered by gzserver without gzclient.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$root/scripts/compute_guard.sh"
acquire_compute_guard "$(compute_guard_path)"
source "$root/scripts/ros1_graphics.sh"
configure_ros1_gpu
: "${DISPLAY:?D435 verification requires a host X11 DISPLAY}"
auth_file="${XAUTHORITY:-$HOME/.Xauthority}"
[[ -r "$auth_file" ]] || { echo "X11 authority is not readable: $auth_file" >&2; exit 1; }
mkdir -p "$root/logs"
output="$(mktemp -d "$root/logs/d435-trial-XXXXXX")"
echo "D435 verification records: $output"
docker run --rm --init --network none --name aliengo-d435-trial \
    --memory=4g --memory-swap=4g --cpus=4 --shm-size=512m --pids-limit=512 \
    --user "$(id -u):$(id -g)" \
    -e HOME=/tmp/aliengo-d435-trial -e ROS_HOME=/tmp/aliengo-d435-trial/.ros \
    -e ROS_LOG_DIR=/tmp/aliengo-d435-trial/.ros/log \
    -e ROS_HOSTNAME=localhost -e ROS_IP=127.0.0.1 \
    -e OMP_NUM_THREADS=1 -e OPENBLAS_NUM_THREADS=1 \
    -e GAZEBO_MODEL_DATABASE_URI= -e GAZEBO_PLUGIN_PATH=/work/devel/lib \
    -e GAZEBO_MODEL_PATH=/course/ros1/worlds/models:/usr/share/gazebo-11/models \
    -e "DISPLAY=$DISPLAY" -e QT_X11_NO_MITSHM=1 -e XAUTHORITY=/tmp/aliengo.xauthority \
    --mount type=bind,source=/tmp/.X11-unix,target=/tmp/.X11-unix,readonly \
    --mount "type=bind,source=$auth_file,target=/tmp/aliengo.xauthority,readonly" \
    --mount "type=bind,source=$root/.ros1,target=/work,readonly" \
    --mount "type=bind,source=$root,target=/course,readonly" \
    --mount "type=bind,source=$output,target=/result" "${gpu_args[@]}" \
    --entrypoint bash aliengo-learning:noetic -c '
set -e
mkdir -p "$ROS_LOG_DIR"
source /opt/ros/noetic/setup.bash
source /work/devel/setup.bash
export LD_LIBRARY_PATH="${ALIENGO_NVIDIA_LIBRARY_PATH:+$ALIENGO_NVIDIA_LIBRARY_PATH:}${LD_LIBRARY_PATH:-}"
export ROS_PACKAGE_PATH="/course/ros1:$ROS_PACKAGE_PATH"
exec python3 /course/scripts/verification/d435_trial.py "$@" --output /result' bash "$@"
