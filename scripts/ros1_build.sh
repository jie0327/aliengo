#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
image=aliengo-learning:noetic
workspace="$root/.ros1"
source "$root/scripts/compute_guard.sh"
acquire_compute_guard "$(compute_guard_path)"

if ! docker image inspect "$image" >/dev/null 2>&1; then
    echo "缺少 ROS 1 仿真镜像 $image；请先运行 ./scripts/ros1_build_env.sh。" >&2
    exit 1
fi
if [[ ! -d "$workspace/src" ]]; then
    if [[ -e "$workspace" ]]; then
        echo "工作区 $workspace 已存在但没有 src，保留原有文件；请先检查该目录。" >&2
        exit 1
    fi
    staging="$(mktemp -d "$root/.ros1-init.XXXXXX")"
    trap 'rm -rf "$staging"' EXIT
    mkdir "$staging/src"
    cp -a "$root/references/unitree_guide/unitree_guide" "$staging/src/"
    cp -a "$root/references/unitree_ros/unitree_gazebo" "$staging/src/"
    cp -a "$root/references/unitree_ros/unitree_controller" "$staging/src/"
    cp -a "$root/references/unitree_ros/unitree_legged_control" "$staging/src/"
    cp -a "$root/references/unitree_ros/robots/aliengo_description" "$staging/src/"
    cp -a "$root/references/unitree_ros/unitree_ros_to_real/unitree_legged_msgs" "$staging/src/"
    patch --batch -d "$staging/src" -p0 < "$root/ros1/aliengo-guide.patch"
    patch --batch -d "$staging/src" -p0 < "$root/ros1/patches/guide_sim_clock.patch"
    mv -T "$staging" "$workspace"
    trap - EXIT
fi
# Apply only the autonomy delta. Dry runs prevent partial modifications to custom
# workspaces; never recopy the reference controller over an existing source tree.
autonomy_patch="$root/ros1/patches/guide_autonomy.patch"
if patch --batch --forward --dry-run -d "$workspace/src" -p0 < "$autonomy_patch" >/dev/null 2>&1; then
    patch --batch --forward -d "$workspace/src" -p0 < "$autonomy_patch"
elif patch --batch --reverse --dry-run -d "$workspace/src" -p0 < "$autonomy_patch" >/dev/null 2>&1; then
    echo "Guide autonomy patch already applied."
else
    echo "Guide autonomy patch conflicts with existing source; preserved workspace: $workspace/src/unitree_guide" >&2
    exit 1
fi

if [[ ! -e "$workspace/src/livox_laser_simulation" ]]; then
    # Preserve existing workspaces while installing our pinned, patched plugin.
    plugin_stage="$(mktemp -d "$workspace/.livox-init.XXXXXX")"
    trap 'rm -rf -- "$plugin_stage"' EXIT
    cp -a "$root/references/livox_laser_simulation" "$plugin_stage/livox_laser_simulation"
    patch --batch -d "$plugin_stage" -p0 < "$root/ros1/patches/livox_sim_noetic.patch"
    patch --batch -d "$plugin_stage" -p0 < "$root/ros1/patches/livox_snapshot_stamp.patch"
    mv -T "$plugin_stage/livox_laser_simulation" "$workspace/src/livox_laser_simulation"
    rmdir "$plugin_stage"
    trap - EXIT
fi
if [[ ! -e "$workspace/src/realsense_gazebo_plugin" ]]; then
    camera_stage="$(mktemp -d "$workspace/.d435-init.XXXXXX")"
    trap 'rm -rf -- "$camera_stage"' EXIT
    cp -a "$root/references/realsense_gazebo_plugin" "$camera_stage/realsense_gazebo_plugin"
    patch --batch -d "$camera_stage" -p0 < "$root/ros1/patches/realsense_rgbd.patch"
    mv -T "$camera_stage/realsense_gazebo_plugin" "$workspace/src/realsense_gazebo_plugin"
    rmdir "$camera_stage"
    trap - EXIT
fi
docker run --rm --init --network none --memory=3g --memory-swap=3g --cpus=2 --user "$(id -u):$(id -g)" \
    -e HOME=/tmp/aliengo-ros1 -e ROS_HOME=/tmp/aliengo-ros1/.ros \
    --mount "type=bind,source=$workspace,target=/work" \
    -w /work --entrypoint bash "$image" -c '
set -e
mkdir -p "$HOME"
source /opt/ros/noetic/setup.bash
catkin_make -j2 -DCMAKE_EXPORT_COMPILE_COMMANDS=ON'

if [[ -f "$workspace/editor/image-id" ]]; then
    python3 "$root/scripts/ros1_editor_config.py"
fi
