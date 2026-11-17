#!/usr/bin/env bash
# 给宿主机编辑器导出 Docker 中的头文件和真实编译配置，不启动仿真。
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
image=aliengo-learning:noetic
workspace="$root/.ros1"
editor="$workspace/editor"
source "$root/scripts/compute_guard.sh"
acquire_compute_guard "$(compute_guard_path)"

image_id="$(docker image inspect "$image" --format '{{.Id}}')"
mkdir -p "$editor/sysroot"
docker run --rm --init --network none --memory=2g --memory-swap=2g --cpus=1 --pids-limit=128 \
    --user "$(id -u):$(id -g)" \
    -e EDITOR_RUNTIME_DIR=/tmp/aliengo-editor -e ROS_HOME=/tmp/aliengo-editor/.ros \
    -e EDITOR_IMAGE_ID="$image_id" \
    --mount "type=bind,source=$workspace,target=/work" \
    --mount "type=bind,source=$editor,target=/editor" \
    -w /work --entrypoint bash "$image" -c '
set -e
mkdir -p "$EDITOR_RUNTIME_DIR"
source /opt/ros/noetic/setup.bash
cmake -S /work/src -B /work/build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON > /editor/cmake.log 2>&1
if [[ ! -f /editor/image-id ]] || [[ "$(cat /editor/image-id)" != "$EDITOR_IMAGE_ID" ]]; then
    mkdir -p /editor/sysroot/usr /editor/sysroot/opt/ros/noetic
    cp -R /usr/include /editor/sysroot/usr/
    cp -R /opt/ros/noetic/include /editor/sysroot/opt/ros/noetic/
    printf "%s\n" "$EDITOR_IMAGE_ID" > /editor/image-id
fi'

python3 "$root/scripts/ros1_editor_config.py"
echo 'ROS 1 编辑器配置已更新；VS Code 会读取 .vscode/c_cpp_properties.json。'
