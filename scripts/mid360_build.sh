#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
workspace="$root/.mid360"
source "$root/scripts/compute_guard.sh"
acquire_compute_guard "$(compute_guard_path)"
if [[ -e "$workspace" ]]; then
    echo "已有 $workspace；为保护现有文件，拒绝覆盖。" >&2; exit 1
fi
if ! docker image inspect aliengo-learning:noetic >/dev/null 2>&1; then
    echo '请先运行 ./scripts/ros1_build_env.sh 构建基础镜像。' >&2; exit 1
fi
staging="$(mktemp -d "$root/.mid360-init.XXXXXX")"
trap 'rm -rf -- "$staging"' EXIT
fetch_pinned() {
    local url="$1" revision="$2" dir="$3"
    git init -q "$dir"
    git -C "$dir" remote add origin "$url"
    git -C "$dir" fetch -q --depth 1 origin "$revision"
    git -C "$dir" checkout -q --detach FETCH_HEAD
    [[ "$(git -C "$dir" rev-parse HEAD)" == "$revision" ]]
}
mkdir -p "$staging/src"
fetch_pinned https://github.com/Livox-SDK/Livox-SDK2.git c0796f04c143143899c87a773d9f6b7136453c0b "$staging/sdk"
fetch_pinned https://github.com/Livox-SDK/livox_ros_driver2.git 21445540f0d100dc86a7e6df312dd70bbdb4afdf "$staging/src/livox_ros_driver2"
cp "$staging/src/livox_ros_driver2/package_ROS1.xml" "$staging/src/livox_ros_driver2/package.xml"
docker build --memory=3g --memory-swap=3g --cpu-period=100000 --cpu-quota=200000 -f "$root/docker/Dockerfile.mid360" -t aliengo-learning:mid360 "$root"
docker run --rm --init --network none --memory=3g --memory-swap=3g --cpus=2 --user "$(id -u):$(id -g)" \
    -e HOME=/tmp/aliengo-mid360 -e ROS_HOME=/tmp/aliengo-mid360/.ros \
    --mount "type=bind,source=$staging,target=/work" -w /work --entrypoint bash aliengo-learning:mid360 -c '
set -e
mkdir -p "$HOME"
cmake -S sdk -B sdk-build -DCMAKE_INSTALL_PREFIX=/work/sdk-install -DCMAKE_BUILD_TYPE=Release
cmake --build sdk-build --parallel 2
cmake --install sdk-build
source /opt/ros/noetic/setup.bash
catkin_make -j2 -DROS_EDITION=ROS1 -DCMAKE_CXX_FLAGS=-I/work/sdk-install/include -DLIVOX_LIDAR_SDK_LIBRARY=/work/sdk-install/lib/liblivox_lidar_sdk_static.a'
mv -T -- "$staging" "$workspace"
trap - EXIT
echo 'MID-360 驱动构建完成。接下来按 docs/mid360_zh.md 设置地址。'
