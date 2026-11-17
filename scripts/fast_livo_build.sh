#!/usr/bin/env bash
# Build pinned sources in an isolated writable workspace, with bounded resources.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root/scripts/compute_guard.sh"
acquire_compute_guard "$(compute_guard_path)"
python3 "$root/scripts/fast_livo_setup.py"
if ! docker image inspect aliengo-learning:noetic-fast-livo >/dev/null 2>&1; then
    echo '缺少 FAST-LIVO2 镜像；请先运行 scripts/fast_livo_build_env.sh。' >&2
    exit 1
fi
docker run --rm --init --network none --name aliengo-fast-livo-build \
    --memory=4g --memory-swap=4g --cpus=2 --user "$(id -u):$(id -g)" \
    -e HOME=/tmp/aliengo-fast-livo-build -e OMP_NUM_THREADS=1 -e OPENBLAS_NUM_THREADS=1 \
    --mount "type=bind,source=$root/.fast_livo,target=/fast_livo" \
    --mount "type=bind,source=$root,target=/course,readonly" \
    -w /fast_livo --entrypoint bash aliengo-learning:noetic-fast-livo -c '
set -e
mkdir -p "$HOME"
source /opt/ros/noetic/setup.bash
cmake -S /fast_livo/src/Sophus -B /fast_livo/sophus-build \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS_RELEASE="-O2 -DNDEBUG" \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build /fast_livo/sophus-build --target Sophus -- -j1 -l2
export LD_LIBRARY_PATH="/fast_livo/sophus-build:${LD_LIBRARY_PATH:-}"
catkin_make -j1 -l2 -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS_RELEASE="-O2 -DNDEBUG" -DSophus_DIR=/fast_livo/sophus-build \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.8.so \
    -DPYTHON_INCLUDE_DIR=/usr/include/python3.8 -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
test -x /fast_livo/devel/lib/fast_livo/fastlivo_mapping'
