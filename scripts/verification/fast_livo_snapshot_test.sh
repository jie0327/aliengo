#!/usr/bin/env bash
# Compile and exercise the real patched XYZ preprocessing and IMU compensation.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$root/scripts/compute_guard.sh"
acquire_compute_guard "$(compute_guard_path)"
if [[ ! -f "$root/.fast_livo/src/fast_livo/src/preprocess.cpp" ||
      ! -f "$root/.fast_livo/src/Sophus/sophus/se3.h" ]]; then
    echo '请先运行 ./scripts/fast_livo_build.sh。' >&2
    exit 1
fi
docker run --rm --init --network none --memory=3g --memory-swap=3g --cpus=2 \
    --pids-limit=256 --user "$(id -u):$(id -g)" \
    --mount "type=bind,source=$root/.fast_livo,target=/fast_livo,readonly" \
    --mount "type=bind,source=$root,target=/course,readonly" \
    --entrypoint bash aliengo-learning:noetic-fast-livo -c '
set -eo pipefail
source /opt/ros/noetic/setup.bash
set -u
g++ -std=c++17 -O0 -g0 -fopenmp -DROOT_DIR=\"/tmp/\" \
    -I/fast_livo/src/fast_livo/include -I/fast_livo/src/Sophus \
    -I/usr/include/eigen3 -I/usr/include/pcl-1.10 \
    $(pkg-config --cflags roscpp opencv4) \
    /course/tests/fast_livo_snapshot_test.cpp -o /tmp/fast_livo_snapshot_test \
    $(pkg-config --libs roscpp opencv4) -lpcl_common
exec /tmp/fast_livo_snapshot_test "$@"' bash "$@"
