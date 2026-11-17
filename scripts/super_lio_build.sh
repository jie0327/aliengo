#!/usr/bin/env bash
# 在独立工作区编译；参考源码和现有 ROS1 控制工作区不被覆盖。
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root/scripts/compute_guard.sh"
acquire_compute_guard "$(compute_guard_path)"
python3 "$root/scripts/super_lio_setup.py"
docker run --rm --init --network none --name aliengo-super-lio-build \
    --memory=3g --memory-swap=3g --cpus=2 --user "$(id -u):$(id -g)" \
    -e HOME=/tmp/aliengo-lio-build -e OMP_NUM_THREADS=1 -e OPENBLAS_NUM_THREADS=1 \
    --mount "type=bind,source=$root/.super_lio,target=/lio" \
    -w /lio --entrypoint bash aliengo-learning:noetic-lio -c '
set -e
mkdir -p "$HOME"
source /opt/ros/noetic/setup.bash
catkin_make -j1 -l2 -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.8.so \
    -DPYTHON_INCLUDE_DIR=/usr/include/python3.8 -DCMAKE_EXPORT_COMPILE_COMMANDS=ON'
