#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root/scripts/compute_guard.sh"
acquire_compute_guard "$(compute_guard_path)"
if ! docker image inspect aliengo-learning:noetic >/dev/null 2>&1; then
    echo '缺少基础镜像，请先完成 ./scripts/ros1_build_env.sh。' >&2; exit 1
fi
docker build --memory=3g --memory-swap=3g --cpu-period=100000 --cpu-quota=200000 \
    -f "$root/docker/Dockerfile.ppo" -t aliengo-learning:ppo-cpu "$root"
