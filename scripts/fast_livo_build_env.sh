#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root/scripts/compute_guard.sh"
acquire_compute_guard "$(compute_guard_path)"
docker build --memory=3g --memory-swap=3g --cpu-period=100000 --cpu-quota=200000 \
    -f "$root/docker/Dockerfile.fast-livo" -t aliengo-learning:noetic-fast-livo "$root/docker"
