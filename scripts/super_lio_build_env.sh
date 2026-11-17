#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root/scripts/compute_guard.sh"
acquire_compute_guard "$(compute_guard_path)"
docker build --memory=2g --memory-swap=2g --cpu-period=100000 --cpu-quota=200000 \
    -f "$root/docker/Dockerfile.super-lio" -t aliengo-learning:noetic-lio "$root/docker"
