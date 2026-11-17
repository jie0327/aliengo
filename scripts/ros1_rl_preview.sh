#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
container=aliengo-ros1-sim
if [[ "$(docker inspect -f '{{.State.Running}}' "$container" 2>/dev/null || true)" != true ]]; then
    echo '请先运行 ./scripts/ros1_sim.sh。' >&2; exit 1
fi
if [[ ! -f "$root/.ros1_rl/site/.verified-runtime" || ! -f "$root/.ros1_rl/cache/body.jit" ]]; then
    echo '请先运行 python3 ./scripts/ros1_rl_setup.py。' >&2; exit 1
fi
exec docker exec -i "$container" bash -c '
set -e
source /opt/ros/noetic/setup.bash
source /work/devel/setup.bash
export PYTHONPATH=/course/.ros1_rl/site:/course/ros1:${PYTHONPATH:-}
exec python3 /course/ros1/unio4_preview.py "$@"' bash "$@"
