#!/usr/bin/env bash
# Compatibility entry for the original PPO-only command.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec "$root/scripts/rl_train.sh" ppo "$@"
