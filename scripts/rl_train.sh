#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
algorithm="${1:-}"
run_name="${2:-}"
steps="${3:-4096}"
if [[ $# -gt 3 || ! "$algorithm" =~ ^(ppo|a2c|sac)$ || ! "$run_name" =~ ^[a-zA-Z0-9][a-zA-Z0-9_.-]*$ || ! "$steps" =~ ^[0-9]+$ ]]; then
    echo '用法：./scripts/rl_train.sh ppo|a2c|sac 运行名称 [训练步数]' >&2; exit 2
fi
if (( 10#$steps <= 0 || 10#$steps % 256 != 0 )); then
    echo '步数须为 256 的正整数倍。' >&2; exit 2
fi
source "$root/scripts/compute_guard.sh"
acquire_compute_guard "$(compute_guard_path)"
if [[ ! -r "$root/.ppo-work/aliengo.urdf" ]]; then
    echo '缺少 URDF，请先运行 ./scripts/ppo_prepare_urdf.sh。' >&2; exit 1
fi
site="$root/.ros1_rl/site"
expected=0c0a7cc4f7c74ff024d5a5e21230a01289b65346b27a626f6c815d94b4b8c955
if [[ ! -r "$site/.verified-runtime" || "$(<"$site/.verified-runtime")" != "$expected" || ! -d "$site/torch" ]]; then
    echo '缺少已验证的本地 CPU PyTorch 运行时，未启动训练。' >&2; exit 1
fi
if [[ -e "$root/.ppo-work/$run_name" ]]; then
    echo '输出目录已存在，不覆盖。' >&2; exit 1
fi
if ! docker image inspect aliengo-learning:ppo-cpu >/dev/null 2>&1; then
    echo '缺少训练镜像，请先运行 ./scripts/ppo_build_env.sh。' >&2; exit 1
fi
exec docker run --rm --init --network none --name aliengo-ppo-train \
    --memory=4g --memory-swap=4g --cpus=2 --pids-limit=64 --user "$(id -u):$(id -g)" \
    -e HOME=/tmp/aliengo-ppo -e PYTHONPATH=/torchsite -e PYTHONDONTWRITEBYTECODE=1 \
    --mount "type=bind,source=$site,target=/torchsite,readonly" \
    --mount "type=bind,source=$root/training/aliengo_mujoco,target=/program,readonly" \
    --mount "type=bind,source=$root/.ppo-work,target=/output" \
    --entrypoint python3 aliengo-learning:ppo-cpu /program/run.py \
    --urdf /output/aliengo.urdf --output "/output/$run_name" --steps "$steps" --seed 19 --memory-mib 4096 --algorithm "$algorithm"
