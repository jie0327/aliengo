#!/usr/bin/env bash
# 使用独立 PyTorch 环境运行课程训练，与已有仿真/训练脚本共用互斥锁。
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
python="$root/.venv-ppo/bin/python"
if [[ ! -x "$python" ]]; then
    echo '缺少 .venv-ppo，请先按 docs/lesson_10_zh.md 安装独立训练环境。' >&2
    exit 1
fi
if [[ "${1:-}" != --help && "${1:-}" != -h ]]; then
    source "$root/scripts/compute_guard.sh"
    acquire_compute_guard "$(compute_guard_path)"
fi
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
export CUBLAS_WORKSPACE_CONFIG=:4096:8
# 忽略终端里可能由 ROS setup 注入的 PYTHONPATH/PYTHONHOME。
exec "$python" -E "$root/examples/lesson10/train_ppo.py" "$@"
