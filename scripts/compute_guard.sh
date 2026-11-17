#!/usr/bin/env bash
# Source this in foreground launchers. Descriptor 9 holds the lock until exit.
compute_guard_path() {
    printf '/tmp/aliengo-learning-compute-%s.lock\n' "$(id -u)"
}

acquire_compute_guard() {
    local lock_path="$1" active name
    if ! command -v flock >/dev/null 2>&1; then
        echo '缺少 flock，不能检查仿真和训练互斥。' >&2; return 1
    fi
    exec 9>"$lock_path"
    if ! flock -n 9; then
        echo '已有仿真、训练或构建正在运行，请先结束后再启动。' >&2; return 1
    fi
    if ! active="$(docker ps --format '{{.Names}}')"; then
        echo '无法检查 Docker 容器状态，未启动。' >&2; return 1
    fi
    for name in aliengo-ros1-sim aliengo-ppo-train; do
        if [[ "$active" == "$name" || "$active" == "$name"$'\n'* ||
              "$active" == *$'\n'"$name" || "$active" == *$'\n'"$name"$'\n'* ]]; then
            echo "容器 $name 正在运行；请先结束后再启动。" >&2; return 1
        fi
    done
}
