#!/usr/bin/env bash
# Explicit SSH/scp operations only when invoked; no robot contact during build.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
action="${1:-}"
target="${2:-}"
if [[ -z "$target" || "$target" == -* ]]; then
    echo '用法：gbplanner_camera_onboard.sh {probe|stage|onboard|sync} 用户名@192.168.123.12 [相机序列号]' >&2; exit 2
fi
case "$action" in
probe)
    ssh "$target" bash -s <<'REMOTE'
set -u
uname -m
cat /etc/os-release
printf '\nROS installations:\n'
ls -d /opt/ros/* 2>/dev/null || true
printf '\nCamera/vision processes:\n'
pgrep -af 'RobotVision|realsense' || true
printf '\nUSB cameras:\n'
lsusb 2>/dev/null || true
if command -v rs-enumerate-devices >/dev/null 2>&1; then rs-enumerate-devices -s; fi
printf '\nClock (compare with laptop):\n'
date -u '+%Y-%m-%dT%H:%M:%S.%N UTC'
for distro in noetic melodic kinetic; do
    if [ -f "/opt/ros/$distro/setup.bash" ]; then
        set +u
        . "/opt/ros/$distro/setup.bash"
        set -u
        rospack find realsense2_camera || true
        break
    fi
done
pkg-config --modversion realsense2 2>/dev/null || true
REMOTE
    ;;
stage)
    ssh "$target" 'mkdir -p "$HOME/aliengo-gbplanner-camera"'
    scp "$root/ros1/hardware/camera.launch" "$root/ros1/hardware/onboard_camera.sh" \
        "$root/ros1/hardware/sync_clock.py" \
        "$root/.hardware/onboard-kit/realsense-ros-2.3.2.tar.gz" \
        "$target:aliengo-gbplanner-camera/"
    echo '文件已复制；未安装系统包、结束进程或启动相机。'
    ;;
sync)
    exec ssh -t "$target" 'sudo python3 "$HOME/aliengo-gbplanner-camera/sync_clock.py" --set'
    ;;
onboard)
    serial="${3:-}"
    printf -v quoted '%q' "$serial"
    printf -v master_uri '%q' "${ROS_MASTER_URI:-http://192.168.123.100:11311}"
    exec ssh -t "$target" "ROS_MASTER_URI=$master_uri bash \"\$HOME/aliengo-gbplanner-camera/onboard_camera.sh\" $quoted"
    ;;
*) echo '未知操作：probe|stage|onboard|sync' >&2; exit 2 ;;
esac
