#!/usr/bin/env bash
# Expand the ROS 1 AlienGo model for offline kinematics exercises.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
workspace="$root/.ros1"
image=aliengo-exploration:simulation
if [[ ! -f "$workspace/devel/setup.bash" ]]; then
    echo '请先运行 ./scripts/ros1_build.sh。' >&2; exit 1
fi
if ! docker image inspect "$image" >/dev/null 2>&1; then
    echo "缺少 ROS 1 教学镜像 $image。" >&2; exit 1
fi
temp="$(mktemp "$workspace/.aliengo.urdf.XXXXXX")"
trap 'rm -f -- "$temp"' EXIT
docker run --rm --init --network none --user "$(id -u):$(id -g)" \
    --mount "type=bind,source=$workspace,target=/work,readonly" \
    --entrypoint bash "$image" -c '
set -e
source /opt/ros/noetic/setup.bash
source /work/devel/setup.bash
exec xacro /work/src/aliengo_description/xacro/robot.xacro DEBUG:=false' > "$temp"
mv -f -- "$temp" "$workspace/aliengo.urdf"
echo "已生成 $workspace/aliengo.urdf"
