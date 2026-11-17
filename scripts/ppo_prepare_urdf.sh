#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root/scripts/compute_guard.sh"
acquire_compute_guard "$(compute_guard_path)"
if [[ ! -r "$root/.ros1/devel/setup.bash" ]]; then
    echo '缺少 ROS1 模型工作区，请先运行 ./scripts/ros1_build.sh。' >&2; exit 1
fi
mkdir -p "$root/.ppo-work"
out="$root/.ppo-work/aliengo.urdf"
if [[ -e "$out" ]]; then echo "文件 $out 已存在，不覆盖。" >&2; exit 1; fi
staging="$(mktemp "$root/.ppo-work/.aliengo.XXXXXX")"
trap 'rm -f "$staging"' EXIT
docker run --rm --init --network none --memory=1g --memory-swap=1g --cpus=1 \
    --user "$(id -u):$(id -g)" -e HOME=/tmp/aliengo-ros1 -e ROS_HOME=/tmp/aliengo-ros1/.ros \
    --mount "type=bind,source=$root/.ros1,target=/work,readonly" \
    --entrypoint bash aliengo-learning:noetic -c '
set -e
source /opt/ros/noetic/setup.bash
source /work/devel/setup.bash
exec rosrun xacro xacro /work/src/aliengo_description/xacro/robot.xacro DEBUG:=false' > "$staging"
if [[ ! -s "$staging" ]]; then echo 'URDF 展开结果为空。' >&2; exit 1; fi
# Hard link fails rather than overwriting if another process created the output.
ln "$staging" "$out"
echo "已生成 $out"
