#!/usr/bin/env bash
# Run offline experiments or read-only observers in the ROS 1 environment.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
image=aliengo-learning:noetic

usage() {
    echo '用法：./scripts/ros1_course.sh offline|sim 课程脚本.py [脚本参数]'
    echo '脚本路径相对 examples/；offline 在无网络容器运行，sim 使用运行中的 ROS 1 仿真。'
}
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
    usage
    exit 0
fi
if [[ "$#" -lt 2 || ( "$1" != offline && "$1" != sim ) ]]; then
    usage >&2
    exit 2
fi
relative="$2"
mode="$1"
shift 2
example="$(realpath -m -- "$root/examples/$relative")"
case "$example" in
    "$root/examples/"*) ;;
    *) echo '课程脚本必须位于 examples/ 内。' >&2; exit 2 ;;
esac
if [[ ! -f "$example" || "$example" != *.py ]]; then
    echo "找不到Python课程脚本：examples/$relative" >&2
    exit 2
fi
relative="${example#"$root/examples/"}"
if [[ "$mode" == sim ]]; then
    if [[ "$(docker inspect -f '{{.State.Running}}' aliengo-ros1-sim 2>/dev/null || true)" != true ]]; then
        echo 'ROS 1 仿真未运行，请先运行 ./scripts/ros1_sim.sh。' >&2
        exit 1
    fi
    exec docker exec aliengo-ros1-sim bash -c '
set -e
source /opt/ros/noetic/setup.bash
source /work/devel/setup.bash
exec python3 "$@"' bash "/course/examples/$relative" "$@"
fi
if ! docker image inspect "$image" >/dev/null 2>&1; then
    echo "缺少 ROS 1 教学镜像 $image。" >&2
    exit 1
fi
exec docker run --rm --init --network none --user "$(id -u):$(id -g)" \
    -e HOME=/tmp/aliengo-course -e PYTHONDONTWRITEBYTECODE=1 \
    --mount "type=bind,source=$root,target=/work" -w /work \
    --entrypoint python3 "$image" "/work/examples/$relative" "$@"
