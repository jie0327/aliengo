#!/usr/bin/env bash
# Real hardware only; each long-running subcommand occupies its terminal.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
hardware="$root/.hardware"
sdk="${ALIENGO_SDK_DIR:-$root/unitree_legged_sdk_v3.8.4}"
image=aliengo-learning:gbplanner-real
action="${1:-help}"
if (( $# )); then shift; fi
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://192.168.123.100:11311}"
export ROS_IP="${ROS_IP:-192.168.123.100}"
unset ROS_HOSTNAME
mkdir -p "$hardware"

render_config() {
    python3 "$root/scripts/hardware_config.py" "$hardware/robot.yaml" \
        --sensor-output "$hardware/planner_sensor.yaml" --runtime-output "$hardware/runtime.yaml"
}
runtime() {
    local name="$1"
    shift
    local options=()
    if [[ -t 0 && -t 1 ]]; then options+=(-it); else options+=(-i); fi
    if [[ -f "${XAUTHORITY:-$HOME/.Xauthority}" ]]; then
        options+=(-v "${XAUTHORITY:-$HOME/.Xauthority}:/tmp/host-xauthority:ro")
    fi
    if [[ -d /tmp/.X11-unix ]]; then options+=(-v /tmp/.X11-unix:/tmp/.X11-unix:ro); fi
    if [[ -d /usr/share/fonts/opentype/noto ]]; then
        options+=(-v /usr/share/fonts/opentype/noto:/usr/share/fonts/opentype/noto:ro)
    fi
    docker run --rm --init --network host --name "aliengo-real-$name" \
        "${options[@]}" --user "$(id -u):$(id -g)" -e HOME=/tmp/aliengo-real \
        -e ROS_HOME=/tmp/aliengo-real/.ros -e ROS_LOG_DIR=/tmp/aliengo-real/log \
        -e ROS_MASTER_URI -e ROS_IP -e ROS_HOSTNAME -e DISPLAY \
        -e LIBGL_ALWAYS_SOFTWARE=1 -e XAUTHORITY=/tmp/host-xauthority \
        --mount "type=bind,source=$root/ros1,target=/course/ros1,readonly" \
        --mount "type=bind,source=$root/.gbplanner,target=/gbp,readonly" \
        --mount "type=bind,source=$root/.super_lio,target=/lio,readonly" \
        --mount "type=bind,source=$root/.mid360,target=/work,readonly" \
        --mount "type=bind,source=$hardware,target=/hardware" \
        --mount "type=bind,source=$sdk,target=/sdk,readonly" \
        --entrypoint bash "$image" /course/ros1/hardware/scripts/runtime.sh "$@"
}
require_master() {
    if [[ "$(docker inspect -f '{{.State.Running}}' aliengo-real-master 2>/dev/null || true)" != true ]]; then
        echo '先在另一个终端运行 ./scripts/gbplanner_real.sh master。' >&2; exit 1
    fi
}
case "$action" in
init)
    if [[ ! -e "$hardware/robot.yaml" ]]; then cp "$root/ros1/hardware/robot.example.yaml" "$hardware/robot.yaml"; fi
    echo "配置：$hardware/robot.yaml；填入实测外参。已有配置不覆盖。"
    ;;
build)
    source "$root/scripts/compute_guard.sh"
    acquire_compute_guard "$(compute_guard_path)"
    docker build -f "$root/docker/Dockerfile.hardware" -t "$image" "$root"
    mkdir -p "$hardware/bin"
    docker run --rm --network none --user "$(id -u):$(id -g)" \
        --mount "type=bind,source=$root/ros1/hardware,target=/source,readonly" \
        --mount "type=bind,source=$sdk,target=/sdk,readonly" \
        --mount "type=bind,source=$hardware,target=/hardware" \
        --entrypoint bash "$image" -c '
set -e
source /opt/ros/noetic/setup.bash
case "$(uname -m)" in x86_64) arch=amd64;; aarch64) arch=arm64;; *) exit 2;; esac
g++ -std=c++14 -O2 /source/high_level_bridge.cpp -I/source/include -I/sdk/include \
    $(pkg-config --cflags --libs roscpp) -L/sdk/lib/cpp/$arch \
    -Wl,-rpath,/sdk/lib/cpp/$arch -lunitree_legged_sdk -lpthread -o /hardware/bin/high_level_bridge
ldd /hardware/bin/high_level_bridge
'
    python3 - "$sdk" "$hardware" <<'PY'
import hashlib,json,pathlib,platform,subprocess,sys
sdk,out=map(pathlib.Path,sys.argv[1:])
arch={'x86_64':'amd64','aarch64':'arm64'}[platform.machine()]
paths=[sdk/'include/unitree_legged_sdk/comm.h',sdk/('lib/cpp/'+arch+'/libunitree_legged_sdk.so')]
metadata={'sdk_path':str(sdk.resolve()),'sdk_commit':subprocess.check_output(
    ['git','-C',str(sdk),'rev-parse','HEAD'],text=True).strip(),
    'sha256':{str(p.relative_to(sdk)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},
    'image':subprocess.check_output(['docker','image','inspect','aliengo-learning:gbplanner-real',
                                   '--format','{{.Id}}'],text=True).strip()}
(out/'build.json').write_text(json.dumps(metadata,indent=2)+'\n')
print('High-level bridge built:',metadata['sdk_commit'])
PY
    ;;
master)
    source "$root/scripts/compute_guard.sh"
    acquire_compute_guard "$(compute_guard_path)"
    runtime master roscore
    ;;
lidar)
    require_master
    if [[ ! -f "$hardware/MID360.json" ]]; then echo '先用 scripts/mid360_config.py 生成 .hardware/MID360.json。' >&2; exit 1; fi
    exec "$root/scripts/mid360_run.sh" "$hardware/MID360.json" xfer_format:=1 \
        lidar_topic:=/livox/raw_lidar imu_topic:=/livox/raw_imu "$@"
    ;;
camera)
    require_master
    # Camera enumeration does not need body extrinsics yet.
    serial="$(python3 - "$hardware/robot.yaml" <<'PY'
import sys,yaml
print(yaml.safe_load(open(sys.argv[1])).get('camera_serial',''))
PY
)"
    camera_devices=()
    for device in /dev/video* /dev/media*; do
        if [[ -c "$device" ]]; then camera_devices+=(--device "$device"); fi
    done
    docker run --rm --init --network host --name aliengo-real-camera \
        "${camera_devices[@]}" \
        --device-cgroup-rule='c 189:* rmw' -v /dev/bus/usb:/dev/bus/usb \
        -v /run/udev:/run/udev:ro \
        -e ROS_HOME=/tmp/aliengo-real/.ros -e ROS_LOG_DIR=/tmp/aliengo-real/log \
        -e HOME=/tmp/aliengo-real -e ROS_MASTER_URI -e ROS_IP -e ROS_HOSTNAME \
        --mount "type=bind,source=$root/ros1/hardware,target=/hardware-source,readonly" \
        --entrypoint bash "$image" -c '
source /opt/ros/noetic/setup.bash
unset ROS_HOSTNAME
exec roslaunch /hardware-source/camera.launch "serial:=$1"' bash "$serial"
    ;;
camera-probe|camera-stage|camera-onboard|camera-sync)
    if [[ "$action" == camera-onboard && $# == 1 ]]; then
        serial="$(python3 - "$hardware/robot.yaml" <<'PY'
import sys,yaml
print(yaml.safe_load(open(sys.argv[1])).get('camera_serial',''))
PY
)"
        set -- "$1" "$serial"
    fi
    exec "$root/scripts/gbplanner_camera_onboard.sh" "${action#camera-}" "$@"
    ;;
time-server)
    require_master
    runtime time python3 /course/ros1/hardware/sync_clock.py --serve --bind "$ROS_IP"
    ;;
map)
    require_master
    render_config
    session="$(date +%Y%m%d-%H%M%S)-$$"
    mkdir -p "$hardware/sessions/$session/maps"
    cp "$hardware/robot.yaml" "$hardware/sessions/$session/robot.yaml"
    if [[ -f "$hardware/build.json" ]]; then cp "$hardware/build.json" "$hardware/sessions/$session/build.json"; fi
    ln -sfn "sessions/$session" "$hardware/latest-session"
    echo "站稳并保持静止，等待 Map init done。数据：.hardware/sessions/$session"
    runtime map roslaunch /course/ros1/hardware/real.launch \
        "map_dir:=../../../hardware/sessions/$session/maps" "$@" 2>&1 | tee -i "$hardware/sessions/$session/launch.log"
    ;;
sdk)
    require_master
    render_config
    if [[ ! -x "$hardware/bin/high_level_bridge" ]]; then echo '先执行 gbplanner_real.sh build。' >&2; exit 1; fi
    python3 - "$sdk" "$hardware/build.json" <<'PY'
import hashlib,json,pathlib,sys
sdk=pathlib.Path(sys.argv[1]); metadata=json.loads(pathlib.Path(sys.argv[2]).read_text())
for name,want in metadata['sha256'].items():
    if hashlib.sha256((sdk/name).read_bytes()).hexdigest()!=want:
        raise SystemExit('SDK 与已编译桥接不一致；先重新运行 gbplanner_real.sh build。')
PY
    echo '接入原厂高层 SDK：机器人须已由遥控器进入运动模式并站稳；不运行厂商的自动行走示例。'
    runtime sdk bash -c 'rosparam load /hardware/runtime.yaml /aliengo_hardware
exec /hardware/bin/high_level_bridge "$@"' bash "$@"
    ;;
enable|stop)
    require_master
    value=false
    if [[ "$action" == enable ]]; then value=true; fi
    runtime "$action-$$" python3 /course/ros1/hardware/scripts/set_enabled.py "$value"
    ;;
check)
    render_config
    require_master
    runtime "check-$$" python3 /course/ros1/hardware/scripts/readiness.py "$@"
    ;;
record)
    require_master
    if [[ ! -d "$hardware/latest-session" ]]; then echo '先运行 map。' >&2; exit 1; fi
    runtime record rosbag record --lz4 --split --size=1024 -O /hardware/latest-session/run \
        /livox/raw_lidar /livox/raw_imu /livox/imu /lio/odom /lio/cloud_world \
        /tf /tf_static /camera/depth/image_rect_raw /camera/color/image_raw \
        /camera/depth/camera_info /camera/color/camera_info /exploration/path \
        /exploration/cmd_vel /exploration/status /exploration/terrain_status \
        /exploration/map_status /aliengo/controller/status
    ;;
rviz)
    require_master
    runtime rviz rviz -d /course/ros1/hardware/real.rviz
    ;;
shell)
    require_master
    runtime "shell-$$" bash
    ;;
*)
    echo '用法：./scripts/gbplanner_real.sh {init|build|master|lidar|camera-probe|camera-stage|camera-sync|camera-onboard|time-server|map|sdk|check|record|rviz|enable|stop|shell}'
    echo '实机操作说明：docs/gbplanner_real_zh.md'
    if [[ "$action" != help ]]; then exit 2; fi
    ;;
esac
