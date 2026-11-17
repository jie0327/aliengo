#!/usr/bin/env bash
# Run on the robot's perception computer, not on the laptop.
set -euo pipefail
kit="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
camera_master_uri="${ROS_MASTER_URI:-http://192.168.123.100:11311}"
camera_ros_ip="${ROS_IP:-192.168.123.12}"
export ROS_MASTER_URI="$camera_master_uri" ROS_IP="$camera_ros_ip"
ros_setup=''
for distro in noetic melodic kinetic; do
    if [[ -f "/opt/ros/$distro/setup.bash" ]]; then ros_setup="/opt/ros/$distro/setup.bash"; break; fi
done
if [[ -z "$ros_setup" ]]; then echo '机载电脑没有找到 ROS1；先确认原厂相机软件环境，勿在 TX2 上直接安装笔记本的 x86 Noetic 镜像。' >&2; exit 1; fi
set +u
source "$ros_setup"
set -u
export ROS_MASTER_URI="$camera_master_uri" ROS_IP="$camera_ros_ip"
unset ROS_HOSTNAME
if [[ "${1:-}" == --build ]]; then
    version="$(pkg-config --modversion realsense2 2>/dev/null || true)"
    if [[ -z "$version" ]]; then echo '缺少机载 librealsense2 开发包；请按 TX2 的 JetPack/相机固件配置准备。' >&2; exit 1; fi
    if ! dpkg --compare-versions "$version" ge 2.50.0; then
        echo "ROS wrapper 2.3.2 需要 librealsense >=2.50.0；机上是 $version，先核对配套版本。" >&2; exit 1
    fi
    missing=()
    for package in catkin roscpp cv_bridge image_transport nodelet tf ddynamic_reconfigure diagnostic_updater; do
        if ! rospack find "$package" >/dev/null 2>&1; then missing+=("$package"); fi
    done
    if (( ${#missing[@]} )); then
        echo "缺少 ROS 依赖：${missing[*]}。按机载 ROS 发行版准备后再构建；本归档只有相机 wrapper 源码。" >&2; exit 1
    fi
    mkdir -p "$kit/ws/src"
    tar -xzf "$kit/realsense-ros-2.3.2.tar.gz" -C "$kit/ws/src"
    cd "$kit/ws"
    catkin_make -j2 -DCMAKE_BUILD_TYPE=Release
    echo '相机 ROS 包已构建。再次不带 --build 运行以启动。'
    exit
fi
if [[ -f "$kit/ws/devel/setup.bash" ]]; then
    set +u
    source "$kit/ws/devel/setup.bash"
    set -u
fi
if ! rospack find realsense2_camera >/dev/null 2>&1; then
    echo '没有 realsense2_camera。kit 中已备 ROS 2.3.2 源码，已有兼容 ROS1/librealsense>=2.50 时可执行 onboard_camera.sh --build。' >&2; exit 1
fi
echo "机载相机 -> $ROS_MASTER_URI，ROS_IP=$ROS_IP；需先释放占用相机的原厂视觉进程，并同步两机时钟。"
exec roslaunch "$kit/camera.launch" "serial:=${1:-}"
