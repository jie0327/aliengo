#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
image="${ALIENGO_ROS1_IMAGE:-aliengo-learning:noetic}"
lio_workspace="${ALIENGO_LIO_WORKSPACE:-}"
fast_livo_workspace="${ALIENGO_FAST_LIVO_WORKSPACE:-}"
gbp_workspace="${ALIENGO_GBP_WORKSPACE:-}"
lio_args=()
fast_livo_args=()
gbp_args=()
container=aliengo-ros1-sim
gui=true
render_mode=gpu
rviz=false
d435_enabled=true
world=depot
mount_args=()
while [[ $# -gt 0 ]]; do
    arg="$1"
    case "$arg" in
        --headless) gui=false ;;
        --software-rendering) render_mode=software ;;
        --rviz) rviz=true ;;
        --world)
            if [[ $# -lt 2 ]]; then echo '--world 后面需要 depot、earth 或 city。' >&2; exit 2; fi
            world="$2"; shift ;;
        sim_ground_truth_tf:=true|sim_ground_truth_tf:=false) mount_args+=("$arg") ;;
        d435_enabled:=true|d435_enabled:=false)
            d435_enabled="${arg#d435_enabled:=}"
            mount_args+=("$arg") ;;
        d435_enabled:=*) echo 'd435_enabled 必须是 true 或 false。' >&2; exit 2 ;;
        d435_color_width:=*|d435_color_height:=*|d435_depth_width:=*|d435_depth_height:=*)
            value="${arg#*:=}"
            if [[ ! "$value" =~ ^[1-9][0-9]*$ ]] || (( ${#value} > 4 )) || (( value < 2 || value > 4096 )); then
                echo 'D435 图像宽高必须是 2 到 4096 的整数。' >&2; exit 2
            fi
            mount_args+=("$arg") ;;
        d435_rate:=*|d435_depth_hfov:=*|d435_color_hfov:=*|d435_min_depth:=*|d435_max_depth:=*|d435_stereo_baseline:=*)
            value="${arg#*:=}"
            if [[ ! "$value" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ || ! "$value" =~ [1-9] ]]; then
                echo 'D435 频率、视场、距离和双目基线必须是正数。' >&2; exit 2
            fi
            mount_args+=("$arg") ;;
        d435_x:=*|d435_y:=*|d435_z:=*|d435_roll:=*|d435_pitch:=*|d435_yaw:=*|d435_color_offset_y:=*)
            value="${arg#*:=}"
            if [[ ! "$value" =~ ^[+-]?([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]]; then
                echo 'D435 位姿和相机间偏移必须是有限十进制数。' >&2; exit 2
            fi
            mount_args+=("$arg") ;;
        mid360_downsample:=*)
            value="${arg#mid360_downsample:=}"
            if [[ ! "$value" =~ ^[1-9][0-9]*$ ]] || (( ${#value} > 5 )) || (( value > 24000 )); then
                echo 'mid360_downsample 必须是 1 到 24000 的整数。' >&2; exit 2
            fi
            mount_args+=("$arg") ;;
        mid360_x:=*|mid360_y:=*|mid360_z:=*|mid360_roll:=*|mid360_pitch:=*|mid360_yaw:=*) mount_args+=("$arg") ;;
        *) echo '用法：./scripts/ros1_sim.sh [--world depot|earth|city] [--rviz] [--headless|--software-rendering] [mid360_*:=值] [d435_*:=值]；相机参数见 docs/d435_sim_zh.md。' >&2; exit 2 ;;
    esac
    shift
done
model_path=/usr/share/gazebo-11/models
resource_path=/usr/share/gazebo-11
case "$world" in
    earth) ;;
    depot)
        if [[ ! -f "$root/ros1/worlds/depot.world" || ! -f "$root/ros1/worlds/models/Depot/meshes/Depot.dae" ]]; then
            echo '请先执行 python3 scripts/ros1_depot_setup.py 准备仓库资源。' >&2; exit 1
        fi
        mount_args+=( world_file:=/course/ros1/worlds/depot.world spawn_x:=-5 )
        model_path="/course/ros1/worlds/models:$model_path" ;;
    city)
        if [[ ! -f "$root/ros1/worlds/city.world" ]]; then
            echo '请先执行 python3 scripts/ros1_city_setup.py 准备城市资源。' >&2; exit 1
        fi
        mount_args+=( world_file:=/course/ros1/worlds/city.world
            spawn_x:=-45 spawn_y:=-25 spawn_z:=5.61 spawn_yaw:=1.5707963267948966 )
        model_path="/course/ros1/worlds/models:/course/references/citysim/models:/course/references/gazebo_city_models:$model_path"
        resource_path="/course/references/citysim:$resource_path" ;;
    *) echo 'world 请选择 depot、earth 或 city。' >&2; exit 2 ;;
esac
if [[ "$rviz" == true && -z "${ALIENGO_ROS1_IMAGE:-}" ]]; then
    image=aliengo-learning:noetic-viz
fi
if [[ "$rviz" == true || -n "${ALIENGO_ROS1_IMAGE:-}" ]]; then
    if ! docker image inspect "$image" >/dev/null 2>&1; then
        if [[ -n "${ALIENGO_ROS1_IMAGE:-}" ]]; then
            echo "缺少 ROS 1 镜像 $image；请先构建对应镜像。" >&2
        else
            echo '请先执行 ./scripts/ros1_viz_build.sh 准备 RViz 镜像。' >&2
        fi
        exit 1
    fi
fi
if [[ -n "$fast_livo_workspace" ]]; then
    if [[ -n "$lio_workspace" || -n "$gbp_workspace" ]]; then
        echo 'FAST-LIVO2 使用独立仿真入口，不能同时挂载 Super-LIO 或 GBPlanner。' >&2; exit 1
    fi
    if [[ ! -f "$fast_livo_workspace/devel/setup.bash" ]]; then
        echo 'FAST-LIVO2 工作区尚未构建，请先运行 ./scripts/fast_livo_build.sh。' >&2; exit 1
    fi
    if [[ "$d435_enabled" != true ]]; then
        echo 'FAST-LIVO2 三传感器融合需要启用 D435。' >&2; exit 1
    fi
    fast_livo_workspace="$(realpath -- "$fast_livo_workspace")"
    fast_livo_args+=( --mount "type=bind,source=$fast_livo_workspace,target=/fast_livo" -e ALIENGO_FAST_LIVO_ENABLED=1 )
fi
if [[ -n "$lio_workspace" ]]; then
    if [[ ! -f "$lio_workspace/devel/setup.bash" ]]; then
        echo "Super LIO 工作区尚未构建：$lio_workspace/devel/setup.bash。请先完成 Super LIO 构建。" >&2
        exit 1
    fi
    lio_workspace="$(realpath -- "$lio_workspace")"
    lio_args+=( --mount "type=bind,source=$lio_workspace,target=/lio" -e ALIENGO_LIO_ENABLED=1 )
fi
if [[ -n "$gbp_workspace" ]]; then
    if [[ ! -f "$gbp_workspace/devel/setup.bash" ]]; then
        # Catkin linked devel uses absolute /gbp links, resolved after mounting.
        gbp_setup_link="$(readlink -- "$gbp_workspace/devel/setup.bash" 2>/dev/null || true)"
        if [[ "$gbp_setup_link" != /gbp/* || ! -f "$gbp_workspace/${gbp_setup_link#/gbp/}" ]]; then
            echo 'GBPlanner 工作区尚未构建，请运行 ./scripts/gbplanner_build.sh。' >&2; exit 1
        fi
    fi
    gbp_workspace="$(realpath -- "$gbp_workspace")"
    gbp_args+=( --mount "type=bind,source=$gbp_workspace,target=/gbp" -e ALIENGO_GBP_ENABLED=1 )
fi
if [[ ! -f "$root/.ros1/devel/setup.bash" ]]; then
    echo 'ROS 1 工作区尚未构建，请先运行 ./scripts/ros1_build.sh。' >&2; exit 1
fi
if [[ ! -f "$root/.ros1/devel/lib/liblivox_laser_simulation.so" || ! -f "$root/.ros1/src/livox_laser_simulation/scan_mode/mid360.csv" ]]; then
    echo 'MID-360 仿真插件或扫描数据缺失，请运行 ./scripts/ros1_build.sh 更新工作区。' >&2; exit 1
fi
if [[ "$d435_enabled" == true && ! -f "$root/.ros1/devel/lib/librealsense_gazebo_plugin.so" ]]; then
    echo 'D435 仿真插件尚未构建，请先运行 ./scripts/ros1_build.sh。' >&2; exit 1
fi
source "$root/scripts/compute_guard.sh"
acquire_compute_guard "$(compute_guard_path)"
if docker container inspect "$container" >/dev/null 2>&1; then
    echo "容器 $container 已存在；请先结束已有仿真。" >&2; exit 1
fi
display_args=()
gpu_args=()
if [[ "$gui" == true || "$rviz" == true || "$d435_enabled" == true ]]; then
    if [[ -z "${DISPLAY:-}" || ! -d /tmp/.X11-unix ]]; then
        if [[ "$rviz" == true ]]; then
            echo 'RViz 需要可用的 X11 显示环境。' >&2; exit 1
        fi
        echo '未检测到宿主 X11，使用无界面仿真；启用相机时将使用虚拟显示。' >&2; gui=false
    else
        display_args+=( -e "DISPLAY=$DISPLAY" -e QT_X11_NO_MITSHM=1 \
            --mount type=bind,source=/tmp/.X11-unix,target=/tmp/.X11-unix,readonly )
        auth_file="${XAUTHORITY:-$HOME/.Xauthority}"
        if [[ -r "$auth_file" ]]; then
            display_args+=( -e XAUTHORITY=/tmp/aliengo.xauthority \
                --mount "type=bind,source=$auth_file,target=/tmp/aliengo.xauthority,readonly" )
        fi
        if [[ -d /usr/share/fonts/opentype/noto ]]; then
            display_args+=( --mount type=bind,source=/usr/share/fonts/opentype/noto,target=/usr/share/fonts/opentype/noto,readonly )
        fi
        if [[ "$render_mode" == gpu ]]; then
            source "$root/scripts/ros1_graphics.sh"
            configure_ros1_gpu
            echo 'Gazebo 使用 NVIDIA GPU 渲染。'
        else
            display_args+=( -e LIBGL_ALWAYS_SOFTWARE=1 -e LP_NUM_THREADS=2 )
        fi
    fi
fi
echo 'ROS 1 AlienGo 仿真启动中；另开终端运行 ./scripts/ros1_keyboard.sh。Ctrl+C 结束。'
docker run --rm --init --network none --name "$container" --user "$(id -u):$(id -g)" \
    --memory=6g --memory-swap=6g --cpus=6 --shm-size=512m \
    -e OMP_NUM_THREADS=1 -e OPENBLAS_NUM_THREADS=1 \
    -e "ALIENGO_D435_ENABLED=$d435_enabled" \
    -e HOME=/tmp/aliengo-ros1 -e ROS_HOME=/tmp/aliengo-ros1/.ros \
    -e ROS_LOG_DIR=/tmp/aliengo-ros1/.ros/log \
    -e ROS_HOSTNAME=localhost -e ROS_IP=127.0.0.1 -e GAZEBO_MODEL_DATABASE_URI= \
    -e "GAZEBO_MODEL_PATH=$model_path" -e "GAZEBO_RESOURCE_PATH=$resource_path" -e GAZEBO_PLUGIN_PATH=/work/devel/lib \
    --mount "type=bind,source=$root/.ros1,target=/work" \
    --mount "type=bind,source=$root,target=/course,readonly" "${lio_args[@]}" "${fast_livo_args[@]}" "${gbp_args[@]}" "${display_args[@]}" "${gpu_args[@]}" \
    --entrypoint bash "$image" -c '
set -e
mkdir -p "$HOME"
source /opt/ros/noetic/setup.bash
if [[ "${ALIENGO_GBP_ENABLED:-}" == 1 ]]; then
    source /opt/gbp_ws/devel/setup.bash --extend
    source /gbp/devel/setup.bash --extend
fi
# Current robot and LIO must shadow the old dependency overlays.
source /work/devel/setup.bash --extend
if [[ "${ALIENGO_LIO_ENABLED:-}" == 1 ]]; then source /lio/devel/setup.bash --extend; fi
if [[ "${ALIENGO_FAST_LIVO_ENABLED:-}" == 1 ]]; then source /fast_livo/devel/setup.bash --extend; fi
export LD_LIBRARY_PATH="${ALIENGO_NVIDIA_LIBRARY_PATH:+$ALIENGO_NVIDIA_LIBRARY_PATH:}${LD_LIBRARY_PATH:-}"
export ROS_PACKAGE_PATH="/course/ros1:$ROS_PACKAGE_PATH"
launch_command=(roslaunch /course/ros1/mid360/gazebo.launch rname:=aliengo "gui:=$1" "rviz:=$2" paused:=false "${@:3}")
if [[ "$ALIENGO_D435_ENABLED" == true && -z "${DISPLAY:-}" ]]; then
    if ! command -v xvfb-run >/dev/null || ! command -v xauth >/dev/null; then
        echo "D435 无显示渲染需要 xvfb/xauth。请运行 ./scripts/ros1_build_env.sh 重建基础镜像（自定义/RViz/Super-LIO 镜像也需重建），或提供 X11 DISPLAY；仅运行雷达可使用 d435_enabled:=false。" >&2
        exit 1
    fi
    export LIBGL_ALWAYS_SOFTWARE=1 LP_NUM_THREADS=2
    exec xvfb-run -a -s "-screen 0 1280x720x24 -nolisten tcp" "${launch_command[@]}"
fi
exec "${launch_command[@]}"' bash "$gui" "$rviz" "${mount_args[@]}"
