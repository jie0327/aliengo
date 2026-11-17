#!/usr/bin/env bash
# Source from GUI launchers and pass "${gpu_args[@]}" to docker run.
# After sourcing ROS inside the container, preserve its library paths with:
# export LD_LIBRARY_PATH="${ALIENGO_NVIDIA_LIBRARY_PATH:+$ALIENGO_NVIDIA_LIBRARY_PATH:}${LD_LIBRARY_PATH:-}"
configure_ros1_gpu() {
    gpu_args=()
    local version cache soname path device group_id
    version="$(nvidia-smi --id=0 --query-gpu=driver_version --format=csv,noheader)" || {
        echo '无法读取 NVIDIA 驱动版本，未启用 GUI。' >&2; return 1;
    }
    cache="$(ldconfig -p)" || return 1
    for soname in libGLX_nvidia.so.0 libnvidia-ml.so.1 \
        "libnvidia-glcore.so.$version" "libnvidia-glsi.so.$version" \
        "libnvidia-tls.so.$version" "libnvidia-gpucomp.so.$version" \
        "libnvidia-glvkspirv.so.$version"; do
        path="$(awk -v name="$soname" '$1 == name && /x86-64/ {print $NF; exit}' <<< "$cache")"
        if [[ ! -r "$path" ]]; then
            printf '缺少可读的 NVIDIA 驱动库：%s。\n' "$soname" >&2
            gpu_args=(); return 1
        fi
        path="$(readlink -f -- "$path")"
        gpu_args+=( --mount "type=bind,source=$path,target=/usr/local/nvidia/lib64/$soname,readonly" )
    done
    gpu_args+=( --mount "type=bind,source=$(command -v nvidia-smi),target=/usr/bin/nvidia-smi,readonly" )
    for device in /dev/nvidia0 /dev/nvidiactl /dev/nvidia-modeset /dev/nvidia-uvm /dev/dri; do
        gpu_args+=( --device "$device" )
    done
    while read -r group_id; do
        gpu_args+=( --group-add "$group_id" )
    done < <(stat -c %g /dev/dri/card* /dev/dri/renderD* | sort -u)
    gpu_args+=(
        -e ALIENGO_NVIDIA_LIBRARY_PATH=/usr/local/nvidia/lib64
        -e __NV_PRIME_RENDER_OFFLOAD=1
        -e __GLX_VENDOR_LIBRARY_NAME=nvidia
    )
}
