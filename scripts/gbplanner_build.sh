#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root/scripts/compute_guard.sh"
acquire_compute_guard "$(compute_guard_path)"
dependency=aliengo-exploration:simulation
image=aliengo-learning:exploration
workspace="$root/.gbplanner"
expected=sha256:1874ecd35a34f6dd47d32becda8fb85b2c67f273a400413d95dfd297c8e1c5d8
actual="$(docker image inspect "$dependency" --format '{{.Id}}')"
if [[ "$actual" != "$expected" ]]; then
  echo "GBPlanner dependency image mismatch: $actual, expected $expected" >&2; exit 1
fi
mkdir -p "$workspace/src/gbp_deps" "$workspace/provenance"
# Reconstruct from pinned originals, preserving unchanged source timestamps so
# repeat builds can reuse compiled objects. The generated overlay is disposable.
stage="$(mktemp -d "$workspace/.sources.XXXXXX")"
trap 'rm -rf -- "$stage"' EXIT
cp -a "$root/references/gbplanner" "$stage/gbplanner"
cp -a "$root/references/gbp_deps/voxblox" "$stage/voxblox"
patch --batch -d "$stage/gbplanner" -p1 < "$root/ros1/patches/gbplanner_backend.patch"
patch --batch -d "$stage/gbplanner" -p1 < "$root/ros1/patches/gbplanner_turn_clearance.patch"
patch --batch -d "$stage/voxblox" -p1 < "$root/ros1/patches/gbplanner_voxblox_native.patch"
patch --batch -d "$stage/voxblox" -p1 < "$root/ros1/patches/gbplanner_voxblox_raycast.patch"
if ! diff -qr "$stage/gbplanner" "$workspace/src/gbplanner" >/dev/null 2>&1; then
  rm -rf "$workspace/src/gbplanner"
  mv "$stage/gbplanner" "$workspace/src/gbplanner"
fi
if ! diff -qr "$stage/voxblox" "$workspace/src/gbp_deps/voxblox" >/dev/null 2>&1; then
  rm -rf "$workspace/src/gbp_deps/voxblox"
  mv "$stage/voxblox" "$workspace/src/gbp_deps/voxblox"
fi
rm -rf -- "$stage"
trap - EXIT
cp "$root/references/gbplanner-sources.lock.json" "$workspace/provenance/sources.lock.json"
printf '%s\n' "$actual" > "$workspace/provenance/dependency-image-id"
sha256sum "$root"/ros1/patches/gbplanner*.patch > "$workspace/provenance/patches.sha256"
docker build --network none -f "$root/docker/Dockerfile.exploration" -t "$image" "$root"
docker image inspect "$image" --format '{{.Id}}' > "$workspace/provenance/runtime-image-id"
docker run --rm --init --network none --cpus=2 --memory=5g --memory-swap=5g \
  --user "$(id -u):$(id -g)" -e HOME=/tmp/aliengo-gbp \
  --mount "type=bind,source=$workspace,target=/gbp" \
  --mount "type=bind,source=$root/tests/gbplanner,target=/tests,readonly" \
  --entrypoint bash -w /gbp "$image" -c '
set -e
mkdir -p "$HOME"
source /opt/gbp_ws/devel/setup.bash
catkin config --extend /opt/gbp_ws/devel --cmake-args -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=ON
catkin build voxblox voxblox_ros planner_msgs planner_common map_manager gbplanner --no-deps --no-status --summarize -j1 -p1
g++ -std=c++17 /tests/backend_core.cpp -Isrc/gbplanner/gbplanner/include -Isrc/gbp_deps/voxblox/voxblox_ros/include -I/opt/ros/noetic/include -I/usr/include/eigen3 -L/opt/ros/noetic/lib -lgrid_map_core -o /tmp/backend_core
LD_LIBRARY_PATH=/opt/ros/noetic/lib /tmp/backend_core
source /gbp/devel/setup.bash
python3 -c "from planner_msgs.srv import planner_srv; print(planner_srv._type)"
'
echo "GBPlanner ready: $image, source /opt/gbp_ws/devel/setup.bash then /gbp/devel/setup.bash --extend"
