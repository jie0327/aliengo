#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$root/scripts/compute_guard.sh"
acquire_compute_guard "$(compute_guard_path)"
docker run --rm --init --network none --cpus=2 --memory=2g --memory-swap=2g \
  --user "$(id -u):$(id -g)" -e HOME=/tmp/aliengo-gbp-tests -e ROS_LOG_DIR=/tmp/aliengo-gbp-tests/log -e ROS_HOME=/tmp/aliengo-gbp-tests/.ros \
  --mount "type=bind,source=$root,target=/course,readonly" \
  --mount "type=bind,source=$root/.gbplanner,target=/gbp,readonly" \
  --entrypoint bash -w /course aliengo-learning:exploration -c '
set -e
mkdir -p "$HOME"
source /opt/gbp_ws/devel/setup.bash
source /gbp/devel/setup.bash --extend
export ROS_PACKAGE_PATH="/course/ros1:$ROS_PACKAGE_PATH"
g++ -std=c++17 /course/tests/gbplanner/backend_core.cpp -I/gbp/src/gbplanner/gbplanner/include -I/gbp/src/gbp_deps/voxblox/voxblox_ros/include $(pkg-config --cflags grid_map_core) $(pkg-config --libs grid_map_core) -o /tmp/backend_core
/tmp/backend_core
g++ -std=c++17 /course/tests/gbplanner/lidar_gain_fov.cpp $(pkg-config --cflags planner_common) $(pkg-config --libs planner_common) -o /tmp/lidar_gain_fov
/tmp/lidar_gain_fov
g++ -std=c++17 /course/tests/gbplanner/raycaster_axes.cpp $(pkg-config --cflags voxblox) $(pkg-config --libs voxblox) -o /tmp/raycaster_axes
/tmp/raycaster_axes
bash /course/tests/gbplanner/turn_clearance_ros.sh
python3 /course/tests/gbplanner/ros_backend_check.py
python3 -m unittest discover -s /course/tests -p test_exploration_coordinator.py
'
