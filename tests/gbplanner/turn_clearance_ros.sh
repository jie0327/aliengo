#!/usr/bin/env bash
# Run inside a sourced Noetic/GBPlanner environment.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
export ROS_MASTER_URI=http://127.0.0.1:11323 ROS_IP=127.0.0.1 ROS_HOSTNAME=localhost
g++ -std=c++17 "$root/tests/gbplanner/turn_clearance.cpp" $(pkg-config --cflags --libs map_manager) -o /tmp/turn_clearance
g++ -std=c++17 -fno-access-control -Wl,--export-dynamic "$root/tests/gbplanner/path_output_gate.cpp" $(pkg-config --cflags --libs gbplanner) -o /tmp/path_output_gate
roscore -p 11323 > /tmp/turn-roscore.log 2>&1 &
master_pid=$!
trap 'kill "$master_pid" 2>/dev/null || true; wait "$master_pid" 2>/dev/null || true' EXIT
python3 - <<'PY'
import time, xmlrpc.client
master = xmlrpc.client.ServerProxy('http://127.0.0.1:11323')
deadline = time.monotonic()+15
while time.monotonic() < deadline:
    try:
        if master.getUri('/test')[0] == 1:
            break
    except OSError:
        pass
    time.sleep(.05)
else:
    raise RuntimeError('ROS master startup failed')
PY
timeout 40 /tmp/turn_clearance
timeout 40 /tmp/path_output_gate
