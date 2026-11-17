#!/usr/bin/env bash
set -euo pipefail
exec docker exec -i aliengo-ros1-sim bash -c '
source /opt/ros/noetic/setup.bash
python3 - <<"PY"
import rospy
from std_srvs.srv import SetBool
rospy.init_node("enable_aliengo_exploration", anonymous=True)
rospy.wait_for_service("/exploration/enable", timeout=5)
reply=rospy.ServiceProxy("/exploration/enable", SetBool)(True)
print(reply.message)
raise SystemExit(0 if reply.success else 1)
PY'
