#!/usr/bin/env python3
"""Explicit enable/stop; service rejection is a nonzero command exit."""
import sys
import rospy
from std_srvs.srv import SetBool

rospy.init_node('hardware_exploration_switch', anonymous=True)
rospy.wait_for_service('/exploration/enable', timeout=5)
reply = rospy.ServiceProxy('/exploration/enable', SetBool)(sys.argv[1] == 'true')
print(reply.message)
raise SystemExit(0 if reply.success else 1)
