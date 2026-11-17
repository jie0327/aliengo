#!/usr/bin/env python3
"""Discard Gazebo's ground-truth orientation from a simulated raw IMU."""
import rospy
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu


def convert(message):
    message.orientation = Quaternion(0, 0, 0, 1)
    message.orientation_covariance = [-1.0] + [0.0] * 8
    return message


def main():
    rospy.init_node("mid360_imu_converter")
    publisher = rospy.Publisher("/livox/imu", Imu, queue_size=10)
    rospy.Subscriber("/mid360_sim/imu_raw", Imu,
                     lambda message: publisher.publish(convert(message)), queue_size=10)
    rospy.spin()


if __name__ == "__main__":
    main()
