#!/usr/bin/env python3
"""Convert the Gazebo block laser's legacy cloud into PointCloud2 XYZ data."""
import rospy
from sensor_msgs.msg import PointCloud, PointCloud2
from sensor_msgs import point_cloud2


def convert(message):
    points = [(point.x, point.y, point.z) for point in message.points]
    return point_cloud2.create_cloud_xyz32(message.header, points)


def main():
    rospy.init_node("mid360_cloud_converter")
    publisher = rospy.Publisher("/livox/lidar", PointCloud2, queue_size=1)
    rospy.Subscriber("/mid360_sim/points_raw", PointCloud,
                     lambda message: publisher.publish(convert(message)), queue_size=1)
    rospy.spin()


if __name__ == "__main__":
    main()
