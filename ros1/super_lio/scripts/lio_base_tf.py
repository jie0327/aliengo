#!/usr/bin/env python3
"""由 LIO 的 IMU 估计位姿和固定安装外参发布 base TF。"""
from copy import deepcopy

import rospy
import tf2_ros
from geometry_msgs.msg import Quaternion, TransformStamped, Vector3
from nav_msgs.msg import Odometry
from tf.transformations import (concatenate_matrices, inverse_matrix,
                                quaternion_from_matrix, quaternion_matrix,
                                translation_matrix)


def odom_to_base_transform(odom, base_to_imu):
    """T_LB = T_LI · inverse(T_BI)，不修改输入消息。"""
    position, rotation = odom.pose.pose.position, odom.pose.pose.orientation
    lidar_map_to_imu = concatenate_matrices(
        translation_matrix((position.x, position.y, position.z)),
        quaternion_matrix((rotation.x, rotation.y, rotation.z, rotation.w)))
    position = base_to_imu.transform.translation
    rotation = base_to_imu.transform.rotation
    mount = concatenate_matrices(
        translation_matrix((position.x, position.y, position.z)),
        quaternion_matrix((rotation.x, rotation.y, rotation.z, rotation.w)))
    lidar_map_to_base = concatenate_matrices(lidar_map_to_imu, inverse_matrix(mount))

    transform = TransformStamped()
    transform.header = deepcopy(odom.header)
    transform.child_frame_id = 'base'
    transform.transform.translation = Vector3(*lidar_map_to_base[:3, 3])
    transform.transform.rotation = Quaternion(*quaternion_from_matrix(lidar_map_to_base))
    return transform


def main():
    rospy.init_node('lio_base_tf')
    # 只查询一次现有静态安装链，并保留监听器引用。
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    base_to_imu = tf_buffer.lookup_transform(
        'base', 'mid360_imu_link', rospy.Time(0), rospy.Duration(10))
    broadcaster = tf2_ros.TransformBroadcaster()
    subscriber = rospy.Subscriber(
        '/lio/odom', Odometry,
        lambda odom: broadcaster.sendTransform(odom_to_base_transform(odom, base_to_imu)),
        queue_size=10)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
