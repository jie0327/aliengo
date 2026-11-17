#!/usr/bin/env python3
"""Only sensor mount TFs. Camera-internal TFs belong to the RealSense driver."""
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3
from tf.transformations import quaternion_from_euler


def main():
    rospy.init_node('hardware_sensor_tf')
    transforms = []
    for child, key in (('mid360_link', 'base_to_mid360'), ('d435_link', 'base_to_d435')):
        mount = rospy.get_param('/aliengo_hardware/'+key)
        message = TransformStamped()
        message.header.stamp, message.header.frame_id = rospy.Time.now(), 'base'
        message.child_frame_id = child
        message.transform.translation = Vector3(*mount['xyz'])
        message.transform.rotation = Quaternion(*quaternion_from_euler(*mount['rpy']))
        transforms.append(message)
    imu = TransformStamped()
    imu.header.stamp, imu.header.frame_id = rospy.Time.now(), 'mid360_link'
    imu.child_frame_id = 'mid360_imu_link'
    imu.transform.translation = Vector3(.011, .02329, -.04412)
    imu.transform.rotation.w = 1.
    transforms.append(imu)
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform(transforms)
    rospy.spin()


if __name__ == '__main__':
    main()
