#!/usr/bin/env python3
"""Real CustomMsg -> LIO; SI IMU; deskewed LIO cloud -> native sensor rays."""
from copy import deepcopy
import threading
from collections import deque

import numpy as np
import rospy
import tf2_ros
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from tf.transformations import quaternion_matrix

from sensor_core import imu_si, xyz_returns, near_returns, deskew_near


def transform_matrix(transform):
    q, t = transform.rotation, transform.translation
    matrix = quaternion_matrix((q.x, q.y, q.z, q.w))
    matrix[:3, 3] = (t.x, t.y, t.z)
    return matrix


class Adapter:
    def __init__(self):
        self.scale = float(rospy.get_param('/aliengo_hardware/imu_acceleration_scale', 9.80665))
        self.registration_blind = float(rospy.get_param('/lio/sensor/blind', 1.))
        # Embedded Super-LIO Driver1 headers and Driver2 have identical wire MD5.
        if CustomMsg._md5sum != 'e4d6829bdfe657cb6c21a746c86b21a6':
            raise ValueError('CustomMsg no longer matches pinned Super-LIO headers')
        self.scan_pub = rospy.Publisher('/livox/lidar_custom', CustomMsg, queue_size=5)
        self.imu_pub = rospy.Publisher('/livox/imu', Imu, queue_size=100)
        self.raw_pub = rospy.Publisher('/livox/raw_points', PointCloud2, queue_size=1)
        self.cloud_pub = rospy.Publisher('/livox/lidar', PointCloud2, queue_size=2)
        self.buffer = tf2_ros.Buffer(cache_time=rospy.Duration(20))
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.pending, self.lock = deque(maxlen=5), threading.Lock()
        self.near_scans = deque(maxlen=30)
        self.subscribers = [
            rospy.Subscriber('/livox/raw_lidar', CustomMsg, self.scan, queue_size=5),
            rospy.Subscriber('/livox/raw_imu', Imu, self.imu, queue_size=100),
            rospy.Subscriber('/lio/cloud_world', PointCloud2, self.registered, queue_size=2)]
        self.timer = rospy.Timer(rospy.Duration(.01), self.flush)

    def scan(self, msg):
        if msg.header.frame_id != 'mid360_link' or msg.header.stamp.to_sec() <= 0:
            rospy.logwarn_throttle(5, 'Reject raw scan: frame/stamp mismatch')
            return
        try:
            points = xyz_returns(msg)
        except ValueError as error:
            rospy.logwarn_throttle(5, str(error))
            return
        start = msg.header.stamp.to_nsec()
        near = near_returns(msg, self.registration_blind)
        end = start+max((p.offset_time for p in msg.points), default=0)
        with self.lock:
            self.near_scans.append((start, end, near))
        self.scan_pub.publish(msg)  # Original stamp/frame, timebase, tag, line, offsets.
        if self.raw_pub.get_num_connections():
            self.raw_pub.publish(point_cloud2.create_cloud_xyz32(msg.header, points))

    def imu(self, msg):
        try:
            self.imu_pub.publish(imu_si(msg, self.scale))
        except ValueError as error:
            rospy.logwarn_throttle(5, str(error))

    def registered(self, msg):
        if msg.header.frame_id == 'lio_map':
            with self.lock:
                stamp = msg.header.stamp.to_nsec()
                scans = [scan for scan in self.near_scans if scan[0] <= stamp <= scan[1]+1000]
                if not scans:
                    rospy.logwarn_throttle(5, 'No raw scan matches LIO output; waiting for paired data')
                    return
                start, unused_end, near = min(scans, key=lambda scan: abs(scan[1]-stamp))
                self.pending.append((msg, start, near))

    def flush(self, unused):
        with self.lock:
            if not self.pending:
                return
            job = self.pending[0]
            msg, start, near = job
        try:
            tf = self.buffer.lookup_transform('mid360_link', 'lio_map', msg.header.stamp,
                                              rospy.Duration(0)).transform
            reference = transform_matrix(tf)
            def pose_at_ns(stamp):
                seconds, nanoseconds = divmod(stamp, 1000000000)
                sample = self.buffer.lookup_transform('lio_map', 'mid360_link',
                    rospy.Time(seconds, nanoseconds), rospy.Duration(0)).transform
                return transform_matrix(sample)
            supplemented = deskew_near(near, start, reference, pose_at_ns)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return
        xyz = np.asarray(list(point_cloud2.read_points(msg, field_names=('x', 'y', 'z'),
                                                       skip_nans=True)), dtype=np.float32)
        if xyz.size:
            native = xyz @ reference[:3, :3].T + reference[:3, 3]
            native = np.vstack((native, supplemented))
            header = Header(stamp=deepcopy(msg.header.stamp), frame_id='mid360_link')
            self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, native))
        with self.lock:
            if self.pending and self.pending[0] is job:
                self.pending.popleft()


if __name__ == '__main__':
    rospy.init_node('livox_real_adapter')
    adapter = Adapter()
    rospy.spin()
