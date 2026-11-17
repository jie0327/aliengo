#!/usr/bin/env python3
"""Native-ray cloud frontend and persistent, measured Depot ground map.

No TF is broadcast here. Super-LIO and the existing lio_base_tf node remain
pose authorities. Work runs on a wall-clock worker so unavailable TF and a
paused simulator cannot cause unbounded callback queues.
"""
from collections import deque
from copy import deepcopy
import json
import math
import threading
import time

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import Point, Quaternion, Vector3
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header, String
from tf.transformations import quaternion_from_matrix, quaternion_matrix

from grid_map_codec import to_grid_message
from mapping_core import (GyroBuffer, ObservedTerrain, SOURCES, TransformQueue, filter_native_points,
                          imu_to_base_pose, imu_to_base_twist, transform_points, unpack_xyz)


def pose_matrix(position, rotation):
    xyz = np.array([position.x, position.y, position.z], dtype=float)
    quaternion = np.array([rotation.x, rotation.y, rotation.z, rotation.w], dtype=float)
    if not np.all(np.isfinite(xyz)) or not np.all(np.isfinite(quaternion)) or np.linalg.norm(quaternion) < .5:
        raise ValueError('nonfinite or invalid pose')
    result = quaternion_matrix(quaternion)
    result[:3, 3] = xyz
    return result


def transform_matrix(message):
    return pose_matrix(message.transform.translation, message.transform.rotation)


def xyzyaw(matrix):
    return (*map(float, matrix[:3, 3]), float(math.atan2(matrix[1, 0], matrix[0, 0])))


def prepare_cloud(message, base_to_sensor, map_to_sensor, min_range, max_range, voxel):
    """Returns native-frame/stamp cloud and separate map-frame terrain points."""
    fields = [(f.name, f.offset, f.datatype, f.count) for f in message.fields]
    points = unpack_xyz(message.data, message.width, message.height,
                        message.point_step, message.row_step, fields, message.is_bigendian)
    native = filter_native_points(points, base_to_sensor, min_range, max_range, voxel)
    if not len(native):
        raise ValueError('cloud contains no usable surface points')
    cloud = point_cloud2.create_cloud_xyz32(deepcopy(message.header), native.astype(np.float32))
    return cloud, transform_points(native, map_to_sensor)


def _skew(vector):
    x, y, z = vector
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def buffer_imu(samples, message):
    """Accept only a finite gyro from the configured physical IMU frame."""
    angular = message.angular_velocity
    return samples.add(message.header.stamp.to_sec(), message.header.frame_id,
                       (angular.x, angular.y, angular.z),
                       np.asarray(message.angular_velocity_covariance).reshape(3, 3))


def base_odometry(message, base_to_imu, gyro=None):
    """Convert pose and gyro-assisted twist; missing gyro makes twist untrusted.

    Super-LIO's /lio/odom omits angular velocity. A close acquisition-time IMU
    sample supplies it. Without one, retain usable pose and assign high twist
    uncertainty; zero angular placeholders must not be read as measured rest.
    """
    output = deepcopy(message)
    map_to_imu = pose_matrix(message.pose.pose.position, message.pose.pose.orientation)
    map_to_base = imu_to_base_pose(map_to_imu, base_to_imu)
    output.pose.pose.position = Point(*map_to_base[:3, 3])
    output.pose.pose.orientation = Quaternion(*quaternion_from_matrix(map_to_base))
    output.child_frame_id = 'base'
    twist = message.twist.twist
    angular_imu = np.zeros(3) if gyro is None else gyro[0]
    linear, angular = imu_to_base_twist(
        (twist.linear.x, twist.linear.y, twist.linear.z),
        angular_imu, base_to_imu)
    if not np.all(np.isfinite(np.r_[linear, angular])):
        raise ValueError('nonfinite IMU twist')
    output.twist.twist.linear = Vector3(*linear)
    output.twist.twist.angular = Vector3(*angular)
    # Both covariance Jacobians include the displaced origin, not just axes.
    rotation, offset = base_to_imu[:3, :3], base_to_imu[:3, 3]
    jacobian = np.zeros((6, 6))
    jacobian[:3, :3] = jacobian[3:, 3:] = rotation
    jacobian[:3, 3:] = _skew(offset) @ rotation
    covariance = np.asarray(message.twist.covariance).reshape(6, 6)
    if gyro is None:
        output.twist.covariance = (np.eye(6)*1.e6).reshape(-1).tolist()
    else:
        covariance = covariance.copy()
        covariance[:3, 3:] = covariance[3:, :3] = 0.
        covariance[3:, 3:] = gyro[1]
        output.twist.covariance = (jacobian @ covariance @ jacobian.T).reshape(-1).tolist()
    jacobian = np.eye(6)
    jacobian[:3, 3:] = _skew(map_to_base[:3, :3] @ offset)
    covariance = np.asarray(message.pose.covariance).reshape(6, 6)
    output.pose.covariance = (jacobian @ covariance @ jacobian.T).reshape(-1).tolist()
    return output, xyzyaw(map_to_base)


class MappingNode:
    def __init__(self):
        self.frame = rospy.get_param('~frame_id', 'lio_map')
        if self.frame != 'lio_map':
            raise ValueError('this fixed-world Depot mapper requires frame_id=lio_map')
        self.base_frame = 'base'
        self.imu_frame = rospy.get_param('~imu_frame', 'mid360_imu_link')
        self.frames = {'lidar': rospy.get_param('~lidar_frame', 'mid360_link'),
                       'camera': rospy.get_param('~camera_frame', 'd435_depth_optical_frame')}
        self.max_range = {'lidar': float(rospy.get_param('~lidar_max_range', 20.)),
                          'camera': float(rospy.get_param('~camera_max_range', 8.))}
        self.min_range = {'lidar': .2, 'camera': .2}
        self.voxel = float(rospy.get_param('~voxel_size', .04))
        self.period = {'lidar': 1./float(rospy.get_param('~lidar_hz', 10.)),
                       'camera': 1./float(rospy.get_param('~camera_hz', 5.))}
        self.queue = TransformQueue(int(rospy.get_param('~queue_size', 8)),
                                    float(rospy.get_param('~tf_wait_wall', 1.)))
        self.lock = threading.RLock()
        self.gyros = GyroBuffer(self.imu_frame, max_age=float(rospy.get_param('~gyro_max_age', .025)))
        self.odom_queue = deque(maxlen=20)
        self.last_selected = {source: 0. for source in SOURCES}
        self.last_base_stamp = 0.
        self.base_pose = None
        self.base_twist_valid = False
        self.point_counts = {source: 0 for source in SOURCES}
        self.terrain = ObservedTerrain(
            resolution=float(rospy.get_param('~resolution', .1)),
            length=float(rospy.get_param('~map_length', 16.)),
            nominal_base_height=float(rospy.get_param('~nominal_base_height', .43)))
        self.tf = tf2_ros.Buffer(cache_time=rospy.Duration(20.))
        self.listener = tf2_ros.TransformListener(self.tf)
        self.cloud_pub = rospy.Publisher('/exploration/cloud', PointCloud2, queue_size=8)
        self.grid_pub = rospy.Publisher('/exploration/elevation_map', GridMap, queue_size=1)
        self.status_pub = rospy.Publisher('/exploration/terrain_status', String, queue_size=1)
        self.base_pub = rospy.Publisher('/exploration/base_odom', Odometry, queue_size=10)
        self.points_pub = rospy.Publisher('/exploration/terrain_points', PointCloud2, queue_size=1)
        self.subscribers = [
            rospy.Subscriber('/livox/lidar', PointCloud2, self._cloud, callback_args='lidar',
                             queue_size=2, buff_size=4*1024*1024),
            rospy.Subscriber('/camera/depth/color/points', PointCloud2, self._cloud, callback_args='camera',
                             queue_size=2, buff_size=32*1024*1024),
            rospy.Subscriber('/lio/odom', Odometry, self._odom, queue_size=20),
            rospy.Subscriber('/livox/imu', Imu, self._imu, queue_size=100)]
        self.stop = threading.Event()
        self.worker = threading.Thread(target=self._run, name='native-terrain-worker', daemon=True)
        rospy.on_shutdown(self.stop.set)
        self.worker.start()

    def _cloud(self, message, source):
        stamp = message.header.stamp.to_sec()
        if message.header.frame_id.lstrip('/') != self.frames[source]:
            rospy.logwarn_throttle(5., 'Rejecting %s cloud frame %s', source, message.header.frame_id)
            return
        with self.lock:
            if stamp <= 0 or stamp-self.last_selected[source] < self.period[source] - .003:
                return
            self.last_selected[source] = stamp
            self.queue.enqueue(source, stamp, message, time.monotonic())

    def _odom(self, message):
        if message.header.frame_id != self.frame or message.header.stamp.to_sec() <= 0:
            return
        with self.lock:
            self.odom_queue.append(message)

    def _imu(self, message):
        with self.lock:
            buffer_imu(self.gyros, message)

    def _transform(self, target, source, stamp):
        try:
            return transform_matrix(self.tf.lookup_transform(target, source, stamp, rospy.Duration(0)))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException, ValueError):
            return None

    def _lookup_cloud(self, source, unused_stamp):
        # Preserve the exact original ROS nanoseconds in every TF query.
        message = self.queue.queues[source][0][1]
        stamp, frame = message.header.stamp, message.header.frame_id
        world = self._transform(self.frame, frame, stamp)
        mount = self._transform(self.base_frame, frame, stamp)
        base = self._transform(self.frame, self.base_frame, stamp)
        if world is None or mount is None or base is None:
            return None
        return world, mount, xyzyaw(base)

    def _process_odom(self):
        with self.lock:
            if not self.odom_queue:
                return
            message = self.odom_queue[-1]
        if message.header.stamp.to_sec() <= self.last_base_stamp:
            return
        mount = self._transform(self.base_frame, self.imu_frame, message.header.stamp)
        if mount is None:
            return
        with self.lock:
            gyro = self.gyros.nearest(message.header.stamp.to_sec())
        try:
            output, pose = base_odometry(message, mount, gyro)
        except (ValueError, np.linalg.LinAlgError) as error:
            rospy.logwarn_throttle(5., 'Invalid LIO odometry: %s', error)
            return
        self.base_pub.publish(output)
        self.base_pose = pose
        self.base_twist_valid = gyro is not None
        self.last_base_stamp = message.header.stamp.to_sec()
        with self.lock:
            while self.odom_queue and self.odom_queue[0].header.stamp <= message.header.stamp:
                self.odom_queue.popleft()

    def _publish_terrain(self):
        now = rospy.Time.now()
        # Map timestamp belongs to real input, so stopped sensors cannot look new.
        stamp = max(state['stamp'] for state in self.terrain.sources.values())
        values, center = self.terrain.grid()
        self.grid_pub.publish(to_grid_message(values, self.terrain.resolution, center,
                                               rospy.Time.from_sec(stamp), self.frame))
        status = self.terrain.status(now.to_sec(), self.base_pose)
        status['base_stamp'] = self.last_base_stamp
        status['base_twist_valid'] = self.base_twist_valid
        status['points'] = dict(self.point_counts)
        with self.lock:
            status['pending'] = {s: self.queue.pending(s) for s in SOURCES}
            status['dropped'] = dict(self.queue.dropped)
        self.status_pub.publish(String(data=json.dumps(status, allow_nan=False, separators=(',', ':'))))
        header = Header(stamp=rospy.Time.from_sec(stamp), frame_id=self.frame)
        self.points_pub.publish(point_cloud2.create_cloud_xyz32(header, self.terrain.ground_points()))

    def _run(self):
        next_publish = 0.
        while not self.stop.is_set() and not rospy.is_shutdown():
            try:
                self._process_odom()
                with self.lock:
                    ready = self.queue.ready(self._lookup_cloud, time.monotonic())
                for source, stamp, message, transforms in ready:
                    world, mount, base = transforms
                    try:
                        native, points = prepare_cloud(message, mount, world,
                                                       self.min_range[source], self.max_range[source], self.voxel)
                    except (ValueError, TypeError, OverflowError) as error:
                        rospy.logwarn_throttle(5., 'Rejected %s surface cloud: %s', source, error)
                        continue
                    if self.terrain.update(source, stamp, points, base):
                        self.cloud_pub.publish(native)
                        self.point_counts[source] = len(points)
                wall = time.monotonic()
                if wall >= next_publish:
                    self._publish_terrain()
                    next_publish = wall+.2
            except Exception as error:
                # Worker failures are visible; no fabricated liveness is emitted.
                rospy.logerr_throttle(2., 'Terrain worker error: %s', error)
            self.stop.wait(.01)


def main():
    rospy.init_node('exploration_mapping')
    node = MappingNode()
    rospy.spin()
    node.stop.set()
    node.worker.join(timeout=2.)


if __name__ == '__main__':
    main()
