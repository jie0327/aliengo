#!/usr/bin/env python3
"""roslaunch prefix: load measured simulation calibration, then exec FAST-LIVO2."""
from collections import OrderedDict
import json
import os
import sys
import threading
import time

import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import CameraInfo, Image
from tf.transformations import quaternion_matrix

from calibration_core import camera_parameters, extrinsic_parameters


def camera_record(message):
    roi = message.roi
    return dict(width=message.width, height=message.height, K=list(message.K), D=list(message.D),
                R=list(message.R), P=list(message.P), distortion_model=message.distortion_model,
                frame_id=message.header.frame_id, stamp=message.header.stamp.to_sec(),
                binning_x=message.binning_x, binning_y=message.binning_y,
                roi=dict(x_offset=roi.x_offset, y_offset=roi.y_offset, width=roi.width, height=roi.height))


def image_record(message):
    return dict(width=message.width, height=message.height, encoding=message.encoding,
                frame_id=message.header.frame_id, stamp=message.header.stamp.to_sec())


def transform_matrix(message):
    p, q = message.transform.translation, message.transform.rotation
    quaternion = np.array([q.x, q.y, q.z, q.w])
    if not np.isfinite(quaternion).all() or abs(np.linalg.norm(quaternion) - 1.) > 1.e-4:
        raise ValueError('sensor TF contains an invalid quaternion')
    matrix = quaternion_matrix(quaternion)
    matrix[:3, 3] = [p.x, p.y, p.z]
    return matrix


def main():
    if len(sys.argv) < 2:
        raise ValueError('calibrated_exec.py requires the FAST-LIVO2 executable and ROS arguments')
    command = sys.argv[1:]
    # The temporary node uses its own name so its cleanup cannot unregister the
    # estimator that immediately replaces this process. Preserve launch namespace.
    bootstrap_args = [sys.argv[0]] + [arg for arg in command[1:] if arg.startswith('__ns:=')]
    rospy.init_node('fast_livo_calibration', argv=bootstrap_args, anonymous=True)
    if not rospy.get_param('/use_sim_time', False):
        raise ValueError('this FAST-LIVO2 launch requires simulated sensor time')
    map_dir = rospy.get_param('pcd_save/output_dir', '/fast_livo/maps/current')
    if not os.path.isabs(map_dir):
        raise ValueError('pcd_save/output_dir must be an absolute directory')
    os.makedirs(map_dir, exist_ok=True)
    image_topic = rospy.get_param('common/img_topic', '/camera/color/image_raw')
    info_topic = rospy.get_param('calibration/camera_info_topic', '/camera/color/camera_info')
    imu_frame = rospy.get_param('calibration/imu_frame', 'mid360_imu_link')
    lidar_frame = rospy.get_param('calibration/lidar_frame', 'mid360_link')
    timeout = float(rospy.get_param('calibration/timeout_wall', 45.))
    lock = threading.Lock()
    samples = {'image': OrderedDict(), 'info': OrderedDict()}

    def capture(kind, record, message):
        with lock:
            values = samples[kind]
            values[message.header.stamp.to_nsec()] = record(message)
            while len(values) > 8:
                values.popitem(last=False)

    subscribers = [
        rospy.Subscriber(image_topic, Image, lambda msg: capture('image', image_record, msg),
                         queue_size=1, buff_size=16*1024*1024),
        rospy.Subscriber(info_topic, CameraInfo, lambda msg: capture('info', camera_record, msg), queue_size=1)]
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)
    deadline = time.monotonic() + timeout
    rospy.loginfo('Waiting for synchronized RGB CameraInfo and static sensor TFs')
    try:
        while not rospy.is_shutdown():
            if time.monotonic() >= deadline:
                raise RuntimeError('timed out waiting for live matching camera calibration and sensor TFs')
            with lock:
                matches = samples['image'].keys() & samples['info'].keys()
                stamp = max(matches) if matches else None
                info = samples['info'].get(stamp)
                image = samples['image'].get(stamp)
            now = rospy.Time.now().to_sec()
            if info is None or now <= 0 or not -.01 <= now - info['stamp'] <= .5:
                time.sleep(.02)
                continue
            camera = camera_parameters(info, image, now)
            try:
                imu_tf = buffer.lookup_transform(imu_frame, lidar_frame, rospy.Time(0))
                camera_tf = buffer.lookup_transform(info['frame_id'], lidar_frame, rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                time.sleep(.02)
                continue
            extrinsics = extrinsic_parameters(transform_matrix(imu_tf), transform_matrix(camera_tf))
            # Upstream uses a normal NodeHandle for extrinsics and an explicit
            # relative 'laserMapping' namespace for vikit's camera model.
            rospy.set_param('extrin_calib', extrinsics)
            rospy.set_param('publish/camera_frame', info['frame_id'])
            for name, value in camera.items():
                rospy.set_param('laserMapping/' + name, value)
            report = dict(image_topic=image_topic, camera_info_topic=info_topic,
                          image_stamp=info['stamp'], image_frame=info['frame_id'],
                          lidar_frame=lidar_frame, imu_frame=imu_frame,
                          camera=camera, extrinsics=extrinsics)
            rospy.set_param('calibration/loaded', report)
            rospy.loginfo('FAST-LIVO2 runtime calibration: %s', json.dumps(report, sort_keys=True))
            break
        else:
            return 1
    finally:
        for subscriber in subscribers:
            subscriber.unregister()
        listener.unregister()
        rospy.signal_shutdown('calibration bootstrap finished')
    os.execv(command[0], command)
    return 1


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Exception as error:
        print('FAST-LIVO2 calibration failed: ' + str(error), file=sys.stderr, flush=True)
        sys.exit(1)
