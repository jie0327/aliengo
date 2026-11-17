#!/usr/bin/env python3
"""Bounded FAST-LIVO2 simulation or sensor-only replay, with explicit VIO evidence."""
import argparse
import json
import math
import os
from pathlib import Path
import pty
import re
import signal
import struct
import threading
import time
import traceback

import numpy as np
import rosbag
import rosgraph
import rospy
import tf2_ros
from nav_msgs.msg import Odometry, Path as RosPath
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, Imu, PointCloud2

from super_lio_trial import Trial, angles, interpolate_truth, pcd_info, pose_data, rigid, trajectory_metrics

SENSOR_TOPICS = {'/clock', '/livox/lidar', '/livox/imu', '/camera/color/image_raw',
                 '/camera/color/camera_info', '/tf_static'}


class FastLivoTrial(Trial):
    def __init__(self, args):
        super().__init__(args)
        self.deadline = self.started + 600
        self.work_deadline = self.deadline - 40
        self.report.update(algorithm='FAST-LIVO2', replay=bool(args.bag), wall_timeout_seconds=600)
        self.state.update(image_count=0, tracked_count=0, last_image_stamp=None,
                          last_lidar_stamp=None, last_imu_stamp=None)
        self.camera_stamps = set()
        self.maps = self.output / 'maps'

    def image(self, message):
        with self.lock:
            self.state['image_count'] += 1
            self.state['last_image_stamp'] = message.header.stamp.to_sec()
            self.camera_stamps.add(message.header.stamp.to_nsec())

    def input_cloud(self, message):
        super().input_cloud(message)
        with self.lock:
            self.state['last_lidar_stamp'] = message.header.stamp.to_sec()

    def input_imu(self, message):
        super().input_imu(message)
        with self.lock:
            self.state['last_imu_stamp'] = message.header.stamp.to_sec()

    @staticmethod
    def fusion_lag(state):
        if state['odom'] is None or state['last_image_stamp'] is None or state['last_lidar_stamp'] is None:
            return None
        # VIO can wait for the next cloud; compare with the slower input stream.
        return min(state['last_image_stamp'], state['last_lidar_stamp']) - state['odom']['stamp']

    @staticmethod
    def sensor_lag(state):
        stamps = [state[name] for name in ('last_image_stamp', 'last_lidar_stamp', 'last_imu_stamp')]
        if state['sim'] is None or any(stamp is None for stamp in stamps):
            return None
        return state['sim'] - min(stamps)

    def tick(self):
        state = super().tick()
        lag = self.fusion_lag(state)
        if lag is not None and lag > 2.0:
            raise RuntimeError('fusion stopped progressing: %.3f sensor seconds behind' % lag)
        input_lag = self.sensor_lag(state)
        if state['odom'] is not None and input_lag is not None and input_lag > 2.0:
            raise RuntimeError('sensor stream stopped progressing: %.3f simulated seconds behind' % input_lag)
        return state

    def tracked(self, message):
        with self.lock:
            self.state['tracked_count'] += 1

    def odom(self, message):
        try:
            if message.header.frame_id != 'livo_map' or message.child_frame_id != 'mid360_imu_link':
                raise ValueError('unexpected FAST-LIVO2 odometry frames')
            if self.base_imu is None:
                return
            imu = pose_data(message.pose.pose)
            base = np.asarray(imu['matrix']) @ np.linalg.inv(self.base_imu)
            row = dict(stamp=message.header.stamp.to_sec(), stamp_ns=message.header.stamp.to_nsec(),
                       wall=time.monotonic() - self.started, phase=self.phase_name,
                       imu=imu, matrix=base.tolist(), position=base[:3, 3].tolist(), **angles(base))
            with self.lock:
                if self.odometry and row['stamp'] <= self.odometry[-1]['stamp']:
                    self.state['error'] = 'non-increasing FAST-LIVO2 odometry stamp'
                self.odometry.append(row)
                self.state['odom'] = row
        except Exception as error:
            with self.lock:
                self.state['error'] = 'odometry callback: ' + str(error)

    def cloud(self, message):
        rgb = next((field for field in message.fields if field.name == 'rgb'), None)
        colored = 0
        if rgb is not None:
            for index in range(min(256, message.width * message.height)):
                row, column = divmod(index, message.width)
                offset = row * message.row_step + column * message.point_step + rgb.offset
                packed = struct.unpack_from('>I' if message.is_bigendian else '<I', message.data, offset)[0]
                colored += bool(packed & 0x00ffffff)
        with self.lock:
            self.state['cloud_count'] += 1
            self.clouds.append(dict(stamp=message.header.stamp.to_sec(), points=message.width * message.height,
                                    frame=message.header.frame_id, fields=[f.name for f in message.fields],
                                    sampled_nonblack_points=colored))
            if message.header.frame_id != 'livo_map':
                self.state['error'] = 'unexpected FAST-LIVO2 cloud frame'

    def tf_probe(self, row):
        stamp = rospy.Time(row['stamp_ns'] // 1000000000, row['stamp_ns'] % 1000000000)
        self.tf_report['attempts'] += 1
        try:
            transform = self.tf_buffer.lookup_transform('livo_map', 'base', stamp, rospy.Duration(0))
            t, q = transform.transform.translation, transform.transform.rotation
            actual = rigid([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])
            error = float(np.max(np.abs(actual - np.asarray(row['matrix']))))
            valid = error < 1e-5
            if valid:
                self.tf_success.add(row['stamp_ns'])
            self.tf_report.update(successful_stamps=len(self.tf_success), matrix_max_error=error,
                                  last_error=None, odom_stamp_ns=row['stamp_ns'],
                                  transform_stamp_ns=transform.header.stamp.to_nsec())
            return valid
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as error:
            self.tf_report['last_error'] = str(error)
            return False

    def get_extrinsic(self, state):
        try:
            transform = self.tf_buffer.lookup_transform('base', 'mid360_imu_link', rospy.Time(0), rospy.Duration(0))
            t, q = transform.transform.translation, transform.transform.rotation
            self.base_imu = rigid([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])
            self.report['base_to_imu'] = self.base_imu.tolist()
            return True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return False

    def input_graph(self):
        publishers, subscribers, services = rosgraph.Master(rospy.get_name()).getSystemState()
        owners = dict(publishers).get('/fast_livo/odom', [])
        inputs = {node: sorted(topic for topic, nodes in subscribers if node in nodes) for node in owners}
        required = {'/clock', '/livox/lidar', '/livox/imu', '/camera/color/image_raw'}
        if len(owners) != 1 or any(set(topics) != required for topics in inputs.values()):
            raise RuntimeError('unexpected estimator subscriptions: ' + str(inputs))
        if self.args.bag:
            if any(topic.startswith('/gazebo/') for topic, _ in publishers + services):
                raise RuntimeError('replay graph contains Gazebo interfaces')
        result = dict(publishers=owners, estimator_inputs=inputs, sensor_only=True)
        self.report['input_graph'] = result
        self.report['calibration'] = rospy.get_param('/fast_livo/calibration/loaded')
        return result

    def visual_features(self):
        path = self.output / 'lio.log'
        log = path.read_text(errors='replace') if path.exists() else ''
        counts = [int(value) for value in re.findall(r'Retrieve (\d+) points from visual sparse map', log)]
        return dict(frames=len(counts), frames_with_features=sum(n > 0 for n in counts),
                    max_features=max(counts, default=0))

    def initialized(self, state):
        return (state['odom'] is not None and state['cloud_count'] > 0
                and state['tracked_count'] > 0 and self.visual_features()['frames_with_features'] >= 3
                and state['odom']['stamp_ns'] in self.tf_success)

    def start_estimator(self):
        self.launch('lio', ['roslaunch', '--sigint-timeout=15', '--sigterm-timeout=2',
                            '/course/ros1/fast_livo/mapping.launch',
                            'rviz:=' + str(self.args.rviz).lower(), 'map_dir:=' + str(self.maps)])

    def execute(self):
        if self.maps.exists() and any(self.maps.iterdir()):
            raise RuntimeError('choose a fresh output directory')
        self.maps.mkdir(exist_ok=True)
        if self.args.bag:
            with rosbag.Bag(str(self.args.bag)) as bag:
                topics = bag.get_type_and_topic_info().topics
                if set(topics) != SENSOR_TOPICS or any(t.message_count == 0 for t in topics.values()):
                    raise RuntimeError('replay requires the complete sensor-only FAST-LIVO2 bag')
                self.report['bag_topics'] = {k: v.message_count for k, v in topics.items()}
            self.launch('master', ['roscore'])
        else:
            self.launch('launch', ['roslaunch', '--sigint-timeout=5', '--sigterm-timeout=2',
                                  '/course/ros1/mid360/gazebo.launch', 'rname:=aliengo',
                                  'world_file:=/course/ros1/worlds/depot.world', 'spawn_x:=-5',
                                  'spawn_y:=0', 'spawn_z:=0.6', 'spawn_yaw:=0', 'paused:=false',
                                  'sim_ground_truth_tf:=false', 'rviz:=false', 'd435_enabled:=true',
                                  'd435_color_width:=640', 'd435_color_height:=360',
                                  'd435_depth_width:=424', 'd435_depth_height:=240', 'd435_rate:=15',
                                  'gui:=' + str(self.args.gui).lower()])
        self.wait(lambda state: rosgraph.is_master_online(), 90, 'ROS master')
        rospy.set_param('/use_sim_time', True)
        rospy.init_node('fast_livo_trial', disable_signals=True)
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(120))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.subscribers = [
            rospy.Subscriber('/clock', Clock, self.clock, queue_size=10),
            rospy.Subscriber('/livox/lidar', PointCloud2, self.input_cloud, queue_size=5),
            rospy.Subscriber('/livox/imu', Imu, self.input_imu, queue_size=100),
            rospy.Subscriber('/camera/color/image_raw', Image, self.image, queue_size=10),
            rospy.Subscriber('/fast_livo/tracked_image', Image, self.tracked, queue_size=10),
            rospy.Subscriber('/fast_livo/odom', Odometry, self.odom, queue_size=1000),
            rospy.Subscriber('/fast_livo/cloud_world', PointCloud2, self.cloud, queue_size=10),
            rospy.Subscriber('/fast_livo/path', RosPath, self.path, queue_size=2)]
        if self.args.bag:
            self.start_estimator()
            self.launch('playback', ['rosbag', 'play', '--clock', '--rate=0.5', '--delay=2',
                                     str(self.args.bag), '--topics', *sorted(SENSOR_TOPICS - {'/clock'})])
            self.wait(self.get_extrinsic, 30, 'recorded static extrinsics')
            self.wait(self.initialized, 120, 'replayed fusion and visual features')
            self.input_graph()
            self.phase_name = 'replay'
            # A completed player is expected; other child exits are failures.
            player = self.processes.pop('playback')
            try:
                while player.poll() is None:
                    self.tick()
                    time.sleep(.05)
                if player.returncode != 0:
                    raise RuntimeError('rosbag player failed')
            finally:
                self.processes['playback'] = player
            # Remove a successfully completed player from inherited liveness checks.
            self.processes.pop('playback')
            self.report['playback_returncode'] = player.returncode
            end = time.monotonic() + 2
            while time.monotonic() < end:
                self.tick()
                time.sleep(.05)
            self.input_graph()
            return

        rospy.wait_for_service('/gazebo/get_model_state', timeout=60)
        self.query_thread = threading.Thread(target=self.query_truth, daemon=True)
        self.query_thread.start()
        self.wait(lambda s: s['truth'] is not None and s['sim'] is not None and s['sim'] > 0
                  and s['lidar_count'] > 0 and s['imu_count'] >= 50 and s['image_count'] > 0,
                  90, 'robot, clock and three sensors')
        self.wait(self.get_extrinsic, 15, 'static sensor extrinsics')
        self.terminal, slave = pty.openpty()
        try:
            self.launch('controller', ['/work/devel/lib/unitree_guide/junior_ctrl'], stdin=slave)
        finally:
            os.close(slave)
        self.sim_wait(1)
        self.run_phase('stand', 8, b'2')
        self.check_pose = True
        if self.args.record_bag:
            self.launch('rosbag', ['rosbag', 'record', '--lz4', '--buffsize=64',
                                   '-O', str(self.output / 'sensors.bag'), *sorted(SENSOR_TOPICS),
                                   '__name:=fast_livo_sensor_record'])
            self.wait(lambda s: all('/fast_livo_sensor_record' in dict(rosgraph.Master(rospy.get_name()).getSystemState()[1]).get(t, [])
                                   for t in SENSOR_TOPICS), 15, 'sensor bag subscriptions')
        self.run_phase('sensor_leadin', 1)
        self.start_estimator()
        self.wait(self.initialized, 120, 'IMU initialization, visual features, odometry and stamped TF')
        self.input_graph()
        initial = self.snapshot()['odom']

        def align_once(state):
            with self.lock:
                reference = interpolate_truth(self.truth, initial['stamp'])
            if reference is None:
                return False
            self.alignment = reference @ np.linalg.inv(np.asarray(initial['matrix']))
            return True

        self.wait(align_once, 10, 'one evaluation-only initial alignment')
        self.report['initial_alignment'] = dict(stamp=initial['stamp'], matrix=self.alignment.tolist(), updates=1)
        self.run_phase('trot_zero', 2, b'4')
        self.key(b'w')
        self.sim_wait(.1)
        self.run_phase('walk', self.args.seconds, b'w')
        self.run_phase('stop', 5, b' ')
        self.key(b'j')
        self.sim_wait(.1)
        self.run_phase('turn', 8, b'j')
        self.run_phase('turn_stop', 3, b' ')
        final = self.snapshot()['odom']
        self.wait(lambda s: self.tf_probe(final), 5, 'final stamped TF')
        self.wait(lambda s: interpolate_truth(list(self.truth), final['stamp']) is not None, 5, 'final truth pose')
        with self.lock:
            metrics, self.trajectory = trajectory_metrics(self.odometry, self.truth, self.alignment,
                                                        initial['stamp'], final['stamp'], self.report['phases'])
        self.report['trajectory'] = metrics
        limits = self.report['accuracy_limits']
        checks = self.report['checks']
        checks.update(forward_motion=metrics['walk']['truth_forward_m'] > self.args.seconds * .01,
                      forward_error=metrics['walk']['forward_error_m'] < limits['forward_error_m'],
                      trajectory_rmse=metrics['position_rmse_m'] < limits['position_rmse_m'],
                      truth_match_coverage=metrics['matched_fraction'] >= .95,
                      turn_motion=abs(metrics['turn']['truth_yaw_change_rad']) > .1,
                      turn_error=metrics['turn']['yaw_change_error_rad'] < limits['yaw_change_error_rad'])
        for name in ('stop', 'turn_stop'):
            checks[name + '_truth_drift'] = metrics[name]['truth_displacement_xy_m'] < limits['truth_stop_drift_m']
            checks[name + '_estimate_drift'] = metrics[name]['estimate_displacement_xy_m'] < limits['estimated_stop_drift_m']
        self.input_graph()

    def finish(self):
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if self.terminal is not None:
            self.key(b' ')
        self.report['phase_at_stop'] = self.phase_name
        # Inputs intentionally remain live while mapping writes its PCD. Coverage
        # must be measured before that shutdown interval, not after Gazebo exits.
        final_active_lag = self.fusion_lag(self.snapshot())
        final_sensor_lag = self.sensor_lag(self.snapshot())
        self.phase_name = 'shutdown'
        for name, seconds in [('playback', 2), ('lio', 18), ('rosbag', 10), ('controller', 3), ('launch', 12), ('master', 3)]:
            self.stop_process(name, seconds)
        self.stop_queries.set()
        if self.query_thread:
            self.query_thread.join(timeout=1)
        if self.terminal is not None:
            os.close(self.terminal)
        if rospy.core.is_initialized():
            rospy.signal_shutdown('FAST-LIVO2 verification finished')
        self.report['visual'] = self.visual_features()
        pcds = []
        for path in sorted(self.maps.glob('*.pcd')):
            try:
                pcds.append(pcd_info(path))
            except Exception as error:
                self.fail('invalid PCD %s: %s' % (path, error))
        self.report['pcd_files'] = pcds
        state = self.snapshot()
        checks = self.report['checks']
        stamp_matches = sum(row['stamp_ns'] in self.camera_stamps for row in self.odometry)
        self.report['odometry_image_stamp_matches'] = stamp_matches
        lag = final_active_lag
        self.report['final_fusion_lag_seconds'] = lag
        self.report['final_sensor_lag_seconds'] = final_sensor_lag
        checks.update(odometry_received=len(self.odometry) > 10,
                      colored_cloud_nonempty=any(c['points'] > 0 and c['sampled_nonblack_points'] > 0 for c in self.clouds),
                      path_received=bool(state['path'] and state['path']['frame'] == 'livo_map' and len(state['path']['poses']) > 2),
                      visual_features=self.report['visual']['frames_with_features'] >= 3,
                      tracked_images=state['tracked_count'] >= 3,
                      sensor_inputs=all(state[k] > 0 for k in ('lidar_count', 'imu_count', 'image_count')),
                      stamped_tf=len(self.tf_success) > 5,
                      measurement_timestamps=bool(self.odometry) and stamp_matches / len(self.odometry) > .95,
                      fusion_reaches_input_end=lag is not None and -.01 <= lag <= 1.0,
                      all_sensors_reach_end=final_sensor_lag is not None and -.1 <= final_sensor_lag <= 1.0,
                      map_saved=any(p['valid'] and Path(p['path']).name == 'all_downsampled_points.pcd' for p in pcds))
        if self.args.record_bag:
            try:
                with rosbag.Bag(str(self.output / 'sensors.bag')) as bag:
                    topics = bag.get_type_and_topic_info().topics
                    self.report['bag_topics'] = {k: v.message_count for k, v in topics.items()}
                    checks['bag_indexed'] = set(topics) == SENSOR_TOPICS and all(v.message_count for v in topics.values())
            except Exception as error:
                checks['bag_indexed'] = False
                self.fail('bag verification: ' + str(error))
        for name, passed in checks.items():
            if not passed:
                self.fail('check failed: ' + name)
        self.report.update(odom_count=len(self.odometry), cloud_count=len(self.clouds), image_count=state['image_count'],
                           tracked_image_count=state['tracked_count'], tf=self.tf_report,
                           wall_seconds=time.monotonic() - self.started)
        self.report['result'] = 'FAIL' if self.report['failures'] else 'PASS'
        (self.output / 'samples.json').write_text(json.dumps(dict(odometry=self.odometry, truth=self.truth, clouds=self.clouds), indent=2, allow_nan=False))
        (self.output / 'trajectory.json').write_text(json.dumps(self.trajectory, indent=2, allow_nan=False))
        (self.output / 'summary.json').write_text(json.dumps(self.report, indent=2, allow_nan=False))
        for stream in self.logs.values():
            stream.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('/result'))
    parser.add_argument('--seconds', type=float, default=20)
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--rviz', action='store_true')
    parser.add_argument('--record-bag', action='store_true')
    parser.add_argument('--bag', type=Path)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 0 < args.seconds <= 60:
        parser.error('--seconds must be in (0, 60]')
    if args.bag and (args.gui or args.record_bag):
        parser.error('replay cannot enable simulation GUI or record another bag')
    trial = FastLivoTrial(args)

    def interrupted(signum, frame):
        raise KeyboardInterrupt('signal %s' % signum)

    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    try:
        trial.execute()
    except BaseException as error:
        trial.fail('%s: %s' % (type(error).__name__, error))
        trial.report['traceback'] = traceback.format_exc()
    finally:
        trial.finish()
    print('FAST-LIVO2 trial:', trial.report['result'], trial.report['failures'], flush=True)
    return 0 if trial.report['result'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
