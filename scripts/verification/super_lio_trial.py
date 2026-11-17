#!/usr/bin/env python3
"""Run a bounded, independently evaluated Super-LIO trial inside the ROS container."""
import argparse
import bisect
import json
import math
import os
from pathlib import Path
import pty
import shutil
import signal
import subprocess
import threading
import time
import traceback

import numpy as np
import rosgraph
import rospy
import tf2_ros
from gazebo_msgs.srv import GetModelState
from nav_msgs.msg import Odometry, Path as RosPath
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, PointCloud2


def rigid(position, quaternion):
    q = np.asarray(quaternion, dtype=float)
    if not np.isfinite(q).all() or np.linalg.norm(q) < 1e-12:
        raise ValueError('invalid pose quaternion')
    x, y, z, w = q / np.linalg.norm(q)
    result = np.eye(4)
    result[:3, :3] = [
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]]
    result[:3, 3] = position
    if not np.isfinite(result).all():
        raise ValueError('non-finite pose')
    return result


def pose_data(pose):
    p, q = pose.position, pose.orientation
    position, quaternion = [p.x, p.y, p.z], [q.x, q.y, q.z, q.w]
    return dict(position=position, quaternion=quaternion,
                matrix=rigid(position, quaternion).tolist())


def angles(matrix):
    r = np.asarray(matrix)[:3, :3]
    return dict(roll=math.atan2(r[2, 1], r[2, 2]),
                pitch=math.atan2(-r[2, 0], math.hypot(r[2, 1], r[2, 2])),
                yaw=math.atan2(r[1, 0], r[0, 0]))


def interpolate_truth(rows, stamp):
    """Interpolate evaluation poses only; never publish them to the estimator."""
    index = bisect.bisect_left([row['stamp'] for row in rows], stamp)
    if index < len(rows) and abs(rows[index]['stamp'] - stamp) < 1e-9:
        return np.asarray(rows[index]['matrix'])
    if index == 0 or index == len(rows):
        return None
    first, last = rows[index - 1], rows[index]
    dt = last['stamp'] - first['stamp']
    if dt <= 0 or dt > .5:
        return None
    fraction = (stamp - first['stamp']) / dt
    p = (1 - fraction)*np.asarray(first['position']) + fraction*np.asarray(last['position'])
    q0, q1 = np.asarray(first['quaternion']), np.asarray(last['quaternion'])
    q0, q1 = q0 / np.linalg.norm(q0), q1 / np.linalg.norm(q1)
    dot = float(np.dot(q0, q1))
    if dot < 0:
        q1, dot = -q1, -dot
    if dot > .9995:
        q = (1 - fraction)*q0 + fraction*q1
    else:
        theta = math.acos(max(-1., min(1., dot)))
        q = (math.sin((1 - fraction)*theta)*q0 + math.sin(fraction*theta)*q1) / math.sin(theta)
    return rigid(p, q)


def trajectory_metrics(odometry, truth, alignment, start, end, phases):
    pairs, eligible = [], 0
    for row in odometry:
        if not start <= row['stamp'] <= end:
            continue
        eligible += 1
        reference = interpolate_truth(truth, row['stamp'])
        if reference is None:
            continue
        estimate = alignment @ np.asarray(row['matrix'])
        error = float(np.linalg.norm(estimate[:3, 3] - reference[:3, 3]))
        pairs.append(dict(stamp=row['stamp'], phase=row['phase'],
                          estimate=estimate[:3, 3].tolist(), truth=reference[:3, 3].tolist(),
                          estimate_yaw=angles(estimate)['yaw'], truth_yaw=angles(reference)['yaw'],
                          position_error_m=error))
    if len(pairs) < 10:
        raise RuntimeError('too few timestamp-matched trajectory samples')
    metrics = dict(paired_samples=len(pairs), eligible_samples=eligible,
                   matched_fraction=len(pairs) / eligible,
                   position_rmse_m=math.sqrt(sum(p['position_error_m']**2 for p in pairs) / len(pairs)),
                   max_position_error_m=max(p['position_error_m'] for p in pairs))
    for name in ('walk', 'stop', 'turn', 'turn_stop'):
        bounds = phases[name]
        selected = [p for p in pairs if bounds['start_sim'] <= p['stamp'] <= bounds['end_sim']]
        if len(selected) < 2:
            raise RuntimeError('not enough matched samples for ' + name)
        a, b = selected[0], selected[-1]
        direction = np.array([math.cos(a['truth_yaw']), math.sin(a['truth_yaw'])])
        gt_delta = np.array(b['truth']) - a['truth']
        estimate_delta = np.array(b['estimate']) - a['estimate']
        gt_yaw = np.unwrap([p['truth_yaw'] for p in selected])
        estimate_yaw = np.unwrap([p['estimate_yaw'] for p in selected])
        metrics[name] = dict(samples=len(selected),
                             truth_forward_m=float(gt_delta[:2] @ direction),
                             estimate_forward_m=float(estimate_delta[:2] @ direction),
                             forward_error_m=float(abs((estimate_delta[:2] - gt_delta[:2]) @ direction)),
                             truth_displacement_xy_m=float(np.linalg.norm(gt_delta[:2])),
                             estimate_displacement_xy_m=float(np.linalg.norm(estimate_delta[:2])),
                             truth_yaw_change_rad=float(gt_yaw[-1] - gt_yaw[0]),
                             estimate_yaw_change_rad=float(estimate_yaw[-1] - estimate_yaw[0]),
                             yaw_change_error_rad=float(abs((estimate_yaw[-1] - estimate_yaw[0])
                                                            - (gt_yaw[-1] - gt_yaw[0]))))
    return metrics, pairs


def pcd_info(path):
    with path.open('rb') as stream:
        header = {}
        for _ in range(100):
            line = stream.readline(4096).decode('ascii').strip()
            fields = line.split()
            if fields and not fields[0].startswith('#'):
                header[fields[0]] = fields[1:]
                if fields[0] == 'DATA':
                    break
        else:
            raise ValueError('PCD header has no DATA line: ' + str(path))
        points = int(header['POINTS'][0])
        payload = path.stat().st_size - stream.tell()
    return dict(path=str(path), bytes=path.stat().st_size, points=points,
                payload_bytes=payload, valid=points > 0 and payload > 0)


class Trial:
    def __init__(self, args):
        self.args = args
        self.output = args.output.resolve()
        self.output.mkdir(parents=True, exist_ok=True)
        self.started = time.monotonic()
        self.deadline = self.started + 300
        self.work_deadline = self.deadline - 40
        self.lock = threading.Lock()
        self.stop_queries = threading.Event()
        self.phase_name = 'startup'
        self.check_pose = False
        self.state = dict(sim=None, truth=None, odom=None, lidar_count=0, imu_count=0,
                          cloud_count=0, path_count=0, path=None, error=None)
        self.truth, self.odometry, self.clouds = [], [], []
        self.processes, self.logs = {}, {}
        self.terminal = self.query_thread = self.tf_buffer = self.tf_listener = None
        self.base_imu = None
        self.next_tf = 0
        self.tf_success = set()
        self.tf_report = dict(attempts=0, successful_stamps=0)
        self.alignment = None
        self.trajectory = []
        self.report = dict(result='FAIL', seconds=args.seconds, gui=args.gui, rviz=args.rviz,
                           record_bag=args.record_bag, phases={}, checks={}, failures=[],
                           posture_limits=dict(min_z_m=.18, max_abs_roll_rad=.8, max_abs_pitch_rad=.8),
                           accuracy_limits=dict(forward_error_m=.2, position_rmse_m=.15,
                                                yaw_change_error_rad=.2, estimated_stop_drift_m=.1,
                                                truth_stop_drift_m=.05))

    def fail(self, error):
        self.report['failures'].append(str(error))

    def snapshot(self):
        with self.lock:
            return dict(self.state)

    def clock(self, message):
        value = message.clock.to_sec()
        with self.lock:
            previous = self.state['sim']
            if previous is not None and value < previous - 1e-6:
                self.state['error'] = 'simulation clock moved backwards'
            self.state['sim'] = value

    def input_cloud(self, message):
        with self.lock:
            self.state['lidar_count'] += 1

    def input_imu(self, message):
        with self.lock:
            self.state['imu_count'] += 1

    def odom(self, message):
        try:
            if message.header.frame_id != 'lio_map' or message.child_frame_id != 'lio_imu':
                raise ValueError('unexpected LIO odometry frames')
            imu = pose_data(message.pose.pose)
            base = np.asarray(imu['matrix']) @ np.linalg.inv(self.base_imu)
            row = dict(stamp=message.header.stamp.to_sec(), stamp_ns=message.header.stamp.to_nsec(),
                       wall=time.monotonic() - self.started, phase=self.phase_name,
                       imu=imu, matrix=base.tolist(), position=base[:3, 3].tolist(), **angles(base))
            with self.lock:
                if self.odometry and row['stamp'] <= self.odometry[-1]['stamp']:
                    self.state['error'] = 'non-increasing LIO odometry stamp'
                self.odometry.append(row)
                self.state['odom'] = row
        except Exception as error:
            with self.lock:
                self.state['error'] = 'odometry callback: ' + str(error)

    def cloud(self, message):
        with self.lock:
            self.state['cloud_count'] += 1
            self.clouds.append(dict(stamp=message.header.stamp.to_sec(),
                                    points=message.width * message.height,
                                    frame=message.header.frame_id, phase=self.phase_name))
            if message.header.frame_id != 'lio_map':
                self.state['error'] = 'unexpected LIO cloud frame'

    def path(self, message):
        with self.lock:
            self.state['path_count'] += 1
            self.state['path'] = dict(frame=message.header.frame_id, stamp=message.header.stamp.to_sec(),
                                      poses=[pose_data(p.pose) for p in message.poses])

    def query_truth(self):
        proxy = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        next_query = time.monotonic()
        while not self.stop_queries.is_set():
            try:
                response = proxy('aliengo_gazebo', 'world')
                if response.success and response.header.stamp.to_nsec() > 0:
                    row = dict(stamp=response.header.stamp.to_sec(),
                               wall=time.monotonic() - self.started, phase=self.phase_name,
                               **pose_data(response.pose))
                    row.update(angles(row['matrix']))
                    with self.lock:
                        if not self.truth or row['stamp'] > self.truth[-1]['stamp']:
                            self.truth.append(row)
                        self.state['truth'] = row
            except Exception as error:
                with self.lock:
                    self.state['truth_query_error'] = str(error)
            next_query = max(next_query + .1, time.monotonic())
            self.stop_queries.wait(max(0, next_query - time.monotonic()))

    def tf_probe(self, row):
        stamp = rospy.Time(row['stamp_ns'] // 1000000000, row['stamp_ns'] % 1000000000)
        self.tf_report['attempts'] += 1
        try:
            transform = self.tf_buffer.lookup_transform('lio_map', 'base', stamp, rospy.Duration(0))
            t, q = transform.transform.translation, transform.transform.rotation
            matrix = rigid([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])
            error = float(np.linalg.norm(matrix[:3, 3] - np.asarray(row['position'])))
            rotation = matrix[:3, :3] @ np.asarray(row['matrix'])[:3, :3].T
            angle_error = math.acos(max(-1., min(1., (np.trace(rotation) - 1) / 2)))
            valid = error < .01 and angle_error < .01
            if valid:
                self.tf_success.add(row['stamp_ns'])
            self.tf_report.update(successful_stamps=len(self.tf_success), last_error=None,
                                  odom_stamp_ns=row['stamp_ns'],
                                  transform_stamp_ns=transform.header.stamp.to_nsec(),
                                  base_position_error_m=error, base_angle_error_rad=angle_error)
            return valid
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as error:
            self.tf_report['last_error'] = str(error)
            return False

    def tick(self):
        now = time.monotonic()
        if now >= self.work_deadline:
            raise RuntimeError('wall-clock work deadline reached; reserving shutdown time within 300 seconds')
        for name, process in self.processes.items():
            if process.poll() is not None:
                raise RuntimeError('%s exited early: %s' % (name, process.returncode))
        state = self.snapshot()
        if state['error']:
            raise RuntimeError(state['error'])
        pose = state['truth']
        if self.check_pose:
            if pose is None or now - self.started - pose['wall'] > 2:
                raise RuntimeError('evaluation pose stream is stale')
            if pose['position'][2] < .18 or abs(pose['roll']) > .8 or abs(pose['pitch']) > .8:
                raise RuntimeError('posture threshold exceeded: ' + str(pose))
        if state['odom'] is not None and self.tf_buffer is not None and now >= self.next_tf:
            self.tf_probe(state['odom'])
            self.next_tf = now + .05
        return state

    def wait(self, predicate, seconds, label):
        end = min(self.work_deadline, time.monotonic() + seconds)
        while True:
            state = self.tick()
            if predicate(state):
                return state
            if time.monotonic() >= end:
                raise RuntimeError(label + ' timed out')
            time.sleep(.01)

    def sim_wait(self, seconds):
        initial = self.snapshot()['sim']
        self.wait(lambda state: state['sim'] is not None and state['sim'] - initial >= seconds,
                  300, self.phase_name)

    def key(self, value):
        os.write(self.terminal, value)

    def run_phase(self, name, seconds, key=None):
        self.phase_name = name
        before = self.tick()
        started = time.monotonic()
        phase = self.report['phases'][name] = dict(start_sim=before['sim'])
        print('Phase:', name, seconds, 'simulated seconds', flush=True)
        try:
            if key is not None:
                self.key(key)
            self.sim_wait(seconds)
        finally:
            after = self.snapshot()
            phase.update(end_sim=after['sim'], wall_seconds=time.monotonic() - started,
                         start_truth=before['truth'], end_truth=after['truth'])

    def launch(self, name, command, stdin=None):
        self.logs[name] = (self.output / (name + '.log')).open('wb')
        self.report[name + '_command'] = command
        self.processes[name] = subprocess.Popen(command, stdin=stdin, stdout=self.logs[name],
                                                stderr=subprocess.STDOUT, start_new_session=True)

    def input_graph(self):
        publishers, subscribers, services = rosgraph.Master('/super_lio_trial').getSystemState()
        by_topic = dict(subscribers)
        nodes = set(by_topic.get('/livox/lidar', [])) & set(by_topic.get('/livox/imu', []))
        nodes = {node for node in nodes if node != rospy.get_name()
                 and not node.startswith('/super_lio_sensor_record')}
        inputs = {node: sorted(topic for topic, owners in subscribers if node in owners) for node in nodes}
        forbidden = {node: [topic for topic in topics
                            if any(token in topic.lower() for token in ('gazebo', 'truth', 'model_states', 'link_states'))]
                     for node, topics in inputs.items()}
        result = dict(lio_nodes=sorted(nodes), lio_subscriptions=inputs,
                      forbidden_subscriptions=forbidden, all_subscriptions=subscribers,
                      model_states_subscribers=by_topic.get('/gazebo/model_states', []))
        self.report['input_graph_last'] = result
        if not nodes or any(forbidden.values()):
            raise RuntimeError('LIO sensor-only subscription check failed: ' + str(result))
        if rospy.get_name() in result['model_states_subscribers']:
            raise RuntimeError('evaluation unexpectedly subscribed to ModelStates')
        return result

    def execute(self):
        maps = self.output / 'maps'
        if maps.exists() and any(maps.rglob('*.pcd')):
            raise RuntimeError('choose a fresh output directory; existing PCD fragments would contaminate the map')
        if self.args.record_bag and any(self.output.glob('sensors.bag*')):
            raise RuntimeError('choose a fresh output directory; sensors.bag already exists')
        (maps / 'PCD').mkdir(parents=True, exist_ok=True)
        if self.args.gui or self.args.rviz:
            result = subprocess.run(['python3', '/course/scripts/verification/glx_renderer.py'],
                                    check=True, capture_output=True, text=True, timeout=20)
            self.report['graphics'] = json.loads(result.stdout)
        self.launch('launch', ['roslaunch', '--sigint-timeout=5', '--sigterm-timeout=2',
                              '/course/ros1/mid360/gazebo.launch', 'rname:=aliengo',
                              'world_file:=/course/ros1/worlds/depot.world', 'spawn_x:=-5',
                              'spawn_y:=0', 'spawn_z:=0.6', 'spawn_yaw:=0', 'paused:=false',
                              'sim_ground_truth_tf:=false', 'rviz:=false',
                              'd435_enabled:=' + str(self.args.gui or self.args.rviz).lower(),
                              'gui:=' + str(self.args.gui).lower()])
        self.wait(lambda state: rosgraph.is_master_online(), 90, 'ROS master')
        rospy.init_node('super_lio_trial', disable_signals=True)
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(120))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.subscribers = [rospy.Subscriber('/clock', Clock, self.clock, queue_size=1),
                            rospy.Subscriber('/livox/lidar', PointCloud2, self.input_cloud, queue_size=1),
                            rospy.Subscriber('/livox/imu', Imu, self.input_imu, queue_size=100),
                            rospy.Subscriber('/lio/odom', Odometry, self.odom, queue_size=1000),
                            rospy.Subscriber('/lio/cloud_world', PointCloud2, self.cloud, queue_size=10),
                            rospy.Subscriber('/lio/path', RosPath, self.path, queue_size=1)]
        rospy.wait_for_service('/gazebo/get_model_state', timeout=60)
        self.query_thread = threading.Thread(target=self.query_truth, daemon=True)
        self.query_thread.start()
        self.wait(lambda state: state['sim'] is not None and state['sim'] > 0
                  and state['truth'] is not None and state['lidar_count'] > 0 and state['imu_count'] >= 50,
                  60, 'clock, robot and sensors')

        def get_extrinsic(state):
            try:
                transform = self.tf_buffer.lookup_transform('base', 'mid360_imu_link',
                                                           rospy.Time(0), rospy.Duration(0))
                t, q = transform.transform.translation, transform.transform.rotation
                self.base_imu = rigid([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])
                return True
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                return False

        self.wait(get_extrinsic, 15, 'URDF base to MID360 IMU extrinsic')
        self.report['base_to_imu'] = self.base_imu.tolist()
        self.terminal, slave = pty.openpty()
        try:
            self.launch('controller', ['/work/devel/lib/unitree_guide/junior_ctrl'], stdin=slave)
        finally:
            os.close(slave)
        self.sim_wait(1)
        self.run_phase('stand', 8, b'2')
        self.check_pose = True
        self.tick()
        if self.args.record_bag:
            self.launch('rosbag', ['rosbag', 'record', '--buffsize=64', '-O', str(self.output / 'sensors.bag'),
                                   '/clock', '/livox/lidar', '/livox/imu', '__name:=super_lio_sensor_record'])

            def recording(state):
                subscriptions = dict(rosgraph.Master('/super_lio_trial').getSystemState()[1])
                return all(any(node.startswith('/super_lio_sensor_record') for node in subscriptions.get(topic, []))
                           for topic in ('/clock', '/livox/lidar', '/livox/imu'))

            self.wait(recording, 15, 'sensor bag subscriptions')
        self.run_phase('sensor_leadin', 1)
        # ROOT in the bounded build is /lio/src/super_lio/. Match the upstream
        # concatenated save_map_dir convention; /result gives ../../../result/maps.
        map_argument = os.path.relpath(maps, '/lio/src/super_lio')
        self.launch('lio', ['roslaunch', '/course/ros1/super_lio/mapping.launch',
                            'rviz:=' + str(self.args.rviz).lower(), 'map_dir:=' + map_argument])

        def initialized(state):
            log = (self.output / 'lio.log').read_text(errors='replace')
            return ('Map init done' in log and state['odom'] is not None
                    and state['cloud_count'] > 0 and state['odom']['stamp_ns'] in self.tf_success)

        self.wait(initialized, 60, 'Map init done, estimated odometry/cloud and stamped TF')
        self.report['map_name'] = rospy.get_param('/lio/map/map_name')
        self.report['input_graph_start'] = self.input_graph()
        if self.args.rviz:
            from rviz.srv import SendFilePath, SendFilePathRequest
            import yaml
            rospy.wait_for_service('/super_lio_rviz/save_config', timeout=10)
            request = SendFilePathRequest()
            request.path.data = str(self.output / 'rviz-runtime.rviz')
            saved = rospy.ServiceProxy('/super_lio_rviz/save_config', SendFilePath)(request)
            assert saved.success, 'RViz runtime export failed'
            config = yaml.safe_load(Path(request.path.data).read_text())
            manager = config['Visualization Manager']
            subscribers = dict(rosgraph.Master('/super_lio_trial').getSystemState()[1])
            assert '/super_lio_rviz' in subscribers.get('/lio/cloud_world', []), 'RViz cloud subscription missing'
            assert manager['Global Options']['Fixed Frame'] == 'lio_map', 'RViz is not displaying estimated map frame'
            assert manager['Views']['Current']['Target Frame'] == '<Fixed Frame>', 'RViz camera follows robot'
            self.report['rviz_runtime'] = dict(fixed_frame='lio_map', registered_cloud_subscribed=True,
                                               config_file='rviz-runtime.rviz')
        initial = self.snapshot()['odom']

        def align_once(state):
            with self.lock:
                reference = interpolate_truth(self.truth, initial['stamp'])
            if reference is None:
                return False
            self.alignment = reference @ np.linalg.inv(np.asarray(initial['matrix']))
            return True

        self.wait(align_once, 5, 'one initial evaluation SE3 alignment')
        self.report['initial_alignment'] = dict(stamp=initial['stamp'], matrix=self.alignment.tolist(),
                                                 evaluation_only=True, updates=1)
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
        self.wait(lambda state: self.tf_probe(final), 5, 'TF at the final exact estimated stamp')
        self.wait(lambda state: interpolate_truth(list(self.truth), final['stamp']) is not None,
                  5, 'final timestamp-matched evaluation pose')
        with self.lock:
            metrics, self.trajectory = trajectory_metrics(
                list(self.odometry), list(self.truth), self.alignment,
                initial['stamp'], final['stamp'], self.report['phases'])
        self.report['trajectory'] = metrics
        limits = self.report['accuracy_limits']
        checks = self.report['checks']
        checks.update(forward_motion=metrics['walk']['truth_forward_m'] > self.args.seconds*.01,
                      forward_error=metrics['walk']['forward_error_m'] < limits['forward_error_m'],
                      trajectory_rmse=metrics['position_rmse_m'] < limits['position_rmse_m'],
                      truth_match_coverage=metrics['matched_fraction'] >= .95,
                      turn_motion=abs(metrics['turn']['truth_yaw_change_rad']) > .1,
                      turn_error=metrics['turn']['yaw_change_error_rad'] < limits['yaw_change_error_rad'],
                      tf_at_estimated_stamp=True)
        for name in ('stop', 'turn_stop'):
            checks[name + '_truth_drift'] = metrics[name]['truth_displacement_xy_m'] < limits['truth_stop_drift_m']
            checks[name + '_estimate_drift'] = metrics[name]['estimate_displacement_xy_m'] < limits['estimated_stop_drift_m']
        state = self.snapshot()
        checks['cloud_nonempty'] = any(row['points'] > 0 for row in self.clouds)
        checks['path_received'] = bool(state['path'] and state['path']['frame'] == 'lio_map'
                                       and len(state['path']['poses']) >= 2)
        self.report['input_graph_end'] = self.input_graph()
        for name, passed in checks.items():
            if not passed:
                self.fail('check failed: ' + name)

    def stop_process(self, name, timeout):
        process = self.processes.get(name)
        if process is None:
            return
        started = time.monotonic()
        exited_before_signal = process.poll() is not None
        graceful = exited_before_signal
        if exited_before_signal:
            self.fail('%s exited before requested shutdown: %s' % (name, process.returncode))
        if not graceful:
            try:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=max(.01, min(timeout, self.deadline - time.monotonic() - 1)))
                graceful = True
            except (OSError, subprocess.TimeoutExpired):
                pass
        # Also reap a process group whose roslaunch parent exited unexpectedly.
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except OSError:
            pass
        try:
            process.wait(timeout=.5)
        except subprocess.TimeoutExpired:
            pass
        self.report.setdefault('shutdown', {})[name] = dict(graceful=graceful,
                                                            exited_before_signal=exited_before_signal,
                                                            returncode=process.returncode,
                                                            wall_seconds=time.monotonic() - started)
        if not graceful:
            self.fail(name + ' did not finish after SIGINT within its shutdown budget')

    def finish(self):
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if self.terminal is not None:
            try:
                self.key(b' ')
            except OSError:
                pass
        self.report['phase_at_stop'] = self.phase_name
        self.phase_name = 'shutdown'
        # Keep sensors and /clock alive while LIO saves, then finalize bag indexes.
        self.stop_process('lio', 10)
        self.stop_process('rosbag', 10)
        self.stop_process('controller', 3)
        self.stop_process('launch', 12)
        self.stop_queries.set()
        if self.query_thread is not None:
            self.query_thread.join(timeout=.5)
        if self.terminal is not None:
            os.close(self.terminal)
        if rospy.core.is_initialized():
            rospy.signal_shutdown('Super-LIO trial finished')
        if 'lio' in self.processes:
            try:
                files = [pcd_info(path) for path in sorted((self.output / 'maps').rglob('*.pcd'))]
                self.report['pcd_files'] = files
                map_name = self.report.get('map_name')
                valid = any(Path(item['path']).name == map_name and item['valid'] for item in files)
                self.report['checks']['map_pcd_nonempty'] = valid
                if not valid:
                    self.fail('nonempty final configured PCD was not saved: ' + str(map_name))
            except Exception as error:
                self.fail('PCD verification: ' + str(error))
        if self.args.record_bag and 'rosbag' in self.processes:
            try:
                import rosbag
                bag_path = self.output / 'sensors.bag'
                with rosbag.Bag(str(bag_path), 'r') as bag:
                    topics = bag.get_type_and_topic_info().topics
                    counts = {topic: info.message_count for topic, info in topics.items()}
                self.report['bag'] = dict(path=str(bag_path), bytes=bag_path.stat().st_size, topics=counts)
                valid = (set(counts) == {'/clock', '/livox/lidar', '/livox/imu'}
                         and all(count > 0 for count in counts.values())
                         and not (self.output / 'sensors.bag.active').exists())
                self.report['checks']['sensor_bag_indexed'] = valid
                if not valid:
                    self.fail('sensor bag topics, counts or finalized index check failed')
            except Exception as error:
                self.fail('bag verification: ' + str(error))
        ogre = Path.home() / '.gazebo/ogre.log'
        if ogre.is_file():
            shutil.copyfile(str(ogre), str(self.output / 'ogre.log'))
        with self.lock:
            self.report.update(lidar_count=self.state['lidar_count'], imu_count=self.state['imu_count'],
                               odom_count=len(self.odometry), cloud_count=self.state['cloud_count'],
                               path_count=self.state['path_count'], truth_count=len(self.truth),
                               tf=self.tf_report, final_phase=self.phase_name)
            samples = dict(truth=list(self.truth), odometry=list(self.odometry),
                           clouds=list(self.clouds), path=self.state['path'])
        self.report['wall_seconds'] = time.monotonic() - self.started
        self.report['result'] = 'PASS' if not self.report['failures'] else 'FAIL'
        (self.output / 'samples.json').write_text(json.dumps(samples, indent=2, allow_nan=False))
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
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error('--seconds must be finite and positive')
    trial = Trial(args)

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
    print('Super-LIO trial:', trial.report['result'], trial.report['failures'], flush=True)
    return 0 if trial.report['result'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
