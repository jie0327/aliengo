#!/usr/bin/env python3
"""Replay sensor-only data and verify Super-LIO without Gazebo or truth inputs."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import traceback

import rosbag
import rosgraph
import rospy
from nav_msgs.msg import Odometry, Path as RosPath
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, PointCloud2


INPUT_TOPICS = {'/clock', '/livox/lidar', '/livox/imu'}
OUTPUT_TOPICS = {'/lio/odom', '/lio/cloud_world', '/lio/path'}


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def xyz(vector):
    return [vector.x, vector.y, vector.z]


def pose_values(pose):
    q = pose.orientation
    return xyz(pose.position), [q.x, q.y, q.z, q.w]


def pcd_info(path):
    with path.open('rb') as stream:
        header = {}
        for _ in range(100):
            fields = stream.readline(4096).decode('ascii').strip().split()
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


class Replay:
    def __init__(self, args):
        self.args = args
        self.output = args.output.resolve()
        self.output.mkdir(parents=True, exist_ok=True)
        self.maps = self.output / 'maps'
        self.started = time.monotonic()
        self.deadline = self.started + 300
        self.work_deadline = self.deadline - 15  # 为停止回放与保存地图预留墙钟预算。
        self.lock = threading.Lock()
        self.odometry, self.clouds, self.paths = [], [], []
        self.counts = dict(clock=0, lidar=0, imu=0)
        self.processes, self.logs = {}, {}
        self.failures = []
        self.master = None
        self.subscribers = []
        self.initial_maps = {}
        self.map_name = 'depot.pcd'
        self.report = dict(result='FAIL', bag=str(args.bag), output=str(self.output),
                           wall_timeout_seconds=300, commands={}, graph_snapshots=[],
                           failures=self.failures)

    def fail(self, error):
        with self.lock:
            self.failures.append(str(error))

    def count(self, name):
        with self.lock:
            self.counts[name] += 1

    def odom(self, message):
        try:
            position, quaternion = pose_values(message.pose.pose)
            row = dict(stamp=message.header.stamp.to_sec(),
                       stamp_ns=message.header.stamp.to_nsec(), seq=message.header.seq,
                       frame_id=message.header.frame_id, child_frame_id=message.child_frame_id,
                       position=position, quaternion=quaternion,
                       linear_velocity=xyz(message.twist.twist.linear),
                       angular_velocity=xyz(message.twist.twist.angular),
                       pose_covariance=list(message.pose.covariance),
                       twist_covariance=list(message.twist.covariance))
            with self.lock:
                self.odometry.append(row)
            require(row['frame_id'] == 'lio_map', 'unexpected odometry parent frame')
            require(row['child_frame_id'] == 'lio_imu', 'unexpected odometry child frame')
            require(all(math.isfinite(v) for v in position + quaternion), 'non-finite LIO pose')
            require(sum(v*v for v in quaternion) > 1e-12, 'zero LIO quaternion')
        except Exception as error:
            self.fail('odometry callback: ' + str(error))

    def cloud(self, message):
        with self.lock:
            self.clouds.append(dict(stamp=message.header.stamp.to_sec(),
                                    frame_id=message.header.frame_id,
                                    points=message.width * message.height))
        if message.header.frame_id != 'lio_map':
            self.fail('registered cloud is not in lio_map')

    def path(self, message):
        with self.lock:
            self.paths.append(dict(stamp=message.header.stamp.to_sec(),
                                   frame_id=message.header.frame_id, poses=len(message.poses)))
        if message.header.frame_id != 'lio_map':
            self.fail('estimated path is not in lio_map')

    def start(self, name, command):
        self.report['commands'][name] = command
        self.logs[name] = (self.output / (name + '.log')).open('w')
        self.processes[name] = subprocess.Popen(
            command, stdout=self.logs[name], stderr=subprocess.STDOUT, start_new_session=True)

    def poll(self):
        require(time.monotonic() < self.work_deadline, 'replay wall-clock deadline exceeded')
        require(not self.failures, 'a ROS callback reported invalid output')
        mapping = self.processes['mapping']
        require(mapping.poll() is None, 'mapping exited unexpectedly: %s' % mapping.returncode)

    def check_graph(self, require_outputs=False):
        publishers, subscriptions, services = self.master.getSystemState()
        all_nodes = {node for _, nodes in publishers + subscriptions + services for node in nodes}
        gazebo_nodes = sorted(node for node in all_nodes if node.startswith('/gazebo'))
        model_services = sorted(service for service, _ in services
                                if service.endswith('/get_model_state'))
        truth_topics = sorted(topic for topic, _ in publishers
                              if topic.startswith('/gazebo/') or '/ground_truth/' in topic)
        output_owners = {topic: sorted(nodes) for topic, nodes in publishers
                         if topic in OUTPUT_TOPICS}
        lio_nodes = set(output_owners.get('/lio/odom', []))
        inputs = {node: sorted(topic for topic, nodes in subscriptions if node in nodes)
                  for node in sorted(lio_nodes)}
        snapshot = dict(wall=time.monotonic() - self.started, nodes=sorted(all_nodes),
                        gazebo_nodes=gazebo_nodes, get_model_state_services=model_services,
                        truth_topics=truth_topics, output_publishers=output_owners, lio_inputs=inputs)
        self.report['graph_snapshots'].append(snapshot)
        require(not gazebo_nodes and not model_services and not truth_topics,
                'Gazebo or ground-truth interfaces are present in the replay graph')
        for node, topics in inputs.items():
            require(set(topics) <= INPUT_TOPICS, 'unexpected LIO inputs: %s %s' % (node, topics))
        if require_outputs:
            require(len(lio_nodes) == 1, 'expected exactly one LIO odometry publisher')
            owner = next(iter(lio_nodes))
            require(all(output_owners.get(topic) == [owner] for topic in OUTPUT_TOPICS),
                    'LIO output topics do not share one unique publisher')
            require(set(inputs[owner]) == INPUT_TOPICS,
                    'LIO must subscribe to lidar, IMU and the playback clock only')
            self.report['lio_sensor_only_graph'] = True

    def check_bag(self):
        with rosbag.Bag(str(self.args.bag), 'r') as bag:
            topics = bag.get_type_and_topic_info().topics
            self.report['bag_topics'] = {
                name: dict(type=info.msg_type, messages=info.message_count)
                for name, info in topics.items()}
            self.report['bag_duration_seconds'] = bag.get_end_time() - bag.get_start_time()
        require(set(topics) <= INPUT_TOPICS, 'bag contains topics beyond clock/lidar/IMU')
        for name, msg_type in (('/livox/lidar', 'sensor_msgs/PointCloud2'),
                               ('/livox/imu', 'sensor_msgs/Imu')):
            require(name in topics and topics[name].msg_type == msg_type
                    and topics[name].message_count > 0, 'missing or invalid sensor topic: ' + name)
        self.report['bag_sensor_only'] = True

    def stop(self, name, grace):
        process = self.processes.get(name)
        if process is None:
            return
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=max(.01, min(grace, self.deadline - time.monotonic())))
            except subprocess.TimeoutExpired:
                self.fail(name + ' did not stop within %.1f seconds after SIGINT' % grace)
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait(timeout=1)
        self.report[name + '_returncode'] = process.returncode
        if process.returncode != 0:
            self.fail('%s exited with return code %s' % (name, process.returncode))

    def validate_outputs(self):
        with self.lock:
            odometry, clouds, paths = list(self.odometry), list(self.clouds), list(self.paths)
            self.report['input_message_counts'] = dict(self.counts)
        valid_positions = [row['position'] for row in odometry
                           if all(math.isfinite(v) for v in row['position'])]
        displacement = max((math.hypot(p[0] - valid_positions[0][0],
                                       p[1] - valid_positions[0][1])
                            for p in valid_positions), default=0)
        self.report.update(odom_count=len(odometry), cloud_count=len(clouds), path_count=len(paths),
                           max_displacement_xy_m=displacement)
        checks = dict(odometry_received=bool(odometry),
                      finite_lio_map_poses=bool(odometry) and all(
                          row['frame_id'] == 'lio_map'
                          and all(math.isfinite(v) for v in row['position'] + row['quaternion'])
                          and sum(v*v for v in row['quaternion']) > 1e-12
                          for row in odometry),
                      registered_cloud_nonempty=any(row['points'] > 0 for row in clouds),
                      path_nonempty=any(row['poses'] > 0 for row in paths),
                      estimated_motion_over_0_3m=displacement > .3)
        pcds = []
        for path in sorted(self.maps.rglob('*.pcd')):
            try:
                info = pcd_info(path)
                stat = path.stat()
                info['updated_this_run'] = self.initial_maps.get(str(path)) != (stat.st_mtime_ns, stat.st_size)
                pcds.append(info)
            except Exception as error:
                self.fail('PCD validation: %s: %s' % (path, error))
        self.report['pcd_files'] = pcds
        final_map = str(self.maps / self.map_name)
        checks['nonempty_new_final_pcd'] = any(
            info['path'] == final_map and info['valid'] and info['updated_this_run'] for info in pcds)
        self.report['checks'] = checks
        for name, passed in checks.items():
            if not passed:
                self.fail('output check failed: ' + name)
        (self.output / 'odometry.json').write_text(json.dumps(odometry, indent=2))
        (self.output / 'clouds.json').write_text(json.dumps(clouds, indent=2))
        (self.output / 'paths.json').write_text(json.dumps(paths, indent=2))

    def run(self):
        try:
            self.check_bag()
            self.initial_maps = {str(p): (p.stat().st_mtime_ns, p.stat().st_size)
                                 for p in self.maps.rglob('*.pcd')}
            # ROOT=/lio/src/super_lio/，因此三层上溯后定位 /result/maps。
            map_dir = '../../..' + str(self.maps)
            self.start('mapping', ['roslaunch', 'aliengo_super_lio', 'mapping.launch',
                                   'rviz:=false', 'publish_tf:=false', 'map_dir:=' + map_dir])
            while not rosgraph.is_master_online():
                self.poll()
                time.sleep(.05)
            self.master = rosgraph.Master('/super_lio_replay')
            # master 先于 launch 参数和节点就绪；等 LIO 注册后再读取仿真时间参数。
            while not any(topic == '/lio/odom' and nodes
                          for topic, nodes in self.master.getSystemState()[0]):
                self.poll()
                time.sleep(.05)
            rospy.init_node('super_lio_replay', disable_signals=True)
            self.report['use_sim_time'] = rospy.get_param('/use_sim_time', False)
            require(self.report['use_sim_time'], 'ROS is not using simulation time')
            self.map_name = rospy.get_param('/lio/map/map_name')
            self.master = rosgraph.Master(rospy.get_name())
            self.subscribers = [
                rospy.Subscriber('/lio/odom', Odometry, self.odom, queue_size=100),
                rospy.Subscriber('/lio/cloud_world', PointCloud2, self.cloud, queue_size=2),
                rospy.Subscriber('/lio/path', RosPath, self.path, queue_size=2),
                rospy.Subscriber('/clock', Clock, lambda message: self.count('clock'), queue_size=1),
                rospy.Subscriber('/livox/lidar', PointCloud2, lambda message: self.count('lidar'), queue_size=1),
                rospy.Subscriber('/livox/imu', Imu, lambda message: self.count('imu'), queue_size=10)]
            self.check_graph()
            self.start('playback', ['rosbag', 'play', '--clock', '--hz=500', '--delay=1',
                                    str(self.args.bag), '--topics', '/livox/lidar', '/livox/imu'])
            next_graph = time.monotonic()
            while self.processes['playback'].poll() is None:
                self.poll()
                if time.monotonic() >= next_graph:
                    self.check_graph()
                    next_graph = time.monotonic() + .5
                time.sleep(.05)
            require(self.processes['playback'].returncode == 0, 'rosbag playback failed')
            drain_until = time.monotonic() + .5
            while time.monotonic() < drain_until:
                self.poll()
                time.sleep(.05)  # bag 播完后 /clock 停止，不能使用 rospy.sleep。
            self.check_graph(require_outputs=True)
        except BaseException as error:
            self.fail('%s: %s' % (type(error).__name__, error))
            self.report['traceback'] = traceback.format_exc()
        finally:
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            signal.signal(signal.SIGTERM, signal.SIG_IGN)
            for name, grace in (('playback', 1), ('mapping', 10)):
                try:
                    self.stop(name, grace)
                except Exception as error:
                    self.fail('shutdown %s: %s' % (name, error))
            if rospy.core.is_initialized():
                rospy.signal_shutdown('sensor-only replay finished')
            try:
                self.validate_outputs()
            except Exception as error:
                self.fail('output validation: ' + str(error))
            for log in self.logs.values():
                log.close()
            self.report['wall_seconds'] = time.monotonic() - self.started
            self.report['result'] = 'FAIL' if self.failures else 'PASS'
            (self.output / 'summary.json').write_text(json.dumps(self.report, indent=2))
        print('Super-LIO replay:', self.report['result'], flush=True)
        return 0 if self.report['result'] == 'PASS' else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, default=Path('/input/sensors.bag'))
    parser.add_argument('--output', type=Path, default=Path('/result'))
    args = parser.parse_args()

    def interrupted(signum, frame):
        raise KeyboardInterrupt('signal %s' % signum)

    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    return Replay(args).run()


if __name__ == '__main__':
    raise SystemExit(main())
