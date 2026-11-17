#!/usr/bin/env python3
"""Real exploration acceptance: measured warmup, repeated plans, paused-clock stop.

Gazebo poses are queried exclusively by this evaluator. They are never published
or passed to mapping/planning/control. See summary.json for measured outcomes.
"""
import argparse
import json
import math
import os
from pathlib import Path
import pty
import signal
import threading
import time
import traceback
import xmlrpc.client

import numpy as np
import rosgraph
import rospy
import tf2_ros
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry, Path as RosPath
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Empty, SetBool

from super_lio_trial import Trial, angles, pose_data, rigid


class ExplorationTrial(Trial):
    def __init__(self, args):
        super().__init__(args)
        self.deadline = self.started + args.wall_seconds
        self.work_deadline = self.deadline - 40
        self.state.update(terrain={}, backend={}, exploration={}, controller={}, camera_count=0)
        self.status_rows, self.commands, self.accepted_paths = [], [], []
        self.suspended_pid = None
        self.report['scope'] = 'flat Depot simulation; fixed sensor mounts; real planner/map/Guide'

    def json_status(self, key, message):
        try:
            value = json.loads(message.data)
            with self.lock:
                self.state[key] = value
                if key == 'exploration':
                    self.status_rows.append(dict(wall=time.monotonic()-self.started,
                                                 phase=self.phase_name, **value))
        except (ValueError, TypeError):
            pass

    def command(self, message):
        with self.lock:
            self.commands.append(dict(wall=time.monotonic()-self.started, phase=self.phase_name,
                                      stamp=message.header.stamp.to_sec(),
                                      vx=message.twist.linear.x, wz=message.twist.angular.z))

    def exploration_path(self, message):
        with self.lock:
            self.accepted_paths.append(dict(wall=time.monotonic()-self.started,
                                            stamp=message.header.stamp.to_sec(),
                                            sequence=message.header.seq,
                                            frame=message.header.frame_id,
                                            poses=[pose_data(p.pose) for p in message.poses]))

    def camera(self, message):
        with self.lock:
            self.state['camera_count'] += 1

    def odom(self, message):
        # This adapter is evaluated against Gazebo, never used as its input.
        try:
            if message.header.frame_id != 'lio_map' or message.child_frame_id != 'lio_imu':
                raise ValueError('unexpected current Super-LIO odometry frames')
            imu = pose_data(message.pose.pose)
            base = np.asarray(imu['matrix']) @ np.linalg.inv(self.base_imu)
            row = dict(stamp=message.header.stamp.to_sec(), stamp_ns=message.header.stamp.to_nsec(),
                       wall=time.monotonic()-self.started, phase=self.phase_name,
                       matrix=base.tolist(), position=base[:3, 3].tolist(), **angles(base))
            with self.lock:
                self.odometry.append(row)
                self.state['odom'] = row
        except Exception as error:
            with self.lock:
                self.state['error'] = 'evaluation odometry: '+str(error)

    def bounded(self, operation, timeout, label):
        result, done = [], threading.Event()

        def invoke():
            try:
                result.append(operation())
            except Exception as error:
                result.append(error)
            finally:
                done.set()

        threading.Thread(target=invoke, daemon=True).start()
        end = time.monotonic()+timeout
        while not done.wait(.02):
            self.tick()
            if time.monotonic() > end:
                raise RuntimeError('wall timeout: '+label)
        if isinstance(result[0], Exception):
            raise result[0]
        return result[0]

    def call(self, name, kind, value=None, timeout=20):
        def invoke():
            rospy.wait_for_service(name, timeout=5)
            proxy = rospy.ServiceProxy(name, kind)
            try:
                return proxy() if value is None else proxy(value)
            finally:
                proxy.close()
        return self.bounded(invoke, timeout, name)

    def execute(self):
        maps = self.output/'maps'
        (maps/'PCD').mkdir(parents=True, exist_ok=True)
        self.launch('launch', ['roslaunch', '--sigint-timeout=5', '--sigterm-timeout=2',
                              '/course/ros1/mid360/gazebo.launch', 'rname:=aliengo',
                              'world_file:=/course/ros1/worlds/depot.world', 'spawn_x:=-5',
                              'spawn_y:=0', 'spawn_z:=0.6', 'spawn_yaw:=0', 'paused:=false',
                              'sim_ground_truth_tf:=false', 'rviz:=false',
                              'd435_enabled:=true', 'd435_color_width:=640', 'd435_color_height:=360',
                              'd435_depth_width:=424', 'd435_depth_height:=240', 'd435_rate:=15',
                              'gui:='+str(self.args.gui).lower()])
        self.wait(lambda state: rosgraph.is_master_online() and rospy.get_param('/use_sim_time', False),
                  90, 'ROS master and simulation clock parameter')
        rospy.init_node('gbplanner_evaluation', disable_signals=True)
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(120))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.subscribers = [rospy.Subscriber('/clock', Clock, self.clock, queue_size=1),
                            rospy.Subscriber('/livox/lidar', PointCloud2, self.input_cloud, queue_size=1),
                            rospy.Subscriber('/livox/imu', Imu, self.input_imu, queue_size=100),
                            rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.camera, queue_size=1),
                            rospy.Subscriber('/lio/odom', Odometry, self.odom, queue_size=100),
                            rospy.Subscriber('/lio/cloud_world', PointCloud2, self.cloud, queue_size=1),
                            rospy.Subscriber('/exploration/cmd_vel', TwistStamped, self.command, queue_size=100),
                            rospy.Subscriber('/exploration/path', RosPath, self.exploration_path, queue_size=10)]
        for key, topic in [('terrain', '/exploration/terrain_status'),
                           ('backend', '/exploration/map_status'),
                           ('exploration', '/exploration/status'),
                           ('controller', '/aliengo/controller/status')]:
            self.subscribers.append(rospy.Subscriber(topic, String,
                                                     lambda message, name=key: self.json_status(name, message),
                                                     queue_size=10))
        self.query_thread = threading.Thread(target=self.query_truth, daemon=True)
        self.query_thread.start()
        self.wait(lambda s: s['sim'] and s['truth'] and s['lidar_count'] and s['camera_count'],
                  90, 'clock, robot, native lidar and D435')

        def extrinsic(state):
            try:
                t = self.tf_buffer.lookup_transform('base', 'mid360_imu_link', rospy.Time(0), rospy.Duration(0))
                p, q = t.transform.translation, t.transform.rotation
                self.base_imu = rigid([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
                return True
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                return False

        self.wait(extrinsic, 15, 'current URDF extrinsic')
        self.report['base_to_imu'] = self.base_imu.tolist()
        self.terminal, slave = pty.openpty()
        try:
            self.launch('controller', ['/work/devel/lib/unitree_guide/junior_ctrl'], stdin=slave)
        finally:
            os.close(slave)
        self.sim_wait(1)
        self.run_phase('stand', 8, b'2')
        self.check_pose = True
        self.launch('lio', ['roslaunch', '--sigint-timeout=5', '--sigterm-timeout=2',
                            '/course/ros1/exploration/exploration.launch',
                            'rviz:='+str(self.args.rviz).lower(),
                            'map_dir:='+os.path.relpath(maps, '/lio/src/super_lio')])
        self.wait(lambda s: s['odom'] and s['terrain'] and s['backend'] and s['exploration'],
                  90, 'Super-LIO, fusion, planner and command heartbeat')
        self.report['map_name'] = rospy.get_param('/lio/map/map_name')
        self.report['input_graph_start'] = self.input_graph()
        self.report['initial_terrain'] = self.snapshot()['terrain']
        self.report['checks']['default_disabled'] = (not self.snapshot()['exploration']['enabled']
                                                     and self.snapshot()['controller']['mode'] == 'stand')
        self.report['spatial_checks'] = 'pinned GBPlanner3 with oriented body/turn geometry fixes; original footprint and follower limits; no custom check_path service'
        initial = self.snapshot()['truth']
        self.run_phase('warmup_trot', 2, b'4')
        self.key(b'w')
        self.sim_wait(.1)
        self.run_phase('warmup_walk', self.args.seconds, b'w')
        self.run_phase('warmup_stop', 3, b' ')
        self.run_phase('warmup_stand', 3, b'2')
        state = self.snapshot()
        distance = float(np.linalg.norm(np.array(state['truth']['position'][:2])-initial['position'][:2]))
        self.report['warmup'] = dict(distance_m=distance, terrain=state['terrain'], backend=state['backend'])
        if not 1.5 <= distance <= 2.:
            raise RuntimeError('manual warmup outside accepted 1.5–2m: %.3fm'%distance)
        self.phase_name = 'autonomy'
        start = self.snapshot()
        reply = self.call('/exploration/enable', SetBool, True)
        self.report['enable'] = dict(success=reply.success, message=reply.message)
        if not reply.success:
            raise RuntimeError('measured warmup did not admit autonomy: '+reply.message)
        self.wait(lambda s: s['exploration'].get('enabled'), 3, 'enabled status acknowledgement')

        def completed(state):
            status = state['exploration']
            if not status.get('enabled'):
                raise RuntimeError('autonomy stopped before completion: '+status.get('reason', ''))
            displacement = np.linalg.norm(np.array(state['truth']['position'][:2])
                                          - start['truth']['position'][:2])
            return status.get('completed_paths', 0) >= self.args.plans and displacement >= 1.

        self.wait(completed, self.args.autonomy_wall_seconds, 'repeated autonomous path completion')
        final = self.snapshot()
        displacement = float(np.linalg.norm(np.array(final['truth']['position'][:2])-start['truth']['position'][:2]))
        self.report['autonomy'] = dict(displacement_m=displacement, start=start, end=final,
                                      accepted_paths=len(self.accepted_paths))
        checks = self.report['checks']
        checks['manual_warmup_measured'] = 1.5 <= distance <= 2.
        checks['repeated_real_paths'] = len(self.accepted_paths) >= self.args.plans
        checks['autonomous_motion'] = displacement >= 1.
        checks['both_sources_integrated'] = all(final['backend']['sources'][name]['count'] >
                                                start['backend']['sources'][name]['count']
                                                for name in ('mid360_link', 'd435_depth_optical_frame'))
        checks['map_growth'] = final['terrain']['cells'] > start['terrain']['cells']
        # Freeze only the command producer. Sensors, localization and Guide
        # continue, so this exercises the real wall-time command watchdog.
        self.phase_name = 'command_interruption'
        self.wait(lambda s: s['controller'].get('command_valid')
                  and abs(s['controller'].get('vx', 0))+abs(s['controller'].get('wz', 0)) > .03,
                  20, 'moving command before publisher interruption')
        interrupted_generation = self.snapshot()['exploration']['generation']
        interrupted_sequence = self.accepted_paths[-1]['sequence']

        def query_pid():
            uri = rosgraph.Master(rospy.get_name()).lookupNode('/exploration_coordinator')
            return xmlrpc.client.ServerProxy(uri).getPid(rospy.get_name())

        response = self.bounded(query_pid, 3, 'owned coordinator PID lookup')
        if response[0] != 1 or response[2] <= 1:
            raise RuntimeError('cannot locate owned coordinator process')
        self.suspended_pid = response[2]
        interrupted_at = time.monotonic()
        os.kill(self.suspended_pid, signal.SIGSTOP)
        self.wait(lambda s: not s['controller'].get('command_valid', True), 2, 'Guide command watchdog')
        checks['command_interruption_stop'] = True
        self.report['command_stop_wall_seconds'] = time.monotonic()-interrupted_at
        # Leave the command heartbeat interrupted before resuming its process.
        end = interrupted_at+1.
        while time.monotonic() < end:
            self.tick()
            time.sleep(.01)
        os.kill(self.suspended_pid, signal.SIGCONT)
        self.suspended_pid = None
        self.wait(lambda s: not s['exploration'].get('enabled', True), 3, 'cancel interrupted path')
        self.sim_wait(1)
        checks['command_recovery_no_resume'] = not self.snapshot()['exploration']['enabled']
        self.wait(lambda s: s['controller'].get('mode') == 'stand', 3, 'stand after interrupted command')
        # A new explicit request starts a new generation; no old path is reused.
        reply = self.call('/exploration/enable', SetBool, True)
        if not reply.success:
            raise RuntimeError('explicit re-enable after command interruption: '+reply.message)
        self.wait(lambda s: s['exploration'].get('enabled') and s['exploration'].get('state') == 'tracking'
                  and s['exploration']['generation'] > interrupted_generation
                  and self.accepted_paths[-1]['sequence'] > interrupted_sequence,
                  20, 'new plan before clock interruption')
        checks['new_path_generation_after_explicit_enable'] = True
        self.report['recovery'] = dict(interrupted_generation=interrupted_generation,
                                       new_generation=self.snapshot()['exploration']['generation'],
                                       interrupted_path_sequence=interrupted_sequence,
                                       new_path_sequence=self.accepted_paths[-1]['sequence'])
        # Pause physics and /clock while an active request/heartbeat exists.
        self.phase_name = 'paused_clock'
        paused_at = time.monotonic()
        self.call('/gazebo/pause_physics', Empty)
        self.wait(lambda s: not s['exploration'].get('enabled', True)
                  and not s['controller'].get('command_valid', True), 4, 'wall-time pause stop')
        stopped = self.snapshot()
        self.report['clock_stop_wall_seconds'] = time.monotonic()-paused_at
        checks['paused_clock_stop'] = not stopped['exploration']['enabled']
        self.call('/gazebo/unpause_physics', Empty)
        self.phase_name = 'resumed_clock'
        self.sim_wait(2)
        checks['no_automatic_resume'] = not self.snapshot()['exploration']['enabled']
        self.phase_name = 'operator_stop'
        self.wait(lambda s: s['controller'].get('mode') == 'stand', 3, 'stand after clock recovery')
        reply = self.call('/exploration/enable', SetBool, True)
        if not reply.success:
            raise RuntimeError('explicit re-enable before operator stop: '+reply.message)
        self.wait(lambda s: s['exploration'].get('enabled') and s['exploration'].get('state') == 'tracking'
                  and abs(s['controller'].get('vx', 0))+abs(s['controller'].get('wz', 0)) > .03,
                  20, 'moving command before explicit stop')
        stopped_at = time.monotonic()
        reply = self.call('/exploration/enable', SetBool, False)
        self.wait(lambda s: not s['exploration'].get('enabled', True)
                  and s['controller'].get('mode') == 'stand', 3, 'explicit stop acknowledgement')
        self.report['operator_stop_wall_seconds'] = time.monotonic()-stopped_at
        checks['explicit_stop'] = reply.success and not self.snapshot()['exploration']['enabled']
        self.report['input_graph_end'] = self.input_graph()
        for name, passed in checks.items():
            if not passed:
                self.fail('check failed: '+name)

    def finish(self):
        self.report['latest_state'] = self.snapshot()
        if self.suspended_pid is not None:
            try:
                os.kill(self.suspended_pid, signal.SIGCONT)
            except OSError as error:
                self.fail('resume suspended coordinator during cleanup: '+str(error))
            self.suspended_pid = None
        if rospy.core.is_initialized():
            try:
                self.call('/gazebo/unpause_physics', Empty, timeout=2)
                self.call('/exploration/enable', SetBool, False, timeout=2)
            except Exception:
                pass
        super().finish()
        (self.output/'exploration-status.json').write_text(json.dumps(self.status_rows, indent=2))
        (self.output/'commands.json').write_text(json.dumps(self.commands, indent=2))
        (self.output/'accepted-paths.json').write_text(json.dumps(self.accepted_paths, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('/result'))
    parser.add_argument('--seconds', type=float, default=35, help='manual warmup simulated seconds')
    parser.add_argument('--plans', type=int, default=3)
    parser.add_argument('--wall-seconds', type=float, default=650)
    parser.add_argument('--autonomy-wall-seconds', type=float, default=240)
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--rviz', action='store_true')
    parser.set_defaults(record_bag=False)
    args = parser.parse_args()
    trial = ExplorationTrial(args)

    def interrupted(signum, frame):
        raise KeyboardInterrupt('signal %s'%signum)

    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    try:
        trial.execute()
    except BaseException as error:
        trial.fail('%s: %s'%(type(error).__name__, error))
        trial.report['traceback'] = traceback.format_exc()
    finally:
        trial.finish()
    print('GBPlanner trial:', trial.report['result'], trial.report['failures'], flush=True)
    return 0 if trial.report['result'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
