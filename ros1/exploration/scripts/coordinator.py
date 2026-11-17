#!/usr/bin/env python3
"""Follow official GBPlanner paths; input and command watches use wall time.

Spatial acceptance belongs to upstream GBPlanner. Terrain support is diagnostic;
this adapter adds no root, footprint, obstacle or active-path map validator.
"""
import json
import math
import threading
import time

import rospy
from geometry_msgs.msg import Pose, PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from planner_msgs.srv import planner_srv, planner_srvRequest
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBoolResponse, Trigger

from navigation_core import Freshness, Session, Waypoint, quaternion_yaw, trim_path


def pose_waypoint(pose):
    p, q = pose.position, pose.orientation
    if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
        raise ValueError('nonfinite odometry/path position')
    return Waypoint(p.x, p.y, p.z, quaternion_yaw(q.x, q.y, q.z, q.w))


def ros_pose(point):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = point.x, point.y, point.z
    pose.orientation.z, pose.orientation.w = math.sin(point.yaw/2), math.cos(point.yaw/2)
    return pose


def _invalid_constant(value):
    raise ValueError('nonfinite JSON constant: '+value)


def _number(value, minimum=0.):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < minimum:
        raise ValueError('invalid finite number')
    return float(value)


def _sources(value, names):
    if not isinstance(value, dict):
        raise ValueError('sources must be an object')
    stamps = {}
    for name in names:
        source = value[name]
        if not isinstance(source, dict) or type(source['count']) is not int or source['count'] < 0:
            raise ValueError('invalid source count')
        stamps[name] = _number(source['stamp'])
    return stamps


def terrain_status(payload):
    value = json.loads(payload, parse_constant=_invalid_constant)
    if not isinstance(value, dict):
        raise ValueError('terrain status must be an object')
    stamp = _number(value['stamp'])
    sources = _sources(value['sources'], ('lidar', 'camera'))
    if (not isinstance(value['frame_id'], str) or not value['frame_id']
            or type(value['footprint_known']) is not bool or not isinstance(value['reason'], str)
            or type(value['cells']) is not int or value['cells'] < 0):
        raise ValueError('invalid terrain status fields')
    if value['floor_z'] is not None:
        _number(value['floor_z'], -float('inf'))
    return value, stamp, sources


def map_status(payload):
    value = json.loads(payload, parse_constant=_invalid_constant)
    if not isinstance(value, dict):
        raise ValueError('map status must be an object')
    _number(value['stamp'])
    sources = _sources(value['sources'], ('mid360_link', 'd435_depth_optical_frame'))
    return value, sources


def controller_status(payload):
    value = json.loads(payload, parse_constant=_invalid_constant)
    if not isinstance(value, dict) or value['mode'] not in ('passive', 'stand', 'manual', 'autonomy'):
        raise ValueError('invalid controller mode')
    if (type(value['stand_ready']) is not bool or type(value['command_valid']) is not bool
            or not isinstance(value['command_reason'], str)):
        raise ValueError('invalid controller status fields')
    _number(value['command_stamp'])
    for name in ('vx', 'vy', 'wz'):
        _number(value[name], -float('inf'))
    return value


class Coordinator:
    def __init__(self):
        self.lock = threading.RLock()
        self.session = Session(horizon=float(rospy.get_param('~execution_horizon', .6)))
        self.require_clock = bool(rospy.get_param('~require_clock',
                                                  rospy.get_param('/use_sim_time', True)))
        self.progress = {name: Freshness() for name in
                         ('clock', 'odom', 'terrain', 'lidar', 'camera', 'map_lidar', 'map_camera')
                         if name != 'clock' or self.require_clock}
        self.pose = self.terrain = self.map_status = None
        self.control, self.control_wall = {}, 0.
        self.rpc_busy = set()
        self.stop_event = threading.Event()
        self.enabling = False
        self.mode_stop_pending = False
        self.mode_stop_thread = None
        self.stop_revoke = False
        self.last_mode_action_wall = 0.
        self.planned, self.failures = 0, 0
        self.max_plans = int(rospy.get_param('~max_completed_paths', 0))
        self.wall_gap = float(rospy.get_param('~max_input_wall_gap', 1.5))
        self.plan_timeout = float(rospy.get_param('~planning_timeout', 15.))
        self.frame = 'lio_map'
        self.command_pub = rospy.Publisher('/exploration/cmd_vel', TwistStamped, queue_size=1)
        self.path_pub = rospy.Publisher('/exploration/path', Path, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher('/exploration/status', String, queue_size=1, latch=True)
        self.subscribers = [
            rospy.Subscriber('/clock', Clock, self.clock, queue_size=1),
            rospy.Subscriber('/exploration/base_odom', Odometry, self.odom, queue_size=1),
            rospy.Subscriber('/exploration/terrain_status', String, self.terrain_callback, queue_size=1),
            rospy.Subscriber('/exploration/map_status', String, self.map_callback, queue_size=1),
            rospy.Subscriber('/aliengo/controller/status', String, self.control_callback, queue_size=1)]
        self.enable_service = rospy.Service('/exploration/enable', SetBool, self.enable)
        rospy.on_shutdown(self.shutdown)

    def clock(self, message):
        if not self.require_clock:
            return
        now, wall = message.clock.to_sec(), time.monotonic()
        with self.lock:
            previous = self.progress['clock'].stamp
            if previous is not None and now < previous:
                self.session.cancel('ROS clock reset; restart mapping before re-enabling')
                self.enabling = False
                self.publish_command(0., 0.)
                self.stop_mode()
            self.progress['clock'].observe(now, wall)

    def odom(self, message):
        with self.lock:
            try:
                if message.header.frame_id != self.frame or message.child_frame_id != 'base':
                    raise ValueError('odometry frame must be lio_map -> base')
                point = pose_waypoint(message.pose.pose)
                if self.progress['odom'].observe(message.header.stamp.to_sec(), time.monotonic()):
                    self.pose = point
            except ValueError as error:
                self.session.cancel(str(error))
                self.enabling = False
                self.publish_command(0., 0.)
                self.stop_mode()

    def terrain_callback(self, message):
        try:
            value, stamp, sources = terrain_status(message.data)
        except (ValueError, TypeError, KeyError):
            return
        with self.lock:
            wall = time.monotonic()
            # Reject the whole status before changing any liveness or state.
            if not self.progress['terrain'].observe(stamp, wall):
                return
            self.terrain = value
            for name, stamp in sources.items():
                self.progress[name].observe(stamp, wall)

    def map_callback(self, message):
        try:
            value, sources = map_status(message.data)
        except (ValueError, TypeError, KeyError):
            return
        with self.lock:
            self.map_status = value
            wall = time.monotonic()
            for name, frame in (('lidar', 'mid360_link'), ('camera', 'd435_depth_optical_frame')):
                self.progress['map_'+name].observe(sources[frame], wall)

    def control_callback(self, message):
        try:
            value = controller_status(message.data)
        except (ValueError, TypeError, KeyError):
            return
        with self.lock:
            self.control, self.control_wall = value, time.monotonic()

    def healthy(self, now, wall, during_plan=False):
        if self.pose is None or self.terrain is None:
            return False, 'waiting for LIO and mapping status'
        if self.terrain.get('frame_id') != self.frame:
            return False, 'terrain frame mismatch'
        for name, stream in self.progress.items():
            # The planner and TSDF share a ROS thread. Planning commands zero;
            # integration must catch up before any returned path can be used.
            if during_plan and name.startswith('map_'):
                continue
            gap = .5 if name == 'clock' else self.wall_gap
            if not stream.fresh(now, wall, ros_age=1.5, wall_gap=gap):
                return False, name+' stopped or stale'
        if wall-self.control_wall > 1.5:
            return False, 'controller status stopped'
        return True, ''

    def require_enable(self, token):
        # Caller holds lock; rechecked inside the RPC worker after service wait.
        if (token != self.session.generation or not self.enabling
                or self.mode_stop_pending or self.stop_event.is_set()):
            raise ValueError('enable cancelled')
        healthy, reason = self.healthy(rospy.Time.now().to_sec(), time.monotonic())
        if not healthy:
            raise ValueError(reason)

    def rpc(self, key, service, kind, request=None, timeout=3., token=None):
        with self.lock:
            if token is not None:
                self.require_enable(token)
            if key in self.rpc_busy:
                raise RuntimeError(key+' RPC still outstanding')
            self.rpc_busy.add(key)
        result, finished = {}, threading.Event()

        def worker():
            try:
                rospy.wait_for_service(service, timeout=min(timeout, 2.))
                proxy = rospy.ServiceProxy(service, kind)
                with self.lock:
                    if token is not None:
                        self.require_enable(token)
                    if key in ('stand', 'mode'):
                        self.last_mode_action_wall = time.monotonic()
                # Never hold the lifecycle lock while a remote service blocks.
                # A cancellation during this call persists until a stop RPC
                # follows it and safe controller status is acknowledged.
                try:
                    result['value'] = proxy() if request is None else proxy(request)
                finally:
                    proxy.close()
            except Exception as error:
                result['error'] = error
            finally:
                with self.lock:
                    self.rpc_busy.discard(key)
                    if key in ('stand', 'mode'):
                        self.last_mode_action_wall = time.monotonic()
                finished.set()
        threading.Thread(target=worker, daemon=True).start()
        if not finished.wait(timeout):
            raise RuntimeError(key+' wall timeout')
        if 'error' in result:
            raise RuntimeError(str(result['error']))
        return result['value']

    def enable(self, request):
        if not request.data:
            with self.lock:
                self.session.cancel('operator stop')
                self.enabling = False
                self.publish_command(0., 0.)
                self.stop_mode()
            return SetBoolResponse(True, 'stopped; cancelled paths will not resume')
        with self.lock:
            if self.session.enabled:
                return SetBoolResponse(True, 'already enabled')
            if self.enabling or self.mode_stop_pending or self.stop_event.is_set():
                return SetBoolResponse(False, 'enable or controller stop still pending')
            ok, reason = self.healthy(rospy.Time.now().to_sec(), time.monotonic())
            if not ok:
                return SetBoolResponse(False, reason)
            self.session.generation += 1
            self.session.state, self.session.reason = 'enabling', ''
            self.enabling = True
            token = self.session.generation
        try:
            stand = self.rpc('stand', '/aliengo/controller/stand', Trigger, token=token)
            if not stand.success:
                raise ValueError(stand.message)
            stand_completed_wall = time.monotonic()
            deadline = time.monotonic()+15
            while time.monotonic() < deadline:
                with self.lock:
                    self.require_enable(token)
                    ready = (self.control_wall > stand_completed_wall
                             and time.monotonic()-self.control_wall < 1.5
                             and self.control.get('mode') == 'stand'
                             and self.control.get('stand_ready', False))
                if ready:
                    break
                time.sleep(.05)
            else:
                raise ValueError('controller did not complete standing')
            response = self.rpc('mode', '/aliengo/controller/autonomy', SetBool, True, token=token)
            if not response.success:
                raise ValueError(response.message)
            deadline = time.monotonic()+3
            while time.monotonic() < deadline:
                with self.lock:
                    self.require_enable(token)
                    if self.control.get('mode') == 'autonomy':
                        healthy, reason = self.healthy(rospy.Time.now().to_sec(), time.monotonic())
                        if not healthy:
                            raise ValueError(reason)
                        self.enabling = False
                        self.session.enable()
                        self.failures = 0
                        return SetBoolResponse(True, 'autonomous planning enabled')
                time.sleep(.05)
            raise ValueError('autonomy mode was not acknowledged')
        except (ValueError, RuntimeError) as error:
            with self.lock:
                # A late enable worker cannot cancel a replacement generation.
                if token == self.session.generation:
                    self.session.cancel(str(error))
                    self.enabling = False
                    self.publish_command(0., 0.)
                    self.stop_mode()
            return SetBoolResponse(False, str(error))

    def stop_mode(self):
        """Persist stop intent across busy/timed-out mode calls, off the 20Hz loop."""
        with self.lock:
            self.mode_stop_pending = True
            self.stop_revoke = (self.stop_revoke or self.enabling
                                or bool(self.rpc_busy.intersection(('stand', 'mode')))
                                or self.control.get('mode') == 'autonomy'
                                or self.last_mode_action_wall >= self.control_wall)
            if self.mode_stop_thread is not None and self.mode_stop_thread.is_alive():
                return
            self.mode_stop_thread = threading.Thread(target=self._stop_mode_worker, daemon=True)
            self.mode_stop_thread.start()

    def _stop_mode_worker(self):
        acknowledged_after = 0.
        while not rospy.is_shutdown():
            with self.lock:
                if not self.mode_stop_pending:
                    self.mode_stop_thread = None
                    return
                busy = bool(self.rpc_busy.intersection(('stand', 'mode')))
                fresh = time.monotonic()-self.control_wall <= 1.5
                safe = self.control.get('mode') in ('passive', 'manual', 'stand')
                if (not busy and not self.stop_revoke and fresh and safe
                        and self.control_wall > acknowledged_after):
                    self.mode_stop_pending = False
                    self.mode_stop_thread = None
                    return
                retry = not busy and (self.stop_revoke or not safe)
            if retry:
                try:
                    response = self.rpc('mode', '/aliengo/controller/autonomy', SetBool, False, timeout=2.)
                    with self.lock:
                        if response.success:
                            self.stop_revoke = False
                            acknowledged_after = time.monotonic()
                except RuntimeError as error:
                    rospy.logwarn_throttle(2., 'Autonomy stop pending: %s', error)
            # A controller may enter autonomy after an earlier status report;
            # keep the intent until a post-stop safe-mode report is observed.
            with self.lock:
                if self.control.get('mode') == 'autonomy' and self.control_wall > acknowledged_after:
                    self.stop_revoke = True
            time.sleep(.05)

    def plan(self, token, start):
        try:
            request = planner_srvRequest()
            request.header.frame_id, request.header.stamp = self.frame, rospy.Time.now()
            request.root_pose, request.bound_mode = ros_pose(start), request.kExactBound
            response = self.rpc('planner', '/exploration/plan_raw', planner_srv,
                                request, self.plan_timeout)
            if not response.path or len(response.path) > 2000:
                raise ValueError('official planner returned an empty or oversized path')
            points = [start]+[pose_waypoint(pose) for pose in response.path]
            points = trim_path(points, self.session.horizon)
            deadline = time.monotonic()+3
            while time.monotonic() < deadline:
                with self.lock:
                    if not self.session.enabled or token != self.session.generation:
                        return
                    ok, reason = self.healthy(rospy.Time.now().to_sec(), time.monotonic())
                if ok:
                    break
                time.sleep(.05)
            if not ok:
                raise ValueError(reason)
            with self.lock:
                if self.session.accept_path(token, points, rospy.Time.now().to_sec(), time.monotonic()):
                    self.planned += 1
                    self.failures = 0
                    path = Path()
                    path.header.frame_id, path.header.stamp, path.header.seq = self.frame, rospy.Time.now(), token
                    for point in self.session.controller.path.points:
                        stamped = PoseStamped()
                        stamped.header, stamped.pose = path.header, ros_pose(point)
                        path.poses.append(stamped)
                    self.path_pub.publish(path)
        except (ValueError, RuntimeError) as error:
            with self.lock:
                if token != self.session.generation or not self.session.enabled:
                    return
                self.failures += 1
                if self.failures >= 3:
                    self.session.cancel('official planner path unavailable: '+str(error))
                else:
                    self.session.state, self.session.reason = 'ready', str(error)
            rospy.logwarn('Planner request %s: %s', token, error)

    def publish_command(self, linear, angular, token=None, calculated_wall=None):
        # The generation check and publisher call are one critical section with
        # stop/cancel. Never stamp a pre-cancellation velocity as a new command.
        with self.lock:
            if (not self.session.enabled or self.session.state != 'tracking'
                    or self.mode_stop_pending or self.stop_event.is_set()
                    or self.control.get('mode') != 'autonomy'
                    or (token is not None and token != self.session.generation)
                    or not all(math.isfinite(v) for v in (linear, angular))):
                linear = angular = 0.
            elif linear != 0. or angular != 0.:
                if calculated_wall is None:
                    calculated_wall = self.session.last_tick_wall
                wall = time.monotonic()
                if (not isinstance(calculated_wall, (int, float))
                        or not math.isfinite(calculated_wall)
                        or not 0. <= wall-calculated_wall <= .4):
                    self.session.cancel('expired command calculation')
                    self.enabling = False
                    linear = angular = 0.
                    self.stop_mode()
            message = TwistStamped()
            message.header.frame_id, message.header.stamp = 'base', rospy.Time.now()
            message.twist.linear.x, message.twist.angular.z = linear, angular
            self.command_pub.publish(message)
            return linear, angular

    def run(self):
        was_enabled = False
        while not self.stop_event.is_set() and not rospy.is_shutdown():
            now, wall = rospy.Time.now().to_sec(), time.monotonic()
            plan_job = None
            with self.lock:
                healthy, reason = self.healthy(now, wall, during_plan=self.session.state == 'planning')
                if self.session.enabled and self.control.get('mode') != 'autonomy':
                    healthy, reason = False, 'controller left autonomy mode'
                pose = self.pose or Waypoint(0, 0, 0, 0)
                linear, angular, _ = self.session.tick((pose.x, pose.y, pose.yaw), now, wall, healthy, reason)
                if self.max_plans and self.session.completed >= self.max_plans:
                    self.session.cancel('configured exploration run completed')
                    linear = angular = 0.
                token = self.session.begin_plan() if healthy else None
                if token is not None:
                    plan_job = (token, pose)
                enabled = self.session.enabled
                command_token = self.session.generation
                status = dict(state=self.session.state, enabled=enabled,
                              reason=self.session.reason or reason, inputs_ready=healthy,
                              completed_paths=self.session.completed, planned_paths=self.planned,
                              generation=self.session.generation,
                              progress_m=self.session.controller.progress,
                              pose=dict(x=pose.x, y=pose.y, z=pose.z, yaw=pose.yaw),
                              stamp=now, command=dict(vx=linear, wz=angular))
            linear, angular = self.publish_command(linear, angular, command_token, wall)
            status['command'] = dict(vx=linear, wz=angular)
            self.status_pub.publish(String(data=json.dumps(status, allow_nan=False)))
            if was_enabled and not enabled:
                self.stop_mode()
            was_enabled = enabled
            if plan_job:
                threading.Thread(target=self.plan, args=plan_job, daemon=True).start()
            time.sleep(.05)

    def shutdown(self):
        with self.lock:
            self.session.cancel('coordinator shutdown')
            self.enabling = False
            self.stop_event.set()
        for _ in range(3):
            self.publish_command(0., 0.)
        self.stop_mode()


if __name__ == '__main__':
    rospy.init_node('exploration_coordinator')
    Coordinator().run()
