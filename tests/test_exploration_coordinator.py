"""Coordinator boundary tests with actual ROS topics/services and no simulator.

Only external planner/controller behavior is provided by this harness. Run in a
Noetic environment with the built planner_msgs overlay; the host skips it.
"""
import importlib.util
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time
import unittest

SCRIPTS = Path(__file__).resolve().parents[1] / 'ros1/exploration/scripts'


def until(predicate, timeout=4.):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(.02)
    return False


@unittest.skipUnless(importlib.util.find_spec('rospy'), 'Requires Noetic and built planner_msgs')
class CoordinatorBoundaryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with socket.socket() as sock:
            sock.bind(('127.0.0.1', 0))
            port = sock.getsockname()[1]
        os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % port
        os.environ['ROS_HOSTNAME'] = '127.0.0.1'
        cls.master = subprocess.Popen(['roscore', '-p', str(port)], stdout=subprocess.DEVNULL,
                                      stderr=subprocess.DEVNULL)
        import rospy
        sys.path.insert(0, str(SCRIPTS))
        import coordinator
        cls.rospy, cls.module = rospy, coordinator
        rospy.init_node('coordinator_boundary_test', anonymous=True, disable_signals=True)
        cls.assert_ready = until(lambda: cls.master.poll() is None and cls._master_ready(), 5.)
        if not cls.assert_ready:
            raise RuntimeError('test ROS master did not start')

    @classmethod
    def _master_ready(cls):
        try:
            cls.rospy.get_master().getPid()
            return True
        except Exception:
            return False

    @classmethod
    def tearDownClass(cls):
        cls.rospy.signal_shutdown('boundary tests complete')
        cls.master.terminate()
        cls.master.wait(timeout=5.)

    def setUp(self):
        from geometry_msgs.msg import TwistStamped
        from planner_msgs.srv import planner_srv, planner_srvResponse
        from std_msgs.msg import String
        from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock
        self.SetBool = SetBool
        self.mode = 'stand'
        self.stand_calls, self.mode_calls, self.commands, self.errors = [], [], [], []
        self.hold_stand, self.stand_entered = threading.Event(), threading.Event()
        self.hold_mode, self.mode_entered = threading.Event(), threading.Event()
        self.hold_stand.set()
        self.hold_mode.set()
        self.feeding = True
        self.clock_feeding = self._testMethodName != 'test_system_time_tracks_without_clock_messages'
        self.rospy.set_param('~require_clock', self.clock_feeding)
        self.controller_feeding = True
        self.footprint_known = True
        self.planner_calls = 0
        self.feed_stop = threading.Event()
        self.publishers = [self.rospy.Publisher(name, kind, queue_size=5) for name, kind in (
            ('/clock', Clock), ('/exploration/base_odom', Odometry),
            ('/exploration/terrain_status', String), ('/exploration/map_status', String),
            ('/aliengo/controller/status', String))]
        self.command_sub = self.rospy.Subscriber('/exploration/cmd_vel', TwistStamped,
                                                 lambda message: self.commands.append(message), queue_size=100)
        def stand(request):
            self.stand_calls.append(time.monotonic())
            self.stand_entered.set()
            self.hold_stand.wait(5.)
            self.mode = 'stand'
            return TriggerResponse(True, 'standing')
        def mode(request):
            self.mode_calls.append(request.data)
            if request.data:
                self.mode_entered.set()
                self.hold_mode.wait(5.)
                self.mode = 'autonomy'
            elif self.mode == 'autonomy':
                self.mode = 'stand'
            return SetBoolResponse(True, 'mode accepted')
        def plan(request):
            self.planner_calls += 1
            result = planner_srvResponse()
            point = self.module.ros_pose(self.module.Waypoint(.5, 0., .43, 0.))
            result.path = [request.root_pose, point]
            return result
        self.services = [self.rospy.Service('/exploration/plan_raw', planner_srv, plan),
                         self.rospy.Service('/aliengo/controller/stand', Trigger, stand),
                         self.rospy.Service('/aliengo/controller/autonomy', SetBool, mode)]
        self.node = self.module.Coordinator()
        def run():
            try:
                self.node.run()
            except Exception as error:
                self.errors.append(error)
        self.runner = threading.Thread(target=run, daemon=True)
        self.runner.start()
        self.feeder = threading.Thread(target=self.feed, daemon=True)
        self.feeder.start()
        self.enable = self.rospy.ServiceProxy('/exploration/enable', SetBool)
        self.assertTrue(until(lambda: self.node.healthy(self.rospy.Time.now().to_sec(), time.monotonic())[0]))

    def feed(self):
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock
        from std_msgs.msg import String
        while not self.feed_stop.is_set():
            stamp = self.rospy.Time.now()
            if self.feeding:
                if self.clock_feeding:
                    self.publishers[0].publish(Clock(stamp))
                odom = Odometry()
                odom.header.frame_id, odom.header.stamp, odom.child_frame_id = 'lio_map', stamp, 'base'
                odom.pose.pose.position.z, odom.pose.pose.orientation.w = .43, 1.
                self.publishers[1].publish(odom)
                source = {'stamp': stamp.to_sec(), 'count': 1}
                terrain = dict(stamp=stamp.to_sec(), frame_id='lio_map',
                               floor_z=0. if self.footprint_known else None,
                               cells=100 if self.footprint_known else 0,
                               footprint_known=self.footprint_known, reason='observed support' if self.footprint_known else 'unknown footprint cell', sources={'lidar': source, 'camera': source})
                self.publishers[2].publish(String(json.dumps(terrain)))
                self.publishers[3].publish(String(json.dumps(dict(stamp=stamp.to_sec(), sources={
                    'mid360_link': source, 'd435_depth_optical_frame': source}))))
                if self.controller_feeding:
                    self.publishers[4].publish(String(json.dumps(dict(mode=self.mode, stand_ready=self.mode == 'stand',
                                                                 command_valid=False, command_reason='zero',
                                                                 command_stamp=stamp.to_sec(), vx=0., vy=0., wz=0.))))
            self.feed_stop.wait(.025)

    def tearDown(self):
        self.hold_stand.set()
        self.hold_mode.set()
        self.mode = 'passive'
        self.node.shutdown()
        self.runner.join(timeout=2.)
        until(lambda: not self.node.mode_stop_pending, 1.)
        self.feed_stop.set()
        self.feeder.join(timeout=1.)
        for service in self.services:
            service.shutdown()
        self.node.enable_service.shutdown()
        for subscription in self.node.subscribers+[self.command_sub]:
            subscription.unregister()
        for publisher in self.publishers:
            publisher.unregister()
        time.sleep(.05)

    def test_stop_during_held_stand_prevents_late_autonomy(self):
        self.hold_stand.clear()
        result = []
        caller = threading.Thread(target=lambda: result.append(self.enable(True)), daemon=True)
        caller.start()
        self.assertTrue(self.stand_entered.wait(3.))
        self.assertTrue(self.enable(False).success)
        self.hold_stand.set()
        caller.join(timeout=3.)
        self.assertTrue(result and not result[0].success)
        self.assertNotIn(True, self.mode_calls)

    def test_system_time_tracks_without_clock_messages(self):
        self.assertTrue(self.enable(True).success)
        self.assertTrue(until(lambda: any(m.twist.linear.x > 0 for m in self.commands)))
        self.assertNotIn('clock', self.node.progress)
        time.sleep(.6)
        self.assertTrue(self.node.session.enabled)

    def test_enable_waits_for_stand_acknowledgement_after_this_stand_rpc(self):
        self.assertTrue(until(lambda: self.node.control.get('stand_ready', False)))
        self.controller_feeding = False
        time.sleep(.075)
        result = []
        caller = threading.Thread(target=lambda: result.append(self.enable(True)), daemon=True)
        caller.start()
        try:
            self.assertTrue(self.stand_entered.wait(3.))
            self.assertFalse(self.mode_entered.wait(.2),
                             'cached readiness from before the stand RPC admitted autonomy')
        finally:
            self.controller_feeding = True
            caller.join(timeout=3.)
        self.assertTrue(result and result[0].success)

    def test_unknown_footprint_is_diagnostic_and_official_path_still_tracks(self):
        self.footprint_known = False
        self.assertTrue(until(lambda: self.node.terrain is not None
                              and self.node.terrain['footprint_known'] is False))
        result = self.enable(True)
        self.assertTrue(result.success, result.message)
        self.assertTrue(until(lambda: self.planner_calls > 0
                              and any(m.twist.linear.x > 0 for m in self.commands)))
        self.assertEqual(self.node.terrain['reason'], 'unknown footprint cell')
        self.assertIsNone(self.node.terrain['floor_z'])
        self.assertTrue(self.node.session.enabled)

    def test_stop_during_inflight_autonomy_retries_after_rpc_becomes_free(self):
        self.hold_mode.clear()
        result = []
        caller = threading.Thread(target=lambda: result.append(self.enable(True)), daemon=True)
        caller.start()
        self.assertTrue(self.mode_entered.wait(3.))
        # Let the client time out while the external service remains in flight.
        caller.join(timeout=3.5)
        self.assertTrue(result and not result[0].success)
        self.assertTrue(self.enable(False).success)
        self.hold_mode.set()
        self.assertTrue(until(lambda: False in self.mode_calls and self.mode != 'autonomy', 3.),
                        'late autonomy response escaped the stop intent')
        self.assertTrue(result and not result[0].success)

    def test_old_computed_command_after_stop_is_gated_to_zero(self):
        self.assertTrue(self.enable(True).success)
        self.assertTrue(until(lambda: any(m.twist.linear.x > 0 for m in self.commands)))
        self.assertTrue(self.enable(False).success)
        time.sleep(.1)
        self.commands.clear()
        self.node.publish_command(.1, .2)  # A run iteration computed this before cancellation.
        self.assertTrue(until(lambda: len(self.commands) > 0))
        self.assertTrue(all(m.twist.linear.x == 0 and m.twist.angular.z == 0 for m in self.commands),
                        'a command computed before cancellation was published after stop')

    def test_old_calculation_same_generation_cancels_before_fresh_stamping(self):
        self.assertTrue(self.enable(True).success)
        self.assertTrue(until(lambda: any(m.twist.linear.x > 0 for m in self.commands)))
        with self.node.lock:
            token = self.node.session.generation
        calculated_wall = time.monotonic()
        # Perception and the normal loop continue: only this calculation ages.
        time.sleep(.45)
        self.assertEqual(self.node.session.generation, token)
        command = self.node.publish_command(.1, .2, token, calculated_wall)
        self.assertEqual(command, (0., 0.))
        self.assertFalse(self.node.session.enabled)
        self.assertGreater(self.node.session.generation, token)
        self.assertIn('command', self.node.session.reason)
        time.sleep(.1)
        self.assertTrue(all(m.twist.angular.z != .2 for m in self.commands))

    def test_fresh_calculation_reaches_real_command_subscriber(self):
        self.assertTrue(self.enable(True).success)
        self.assertTrue(until(lambda: self.node.session.state == 'tracking'))
        token = self.node.session.generation
        command = self.node.publish_command(.1, .2, token, time.monotonic()-.2)
        self.assertEqual(command, (.1, .2))
        self.assertTrue(until(lambda: any(m.twist.angular.z == .2 for m in self.commands)))
        self.assertTrue(self.node.session.enabled)

    def test_future_calculation_time_cancels_instead_of_fresh_stamping(self):
        self.assertTrue(self.enable(True).success)
        self.assertTrue(until(lambda: self.node.session.state == 'tracking'))
        command = self.node.publish_command(.1, .2, self.node.session.generation, time.monotonic()+1.)
        self.assertEqual(command, (0., 0.))
        self.assertFalse(self.node.session.enabled)

    def test_nonfinite_calculation_time_cancels_instead_of_fresh_stamping(self):
        self.assertTrue(self.enable(True).success)
        self.assertTrue(until(lambda: self.node.session.state == 'tracking'))
        command = self.node.publish_command(.1, .2, self.node.session.generation, float('nan'))
        self.assertEqual(command, (0., 0.))
        self.assertFalse(self.node.session.enabled)

    def test_keyboard_passive_override_is_preserved_without_forcing_stand(self):
        self.assertTrue(self.enable(True).success)
        self.assertTrue(until(lambda: any(m.twist.linear.x > 0 for m in self.commands)))
        before = len(self.mode_calls)
        self.mode = 'passive'  # External keyboard/control-loop mode selection.
        self.assertTrue(until(lambda: not self.node.session.enabled))
        time.sleep(.2)
        self.assertEqual(self.mode, 'passive')
        self.assertEqual(len(self.mode_calls), before, 'stop changed the operator-selected mode')

    def test_malformed_complete_status_does_not_refresh_any_liveness(self):
        from std_msgs.msg import String
        self.feeding = False
        time.sleep(.1)
        walls = {name: stream.wall for name, stream in self.node.progress.items()}
        control_wall, control = self.node.control_wall, dict(self.node.control)
        terrain = dict(self.node.terrain)
        # All are valid JSON payloads, but none meets the complete wire schema.
        self.publishers[2].publish(String(json.dumps(dict(terrain, cells=True))))
        self.publishers[3].publish(String(json.dumps(dict(stamp=time.time(), sources={
            'mid360_link': dict(stamp=time.time(), count=1),
            'd435_depth_optical_frame': dict(stamp=float('nan'), count=1)}))))
        self.publishers[4].publish(String(json.dumps(dict(control, command_stamp=float('inf')))))
        time.sleep(.2)
        self.assertEqual(self.node.control_wall, control_wall)
        self.assertEqual(self.node.control, control)
        self.assertEqual(self.node.terrain, terrain)
        self.assertEqual({name: stream.wall for name, stream in self.node.progress.items()}, walls)
        self.assertFalse(self.errors)

    def test_malformed_terrain_keeps_node_alive_and_zero_heartbeat_running(self):
        from std_msgs.msg import String
        self.feeding = False
        time.sleep(.1)
        before = len(self.commands)
        self.publishers[2].publish(String('[]'))
        self.assertTrue(until(lambda: len(self.commands) >= before+5, 1.),
                        'malformed status killed command heartbeat')
        self.assertFalse(self.errors)
        self.assertTrue(all(m.twist.linear.x == 0 for m in self.commands[before:]))


if __name__ == '__main__':
    unittest.main()
