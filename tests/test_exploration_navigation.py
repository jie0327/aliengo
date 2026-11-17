"""Exercise autonomy cancellation, actual path following and advancing-input gates."""
import importlib.util
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / 'ros1/exploration/scripts'


def core():
    source = SCRIPTS / 'navigation_core.py'
    assert source.is_file(), 'Autonomy lifecycle and freshness implementation is missing'
    if str(SCRIPTS) not in sys.path:
        sys.path.insert(0, str(SCRIPTS))
    spec = importlib.util.spec_from_file_location('exploration_navigation_core', source)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class NavigationTests(unittest.TestCase):
    def setUp(self):
        self.n = core()

    def points(self, end=.5):
        return [self.n.Waypoint(0, 0, 0, 0), self.n.Waypoint(end, 0, 0, 0)]

    def active(self, points=None):
        session = self.n.Session()
        session.enable()
        token = session.begin_plan()
        self.assertTrue(session.accept_path(token, points or self.points(), 10., 100.))
        return session

    def test_repeated_stamps_do_not_refresh_wall_watchdog(self):
        stream = self.n.Freshness()
        self.assertFalse(stream.fresh(2., 10.))
        self.assertTrue(stream.observe(2., 10.))
        self.assertFalse(stream.observe(2., 10.7))
        self.assertFalse(stream.fresh(2., 11., wall_gap=.8))
        self.assertTrue(stream.observe(2.1, 11.1))
        self.assertTrue(stream.fresh(2.1, 11.2, wall_gap=.8))

    def test_future_old_nonfinite_and_regressive_inputs_rejected(self):
        stream = self.n.Freshness()
        self.assertFalse(stream.observe(float('nan'), 10.))
        stream.observe(2., 10.)
        self.assertFalse(stream.observe(1.9, 10.1))
        self.assertEqual(stream.stamp, 2.)
        self.assertFalse(stream.fresh(1., 10.2))
        self.assertFalse(stream.fresh(4., 10.2, ros_age=1.5))
        self.assertFalse(stream.fresh(2., 9.))

    def test_disabled_and_planning_sessions_command_zero(self):
        session = self.n.Session()
        self.assertEqual(session.tick((0, 0, 0), 10., 100.)[:2], (0., 0.))
        session.enable()
        self.assertIsNotNone(session.begin_plan())
        self.assertIsNone(session.begin_plan(), 'duplicate planning worker')
        self.assertEqual(session.tick((0, 0, 0), 10., 100.)[:2], (0., 0.))

    def test_cancel_invalidates_late_planner_response(self):
        session = self.n.Session()
        session.enable()
        token = session.begin_plan()
        session.cancel('operator stop')
        self.assertFalse(session.accept_path(token, self.points(), 10., 100.))
        self.assertFalse(session.enabled)
        self.assertEqual(session.tick((0, 0, 0), 10., 100.)[:2], (0., 0.))
        session.enable()
        replacement = session.begin_plan()
        self.assertNotEqual(token, replacement)
        self.assertFalse(session.accept_path(token, self.points(), 10., 100.))

    def test_active_input_failure_stops_and_does_not_resume_automatically(self):
        session = self.active()
        self.assertGreater(session.tick((0, 0, 0), 10.1, 100.1)[0], 0)
        self.assertEqual(session.tick((.01, 0, 0), 10.2, 100.2, healthy=False,
                                      reason='camera stopped')[:2], (0., 0.))
        self.assertFalse(session.enabled)
        self.assertEqual(session.tick((.01, 0, 0), 10.3, 100.3, healthy=True)[:2], (0., 0.))

    def test_expired_active_path_uses_both_ros_and_wall_deadlines(self):
        session = self.active()
        self.assertEqual(session.tick((0, 0, 0), 71., 101.)[:2], (0., 0.))
        self.assertFalse(session.enabled)
        session = self.active()
        self.assertEqual(session.tick((0, 0, 0), 10.1, 221.)[:2], (0., 0.))
        self.assertFalse(session.enabled)

    def test_clock_regression_cancels_path(self):
        session = self.active()
        session.tick((0, 0, 0), 10.1, 100.1)
        self.assertEqual(session.tick((0, 0, 0), 9., 100.2)[:2], (0., 0.))
        self.assertFalse(session.enabled)

    def test_wrong_frame_nonfinite_and_no_progress_paths_rejected(self):
        for points, frame in [(self.points(), 'world'),
                              ([self.n.Waypoint(float('nan'), 0, 0, 0)], 'lio_map'),
                              ([self.n.Waypoint(0, 0, 0, 0)] * 2, 'lio_map')]:
            session = self.n.Session()
            session.enable()
            token = session.begin_plan()
            with self.assertRaises(ValueError):
                session.accept_path(token, points, 10., 100., frame=frame)
            self.assertEqual(session.tick((0, 0, 0), 10., 100.)[:2], (0., 0.))

    def test_long_path_trimmed_to_execution_horizon(self):
        session = self.active(self.points(end=3.))
        self.assertAlmostEqual(session.controller.path.length, .6)
        self.assertAlmostEqual(session.controller.path.goal.x, .6)

    def test_truncation_does_not_invent_terminal_turn_to_unrelated_planner_yaw(self):
        session = self.active([self.n.Waypoint(0, 0, 0, 0),
                               self.n.Waypoint(0, 2., 0, 0)])
        self.assertAlmostEqual(session.controller.path.goal.y, .6)
        self.assertAlmostEqual(session.controller.path.goal.yaw, math.pi/2)

    def test_untrimmed_planner_terminal_orientation_is_preserved(self):
        session = self.active([self.n.Waypoint(0, 0, 0, 0),
                               self.n.Waypoint(.3, 0, 0, .7)])
        self.assertAlmostEqual(session.controller.path.goal.yaw, .7)

    def test_following_and_arrival_trigger_exactly_one_new_planning_cycle(self):
        session = self.active()
        velocity = session.tick((0, 0, 0), 10.1, 100.1)
        self.assertGreater(velocity[0], 0.)
        self.assertLessEqual(velocity[0], .12)
        done = session.tick((.5, 0, 0), 10.2, 100.2)
        self.assertEqual(done[:2], (0., 0.))
        self.assertEqual(session.completed, 1)
        session.tick((.5, 0, 0), 10.3, 100.3)
        self.assertEqual(session.completed, 1)
        self.assertIsNotNone(session.begin_plan())

    def test_turning_removes_translation_and_cross_track_failure_cancels(self):
        session = self.active()
        velocity = session.tick((0, 0, math.pi/2), 10.1, 100.1)
        self.assertEqual(velocity[0], 0.)
        self.assertLess(velocity[1], 0.)
        failed = session.tick((0, 1., 0), 10.2, 100.2)
        self.assertEqual(failed[:2], (0., 0.))
        self.assertFalse(session.enabled)

    def test_timing_gap_does_not_exceed_acceleration_limit(self):
        session = self.active()
        velocity = session.tick((0, 0, 0), 11., 100.1)
        self.assertLessEqual(velocity[0], .02)

    def test_control_loop_wall_gap_cancels_even_when_inputs_are_fresh(self):
        session = self.active()
        self.assertGreater(session.tick((0, 0, 0), 10.1, 100.1)[0], 0.)
        self.assertEqual(session.tick((0, 0, 0), 10.7, 100.7, healthy=True)[:2], (0., 0.))
        self.assertFalse(session.enabled)
        self.assertIn('command', session.reason)
        self.assertEqual(session.tick((0, 0, 0), 10.8, 100.8)[:2], (0., 0.))

    def test_planning_loop_gap_invalidates_late_official_planner_response(self):
        session = self.n.Session()
        session.enable()
        token = session.begin_plan()
        session.tick((0, 0, 0), 10., 100.)
        self.assertEqual(session.tick((0, 0, 0), 10.6, 100.6)[:2], (0., 0.))
        self.assertFalse(session.enabled)
        self.assertFalse(session.accept_path(token, self.points(), 10.6, 100.6))

    def test_terminal_approach_remains_above_guide_step_threshold(self):
        session = self.active()
        for i in range(1, 11):
            velocity = session.tick((.42, 0, 0), 10.+i*.1, 100.+i*.1)
        self.assertGreater(velocity[0], .03,
                           'Guide stops stepping before the follower reaches its goal tolerance')


if __name__ == '__main__':
    unittest.main()
