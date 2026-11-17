"""Numerical contracts for lessons 6/7; standard library, no ROS imports."""
import importlib.util
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def load_example(relative):
    path = ROOT / 'examples' / relative
    if not path.exists():
        return None
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


drift = load_example('lesson06/drift_demo.py')
forces = load_example('lesson07/static_forces.py')
mpc = load_example('lesson07/mpc_1d.py')
observe = load_example('lesson06/observe_state.py')


class IMUMathTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(drift, 'lesson06 drift example must exist')

    def test_constant_bias_has_quadratic_position_drift(self):
        rows = drift.integrate_bias(bias=0.02, duration=10.0, dt=0.01)
        self.assertAlmostEqual(rows[-1][1], 0.2, places=10)
        self.assertAlmostEqual(rows[-1][2], 1.0, places=10)
        self.assertAlmostEqual(rows[-1][0], 10.0, places=10)

    def test_finite_bias_that_overflows_integrated_state_is_rejected(self):
        with self.assertRaises(ValueError):
            drift.integrate_bias(bias=1e308, duration=10.0, dt=1.0)

    def test_overflowed_quaternion_norm_is_rejected(self):
        with self.assertRaises(ValueError):
            drift.rotate_xyzw((1e308,) * 4, (0, 0, 1))

    def test_overflowing_drift_cli_reports_error_without_inf_result(self):
        result = subprocess.run([sys.executable, str(ROOT / 'examples/lesson06/drift_demo.py'),
                                 '--bias', '1e308', '--duration', '10', '--dt', '1'],
                                text=True, capture_output=True)
        self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
        self.assertNotIn('position_error=', result.stdout)
        self.assertNotIn('Traceback', result.stderr)

    def test_partial_last_step_preserves_requested_duration(self):
        rows = drift.integrate_bias(bias=2.0, duration=0.25, dt=0.1)
        self.assertAlmostEqual(rows[-1][1], 0.5)
        self.assertAlmostEqual(rows[-1][2], 0.0625)

    def test_zero_bias_stays_stationary(self):
        self.assertEqual(drift.integrate_bias(0.0, 0.2, 0.1)[-1], (0.2, 0.0, 0.0))

    def test_invalid_time_and_nonfinite_data_are_rejected(self):
        for bias, duration, dt in [(math.nan, 1, 0.1), (0, math.inf, 0.1), (0, 1, 0),
                                   (0, 1, -0.1), (0, -1, 0.1), (0, 1, math.nan)]:
            with self.subTest(bias=bias, duration=duration, dt=dt):
                with self.assertRaises(ValueError):
                    drift.integrate_bias(bias, duration, dt)

    def test_stamp_reversal_duplicate_and_large_gap_are_rejected(self):
        self.assertAlmostEqual(drift.checked_dt(1.0, 1.01, max_dt=0.1), 0.01)
        for current in [0.5, 1.0, 1.2, math.nan]:
            with self.subTest(current=current):
                with self.assertRaises(ValueError):
                    drift.checked_dt(1.0, current, max_dt=0.1)

    def test_sdk_quaternion_conversion_and_yaw_rotation(self):
        s = math.sqrt(0.5)
        q = drift.wxyz_to_xyzw((s, 0.0, 0.0, s))
        rotated = drift.rotate_xyzw(q, (1, 0, 0))
        for got, expected in zip(rotated, (0, 1, 0)):
            self.assertAlmostEqual(got, expected)

    def test_quaternion_normalization_and_invalid_inputs(self):
        self.assertEqual(drift.rotate_xyzw((0, 0, 0, 2), (1, 2, 3)), (1.0, 2.0, 3.0))
        for q in [(0, 0, 0, 0), (0, 0, math.nan, 1), (0, 0, 1)]:
            with self.subTest(q=q):
                with self.assertRaises(ValueError):
                    drift.rotate_xyzw(q, (1, 0, 0))
        with self.assertRaises(ValueError):
            drift.rotate_xyzw((0, 0, 0, 1), (math.inf, 0, 0))

    def test_stationary_specific_force_cancels_world_gravity(self):
        self.assertEqual(drift.world_acceleration((0, 0, 0, 1), (0, 0, 9.81)), (0.0, 0.0, 0.0))
        # Sensor rolled by pi: sensor z points down, so measured specific force is -z.
        result = drift.world_acceleration((1, 0, 0, 0), (0, 0, -9.81))
        for got in result:
            self.assertAlmostEqual(got, 0)

    def test_readonly_observer_import_does_not_require_ros(self):
        self.assertIsNotNone(observe, 'read-only observer must be importable without ROS')
        result = subprocess.run([sys.executable, '-S', str(ROOT / 'examples/lesson06/observe_state.py'), '--help'],
                                text=True, capture_output=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('--timeout', result.stdout)

    def test_cli_writes_physical_csv(self):
        with tempfile.TemporaryDirectory() as tmp:
            target = Path(tmp) / 'drift.csv'
            result = subprocess.run([sys.executable, str(ROOT / 'examples/lesson06/drift_demo.py'),
                                     '--duration', '0.25', '--dt', '0.1', '--bias', '2',
                                     '--csv', str(target)], text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('0.062500', result.stdout)
            self.assertEqual(len(target.read_text().splitlines()), 5)


class ObservationBoundaryTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(observe, 'read-only observer must exist')

    def test_ros_stamp_is_seconds_and_validates_nanoseconds(self):
        self.assertAlmostEqual(observe.stamp_seconds(12, 250_000_000), 12.25)
        for sec, nano in [(1, -1), (1, 1_000_000_000)]:
            with self.assertRaises(ValueError):
                observe.stamp_seconds(sec, nano)

    def test_unavailable_orientation_does_not_fabricate_valid_attitude(self):
        result = observe.imu_summary((0, 0, 0, 0), (0, 0, 0), (0, 0, 9.81), False)
        self.assertIsNone(result['quaternion_norm'])
        self.assertAlmostEqual(result['specific_force_norm_m_s2'], 9.81)
        with self.assertRaises(ValueError):
            observe.imu_summary((0, 0, 0, 0), (0, 0, 0), (0, 0, 9.81), True)
        with self.assertRaises(ValueError):
            observe.imu_summary((0, 0, 0, 1), (math.nan, 0, 0), (0, 0, 9.81), True)

    def test_joint_arrays_cannot_silently_mislabel_data(self):
        self.assertEqual(observe.joint_summary(['FR_hip_joint'], [0.5])['position_rad'], {'FR_hip_joint': 0.5})
        for names, positions in [(['FR_hip_joint'], []), (['a', 'a'], [0, 1]), (['a'], [math.inf])]:
            with self.assertRaises(ValueError):
                observe.joint_summary(names, positions)

    def test_equal_stamp_is_duplicate_but_is_not_a_new_time_step(self):
        tracker = observe.SampleTimes()
        self.assertIsNone(tracker.accept('imu', 3.0))
        self.assertEqual(tracker.accept('imu', 3.0), 0.0)
        self.assertEqual(tracker.previous['imu'], 3.0)
        self.assertAlmostEqual(tracker.accept('imu', 3.01), 0.01)

    def test_clock_reset_is_rejected_before_counting_new_sample(self):
        tracker = observe.SampleTimes()
        self.assertIsNone(tracker.accept('imu', 3.0))
        self.assertAlmostEqual(tracker.accept('imu', 3.01), 0.01)
        with self.assertRaises(ValueError):
            tracker.accept('imu', 0.0)
        self.assertEqual(tracker.previous['imu'], 3.01)


class ObservationStartupTests(unittest.TestCase):
    def setUp(self):
        self.assertTrue(hasattr(observe, 'ObserverState'), 'observer needs a bounded startup state')
        self.state = observe.ObserverState(samples=2, timeout=1.0, start_time=10.0)

    @staticmethod
    def imu_sample(stamp, valid=True):
        summary = observe.imu_summary((0, 0, 0, 1 if valid else 0), (0, 0, 0), (0, 0, 9.81))
        return dict(summary, stamp_s=stamp, frame_id='imu_link')

    def test_startup_invalid_then_valid_is_counted_without_weakening_validation(self):
        self.state.consume('imu', lambda: self.imu_sample(0.0, valid=False), now=10.1)
        self.assertFalse(self.state.errors)
        self.assertEqual(self.state.counts['imu'], 0)
        self.assertEqual(self.state.startup_invalid['imu']['count'], 1)
        self.assertIn('unit quaternion', self.state.startup_invalid['imu']['last_reason'])
        self.state.consume('imu', lambda: self.imu_sample(1.0), now=10.2)
        self.state.consume('imu', lambda: self.imu_sample(1.01), now=10.3)
        self.state.consume('joints', lambda: dict(observe.joint_summary(['FR_hip_joint'], [0.1]),
                                                stamp_s=1.0, frame_id=''), now=10.4)
        self.state.consume('joints', lambda: dict(observe.joint_summary(['FR_hip_joint'], [0.1]),
                                                stamp_s=1.01, frame_id=''), now=10.5)
        report = self.state.report(now=10.5)
        self.assertTrue(report['complete'])
        self.assertFalse(report['timed_out'])
        self.assertEqual(report['counts'], {'imu': 2, 'joints': 2})
        self.assertEqual(report['startup_invalid']['imu']['count'], 1)

    def test_persistent_invalid_data_times_out_without_counting_as_valid(self):
        for index in range(20):
            self.state.consume('imu', lambda: self.imu_sample(0.0, valid=False), now=10.0 + index * 0.04)
        report = self.state.report(now=11.0)
        self.assertTrue(self.state.finished(now=11.0))
        self.assertFalse(report['complete'])
        self.assertTrue(report['timed_out'])
        self.assertEqual(report['counts']['imu'], 0)
        self.assertEqual(report['startup_invalid']['imu']['count'], 20)
        self.assertIn('unit quaternion', report['startup_invalid']['imu']['last_reason'])
        self.state.consume('imu', lambda: self.imu_sample(1.0), now=11.1)
        self.assertEqual(self.state.counts['imu'], 0)

    def test_bad_data_after_first_valid_sample_fails_instead_of_becoming_startup_noise(self):
        self.state.consume('imu', lambda: self.imu_sample(1.0), now=10.1)
        self.state.consume('imu', lambda: self.imu_sample(1.01, valid=False), now=10.2)
        self.assertTrue(self.state.errors)
        self.assertTrue(self.state.finished(now=10.2))
        self.assertFalse(self.state.report(now=10.2)['complete'])
        self.assertEqual(self.state.startup_invalid['imu']['count'], 0)

    def test_clock_reversal_after_first_valid_sample_is_fatal(self):
        self.state.consume('imu', lambda: self.imu_sample(1.0), now=10.1)
        self.state.consume('imu', lambda: self.imu_sample(0.0), now=10.2)
        self.assertIn('stamp moved backwards', self.state.errors[0])
        self.assertEqual(self.state.counts['imu'], 1)

    def test_duplicate_then_new_stamp_counts_only_the_new_sample(self):
        self.state.consume('imu', lambda: self.imu_sample(1.0), now=10.1)
        self.state.consume('imu', lambda: self.imu_sample(1.0), now=10.2)
        self.assertFalse(self.state.errors)
        self.assertEqual(self.state.counts['imu'], 1)
        self.assertEqual(self.state.duplicate_stamps['imu'], 1)
        self.assertEqual(self.state.timing.previous['imu'], 1.0)
        self.state.consume('imu', lambda: self.imu_sample(1.01), now=10.3)
        self.assertEqual(self.state.counts['imu'], 2)
        self.assertAlmostEqual(self.state.summaries['imu']['last_dt_s'], 0.01)
        self.assertEqual(self.state.report(now=10.3)['duplicate_stamps']['imu'], 1)

    def test_persistent_duplicates_timeout_without_filling_sample_count(self):
        self.state.consume('imu', lambda: self.imu_sample(1.0), now=10.0)
        for index in range(20):
            self.state.consume('imu', lambda: self.imu_sample(1.0), now=10.01 + index * 0.04)
        report = self.state.report(now=11.0)
        self.assertFalse(report['complete'])
        self.assertTrue(report['timed_out'])
        self.assertFalse(report['errors'])
        self.assertEqual(report['counts']['imu'], 1)
        self.assertEqual(report['duplicate_stamps']['imu'], 20)
        self.assertEqual(self.state.timing.previous['imu'], 1.0)

    def test_backwards_after_duplicate_still_fails(self):
        self.state.consume('imu', lambda: self.imu_sample(1.0), now=10.1)
        self.state.consume('imu', lambda: self.imu_sample(1.0), now=10.2)
        self.assertFalse(self.state.errors)
        self.state.consume('imu', lambda: self.imu_sample(0.99), now=10.3)
        self.assertTrue(self.state.errors)
        self.assertEqual(self.state.counts['imu'], 1)
        self.assertEqual(self.state.timing.previous['imu'], 1.0)

    def test_duplicate_stamp_does_not_hide_invalid_sensor_fields(self):
        self.state.consume('imu', lambda: self.imu_sample(1.0), now=10.1)
        self.state.consume('imu', lambda: self.imu_sample(1.0, valid=False), now=10.2)
        self.assertTrue(self.state.errors)
        self.assertEqual(self.state.duplicate_stamps['imu'], 0)
        self.assertEqual(self.state.counts['imu'], 1)

    def test_each_stream_leaves_startup_independently(self):
        self.state.consume('imu', lambda: self.imu_sample(0.0, valid=False), now=10.1)
        self.state.consume('joints', lambda: dict(observe.joint_summary(['FR_hip_joint'], [0.1]),
                                                stamp_s=1.0, frame_id=''), now=10.2)
        self.state.consume('joints', lambda: observe.joint_summary(['FR_hip_joint'], []), now=10.3)
        self.assertTrue(self.state.errors)
        self.assertEqual(self.state.startup_invalid['imu']['count'], 1)
        self.assertEqual(self.state.startup_invalid['joints']['count'], 0)

    def test_completed_stream_is_still_checked_while_other_stream_is_waiting(self):
        state = observe.ObserverState(samples=1, timeout=1.0, start_time=10.0)
        state.consume('imu', lambda: self.imu_sample(1.0), now=10.1)
        state.consume('imu', lambda: self.imu_sample(1.01, valid=False), now=10.2)
        self.assertTrue(state.errors)
        self.assertEqual(state.counts['imu'], 1)


class ContactForceTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(forces, 'lesson07 contact force example must exist')

    def test_four_equal_contacts_balance_twenty_kg(self):
        values = forces.equal_support_forces(20.0, 4)
        self.assertEqual(len(values), 4)
        self.assertAlmostEqual(values[0][2], 49.05)
        self.assertAlmostEqual(sum(f[2] for f in values), 196.2)

    def test_no_contact_cannot_support_mass(self):
        for mass, contacts in [(20, 0), (-1, 4), (math.nan, 4), (20, 1.5)]:
            with self.subTest(mass=mass, contacts=contacts):
                with self.assertRaises(ValueError):
                    forces.equal_support_forces(mass, contacts)

    def test_friction_cone_checks_combined_horizontal_force(self):
        self.assertTrue(forces.in_friction_cone((3, 4, 10), 0.5))
        self.assertFalse(forces.in_friction_cone((4, 4, 10), 0.5))
        self.assertFalse(forces.in_friction_cone((0, 0, -1), 0.5))
        self.assertTrue(forces.in_friction_cone((0, 0, 0), 0.5))
        self.assertFalse(forces.in_friction_cone((0.01, 0, 0), 0.5))
        with self.assertRaises(ValueError):
            forces.in_friction_cone((0, 0, 10), -0.5)
        with self.assertRaises(ValueError):
            forces.in_friction_cone((math.nan, 0, 10), 0.5)

    def test_actuator_torque_opposes_external_ground_force(self):
        # J in m/rad, F_ground in N. tau + J^T F_ground = 0.
        jacobian = ((0.2, 0.0), (0.0, 0.0), (0.1, -0.2))
        self.assertEqual(forces.compensation_torque(jacobian, (0, 0, 50)), (-5.0, 10.0))
        self.assertEqual(forces.compensation_torque(jacobian, (10, 0, 50)), (-7.0, 10.0))
        with self.assertRaises(ValueError):
            forces.compensation_torque(((1, 0), (1,), (0, 1)), (0, 0, 1))


class PredictiveControlTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(mpc, 'lesson07 finite-horizon example must exist')

    def test_double_integrator_uses_half_acceleration_position_term(self):
        x, v = mpc.predict_step(1.0, 2.0, -1.0, 0.5)
        self.assertAlmostEqual(x, 1.875)
        self.assertAlmostEqual(v, 1.5)

    def test_model_rejects_invalid_time_and_nonfinite_inputs(self):
        for args in [(0, 0, 1, 0), (0, 0, 1, -0.1), (0, math.nan, 1, 0.1)]:
            with self.assertRaises(ValueError):
                mpc.predict_step(*args)

    def test_finite_model_inputs_that_overflow_prediction_are_rejected(self):
        with self.assertRaises(ValueError):
            mpc.predict_step(0, 0, 0, 1e308)

    def test_finite_state_that_overflows_plan_cost_is_rejected(self):
        with self.assertRaises(ValueError):
            mpc.choose_action(1e200, 0, horizon=1)

    def test_overflowing_mpc_cli_reports_error_without_nan_metrics(self):
        result = subprocess.run([sys.executable, str(ROOT / 'examples/lesson07/mpc_1d.py'),
                                 '--dt', '1e308', '--horizon', '1', '--steps', '1'],
                                text=True, capture_output=True)
        self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
        self.assertNotIn('final_abs_error_m=', result.stdout)
        self.assertNotIn('Traceback', result.stderr)

    def test_controller_brakes_before_target_with_longer_horizon(self):
        short = mpc.choose_action(0.8, 1.0, horizon=1, action_cost=0.01)
        long = mpc.choose_action(0.8, 1.0, horizon=5, action_cost=0.01)
        self.assertEqual(short['action'], 0.0)
        self.assertEqual(long['action'], -1.0)
        self.assertEqual(len(long['sequence']), 5)

    def test_action_and_predicted_velocity_constraints_hold(self):
        plan = mpc.choose_action(0.0, 0.9, horizon=4, action_cost=0.01, max_speed=1.0)
        self.assertTrue(all(abs(a) <= 1.0 for a in plan['sequence']))
        self.assertTrue(all(abs(v) <= 1.0 + 1e-12 for x, v in plan['states']))

    def test_infeasible_state_and_invalid_horizon_fail_explicitly(self):
        with self.assertRaises(ValueError):
            mpc.choose_action(0.0, 3.0, horizon=3, max_speed=1.0)
        for horizon in [0, -1, 1.5, 9]:
            with self.subTest(horizon=horizon):
                with self.assertRaises(ValueError):
                    mpc.choose_action(0.0, 0.0, horizon=horizon)
        with self.assertRaises(ValueError):
            mpc.choose_action(0, 0, action_cost=-1)

    def test_rolling_horizon_only_applies_first_action_and_reports_metrics(self):
        rows, metrics = mpc.simulate(horizon=5, action_cost=0.01, steps=40)
        self.assertEqual(len(rows), 41)
        self.assertEqual(rows[0]['position_m'], 0.0)
        self.assertLess(metrics['final_abs_error_m'], 0.1)
        self.assertLessEqual(metrics['max_speed_m_s'], 2.0 + 1e-12)
        self.assertGreater(metrics['action_energy'], 0)

    def test_cli_emits_comparison_csv(self):
        with tempfile.TemporaryDirectory() as tmp:
            target = Path(tmp) / 'mpc.csv'
            result = subprocess.run([sys.executable, str(ROOT / 'examples/lesson07/mpc_1d.py'),
                                     '--compare', '--steps', '12', '--csv', str(target)],
                                    text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('horizon=5', result.stdout)
            self.assertIn('action_cost=1', result.stdout)
            self.assertEqual(len(target.read_text().splitlines()), 40)


if __name__ == '__main__':
    unittest.main()
