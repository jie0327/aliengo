"""Real sensor contracts: unit conversion once, timestamps and scan offsets intact."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT/'ros1/hardware/scripts/sensor_core.py'


class HardwareSensorTests(unittest.TestCase):
    def core(self):
        self.assertTrue(SOURCE.exists(), 'real sensor adapter is missing')
        spec = importlib.util.spec_from_file_location('sensor_core', SOURCE)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def test_g_conversion_preserves_stamp_and_rotates_no_axes(self):
        msg = NS(header=NS(stamp=123.456, frame_id='livox_frame'),
                 linear_acceleration=NS(x=0., y=-1., z=.5),
                 linear_acceleration_covariance=[.01]*9,
                 angular_velocity=NS(x=.1, y=.2, z=.3))
        out = self.core().imu_si(msg, 9.80665)
        self.assertEqual(out.header.stamp, 123.456)
        self.assertEqual(out.header.frame_id, 'mid360_imu_link')
        self.assertAlmostEqual(out.linear_acceleration.y, -9.80665)
        self.assertAlmostEqual(out.linear_acceleration.z, 4.903325)
        self.assertAlmostEqual(out.linear_acceleration_covariance[0], .961703842225)
        self.assertEqual(out.angular_velocity.z, .3)
        self.assertEqual(msg.header.frame_id, 'livox_frame')
        self.assertEqual(msg.linear_acceleration.y, -1.)

    def test_already_si_does_not_double_scale(self):
        msg = NS(header=NS(stamp=1, frame_id='raw'),
                 linear_acceleration=NS(x=0., y=0., z=9.81),
                 linear_acceleration_covariance=[-1.]+[0.]*8)
        out = self.core().imu_si(msg, 1.)
        self.assertEqual(out.linear_acceleration.z, 9.81)
        self.assertEqual(out.linear_acceleration_covariance[0], -1.)

    def test_points_keep_valid_returns_without_mutating_custom_timing(self):
        points = [NS(x=1., y=2., z=3., tag=0, offset_time=4000),
                  NS(x=float('nan'), y=0., z=0., tag=0, offset_time=5000),
                  NS(x=4., y=5., z=6., tag=0x20, offset_time=6000),
                  NS(x=7., y=8., z=9., tag=0x10, offset_time=1000000)]
        msg = NS(point_num=4, points=points)
        self.assertEqual(self.core().xyz_returns(msg), [(1., 2., 3.), (7., 8., 9.)])
        self.assertEqual([p.offset_time for p in msg.points], [4000, 5000, 6000, 1000000])

    def test_corrupt_point_count_is_rejected(self):
        with self.assertRaises(ValueError):
            self.core().xyz_returns(NS(point_num=50, points=[]))

    def test_navigation_keeps_near_returns_excluded_from_registration(self):
        self.assertTrue(hasattr(self.core(), 'near_returns'), 'navigation near-return retention is missing')
        msg = NS(point_num=4, points=[
            NS(x=.4, y=0., z=0., tag=0x10, offset_time=25000000),
            NS(x=.1, y=0., z=0., tag=0x10, offset_time=26000000),
            NS(x=2., y=0., z=0., tag=0x10, offset_time=27000000),
            NS(x=.5, y=0., z=0., tag=0x20, offset_time=28000000)])
        self.assertEqual(self.core().near_returns(msg, 1.), [(.4, 0., 0., 25000000)])

    def test_near_returns_use_their_acquisition_time_and_reference_sensor_pose(self):
        self.assertTrue(hasattr(self.core(), 'deskew_near'), 'near-return deskew is missing')
        import math
        import numpy as np
        reference = np.array([[0.,1.,0.,0.],[-1.,0.,0.,1.],[0.,0.,1.,0.],[0.,0.,0.,1.]])
        def measured_pose(nanoseconds):
            alpha = (nanoseconds-100000000)/25000000.
            c, s = math.cos(alpha*math.pi/2), math.sin(alpha*math.pi/2)
            return np.array([[c,-s,0.,alpha],[s,c,0.,0.],[0.,0.,1.,0.],[0.,0.,0.,1.]])
        points = [( .4, 0., 0., 0), (.4, 0., 0., 25000000)]
        actual = self.core().deskew_near(points, 100000000, reference, measured_pose)
        np.testing.assert_allclose(actual, [[0., .6, 0.], [.4, 0., 0.]], atol=1e-12)


if __name__ == '__main__':
    unittest.main()
