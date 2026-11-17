"""Real ROS message boundary tests, run inside the Noetic dependency image."""
import importlib.util
import math
from pathlib import Path
import sys
import unittest

import numpy as np

SCRIPTS = Path(__file__).resolve().parents[1] / 'ros1/exploration/scripts'
sys.path.insert(0, str(SCRIPTS))


@unittest.skipUnless(importlib.util.find_spec('rospy'), 'Requires ROS Noetic messages')
class MappingRosTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.node = None
        if (SCRIPTS / 'mapping_node.py').is_file():
            import mapping_node
            cls.node = mapping_node

    def test_native_cloud_keeps_original_frame_stamp_and_coordinates(self):
        self.assertIsNotNone(self.node, 'ROS mapping frontend missing')
        import rospy
        from sensor_msgs import point_cloud2
        from std_msgs.msg import Header
        message = point_cloud2.create_cloud_xyz32(
            Header(stamp=rospy.Time(12, 345678901), frame_id='d435_depth_optical_frame'),
            [[1., .2, .1], [1.01, .2, .1], [float('nan'), 0., 0.], [0., 0., 0.]])
        mount, world = np.eye(4), np.eye(4)
        world[:3, 3] = [10., 20., 30.]
        native, points = self.node.prepare_cloud(message, mount, world, .15, 20., .05)
        self.assertEqual(native.header, message.header)
        np.testing.assert_allclose(list(point_cloud2.read_points(native)), [[1., .2, .1]], atol=1.e-6)
        np.testing.assert_allclose(points, [[11., 20.2, 30.1]], atol=1.e-6)
        # Reject malformed fields without ever publishing a fabricated surface.
        message.fields = message.fields[:2]
        with self.assertRaises(ValueError):
            self.node.prepare_cloud(message, mount, world, .15, 20., .05)

    def test_base_odometry_preserves_stamp_and_moves_pose_twist_origin(self):
        self.assertIsNotNone(self.node, 'ROS base odometry adapter missing')
        import rospy
        from nav_msgs.msg import Odometry
        message = Odometry()
        message.header.stamp = rospy.Time(15, 123)
        message.header.frame_id = 'lio_map'
        message.child_frame_id = 'lio_imu'
        message.pose.pose.position.x = 1.
        message.pose.pose.position.y = 2.
        message.pose.pose.position.z = 3.
        message.pose.pose.orientation.z = math.sqrt(.5)
        message.pose.pose.orientation.w = math.sqrt(.5)
        message.twist.twist.linear.x = 1.
        message.twist.twist.angular.z = 2.
        mount = np.eye(4)
        mount[:3, 3] = [.261, .02329, .10088]
        gyro = (np.array([0., 0., 2.]), np.eye(3)*.01)
        output, pose = self.node.base_odometry(message, mount, gyro)
        self.assertEqual(output.header, message.header)
        self.assertEqual(output.child_frame_id, 'base')
        np.testing.assert_allclose(pose, [1.02329, 1.739, 2.89912, math.pi/2])
        self.assertAlmostEqual(output.twist.twist.linear.x, 1.04658)
        self.assertAlmostEqual(output.twist.twist.linear.y, -.522)
        self.assertAlmostEqual(message.pose.pose.position.x, 1.)
        message.pose.pose.orientation.w = float('nan')
        with self.assertRaises(ValueError):
            self.node.base_odometry(message, mount)

    def test_real_producer_zero_angular_twist_uses_stamped_imu_gyro(self):
        import rospy
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu
        from mapping_core import GyroBuffer
        message = Odometry()
        message.header.stamp = rospy.Time(15, 123)
        message.header.frame_id = 'lio_map'
        message.child_frame_id = 'lio_imu'
        message.pose.pose.orientation.w = 1.
        message.twist.twist.linear.x = 1.
        # Current Super-LIO producer leaves angular velocity at the default zero.
        imu = Imu()
        imu.header.stamp = rospy.Time(15, 2000123)
        imu.header.frame_id = 'mid360_imu_link'
        imu.angular_velocity.z = .3
        imu.angular_velocity_covariance = (.01, 0., 0., 0., .01, 0., 0., 0., .01)
        samples = GyroBuffer('mid360_imu_link', max_age=.025)
        self.assertTrue(self.node.buffer_imu(samples, imu))
        mount = np.eye(4)
        mount[:3, 3] = [.261, .02329, .10088]
        output, _ = self.node.base_odometry(message, mount, samples.nearest(message.header.stamp.to_sec()))
        self.assertEqual(output.header, message.header)
        self.assertAlmostEqual(output.twist.twist.angular.z, .3)
        self.assertAlmostEqual(output.twist.twist.linear.x, 1.006987)
        self.assertAlmostEqual(output.twist.twist.linear.y, -.0783)
        self.assertGreater(output.twist.covariance[35], 0.)
        for kind in ('missing', 'wrong_frame', 'stale', 'unavailable'):
            with self.subTest(kind=kind):
                fallback = GyroBuffer('mid360_imu_link', max_age=.025)
                if kind != 'missing':
                    imu.header.frame_id = 'base' if kind == 'wrong_frame' else 'mid360_imu_link'
                    imu.header.stamp = rospy.Time(14) if kind == 'stale' else rospy.Time(15)
                    imu.angular_velocity_covariance = (-1.,)*9 if kind == 'unavailable' else (.01,)*9
                    self.node.buffer_imu(fallback, imu)
                output, pose = self.node.base_odometry(message, mount, fallback.nearest(message.header.stamp.to_sec()))
                self.assertTrue(np.all(np.isfinite(pose)))
                self.assertTrue(np.all(np.diag(np.array(output.twist.covariance).reshape(6, 6)) >= 1.e6))

    def test_grid_message_roundtrip_preserves_dimensions_and_nan(self):
        import io
        import rospy
        from grid_map_msgs.msg import GridMap
        from grid_map_codec import to_grid_message
        message = to_grid_message(np.array([[10., 11., 12.], [20., 21., np.nan]]),
                                  .5, (1., -2.), rospy.Time(123), 'lio_map')
        buffer = io.BytesIO()
        message.serialize(buffer)
        output = GridMap().deserialize(buffer.getvalue())
        self.assertEqual(output.info.header.stamp, rospy.Time(123))
        self.assertEqual((output.info.length_x, output.info.length_y), (1., 1.5))
        np.testing.assert_allclose(output.data[0].data, [10, 20, 11, 21, 12, np.nan])


if __name__ == '__main__':
    unittest.main()
