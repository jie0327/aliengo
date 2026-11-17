"""Run with the Noetic environment sourced; uses real ROS message types."""
import importlib.util
import io
import math
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(importlib.util.find_spec('rospy'), 'Requires ROS Noetic messages')
class LioBaseTfTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location(
            'lio_base_tf', ROOT / 'ros1/super_lio/scripts/lio_base_tf.py')
        cls.node = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.node)

    def odom(self):
        import rospy
        from nav_msgs.msg import Odometry
        message = Odometry()
        message.header.seq = 17
        message.header.stamp = rospy.Time(12, 345678901)
        message.header.frame_id = 'lio_map'
        message.child_frame_id = 'mid360_imu_link'
        message.pose.pose.position.x = 1
        message.pose.pose.position.y = 2
        message.pose.pose.position.z = 3
        message.pose.pose.orientation.z = math.sqrt(.5)
        message.pose.pose.orientation.w = math.sqrt(.5)
        return message

    def mount(self):
        from geometry_msgs.msg import TransformStamped
        message = TransformStamped()
        message.header.frame_id = 'base'
        message.child_frame_id = 'mid360_imu_link'
        message.transform.translation.x = .261
        message.transform.translation.y = .02329
        message.transform.translation.z = .10088
        message.transform.rotation.w = 1
        return message

    def assert_pose(self, transform, position, quaternion):
        actual_position = transform.transform.translation
        for actual, expected in zip(
                (actual_position.x, actual_position.y, actual_position.z), position):
            self.assertAlmostEqual(actual, expected, places=12)
        actual_rotation = transform.transform.rotation
        actual_quaternion = (actual_rotation.x, actual_rotation.y,
                             actual_rotation.z, actual_rotation.w)
        self.assertAlmostEqual(
            abs(sum(actual * expected for actual, expected
                    in zip(actual_quaternion, quaternion))), 1, places=12)

    def test_default_offset_is_subtracted_after_world_yaw_rotation(self):
        result = self.node.odom_to_base_transform(self.odom(), self.mount())
        self.assert_pose(result, (1.02329, 1.739, 2.89912),
                         (0, 0, math.sqrt(.5), math.sqrt(.5)))

    def test_nonidentity_mount_rotation_is_inverted_before_composition(self):
        odom, mount = self.odom(), self.mount()
        mount.transform.translation.x = .2
        mount.transform.translation.y = -.1
        mount.transform.translation.z = .3
        mount.transform.rotation.x = math.sqrt(.5)
        mount.transform.rotation.w = math.sqrt(.5)
        # 已知 base 为 yaw=90°；叠加安装 roll=90° 得到 IMU 位姿。
        odom.pose.pose.position.x = 1.1
        odom.pose.pose.position.y = 2.2
        odom.pose.pose.position.z = 3.3
        odom.pose.pose.orientation.x = .5
        odom.pose.pose.orientation.y = .5
        odom.pose.pose.orientation.z = .5
        odom.pose.pose.orientation.w = .5
        result = self.node.odom_to_base_transform(odom, mount)
        self.assert_pose(result, (1, 2, 3),
                         (0, 0, math.sqrt(.5), math.sqrt(.5)))

    def test_stamp_and_parent_frame_come_from_estimated_odometry(self):
        odom = self.odom()
        odom.header.frame_id = 'another_lio_map'
        result = self.node.odom_to_base_transform(odom, self.mount())
        self.assertEqual(result.header, odom.header)
        self.assertEqual(result.header.frame_id, 'another_lio_map')
        self.assertEqual(result.child_frame_id, 'base')

    def test_conversion_and_output_mutation_leave_both_inputs_unchanged(self):
        odom, mount = self.odom(), self.mount()
        before = []
        for message in (odom, mount):
            buffer = io.BytesIO()
            message.serialize(buffer)
            before.append(buffer.getvalue())
        result = self.node.odom_to_base_transform(odom, mount)
        result.header.frame_id = 'changed_output'
        result.header.stamp.secs += 1
        result.transform.translation.x = 100
        result.transform.rotation.w = 0
        for message, original in zip((odom, mount), before):
            buffer = io.BytesIO()
            message.serialize(buffer)
            self.assertEqual(buffer.getvalue(), original)


if __name__ == '__main__':
    unittest.main()
