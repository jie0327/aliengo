"""Run with the Noetic environment sourced; uses real ROS message types."""
import importlib.util
import io
import math
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(importlib.util.find_spec('rospy'), 'Requires ROS Noetic messages')
class GazeboPoseTfTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location(
            'gazebo_pose_tf', ROOT / 'ros1/mid360/scripts/gazebo_pose_tf.py')
        cls.node = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.node)

    def state(self):
        import rospy
        from gazebo_msgs.srv import GetModelStateResponse
        state = GetModelStateResponse()
        state.success = True
        state.header.stamp = rospy.Time(12, 345)
        state.header.frame_id = 'world'
        state.pose.position.x = -45
        state.pose.position.y = -25
        state.pose.position.z = 5.428
        state.pose.orientation.z = math.sqrt(.5)
        state.pose.orientation.w = math.sqrt(.5)
        state.twist.linear.x = 1
        state.twist.linear.z = .2
        state.twist.angular.z = .3
        return state

    def test_pose_and_simulation_stamp_match_gazebo_including_absolute_height(self):
        state = self.state()
        transform, odom = self.node.state_messages(state)
        self.assertEqual(transform.header.stamp, state.header.stamp)
        self.assertEqual(odom.header.stamp, state.header.stamp)
        self.assertEqual(transform.header.frame_id, 'world')
        self.assertEqual(transform.child_frame_id, 'base')
        self.assertEqual(odom.child_frame_id, 'base')
        self.assertEqual(transform.transform.translation.x, -45)
        self.assertEqual(transform.transform.translation.y, -25)
        self.assertEqual(transform.transform.translation.z, 5.428)
        self.assertEqual(transform.transform.rotation, state.pose.orientation)
        self.assertEqual(odom.pose.pose, state.pose)

    def test_world_velocity_is_rotated_into_child_base_frame(self):
        _, odom = self.node.state_messages(self.state())
        self.assertAlmostEqual(odom.twist.twist.linear.x, 0)
        self.assertAlmostEqual(odom.twist.twist.linear.y, -1)
        self.assertAlmostEqual(odom.twist.twist.linear.z, .2)
        self.assertAlmostEqual(odom.twist.twist.angular.z, .3)

    def test_roll_rotates_both_linear_and_angular_velocity_without_mutating_source(self):
        state = self.state()
        state.pose.orientation.x = math.sqrt(.5)
        state.pose.orientation.z = 0
        state.twist.linear.x, state.twist.linear.y, state.twist.linear.z = 0, 1, 0
        state.twist.angular.z = 1
        before = io.BytesIO()
        state.serialize(before)
        _, odom = self.node.state_messages(state)
        self.assertAlmostEqual(odom.twist.twist.linear.z, -1)
        self.assertAlmostEqual(odom.twist.twist.angular.y, 1)
        self.assertAlmostEqual(odom.twist.twist.angular.z, 0)
        after = io.BytesIO()
        state.serialize(after)
        self.assertEqual(before.getvalue(), after.getvalue())


if __name__ == '__main__':
    unittest.main()
