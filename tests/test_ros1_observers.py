"""ROS 1 observer adapters preserve the original course's data checks."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    'ros1_observe_state', ROOT / 'examples/lesson06/ros1_observe_state.py')
observer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(observer)


class Ros1ObserverTests(unittest.TestCase):
    def test_joint_message_keeps_names_positions_and_ros1_stamp(self):
        message = SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(secs=12, nsecs=500000000), frame_id='base'),
            name=['FR_hip_joint'], position=[0.1], velocity=[0.2], effort=[])
        result = observer.read_sample('joints', message)
        self.assertEqual(result['stamp_s'], 12.5)
        self.assertEqual(result['position_rad'], {'FR_hip_joint': 0.1})

    def test_joint_message_rejects_mismatched_velocity(self):
        message = SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(secs=12, nsecs=0), frame_id='base'),
            name=['a', 'b'], position=[0.1, 0.2], velocity=[0.2], effort=[])
        with self.assertRaises(ValueError):
            observer.read_sample('joints', message)

    def test_imu_message_uses_xyzw_and_marks_missing_orientation(self):
        q = SimpleNamespace(x=0., y=0., z=0., w=1.)
        v = SimpleNamespace(x=0., y=0., z=0.)
        message = SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(secs=2, nsecs=3), frame_id='imu'),
            orientation=q, angular_velocity=v, linear_acceleration=v,
            orientation_covariance=[-1.], angular_velocity_covariance=[0.],
            linear_acceleration_covariance=[0.])
        result = observer.read_sample('imu', message)
        self.assertIsNone(result['quaternion_norm'])
        self.assertAlmostEqual(result['stamp_s'], 2.000000003)


if __name__ == '__main__':
    unittest.main()
