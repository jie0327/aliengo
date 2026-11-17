"""The replay verifier must reject live processes that stop producing poses."""
import importlib.util
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest


@unittest.skipUnless(importlib.util.find_spec('rospy'), 'Requires ROS Noetic messages')
class FusionProgressTests(unittest.TestCase):
    def setUp(self):
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts/verification'))
        self.addCleanup(sys.path.pop, 0)
        from fast_livo_trial import FastLivoTrial
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.trial = FastLivoTrial(SimpleNamespace(output=Path(directory.name), seconds=20,
                                                 gui=False, rviz=False, record_bag=False, bag=None))
        self.trial.state.update(odom={'stamp': 10.0}, last_lidar_stamp=10.1, last_image_stamp=10.2,
                                last_imu_stamp=10.195, sim=10.2)

    def test_live_process_without_new_estimates_is_rejected(self):
        self.trial.state.update(last_lidar_stamp=15.0, last_image_stamp=15.1)
        with self.assertRaisesRegex(RuntimeError, 'fusion stopped progressing'):
            self.trial.tick()

    def test_normal_one_scan_delay_is_accepted(self):
        self.trial.tick()

    def test_camera_stopping_mid_bag_is_rejected(self):
        self.trial.state.update(sim=15.0, last_lidar_stamp=15.0, last_imu_stamp=15.0)
        with self.assertRaisesRegex(RuntimeError, 'sensor stream stopped progressing'):
            self.trial.tick()


if __name__ == '__main__':
    unittest.main()
