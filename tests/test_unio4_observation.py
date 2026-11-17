import importlib.util
import math
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('unio4_observation', ROOT / 'ros1/unio4_observation.py')
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)


class ObservationTests(unittest.TestCase):
    def setUp(self):
        self.joints = {n: {'q': q, 'dq': 0.0} for n, q in zip(mod.JOINTS, mod.DEFAULT_Q)}
        self.obs = mod.ObservationHistory()

    def test_frame_has_documented_layout_and_zero_padding(self):
        history = self.obs.append([0, 0, 0, 1], self.joints, [0.1, 0, 0])
        self.assertEqual(len(history), 290)
        self.assertEqual(history[:232], [0.0] * 232)
        frame = history[-58:]
        self.assertEqual(frame[:3], [0.0, 0.0, -1.0])
        self.assertEqual(frame[3:6], [0.2, 0.0, 0.0])
        self.assertEqual(frame[18:54], [0.0] * 36)
        self.assertEqual(frame[54:], [0.0] * 4)

    def test_action_is_retained_for_next_frame_and_gait_advances(self):
        self.obs.append([0, 0, 0, 1], self.joints, [0, 0, 0])
        self.obs.accept_action([0.5] * 12)
        frame = self.obs.append([0, 0, 0, 1], self.joints, [0, 0, 0])[-58:]
        self.assertEqual(frame[42:54], [0.5] * 12)
        self.assertAlmostEqual(frame[54], math.sin(2 * math.pi * 0.56))

    def test_rejects_bad_data_and_reset_clears_state(self):
        with self.assertRaises(ValueError):
            self.obs.append([0, 0, 0, 0], self.joints, [0, 0, 0])
        with self.assertRaises(ValueError):
            self.obs.append([1e308, 1e308, 1e308, 1e308], self.joints, [0, 0, 0])
        with self.assertRaises(ValueError):
            self.obs.accept_action([float('nan')] * 12)
        self.obs.reset()
        self.assertEqual(self.obs.history, [0.0] * 290)


if __name__ == '__main__':
    unittest.main()
