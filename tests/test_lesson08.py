"""Offline interface admission checks: no SDK or network dependency."""
import copy
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
MODULE = ROOT / "examples/lesson08/offline_bridge.py"


def frame():
    names = ["FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
             "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
             "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
             "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"]
    return {"command_stamp": 9.98, "feedback_stamp": 9.99,
            "feedback_positions": {name: [0.0, 0.67, -1.3][i % 3] for i, name in enumerate(names)},
            "targets": [{"name": name, "q": [0.0, 0.67, -1.3][i % 3],
                         "dq": 0.0, "tau": 0.0, "kp": 10.0, "kd": 1.0}
                        for i, name in enumerate(names)]}


class OfflineBridgeTests(unittest.TestCase):
    def setUp(self):
        self.assertTrue(MODULE.is_file(), "lesson08 offline bridge is not implemented")
        spec = importlib.util.spec_from_file_location("course_offline_bridge", MODULE)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.bridge = module.OfflineBridge()

    def check_frame(self, payload=None, **kwargs):
        return self.bridge.check(frame() if payload is None else payload, now=10.0, **kwargs)

    def test_shuffled_names_keep_sdk_index_and_gain_mapping(self):
        payload = frame()
        payload["targets"][0]["q"] = 0.02
        payload["targets"].reverse()
        result = self.check_frame(payload)
        self.assertTrue(result["accepted"], result)
        self.assertEqual(len(result["sdk_targets"]), 12)
        self.assertEqual(result["sdk_targets"][0]["index"], 0)
        self.assertEqual(result["sdk_targets"][0]["name"], "FR_hip_joint")
        self.assertEqual(result["sdk_targets"][0]["q"], 0.02)
        self.assertEqual(result["sdk_targets"][3]["name"], "FL_hip_joint")
        self.assertEqual(result["sdk_targets"][6]["name"], "RR_hip_joint")
        self.assertEqual(result["sdk_targets"][11]["name"], "RL_calf_joint")
        self.assertEqual(result["sdk_targets"][0]["Kp"], 10.0)

    def test_missing_joint_rejects_whole_frame(self):
        payload = frame(); payload["targets"].pop()
        result = self.check_frame(payload)
        self.assertFalse(result["accepted"])
        self.assertEqual(result["sdk_targets"], [])

    def test_duplicate_name_is_rejected(self):
        payload = frame(); payload["targets"][-1] = copy.deepcopy(payload["targets"][0])
        self.assertFalse(self.check_frame(payload)["accepted"])

    def test_unknown_joint_is_rejected(self):
        payload = frame(); payload["targets"][0]["name"] = "front_motor"
        self.assertFalse(self.check_frame(payload)["accepted"])

    def test_stale_command_and_feedback_are_each_rejected(self):
        for field in ("command_stamp", "feedback_stamp"):
            with self.subTest(field=field):
                payload = frame(); payload[field] = 9.8
                self.assertFalse(self.check_frame(payload)["accepted"])

    def test_future_or_nonfinite_clock_is_rejected(self):
        for value in (10.2, float("nan"), float("inf"), True):
            with self.subTest(value=value):
                payload = frame(); payload["feedback_stamp"] = value
                self.assertFalse(self.check_frame(payload)["accepted"])

    def test_nonfinite_wrong_type_and_missing_fields_are_rejected(self):
        for field, value in (("q", float("nan")), ("dq", float("inf")),
                             ("tau", "2.0"), ("kp", True)):
            with self.subTest(field=field):
                payload = frame(); payload["targets"][0][field] = value
                self.assertFalse(self.check_frame(payload)["accepted"])
        payload = frame(); del payload["targets"][0]["kd"]
        self.assertFalse(self.check_frame(payload)["accepted"])

    def test_position_speed_effort_and_gain_limits_reject_whole_frame(self):
        for field, value in (("q", 0.8), ("dq", 1.1), ("tau", 2.1), ("kp", -1), ("kd", 3)):
            with self.subTest(field=field):
                payload = frame(); payload["targets"][0][field] = value
                result = self.check_frame(payload)
                self.assertFalse(result["accepted"])
                self.assertEqual(result["sdk_targets"], [])

    def test_step_limit_uses_previous_feedback_by_name(self):
        payload = frame(); payload["targets"][0]["q"] = 0.2
        previous = {r["name"]: r["q"] for r in frame()["targets"]}
        self.assertFalse(self.check_frame(payload, previous=previous)["accepted"])
        payload["targets"][0]["q"] = 0.02
        self.assertTrue(self.check_frame(payload, previous=previous)["accepted"])

    def test_incomplete_previous_feedback_cannot_bypass_step_check(self):
        self.assertFalse(self.check_frame(previous={})["accepted"])

    def test_fresh_stamp_without_feedback_positions_is_rejected(self):
        payload = frame(); del payload["feedback_positions"]
        self.assertFalse(self.check_frame(payload)["accepted"])

    def test_json_input_cannot_skip_step_check(self):
        payload = frame(); payload["targets"][0]["q"] = 0.4
        with tempfile.TemporaryDirectory() as folder:
            source = Path(folder) / "frame.json"
            source.write_text(json.dumps(payload))
            result = subprocess.run([sys.executable, str(MODULE), "--input", str(source), "--now", "10"],
                                    text=True, capture_output=True, timeout=5)
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertFalse(json.loads(result.stdout)["accepted"])

    def test_closed_bridge_does_not_admit_new_frame(self):
        self.bridge.close()
        self.assertFalse(self.check_frame()["accepted"])

    def test_check_does_not_mutate_callers_frame(self):
        payload = frame(); original = copy.deepcopy(payload)
        self.check_frame(payload)
        self.assertEqual(payload, original)


if __name__ == "__main__":
    unittest.main()
