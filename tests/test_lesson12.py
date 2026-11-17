"""Hand-checkable policy interface tests; no model, ROS, SDK, or network."""
import copy
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
MODULE = ROOT / "examples/lesson12/offline_policy_contract.py"
NAMES = [f"{leg}_{joint}_joint" for leg in ("FR", "FL", "RR", "RL")
         for joint in ("hip", "thigh", "calf")]


def frame(stamp=9.99):
    return {"stamp": stamp, "body_omega": [2.0, -4.0, 1.0],
            "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
            "command": [0.5, -0.25, 2.0],
            "joints": [{"name": name, "q": [0.0, 0.8, -1.5][i % 3], "dq": 0.0}
                       for i, name in enumerate(NAMES)]}


class PolicyContractTests(unittest.TestCase):
    def setUp(self):
        self.assertTrue(MODULE.is_file(), "lesson12 offline contract is not implemented")
        spec = importlib.util.spec_from_file_location("course_policy_contract", MODULE)
        self.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)
        self.contract = self.module.PolicyContract()

    def step(self, payload=None, action=None, now=10.0):
        return self.contract.step(frame() if payload is None else payload, now=now,
                                  action_fixture=[0.0] * 12 if action is None else action)

    def test_observation_uses_hand_checked_order_scales_and_default_offsets(self):
        payload = frame()
        payload["joints"][0].update(q=0.2, dq=4.0)
        payload["joints"].reverse()
        result = self.step(payload)
        self.assertTrue(result["accepted"], result)
        self.assertEqual(len(result["observation"]), 45)
        self.assertEqual(result["observation"][:9],
                         [0.5, -1.0, 0.25, 0.0, 0.0, -1.0, 1.0, -0.5, 0.5])
        self.assertEqual(result["observation"][9:21], [0.2] + [0.0] * 11)
        self.assertEqual(result["observation"][21:33], [0.2] + [0.0] * 11)
        self.assertEqual(result["observation"][33:45], [0.0] * 12)
        self.assertEqual([target["name"] for target in result["targets"]], NAMES)

    def test_pitch_90_gravity_and_nonunit_quaternion_normalization(self):
        payload = frame()
        payload["orientation_xyzw"] = [0.0, math.sqrt(2.0), 0.0, math.sqrt(2.0)]
        result = self.step(payload)
        self.assertTrue(result["accepted"], result)
        for value, expected in zip(result["observation"][3:6], [1.0, 0.0, 0.0]):
            self.assertAlmostEqual(value, expected, places=12)

    def test_action_is_clipped_before_scaling_and_adding_default_pose(self):
        result = self.step(action=[2.0, -2.0, 0.5] * 4)
        self.assertEqual(result["action_used"], [1.0, -1.0, 0.5] * 4)
        self.assertEqual([r["q_des"] for r in result["targets"]], [0.25, 0.55, -1.375] * 4)
        # The current observation contains the previous action, not this output.
        self.assertEqual(result["observation"][33:], [0.0] * 12)

    def test_decimation_holds_target_and_history_only_advances_on_policy_ticks(self):
        results = []
        for tick in range(11):
            results.append(self.step(frame(9.97 + tick * 0.002), action=[1.0] * 12))
        self.assertTrue(all(r["accepted"] for r in results), results)
        self.assertEqual([i for i, r in enumerate(results) if r["policy_due"]], [0, 10])
        self.assertEqual(results[1]["targets"], results[0]["targets"])
        self.assertEqual(results[9]["history"], results[0]["history"])
        self.assertEqual(results[10]["observation"][33:], [1.0] * 12)
        self.assertEqual(results[0]["history"][:90], [0.0] * 90)
        self.assertEqual(results[10]["history"][45:90], results[0]["observation"])

    def test_reset_clears_previous_action_history_and_clock_sequence(self):
        self.step(action=[1.0] * 12)
        self.contract.reset()
        result = self.step(frame(1.0), now=1.001)
        self.assertTrue(result["accepted"], result)
        self.assertTrue(result["policy_due"])
        self.assertEqual(result["observation"][33:], [0.0] * 12)
        self.assertEqual(result["history"][:90], [0.0] * 90)

    def test_missing_duplicate_and_unknown_joint_reject_entire_frame(self):
        for kind in ("missing", "duplicate", "unknown"):
            with self.subTest(kind=kind):
                self.contract.reset()
                payload = frame()
                if kind == "missing":
                    payload["joints"].pop()
                elif kind == "duplicate":
                    payload["joints"][-1]["name"] = NAMES[0]
                else:
                    payload["joints"][-1]["name"] = "other_joint"
                result = self.step(payload)
                self.assertFalse(result["accepted"])
                self.assertEqual(result["targets"], [])

    def test_nonfinite_or_boolean_data_is_never_hidden_by_clipping(self):
        for field in ("q", "dq", "body_omega", "command", "orientation_xyzw", "action"):
            for invalid in (float("nan"), float("inf"), True, "1.0"):
                with self.subTest(field=field, invalid=invalid):
                    self.contract.reset()
                    payload, action = frame(), [0.0] * 12
                    if field in ("q", "dq"):
                        payload["joints"][0][field] = invalid
                    elif field == "action":
                        action[0] = invalid
                    else:
                        payload[field][0] = invalid
                    self.assertFalse(self.step(payload, action)["accepted"])

    def test_stale_future_and_nonfinite_timestamps_are_rejected(self):
        for stamp in (9.0, 10.001, float("nan"), float("inf"), True):
            with self.subTest(stamp=stamp):
                self.contract.reset()
                self.assertFalse(self.step(frame(stamp))["accepted"])
        self.contract.reset()
        self.assertFalse(self.step(now=float("nan"))["accepted"])

    def test_duplicate_backwards_and_off_period_frames_are_rejected(self):
        for second_stamp in (9.99, 9.988, 9.998):
            with self.subTest(stamp=second_stamp):
                self.contract.reset()
                self.assertTrue(self.step()["accepted"])
                result = self.step(frame(second_stamp))
                self.assertFalse(result["accepted"])
                self.assertEqual(result["targets"], [])

    def test_zero_quaternion_and_wrong_shapes_are_rejected(self):
        for field, value in (("orientation_xyzw", [0.0] * 4),
                             ("body_omega", [0.0] * 2), ("command", [0.0] * 4)):
            self.contract.reset()
            payload = frame(); payload[field] = value
            self.assertFalse(self.step(payload)["accepted"])
        self.contract.reset()
        self.assertFalse(self.step(action=[0.0] * 11)["accepted"])

    def test_rejection_latches_degraded_until_explicit_reset(self):
        invalid = frame(); invalid["joints"].pop()
        self.assertEqual(self.step(invalid)["state"], "DEGRADED")
        self.assertFalse(self.step()["accepted"])
        self.contract.reset()
        self.assertTrue(self.step()["accepted"])

    def test_observation_clip_applies_after_scale(self):
        payload = frame(); payload["body_omega"][0] = 100.0
        self.assertEqual(self.step(payload)["observation"][0], 10.0)

    def test_input_is_not_mutated(self):
        payload = frame(); original = copy.deepcopy(payload)
        self.step(payload)
        self.assertEqual(payload, original)

    def test_metadata_rejects_incompatible_layout_or_period(self):
        for key, value in (("observation_dim", 48), ("quaternion_order", "wxyz"),
                           ("policy_dt", 0.01), ("decimation", 0),
                           ("omega_scale", float("nan"))):
            with self.subTest(key=key):
                metadata = self.module.default_metadata(); metadata[key] = value
                with self.assertRaises(ValueError):
                    self.module.PolicyContract(metadata)

    def test_metadata_bounds_history_and_decimation_before_allocating(self):
        for key, value in (("history_steps", 65), ("decimation", 1001)):
            with self.subTest(key=key):
                metadata = self.module.default_metadata(); metadata[key] = value
                if key == "decimation":
                    metadata["policy_dt"] = 2.002
                with self.assertRaises(ValueError):
                    self.module.PolicyContract(metadata)

    def test_metadata_joint_order_drives_both_input_and_output_mapping(self):
        metadata = self.module.default_metadata()
        metadata["joint_names"] = NAMES[3:6] + NAMES[:3] + NAMES[9:] + NAMES[6:9]
        self.contract = self.module.PolicyContract(metadata)
        payload = frame(); payload["joints"][3]["q"] = 0.125
        result = self.step(payload, action=[0.5] + [0.0] * 11)
        self.assertEqual(result["observation"][9], 0.125)
        self.assertEqual(result["targets"][0], {"index": 0, "name": "FL_hip_joint", "q_des": 0.125})

    def test_cli_demo_and_single_rejection_have_distinct_exit_status(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "decisions.json"
            demo = subprocess.run([sys.executable, str(MODULE), "--demo", "--output", str(output)],
                                  text=True, capture_output=True, timeout=5)
            self.assertEqual(demo.returncode, 0, demo.stdout + demo.stderr)
            saved = json.loads(output.read_text())
            self.assertTrue(saved["demo_passed"])
            rejected = subprocess.run([sys.executable, str(MODULE), "--case", "stale"],
                                      text=True, capture_output=True, timeout=5)
            self.assertEqual(rejected.returncode, 1, rejected.stdout + rejected.stderr)
            self.assertEqual(json.loads(rejected.stdout)["targets"], [])


if __name__ == "__main__":
    unittest.main()
