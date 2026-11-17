"""Observable checks for read-only state extraction and an offline PD toy model."""
import csv
import importlib.util
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


def load_example(name, relative_path):
    path = ROOT / "examples" / relative_path
    if not path.exists():
        return None
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


reader = load_example("lesson02_reader", "lesson02/joint_state_reader.py")
pd = load_example("lesson03_pd", "lesson03/pd_lab.py")


class JointReaderTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(reader, "lesson02/joint_state_reader.py must exist")

    def test_reordered_arrays_still_identify_right_front_leg(self):
        values, issues = reader.extract_fr_positions(
            ["FR_calf_joint", "FL_hip_joint", "FR_hip_joint", "FR_thigh_joint"],
            [-1.4, 8.0, 0.1, 0.7],
        )
        self.assertEqual(values, {"FR_hip_joint": 0.1, "FR_thigh_joint": 0.7,
                                  "FR_calf_joint": -1.4})
        self.assertEqual(issues, [])

    def test_optional_position_and_missing_joint_are_unavailable_not_zero(self):
        values, issues = reader.extract_fr_positions(["FR_hip_joint"], [])
        self.assertIsNone(values["FR_hip_joint"])
        self.assertIsNone(values["FR_thigh_joint"])
        self.assertTrue(any("FR_hip_joint" in item for item in issues))
        self.assertTrue(any("FR_thigh_joint" in item for item in issues))

    def test_duplicate_name_and_nan_never_become_a_valid_sample(self):
        values, issues = reader.extract_fr_positions(
            ["FR_hip_joint", "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint"],
            [0.1, 0.2, float("nan"), -1.4],
        )
        self.assertIsNone(values["FR_hip_joint"])
        self.assertIsNone(values["FR_thigh_joint"])
        self.assertTrue(issues)

    def test_throttle_counts_printed_complete_samples_and_recovers(self):
        state = reader.ReaderState(rate=2.0)
        names = ["FR_hip_joint", "FR_thigh_joint", "FR_calf_joint"]
        self.assertIn("N/A", state.consume(names, [], now=0.0))
        self.assertEqual(state.samples, 0)
        self.assertIsNone(state.consume(names, [0.1, 0.7, -1.4], now=0.1))
        self.assertEqual(state.samples, 0)
        self.assertIn("0.7000", state.consume(names, [0.1, 0.7, -1.4], now=0.5))
        self.assertEqual(state.samples, 1)
        self.assertEqual(state.messages, 3)

    def test_help_and_invalid_inputs_work_without_ros_import(self):
        path = ROOT / "examples/lesson02/joint_state_reader.py"
        help_run = subprocess.run([sys.executable, str(path), "--help"],
                                  capture_output=True, text=True)
        self.assertEqual(help_run.returncode, 0, help_run.stderr)
        for args in (["--samples", "0"], ["--timeout", "nan"], ["--rate", "-1"]):
            result = subprocess.run([sys.executable, str(path), *args],
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 2, result.stderr)

    def test_offline_demo_exposes_reordering_and_missing_data(self):
        result = subprocess.run(
            [sys.executable, str(ROOT / "examples/lesson02/joint_state_reader.py"), "--demo"],
            capture_output=True, text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.count("FR_thigh_joint=0.7000"), 2)
        self.assertIn("FR_calf_joint=N/A", result.stdout)


class PdLabTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(pd, "lesson03/pd_lab.py must exist")

    def test_paper_example_and_symmetric_torque_saturation(self):
        raw, applied = pd.pd_torque(0.4, 0.2, target=0.5, kp=20, kd=1,
                                   feedforward=0, torque_limit=3)
        self.assertAlmostEqual(raw, 1.8)
        self.assertAlmostEqual(applied, 1.8)
        self.assertEqual(pd.pd_torque(0, 0, target=1, kp=20, kd=1,
                                     feedforward=0, torque_limit=3), (20, 3))
        self.assertEqual(pd.pd_torque(2, 0, target=1, kp=20, kd=1,
                                     feedforward=0, torque_limit=3), (-20, -3))

    def test_one_step_uses_inertia_damping_and_resisting_load(self):
        config = pd.Config(inertia=2, damping=0.5, kp=0, kd=0, load=1,
                           feedforward=3, q0=0, dq0=2, dt=0.1, duration=0.1)
        rows = pd.simulate(config)
        self.assertEqual(len(rows), 2)
        # Net torque = 3 - 0.5*2 - 1 = 1; acceleration = 0.5 rad/s².
        self.assertAlmostEqual(rows[-1]["dq_rad_s"], 2.05)
        self.assertAlmostEqual(rows[-1]["q_rad"], 0.205)
        self.assertAlmostEqual(rows[-1]["t_s"], 0.1)

    def test_fixed_load_causes_error_and_matching_feedforward_cancels_it(self):
        no_ff = pd.simulate(pd.Config(duration=6))
        with_ff = pd.simulate(pd.Config(duration=6, feedforward=0.2))
        self.assertAlmostEqual(no_ff[-1]["q_rad"], 0.45, places=4)
        self.assertAlmostEqual(with_ff[-1]["q_rad"], 0.5, places=4)

    def test_larger_damping_reduces_overshoot_in_this_experiment(self):
        low = pd.simulate(pd.Config(kd=0.05, load=0))
        high = pd.simulate(pd.Config(kd=0.8, load=0))
        self.assertGreater(max(row["q_rad"] for row in low), 0.7)
        self.assertLess(max(row["q_rad"] for row in high), 0.51)

    def test_fractional_last_step_ends_at_requested_time(self):
        rows = pd.simulate(pd.Config(duration=0.25, dt=0.1))
        self.assertEqual([row["t_s"] for row in rows], [0.0, 0.1, 0.2, 0.25])

    def test_invalid_or_unbounded_workload_rejected(self):
        for changes in ({"inertia": 0}, {"dt": 0}, {"kd": -1},
                        {"load": float("nan")}, {"dt": 1e-12},
                        {"torque_limit": -1}, {"duration": -1}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                pd.simulate(pd.Config(**changes))

    def test_extreme_finite_time_ratio_is_a_clean_validation_error(self):
        with self.assertRaises(ValueError):
            pd.simulate(pd.Config(duration=1e308, dt=1e-308))

    def test_cli_writes_csv_and_reports_a_failed_input(self):
        path = ROOT / "examples/lesson03/pd_lab.py"
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "nested" / "trace.csv"
            result = subprocess.run([sys.executable, str(path), "--duration", "0.1",
                                     "--dt", "0.05", "--csv", str(output)],
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            with output.open(newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 3)
            self.assertAlmostEqual(float(rows[-1]["t_s"]), 0.1)
            self.assertIn("q(rad)", result.stdout)
        result = subprocess.run([sys.executable, str(path), "--inertia", "0"],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 2, result.stderr)


if __name__ == "__main__":
    unittest.main()
