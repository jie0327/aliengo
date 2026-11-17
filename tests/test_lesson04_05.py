"""Behavioral checks for the offline kinematics and gait lessons (stdlib only)."""

import csv
import importlib.util
import io
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


def example(test, relative):
    path = ROOT / "examples" / relative
    test.assertTrue(path.is_file(), f"Missing lesson implementation: {relative}")
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class UrdfKinematicsTests(unittest.TestCase):
    def model(self):
        module = example(self, "lesson04/urdf_fk.py")
        path = ROOT / ".ros1/aliengo.urdf"
        if not path.is_file():
            self.skipTest("Generate actual AlienGo URDF: ./scripts/ros1_model.sh")
        return module, module.UrdfModel(path)

    def assertPosition(self, actual, expected):
        for value, want in zip(actual, expected):
            self.assertAlmostEqual(value, want, places=11)

    def test_actual_zero_reference_all_legs_includes_hip_and_foot_offsets(self):
        # Missing a fixed foot offset or swapping left/right fails these hand sums.
        module, model = self.model()
        for leg, want in [("FR", (.2407, -.1378, -.5)),
                          ("FL", (.2407, .1378, -.5)),
                          ("RR", (-.2407, -.1378, -.5)),
                          ("RL", (-.2407, .1378, -.5))]:
            with self.subTest(leg=leg):
                self.assertPosition(module.foot_position(model, leg, (0, 0, 0),
                                                        check_limits=False), want)

    def test_actual_bent_leg_rotation_sign_all_legs(self):
        # Downward thigh rotated +pi/2 about +y must move in negative x.
        module, model = self.model()
        for leg, want in [("FR", (-.0093, -.1378, -.25)),
                          ("FL", (-.0093, .1378, -.25)),
                          ("RR", (-.4907, -.1378, -.25)),
                          ("RL", (-.4907, .1378, -.25))]:
            with self.subTest(leg=leg):
                self.assertPosition(module.foot_position(model, leg, (0, math.pi/2, -math.pi/2)), want)

    def test_hip_rotation_changes_lateral_and_vertical_coordinates(self):
        module, model = self.model()
        self.assertPosition(module.foot_position(model, "FR", (math.pi/6, 0, -math.pi/2)),
                            (.4907, -.001171005048489, -.259906350946110))

    def test_zero_calf_is_rejected_unless_mathematical_reference_requested(self):
        module, model = self.model()
        with self.assertRaisesRegex(ValueError, "FR_calf_joint"):
            module.foot_position(model, "FR", (0, 0, 0))
        with self.assertRaises(ValueError):
            module.foot_position(model, "FR", (0, float("nan"), -1))
        with self.assertRaises(ValueError):
            module.foot_position(model, "FR", (0, -1))

    def test_nonzero_origin_rotation_is_applied_before_axis_rotation(self):
        # Parent origin (1,2,3), Rz(pi/2), local Ry(pi/2), then local foot -z.
        module = example(self, "lesson04/urdf_fk.py")
        xml = '''<robot name="fixture"><link name="root"/><link name="arm"/>
        <link name="tip"/><joint name="bend" type="revolute">
        <parent link="root"/><child link="arm"/>
        <origin xyz="1 2 3" rpy="0 0 1.5707963267948966"/>
        <axis xyz="0 1 0"/><limit lower="-2" upper="2"/></joint>
        <joint name="end" type="fixed"><parent link="arm"/><child link="tip"/>
        <origin xyz="0 0 -1"/></joint></robot>'''
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "fixture.urdf"
            path.write_text(xml)
            model = module.UrdfModel(path)
            matrix = model.forward("root", "tip", {"bend": math.pi/2})
            self.assertPosition(tuple(row[3] for row in matrix[:3]), (1, 1, 3))
            with self.assertRaises(ValueError):
                model.forward("root", "tip", {})

    def test_missing_model_cli_reports_generation_command(self):
        path = ROOT / "examples/lesson04/urdf_fk.py"
        self.assertTrue(path.is_file(), "Missing FK CLI")
        result = subprocess.run([sys.executable, str(path), "--urdf", "/missing/model.urdf"],
                                text=True, capture_output=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("./scripts/ros1_model.sh", result.stderr)


class PlanarInverseTests(unittest.TestCase):
    def test_right_angle_target_has_two_geometrically_distinct_solutions(self):
        module = example(self, "lesson04/planar_ik.py")
        solutions = module.planar_ik(.25, .25)
        self.assertEqual(len(solutions), 2)
        solutions = sorted(solutions, key=lambda pair: pair[1])
        for actual, want in zip(solutions, [(math.pi/2, -math.pi/2), (0, math.pi/2)]):
            for angle, expected in zip(actual, want):
                self.assertAlmostEqual(angle, expected, places=12)

    def test_forward_inverse_roundtrip_recovers_position_for_both_branches(self):
        module = example(self, "lesson04/planar_ik.py")
        for q in [(.3, -.9), (-1.1, 1.8), (.7, 0)]:
            target = module.planar_fk(*q)
            for solution in module.planar_ik(*target):
                reconstructed = module.planar_fk(*solution)
                for value, want in zip(reconstructed, target):
                    self.assertAlmostEqual(value, want, places=12)

    def test_unreachable_inner_outer_and_nonfinite_targets_are_rejected(self):
        module = example(self, "lesson04/planar_ik.py")
        for x, y, l1, l2 in [(.51, 0, .25, .25), (.05, 0, .3, .2),
                              (float("nan"), 0, .25, .25), (.1, .1, 0, .25)]:
            with self.subTest(target=(x, y, l1, l2)), self.assertRaises(ValueError):
                module.planar_ik(x, y, l1, l2)

    def test_extended_boundary_has_one_solution_and_origin_is_underdetermined(self):
        module = example(self, "lesson04/planar_ik.py")
        self.assertEqual(len(module.planar_ik(.5, 0)), 1)
        with self.assertRaisesRegex(ValueError, "singular"):
            module.planar_ik(0, 0)

    def test_joint_limits_filter_branch_instead_of_clipping_angles(self):
        module = example(self, "lesson04/planar_ik.py")
        solutions = module.planar_ik(.25, .25, limits=((-2, 2), (-2, -.5)))
        self.assertEqual(len(solutions), 1)
        self.assertAlmostEqual(solutions[0][1], -math.pi/2)
        with self.assertRaisesRegex(ValueError, "limit"):
            module.planar_ik(.25, .25, limits=((-2, 2), (-.4, .4)))


class GaitTableTests(unittest.TestCase):
    def test_diagonal_contact_boundary_and_normalized_phase(self):
        module = example(self, "lesson05/gait_table.py")
        for time, contacts, phase in [(0, (1, 0, 0, 1), 0),
                                      (.125, (1, 0, 0, 1), .5),
                                      (.25, (0, 1, 1, 0), 0),
                                      (.5, (1, 0, 0, 1), 0)]:
            states = module.sample(time, period=.5, stance_ratio=.5)
            self.assertEqual(tuple(state["contact"] for state in states), contacts)
            for state in states:
                self.assertAlmostEqual(state["phase"], phase)

    def test_cycloid_lift_has_zero_endpoint_speed_and_known_apex(self):
        module = example(self, "lesson05/gait_table.py")
        for phase, want in [(0, (0, 0, 0, 0)), (.5, (.05, .04, .4, 0)),
                            (1, (.1, 0, 0, 0))]:
            values = module.swing(phase, duration=.5, stride=.1, height=.04)
            for value, expected in zip(values, want):
                self.assertAlmostEqual(value, expected, places=12)

    def test_invalid_timing_and_geometry_are_rejected(self):
        module = example(self, "lesson05/gait_table.py")
        for kwargs in [dict(period=0), dict(stance_ratio=1), dict(stance_ratio=0),
                       dict(time=-.1), dict(height=-.01), dict(time=float("inf"))]:
            arguments = dict(time=0)
            arguments.update(kwargs)
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                module.sample(**arguments)

    def test_csv_cli_writes_four_rows_per_sample_and_returns_to_first_phase(self):
        path = ROOT / "examples/lesson05/gait_table.py"
        self.assertTrue(path.is_file(), "Missing gait CLI")
        result = subprocess.run([sys.executable, str(path), "--period", ".5", "--duration", ".5",
                                 "--dt", ".25"], text=True, capture_output=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        rows = list(csv.DictReader(io.StringIO(result.stdout)))
        self.assertEqual(len(rows), 12)
        self.assertEqual([row["leg"] for row in rows[:4]], ["FR", "FL", "RR", "RL"])
        self.assertEqual(rows[0]["contact"], rows[8]["contact"])
        self.assertAlmostEqual(float(rows[-1]["time_s"]), .5)

    def test_csv_cli_rejects_unbounded_or_overflowing_sample_count_before_writing(self):
        path = ROOT / "examples/lesson05/gait_table.py"
        self.assertTrue(path.is_file(), "Missing gait CLI")
        for duration, dt in [("1e308", "1e-308"), ("100000", "1")]:
            result = subprocess.run([sys.executable, str(path), "--duration", duration, "--dt", dt],
                                    text=True, capture_output=True, timeout=3)
            self.assertEqual(result.returncode, 2, result.stderr)
            self.assertEqual(result.stdout, "")
            self.assertIn("sample", result.stderr)
            self.assertNotIn("Traceback", result.stderr)


if __name__ == "__main__":
    unittest.main()
