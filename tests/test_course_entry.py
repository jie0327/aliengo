import subprocess
import os
import tempfile
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
ENTRY = ROOT / "scripts/ros1_course.sh"


class CourseEntryTests(unittest.TestCase):
    def run_entry(self, *args):
        self.assertTrue(ENTRY.is_file(), "course entry is not implemented")
        return subprocess.run(["bash", str(ENTRY), *args], text=True, capture_output=True, timeout=10)

    def test_help_explains_both_modes(self):
        result = self.run_entry("--help")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("offline", result.stdout)
        self.assertIn("sim", result.stdout)

    def test_wrong_mode_fails_before_docker(self):
        self.assertEqual(self.run_entry("wrong", "missing.py").returncode, 2)

    def test_missing_example_is_actionable(self):
        result = self.run_entry("offline", "lesson08/missing.py")
        self.assertEqual(result.returncode, 2)
        self.assertIn("lesson08/missing.py", result.stderr)

    def test_path_escape_is_rejected(self):
        result = self.run_entry("offline", "../scripts/ros1_build.sh")
        self.assertEqual(result.returncode, 2)
        self.assertIn("examples", result.stderr)

    def test_offline_uses_current_noetic_image(self):
        with tempfile.TemporaryDirectory() as tmp:
            fake = Path(tmp) / 'docker'
            calls = Path(tmp) / 'docker_calls'
            fake.write_text('#!/bin/sh\nprintf "%s\\n" "$*" >> "$DOCKER_CALLS"\n')
            fake.chmod(0o755)
            env = dict(os.environ, PATH=tmp + os.pathsep + os.environ['PATH'], DOCKER_CALLS=str(calls))
            result = subprocess.run(['bash', str(ENTRY), 'offline', 'lesson09/q_learning_speed.py'],
                                    text=True, capture_output=True, timeout=10, env=env)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('aliengo-learning:noetic', calls.read_text())
            self.assertNotIn('aliengo-exploration:simulation', calls.read_text())


if __name__ == "__main__":
    unittest.main()
