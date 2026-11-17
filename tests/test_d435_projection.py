"""Exercise the RGB projection and depth conversion used by the patched plugin."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class D435ProjectionTest(unittest.TestCase):
    def test_depth_pixels_project_into_distinct_color_optics(self):
        # Removing the extrinsic/FOV projection must break these numeric fixtures.
        with tempfile.TemporaryDirectory(prefix="d435-projection-") as directory:
            stage = Path(directory)
            source = stage / "realsense_gazebo_plugin"
            shutil.copytree(ROOT / "references/realsense_gazebo_plugin", source)
            patch = ROOT / "ros1/patches/realsense_rgbd.patch"
            if patch.exists():
                subprocess.run(["patch", "--batch", "-p0", "-i", str(patch)],
                               cwd=stage, check=True, capture_output=True, text=True)
            self.assertTrue(
                (source / "include/realsense_gazebo_plugin/d435_projection.h").exists(),
                "Original PAL cloud samples matching row/column instead of projecting RGB optics",
            )
            executable = stage / "projection_test"
            compiled = subprocess.run([
                "g++", "-std=c++11", "-Wall", "-Wextra", "-Werror",
                "-I", str(source / "include"),
                str(ROOT / "tests/fixtures/d435_projection.cpp"), "-o", str(executable),
            ], capture_output=True, text=True)
            self.assertEqual(compiled.returncode, 0, compiled.stdout + compiled.stderr)
            result = subprocess.run([str(executable)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
