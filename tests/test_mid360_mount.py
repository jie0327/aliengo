"""Check the expanded mounting geometry actually bridges the visible gap."""
from pathlib import Path
import subprocess
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]


class Mid360MountTest(unittest.TestCase):
    def test_mount_reaches_trunk_and_sensor_after_height_change(self):
        for sensor_z in (0.145, 0.165):
            with self.subTest(sensor_z=sensor_z):
                result = subprocess.run([
                    "docker", "run", "--rm", "--network", "none",
                    "--memory=512m", "--memory-swap=512m", "--cpus=1",
                    "--mount", f"type=bind,source={ROOT},target=/course,readonly",
                    "--mount", f"type=bind,source={ROOT / '.ros1'},target=/work,readonly",
                    "--entrypoint", "bash", "aliengo-learning:noetic", "-c",
                    "set -e; source /opt/ros/noetic/setup.bash; "
                    "source /work/devel/setup.bash; "
                    "xacro /course/ros1/mid360/robot.xacro DEBUG:=false "
                    "mid360_x:=0.3 mid360_z:=\"$1\"", "bash", str(sensor_z),
                ], capture_output=True, text=True, check=True, timeout=30)
                robot = ET.fromstring(result.stdout)
                mount = robot.find("link[@name='mid360_mount_link']")
                self.assertIsNotNone(mount, "sensor is floating without mounting geometry")
                joint = robot.find("joint[@name='mid360_mount_joint']")
                self.assertEqual(joint.get("type"), "fixed")
                self.assertEqual(joint.find("parent").get("link"), "trunk")
                origin = [float(v) for v in joint.find("origin").get("xyz").split()]
                self.assertAlmostEqual(origin[0], 0.3)
                intervals = []
                for collision in mount.findall("collision"):
                    z = float(collision.find("origin").get("xyz").split()[2])
                    height = float(collision.find("geometry/box").get("size").split()[2])
                    self.assertGreater(height, 0)
                    intervals.append((origin[2] + z - height / 2,
                                      origin[2] + z + height / 2))
                intervals.sort()
                self.assertAlmostEqual(intervals[0][0], 0.056)
                covered_to = intervals[0][1]
                for low, high in intervals[1:]:
                    self.assertLessEqual(low, covered_to + 1e-9)
                    covered_to = max(covered_to, high)
                self.assertAlmostEqual(covered_to, sensor_z - 0.047)
                self.assertGreater(float(mount.find("inertial/mass").get("value")), 0)
                inertia = mount.find("inertial/inertia")
                diagonal = [float(inertia.get(key)) for key in ("ixx", "iyy", "izz")]
                self.assertTrue(all(value > 0 for value in diagonal))
                self.assertLessEqual(max(diagonal), sum(diagonal) - max(diagonal) + 1e-12)


if __name__ == "__main__":
    unittest.main()
