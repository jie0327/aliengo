"""Expand the actual ROS 1 simulation model when the Noetic workspace exists."""
from pathlib import Path
import shutil
import subprocess
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


class Mid360SimulationTest(unittest.TestCase):
    def test_point_cloud_conversion_preserves_xyz_and_header(self):
        if not shutil.which("docker") or not (ROOT / ".ros1/devel/setup.bash").exists():
            self.skipTest("ROS 1 Docker workspace is unavailable")
        command = [
            "docker", "run", "--rm", "--network", "none",
            "--mount", f"type=bind,source={ROOT},target=/course,readonly",
            "--entrypoint", "bash", "aliengo-learning:noetic", "-c",
            "source /opt/ros/noetic/setup.bash; "
            "PYTHONPATH=/course/ros1/mid360/scripts:$PYTHONPATH python3 -c "
            "'from cloud_converter import convert; from sensor_msgs.msg import PointCloud; "
            "from geometry_msgs.msg import Point32; from sensor_msgs.point_cloud2 import read_points; "
            "m=PointCloud(); m.header.frame_id=\"mid360_link\"; m.points=[Point32(1,2,3)]; "
            "out=convert(m); assert out.header.frame_id == m.header.frame_id; "
            "assert list(read_points(out)) == [(1.,2.,3.)]'",
        ]
        subprocess.run(command, check=True, capture_output=True, text=True)

    def test_model_has_configurable_lidar_frame_and_cloud(self):
        if not shutil.which("docker") or not (ROOT / ".ros1/devel/setup.bash").exists():
            self.skipTest("ROS 1 Docker workspace is unavailable")
        command = [
            "docker", "run", "--rm", "--network", "none",
            "--mount", f"type=bind,source={ROOT},target=/course,readonly",
            "--mount", f"type=bind,source={ROOT / '.ros1'},target=/work,readonly",
            "--entrypoint", "bash", "aliengo-learning:noetic", "-c",
            "source /opt/ros/noetic/setup.bash; source /work/devel/setup.bash; "
            "xacro /course/ros1/mid360/robot.xacro DEBUG:=false mid360_x:=0.3",
        ]
        result = subprocess.run(command, check=True, capture_output=True, text=True)
        robot = ET.fromstring(result.stdout)
        joint = robot.find("joint[@name='mid360_joint']")
        self.assertIsNotNone(joint)
        self.assertEqual(joint.find("parent").get("link"), "trunk")
        self.assertEqual(joint.find("child").get("link"), "mid360_link")
        self.assertEqual(joint.find("origin").get("xyz").split()[0], "0.3")
        plugin = robot.find("gazebo[@reference='mid360_link']/sensor/plugin")
        self.assertEqual(plugin.get("filename"), "liblivox_laser_simulation.so")
        self.assertEqual(plugin.findtext("frame_name"), "mid360_link")
        self.assertEqual(plugin.findtext("ros_topic"), "/mid360_sim/points_raw")
        self.assertTrue(plugin.findtext("csv_file_name").endswith("/scan_mode/mid360.csv"))
