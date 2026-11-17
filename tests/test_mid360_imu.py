"""Check that Gazebo orientation is not exposed as measured IMU attitude."""
from pathlib import Path
import shutil
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[1]


class Mid360ImuTest(unittest.TestCase):
    def test_converter_preserves_sample_and_marks_orientation_unavailable(self):
        if not shutil.which("docker"):
            self.skipTest("Docker is unavailable")
        command = [
            "docker", "run", "--rm", "--network", "none",
            "--mount", f"type=bind,source={ROOT},target=/course,readonly",
            "--entrypoint", "bash", "aliengo-learning:noetic", "-c",
            "source /opt/ros/noetic/setup.bash; "
            "PYTHONPATH=/course/ros1/mid360/scripts:$PYTHONPATH python3 -c "
            "'from imu_converter import convert; from sensor_msgs.msg import Imu; "
            "m=Imu(); m.header.frame_id=\"mid360_imu_link\"; m.header.stamp.secs=7; "
            "m.angular_velocity.x=2; m.linear_acceleration.z=9.81; m.orientation.x=0.5; "
            "out=convert(m); assert out.header.stamp.secs == 7; "
            "assert out.header.frame_id == \"mid360_imu_link\"; "
            "assert out.angular_velocity.x == 2 and out.linear_acceleration.z == 9.81; "
            "assert out.orientation_covariance[0] == -1 and out.orientation.x == 0'",
        ]
        subprocess.run(command, check=True, capture_output=True, text=True)


if __name__ == "__main__":
    unittest.main()
