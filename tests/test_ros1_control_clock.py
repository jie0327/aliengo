"""Exercise Guide's real C++ timing helper against ROS simulated time."""
from pathlib import Path
import subprocess
import unittest


ROOT = Path(__file__).resolve().parents[1]


class ControlClockTest(unittest.TestCase):
    def compile_and_run(self, source, simulation=False):
        command = [
            "docker", "run", "--rm", "-i", "--network", "none",
            "--memory=512m", "--memory-swap=512m", "--cpus=1", "--pids-limit=64",
            "--mount", f"type=bind,source={ROOT / '.ros1'},target=/work,readonly",
            "--entrypoint", "bash", "aliengo-learning:noetic", "-c",
            "set -e; source /opt/ros/noetic/setup.bash; "
            "g++ -std=c++11 -pthread -I/work/src/unitree_guide/include "
            "-I/opt/ros/noetic/include \"$@\" -x c++ - -o /tmp/control-clock "
            "-L/opt/ros/noetic/lib -lrostime -lcpp_common; /tmp/control-clock",
            "bash",
        ]
        if simulation:
            command.append("-DCOMPILE_WITH_SIMULATION")
        result = subprocess.run(command, input=source, capture_output=True,
                                text=True, timeout=45)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_simulation_clock_pauses_and_waits_for_physics(self):
        self.compile_and_run(r'''
#include <cassert>
#include <chrono>
#include <thread>
#include <ros/time.h>
#include "common/timeMarker.h"
int main() {
    ros::Time::init();
    ros::Time::setNow(ros::Time(12, 345000000));
    assert(getSystemTime() == 12345000);
    usleep(10000);
    assert(getSystemTime() == 12345000);
    auto wall = std::chrono::steady_clock::now();
    std::thread physics([] {
        usleep(100000);
        ros::Time::setNow(ros::Time(12, 347000000));
    });
    absoluteWait(12345000, 2000);
    double waited = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - wall).count();
    physics.join();
    assert(waited >= 0.08);
    assert(getSystemTime() == 12347000);
    ros::Time::shutdown();
}
''', simulation=True)

    def test_non_simulation_clock_still_advances_in_wall_time(self):
        self.compile_and_run(r'''
#include <cassert>
#include "common/timeMarker.h"
int main() {
    auto before = getSystemTime();
    usleep(10000);
    assert(getSystemTime() > before);
}
''')


if __name__ == "__main__":
    unittest.main()
