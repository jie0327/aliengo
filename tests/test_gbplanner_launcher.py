"""Exercise host preflight with Catkin setup files from container builds."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class GbplannerLauncherTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='gbp launcher ')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        (self.root/'scripts').mkdir()
        shutil.copy2(ROOT/'scripts/ros1_sim.sh', self.root/'scripts/ros1_sim.sh')
        (self.root/'scripts/compute_guard.sh').write_text(
            'compute_guard_path() { :; }\nacquire_compute_guard() { :; }\n')
        for name in ('devel/setup.bash', 'devel/lib/liblivox_laser_simulation.so',
                     'src/livox_laser_simulation/scan_mode/mid360.csv'):
            path = self.root/'.ros1'/name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.touch()
        self.workspace = self.root/'planner workspace'
        (self.workspace/'devel').mkdir(parents=True)
        self.commands = self.root/'docker.jsonl'
        binary = self.root/'bin'
        binary.mkdir()
        docker = binary/'docker'
        docker.write_text('''#!/usr/bin/env python3
import json, os, sys
with open(os.environ['LAUNCH_TEST_DOCKER_LOG'], 'a') as stream:
    stream.write(json.dumps(sys.argv[1:])+'\\n')
sys.exit(1 if sys.argv[1:3] == ['container', 'inspect'] else 0)
''')
        docker.chmod(0o755)
        self.env = dict(os.environ, PATH=str(binary)+os.pathsep+os.environ['PATH'],
                        ALIENGO_ROS1_IMAGE='test-exploration', ALIENGO_LIO_WORKSPACE='',
                        ALIENGO_GBP_WORKSPACE=str(self.workspace),
                        LAUNCH_TEST_DOCKER_LOG=str(self.commands))

    def launch(self):
        result = subprocess.run(['bash', str(self.root/'scripts/ros1_sim.sh'),
                                 '--world', 'earth', '--headless', 'd435_enabled:=false'],
                                env=self.env, capture_output=True, text=True, timeout=10)
        calls = [json.loads(line) for line in self.commands.read_text().splitlines()]
        return result, [call for call in calls if call[0] == 'run']

    def assert_launches_with_correct_mount(self):
        result, runs = self.launch()
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
        self.assertEqual(len(runs), 1)
        self.assertIn('type=bind,source=%s,target=/gbp' % self.workspace, runs[0])

    def test_regular_setup_file_launches(self):
        (self.workspace/'devel/setup.bash').write_text('# generated setup\n')
        self.assert_launches_with_correct_mount()

    def test_absolute_container_setup_link_launches(self):
        target = self.workspace/'devel/.private/catkin_tools_prebuild/setup.bash'
        target.parent.mkdir(parents=True)
        target.write_text('# linked Catkin setup\n')
        entry = self.workspace/'devel/setup.bash'
        entry.symlink_to('/gbp/devel/.private/catkin_tools_prebuild/setup.bash')
        self.assertTrue(entry.is_symlink())
        self.assertFalse(entry.is_file(), 'fixture must be unresolved on the host')
        self.assert_launches_with_correct_mount()

    def test_missing_link_target_is_rejected_before_launch(self):
        (self.workspace/'devel/setup.bash').symlink_to(
            '/gbp/devel/.private/catkin_tools_prebuild/setup.bash')
        result, runs = self.launch()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('GBPlanner 工作区尚未构建', result.stderr)
        self.assertFalse(runs)

    def test_missing_setup_is_rejected_before_launch(self):
        result, runs = self.launch()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('GBPlanner 工作区尚未构建', result.stderr)
        self.assertFalse(runs)


if __name__ == '__main__':
    unittest.main()
