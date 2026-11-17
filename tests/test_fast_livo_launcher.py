"""Exercise opt-in startup and preserve the ordinary simulation's defaults."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class FastLivoLauncherTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory(prefix='fast livo launcher ')
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        (self.root / 'scripts').mkdir()
        for name in ('ros1_sim.sh', 'fast_livo_sim.sh'):
            shutil.copy2(ROOT / 'scripts' / name, self.root / 'scripts' / name)
        (self.root / 'scripts/compute_guard.sh').write_text(
            'compute_guard_path() { :; }\nacquire_compute_guard() { :; }\n')
        for name in ('.ros1/devel/setup.bash', '.ros1/devel/lib/liblivox_laser_simulation.so',
                     '.ros1/devel/lib/librealsense_gazebo_plugin.so',
                     '.ros1/src/livox_laser_simulation/scan_mode/mid360.csv', '.fast_livo/devel/setup.bash'):
            path = self.root / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.touch()
        binary = self.root / 'bin'
        binary.mkdir()
        self.calls = self.root / 'docker.jsonl'
        docker = binary / 'docker'
        docker.write_text('''#!/usr/bin/env python3
import json, os, sys
with open(os.environ['LAUNCH_TEST_DOCKER_LOG'], 'a') as stream:
    stream.write(json.dumps(sys.argv[1:]) + '\\n')
sys.exit(1 if sys.argv[1:3] == ['container', 'inspect'] else 0)
''')
        docker.chmod(0o755)
        self.env = dict(os.environ, PATH=str(binary) + os.pathsep + os.environ['PATH'], DISPLAY='',
                        ALIENGO_FAST_LIVO_WORKSPACE='', ALIENGO_LIO_WORKSPACE='', ALIENGO_GBP_WORKSPACE='',
                        ALIENGO_ROS1_IMAGE='', LAUNCH_TEST_DOCKER_LOG=str(self.calls))

    def launch(self, script, *arguments):
        result = subprocess.run(['bash', str(self.root / 'scripts' / script), '--world', 'earth',
                                 '--headless', *arguments], env=self.env,
                                capture_output=True, text=True, timeout=10)
        calls = [json.loads(line) for line in self.calls.read_text().splitlines()] if self.calls.exists() else []
        return result, [call for call in calls if call[0] == 'run']

    def test_ordinary_simulation_keeps_original_image_and_no_fast_livo_mount(self):
        result, runs = self.launch('ros1_sim.sh', 'd435_enabled:=false')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(len(runs), 1)
        self.assertIn('aliengo-learning:noetic', runs[0])
        self.assertNotIn('ALIENGO_FAST_LIVO_ENABLED=1', runs[0])
        self.assertFalse(any('target=/fast_livo' in arg for arg in runs[0]))
        self.assertNotIn('sim_ground_truth_tf:=false', runs[0])

    def test_separate_entry_mounts_only_fast_livo_and_disables_truth_tf(self):
        self.env['ALIENGO_LIO_WORKSPACE'] = '/unrelated/existing/workspace'
        result, runs = self.launch('fast_livo_sim.sh')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(len(runs), 1)
        self.assertIn('aliengo-learning:noetic-fast-livo', runs[0])
        self.assertIn('ALIENGO_FAST_LIVO_ENABLED=1', runs[0])
        self.assertIn('type=bind,source=%s,target=/fast_livo' % (self.root / '.fast_livo'), runs[0])
        self.assertFalse(any('target=/lio' in arg for arg in runs[0]))
        self.assertIn('sim_ground_truth_tf:=false', runs[0])

    def test_disabled_camera_is_rejected_before_container_launch(self):
        result, runs = self.launch('fast_livo_sim.sh', 'd435_enabled:=false')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('需要 D435 RGB', result.stderr)
        self.assertEqual(runs, [])


if __name__ == '__main__':
    unittest.main()
