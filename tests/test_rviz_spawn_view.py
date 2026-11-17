"""Keep RViz's initial world view near the selected spawn without losing ROS remaps."""
import copy
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'ros1/mid360/rviz_spawn_view.py'
TEMPLATE = ROOT / 'ros1/mid360/pointcloud.rviz'


class RvizSpawnViewTests(unittest.TestCase):
    def setUp(self):
        self.assertTrue(SCRIPT.is_file(), 'RViz spawn prefix is not implemented')
        spec = importlib.util.spec_from_file_location('rviz_spawn_view', SCRIPT)
        self.view = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.view)
        self.template = yaml.safe_load(TEMPLATE.read_text())

    def test_depot_view_uses_spawn_in_world_coordinates(self):
        rendered = self.view.render_config(self.template, (-5., 0., .6))
        manager = rendered['Visualization Manager']
        current = manager['Views']['Current']
        grid = next(d for d in manager['Displays'] if d['Class'] == 'rviz/Grid')
        self.assertEqual(manager['Global Options']['Fixed Frame'], 'world')
        self.assertEqual(current['Class'], 'rviz/Orbit')
        self.assertEqual(current['Target Frame'], '<Fixed Frame>')
        self.assertEqual(current['Focal Point'], {'X': -5., 'Y': 0., 'Z': .6})
        self.assertEqual(grid['Reference Frame'], '<Fixed Frame>')
        self.assertEqual(grid['Offset'], {'X': -5., 'Y': 0., 'Z': 0.})

    def test_city_view_keeps_its_elevated_spawn_and_reference_plane(self):
        rendered = self.view.render_config(self.template, (-45., -25., 5.61))
        manager = rendered['Visualization Manager']
        self.assertEqual(manager['Views']['Current']['Focal Point'],
                         {'X': -45., 'Y': -25., 'Z': 5.61})
        grid = next(d for d in manager['Displays'] if d['Class'] == 'rviz/Grid')
        self.assertEqual(grid['Offset']['X'], -45.)
        self.assertEqual(grid['Offset']['Y'], -25.)
        self.assertAlmostEqual(grid['Offset']['Z'], 5.01)

    def test_render_leaves_template_and_unrelated_settings_unchanged(self):
        original = copy.deepcopy(self.template)
        rendered = self.view.render_config(self.template, (-45., -25., 5.61))
        self.assertEqual(self.template, original)
        expected = copy.deepcopy(original)
        manager = expected['Visualization Manager']
        manager['Global Options']['Fixed Frame'] = 'world'
        manager['Views']['Current']['Class'] = 'rviz/Orbit'
        manager['Views']['Current']['Target Frame'] = '<Fixed Frame>'
        manager['Views']['Current']['Focal Point'] = {'X': -45., 'Y': -25., 'Z': 5.61}
        for display in manager['Displays']:
            if display['Class'] == 'rviz/Grid':
                display['Reference Frame'] = '<Fixed Frame>'
                display['Offset'] = {'X': -45., 'Y': -25., 'Z': 5.61 - .6}
        self.assertEqual(rendered, expected)
        rendered['Visualization Manager']['Views']['Current']['Distance'] = 99
        self.assertEqual(self.template, original)

    def test_prefix_executes_original_argv_with_only_display_config_replaced(self):
        with tempfile.TemporaryDirectory(prefix='rviz spawn test ') as directory:
            source = Path(directory) / 'original view.rviz'
            source.write_text(TEMPLATE.read_text())
            original_bytes = source.read_bytes()
            tail = ['--no-stereo', '-d', str(source), '__name:=mid360_rviz',
                    '__log:=/tmp/rviz log with spaces.log', '/livox/lidar:=/livox/lidar']
            capture = 'import json,os,sys; print(json.dumps(dict(pid=os.getpid(), argv=sys.argv[1:])))'
            process = subprocess.Popen(
                [str(SCRIPT), '-45', '-25', '5.61', sys.executable, '-c', capture, *tail],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                env={**os.environ, 'PYTHONDONTWRITEBYTECODE': '1'})
            stdout, stderr = process.communicate(timeout=10)
            self.assertEqual(process.returncode, 0, stderr)
            executed = json.loads(stdout)
            self.assertEqual(executed['pid'], process.pid)
            generated = Path(executed['argv'][2])
            try:
                self.assertNotEqual(generated, source)
                expected_argv = list(tail)
                expected_argv[2] = str(generated)
                self.assertEqual(executed['argv'], expected_argv)
                config = yaml.safe_load(generated.read_text())
                self.assertEqual(config['Visualization Manager']['Views']['Current']['Focal Point'],
                                 {'X': -45., 'Y': -25., 'Z': 5.61})
                self.assertEqual(source.read_bytes(), original_bytes)
            finally:
                generated.unlink()


if __name__ == '__main__':
    unittest.main()
