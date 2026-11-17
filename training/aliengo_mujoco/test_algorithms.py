"""Lightweight algorithm and CLI checks: no ML framework or simulation."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

HERE = Path(__file__).resolve().parent

class AlgorithmTest(unittest.TestCase):
    def test_each_algorithm_is_selectable_without_starting_training(self):
        for algorithm in ('ppo', 'a2c', 'sac'):
            result = subprocess.run([sys.executable, str(HERE / 'run.py'), '--check',
                                     '--algorithm', algorithm], capture_output=True, text=True, check=True)
            config = json.loads(result.stdout)
            self.assertEqual(config['algorithm'], algorithm)
            self.assertEqual(config['device'], 'cpu')
            self.assertEqual(config['threads'], 1)

    def test_training_budget_must_be_aligned_for_fair_comparison(self):
        from config import validate_steps
        self.assertTrue(validate_steps(4096))
        self.assertTrue(validate_steps(100096))
        self.assertFalse(validate_steps(4097))
        self.assertFalse(validate_steps(8193))
        self.assertFalse(validate_steps(255))

    def test_launcher_rejects_invalid_algorithm_and_step_count_before_docker(self):
        launcher = HERE.parents[1] / 'scripts' / 'rl_train.sh'
        for args in (('unknown', 'run-01', '4096'), ('ppo', 'run-01', '4097')):
            result = subprocess.run([str(launcher), *args], capture_output=True, text=True)
            self.assertEqual(result.returncode, 2)

    def test_launcher_accepts_long_budget_before_docker(self):
        launcher = HERE.parents[1] / 'scripts' / 'rl_train.sh'
        # Fake the guard's commands: this checks parsing without using Docker or
        # touching the real user's lock.
        test_id = '9' + str(os.getpid())
        lock = Path('/tmp') / ('aliengo-learning-compute-' + test_id + '.lock')
        with tempfile.TemporaryDirectory() as directory:
            for name, body in (('id', 'echo "$PPO_TEST_ID"'),
                               ('flock', 'exit 0'), ('docker', 'exit 1')):
                path = Path(directory) / name
                path.write_text('#!/bin/sh\n' + body + '\n')
                path.chmod(0o755)
            env = dict(os.environ, PATH=directory + ':' + os.environ['PATH'], PPO_TEST_ID=test_id)
            try:
                result = subprocess.run([str(launcher), 'ppo', 'run-01', '100096'],
                                        capture_output=True, text=True, env=env)
                self.assertEqual(result.returncode, 1)
                self.assertIn('无法检查 Docker', result.stderr)
            finally:
                lock.unlink(missing_ok=True)

if __name__ == '__main__':
    unittest.main()
