"""Exercise lock and container checks with a fake Docker CLI; no containers."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

GUARD = Path(__file__).resolve().parents[2] / 'scripts' / 'compute_guard.sh'

class ComputeGuardTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = Path(self.tmp.name)
        docker = self.path / 'docker'
        docker.write_text('#!/bin/sh\nif [ "$1" = ps ]; then echo "$MOCK_RUNNING"; else exit 1; fi\n')
        docker.chmod(0o755)
        self.env = dict(os.environ, PATH=str(self.path) + ':' + os.environ['PATH'])
        self.lock = self.path / 'guard.lock'
    def tearDown(self): self.tmp.cleanup()
    def invoke(self, env=None):
        return subprocess.run(['bash', '-c', 'source "$1"; acquire_compute_guard "$2"',
                               'bash', str(GUARD), str(self.lock)], env=env or self.env,
                              capture_output=True, text=True)
    def test_existing_sim_blocks_training(self):
        env = dict(self.env, MOCK_RUNNING='aliengo-ros1-sim')
        result = self.invoke(env)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('aliengo-ros1-sim', result.stderr)
    def test_shared_lock_rejects_second_process(self):
        holder = subprocess.Popen(['bash', '-c', 'source "$1"; acquire_compute_guard "$2" || exit; echo READY; read -r line',
                                   'bash', str(GUARD), str(self.lock)], env=self.env,
                                  stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
        try:
            self.assertEqual(holder.stdout.readline().strip(), 'READY')
            result = self.invoke()
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('正在运行', result.stderr)
        finally:
            holder.communicate(input='done\n', timeout=2)

    def test_lock_is_inherited_by_exec(self):
        holder = subprocess.Popen(['bash', '-c',
                                   'source "$1"; acquire_compute_guard "$2" || exit; exec bash -c "echo READY; read -r line"',
                                   'bash', str(GUARD), str(self.lock)], env=self.env,
                                  stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
        try:
            self.assertEqual(holder.stdout.readline().strip(), 'READY')
            result = self.invoke()
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('正在运行', result.stderr)
        finally:
            holder.communicate(input='done\n', timeout=2)

if __name__ == '__main__': unittest.main()
