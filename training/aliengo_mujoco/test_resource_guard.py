"""Low-cost checks: never imports the training stack or starts a simulator."""
import json
import subprocess
import sys
import unittest
from pathlib import Path

ENTRY = Path(__file__).with_name('run.py')

class ResourceGuardTest(unittest.TestCase):
    def test_check_enforces_memory_and_thread_limits(self):
        result = subprocess.run([sys.executable, str(ENTRY), '--check'], capture_output=True, text=True, check=True)
        settings = json.loads(result.stdout)
        self.assertEqual(settings['device'], 'cpu')
        self.assertEqual(settings['address_space_limit_bytes'], 4096 * 1024 * 1024)
        self.assertEqual(settings['threads'], 1)
        self.assertEqual(settings['max_processes'], 1)

    def test_rejects_memory_limit_above_ceiling(self):
        result = subprocess.run([sys.executable, str(ENTRY), '--check', '--memory-mib', '4097'], capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)

if __name__ == '__main__':
    unittest.main()
