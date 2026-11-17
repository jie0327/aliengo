import json
import pathlib
import subprocess
import sys
import tempfile
import unittest

SCRIPT = pathlib.Path(__file__).resolve().parents[1] / 'scripts' / 'mid360_config.py'


class Mid360ConfigTest(unittest.TestCase):
    def test_config_uses_requested_addresses_and_preserves_sensor_coordinates(self):
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / 'sensor.json'
            subprocess.run([sys.executable, str(SCRIPT), str(path), '192.168.7.5', '192.168.7.12'], check=True)
            config = json.loads(path.read_text())
            host = config['MID360']['host_net_info']
            for key in ('cmd_data_ip', 'push_msg_ip', 'point_data_ip', 'imu_data_ip'):
                self.assertEqual(host[key], '192.168.7.5')
            self.assertEqual(config['lidar_configs'][0]['ip'], '192.168.7.12')
            self.assertTrue(all(value == 0 for value in config['lidar_configs'][0]['extrinsic_parameter'].values()))

    def test_refuses_to_replace_existing_configuration(self):
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / 'sensor.json'
            path.write_text('keep')
            result = subprocess.run([sys.executable, str(SCRIPT), str(path), '192.168.7.5', '192.168.7.12'], capture_output=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(path.read_text(), 'keep')

    def test_rejects_invalid_or_identical_addresses(self):
        with tempfile.TemporaryDirectory() as directory:
            for host, lidar in [('invalid', '192.168.7.12'), ('192.168.7.5', '192.168.7.5')]:
                path = pathlib.Path(directory) / 'sensor.json'
                result = subprocess.run([sys.executable, str(SCRIPT), str(path), host, lidar], capture_output=True)
                self.assertNotEqual(result.returncode, 0)
                self.assertFalse(path.exists())


if __name__ == '__main__':
    unittest.main()
