"""Test training report serialization without running ML or physics."""
import importlib.util
import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest import mock

HERE = Path(__file__).resolve().parent


class FakePolicy:
    mutate_policy_kwargs = False

    def __init__(self, _policy, _env, *, policy_kwargs, **_kwargs):
        # A2C/SB3 may mutate the caller's nested dict by adding class objects.
        if self.mutate_policy_kwargs:
            policy_kwargs['optimizer_class'] = type('RMSprop', (), {})
        self.num_timesteps = 0

    def learn(self, total_timesteps):
        self.num_timesteps = total_timesteps

    def save(self, path):
        Path(path + '.zip').write_bytes(b'test')


class FakeA2C(FakePolicy):
    mutate_policy_kwargs = True


class FakeSAC(FakePolicy):
    mutate_policy_kwargs = True


class ReportTest(unittest.TestCase):
    def test_all_algorithms_save_json_when_library_mutates_nested_config(self):
        fake_torch = types.SimpleNamespace(__version__='test', set_num_threads=lambda *_: None,
                                           set_num_interop_threads=lambda *_: None)
        fake_modules = {
            'torch': fake_torch, 'numpy': types.SimpleNamespace(__version__='test'),
            'mujoco': types.SimpleNamespace(__version__='test'),
            'environment': types.SimpleNamespace(AliengoEnv=lambda _: object()),
            'model': types.SimpleNamespace(convert=lambda _u, output: output.write_text('<test/>')),
            'stable_baselines3': types.SimpleNamespace(PPO=FakePolicy, A2C=FakeA2C, SAC=FakeSAC),
        }
        with mock.patch.dict(sys.modules, fake_modules), tempfile.TemporaryDirectory() as directory:
            spec = importlib.util.spec_from_file_location('train_report_test', HERE / 'train.py')
            train = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(train)
            train.evaluate = lambda *_: [{'seed': 1001, 'distance_x_m': 0.0}]
            urdf = Path(directory) / 'model.urdf'
            urdf.write_text('<robot/>')
            for algorithm in ('ppo', 'a2c', 'sac'):
                with self.subTest(algorithm=algorithm):
                    output = Path(directory) / algorithm
                    with contextlib.redirect_stdout(io.StringIO()):
                        train.run(urdf, output, 256, 19, {}, algorithm)
                    report = json.loads((output / 'result.json').read_text())
                    self.assertEqual(report['algorithm'], algorithm)
                    self.assertEqual(report['actual_timesteps'], 256)
                    self.assertNotIn('optimizer_class', report['algorithm_parameters']['policy_kwargs'])
                    self.assertTrue((output / 'policy.zip').is_file())


if __name__ == '__main__':
    unittest.main()
