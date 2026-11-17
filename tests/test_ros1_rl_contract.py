"""Numerical tests for configurable 48-element policy observations and actions."""
import importlib.util
import math
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('rl_contract', ROOT / 'ros1/rl_contract.py')
rl_contract = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rl_contract)


def config():
    names = [f'{leg}_{joint}_joint' for leg in ('FL', 'FR', 'RL', 'RR')
             for joint in ('hip', 'thigh', 'calf')]
    return {'joint_names': names,
            'default_q': [0.1, 0.8, -1.5, -0.1, 0.8, -1.5] * 2,
            'observation_scales': {'lin_vel': 2., 'ang_vel': 0.25, 'gravity': 1.,
                                   'commands': [2., 2., 0.25], 'dof_pos': 1.,
                                   'dof_vel': 0.05, 'previous_action': 1.},
            'observation_clip': 100., 'action_clip': 1., 'action_scale': 0.25,
            'action_multipliers': [0.5 if i % 3 == 0 else 1. for i in range(12)],
            'previous_action_multipliers': [1.] * 12,
            'kp': [20.] * 12, 'kd': [0.75] * 12}


def frame(cfg):
    # Deliberately reversed dictionary insertion order to check name mapping.
    joints = {name: {'q': cfg['default_q'][i] + (i + 1) * .01, 'dq': (i + 1) * .1}
              for i, name in reversed(list(enumerate(cfg['joint_names'])))}
    return {'quaternion_xyzw': [0., 0., 0., 1.], 'linear_velocity_world': [1., 2., 3.],
            'angular_velocity_body': [4., 5., 6.], 'command': [0.5, -0.25, 0.2],
            'joints': joints}


class PolicyContractTests(unittest.TestCase):
    def test_observation_order_scaling_and_named_joint_mapping(self):
        cfg = config(); contract = rl_contract.PolicyContract(cfg)
        obs = contract.observation(frame(cfg))
        self.assertEqual(len(obs), 48)
        self.assertEqual(obs[:12], [2., 4., 6., 1., 1.25, 1.5, 0., 0., -1., 1., -.5, .05])
        for i in range(12):
            self.assertAlmostEqual(obs[12 + i], (i + 1) * .01)
            self.assertAlmostEqual(obs[24 + i], (i + 1) * .005)
        self.assertEqual(obs[36:], [0.] * 12)

    def test_rotation_uses_body_to_world_quaternion_inverse(self):
        cfg = config(); f = frame(cfg)
        f['quaternion_xyzw'] = [0., 0., math.sqrt(.5), math.sqrt(.5)]
        f['linear_velocity_world'] = [1., 0., 0.]
        obs = rl_contract.PolicyContract(cfg).observation(f)
        self.assertAlmostEqual(obs[0], 0., places=12)
        self.assertAlmostEqual(obs[1], -2., places=12)
        self.assertAlmostEqual(obs[2], 0., places=12)
        f['quaternion_xyzw'] = [0., math.sqrt(.5), 0., math.sqrt(.5)]
        obs = rl_contract.PolicyContract(cfg).observation(f)
        self.assertAlmostEqual(obs[6], 1., places=12)
        self.assertAlmostEqual(obs[8], 0., places=12)

    def test_actions_are_clipped_scaled_and_previous_action_reset(self):
        cfg = config(); c = rl_contract.PolicyContract(cfg)
        targets = c.targets([2., -2., .5] * 4)
        self.assertEqual([t['name'] for t in targets], cfg['joint_names'])
        self.assertAlmostEqual(targets[0]['q'], .225)
        self.assertAlmostEqual(targets[1]['q'], .55)
        self.assertEqual((targets[0]['dq'], targets[0]['kp'], targets[0]['kd'], targets[0]['tau']),
                         (0., 20., .75, 0.))
        self.assertEqual(c.observation(frame(cfg))[36:], [1., -1., .5] * 4)
        c.reset()
        self.assertEqual(c.observation(frame(cfg))[36:], [0.] * 12)

    def test_nonfinite_and_invalid_input_rejected_without_updating_history(self):
        cfg = config(); c = rl_contract.PolicyContract(cfg)
        f = frame(cfg); f['quaternion_xyzw'] = [0., 0., 0., 0.]
        with self.assertRaises(ValueError): c.observation(f)
        f = frame(cfg); del f['joints'][cfg['joint_names'][0]]
        with self.assertRaises(ValueError): c.observation(f)
        f = frame(cfg); f['angular_velocity_body'][0] = math.nan
        with self.assertRaises(ValueError): c.observation(f)
        with self.assertRaises(ValueError): c.targets([math.nan] + [0.] * 11)
        self.assertEqual(c.observation(frame(cfg))[36:], [0.] * 12)

    def test_previous_action_scaling_is_explicit_and_independent_of_target_scaling(self):
        cfg = config()
        cfg['previous_action_multipliers'] = [0.5 if i % 3 == 0 else 1. for i in range(12)]
        c = rl_contract.PolicyContract(cfg)
        c.targets([1.] * 12)
        self.assertEqual(c.observation(frame(cfg))[36:], [0.5, 1., 1.] * 4)
        del cfg['previous_action_multipliers']
        with self.assertRaises(ValueError):
            rl_contract.PolicyContract(cfg)

    def test_invalid_config_is_rejected_and_clipping_follows_scaling(self):
        cfg = config(); cfg['joint_names'][1] = cfg['joint_names'][0]
        with self.assertRaises(ValueError): rl_contract.PolicyContract(cfg)
        cfg = config(); cfg['observation_clip'] = .5
        obs = rl_contract.PolicyContract(cfg).observation(frame(cfg))
        self.assertEqual(obs[:3], [.5, .5, .5])
        self.assertEqual(obs[8], -.5)


if __name__ == '__main__':
    unittest.main()
