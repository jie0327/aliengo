"""Small single-process RL experiments with separate seeded evaluations."""
import hashlib
import json
import math
import time
from copy import deepcopy

import mujoco
import numpy as np
import torch
from stable_baselines3 import A2C, PPO, SAC

from environment import AliengoEnv
from model import convert
from config import EVALUATION_SEEDS


def evaluate(policy, model_path, seeds):
    results = []
    for seed in seeds:
        env = AliengoEnv(model_path)
        observation, _ = env.reset(seed=seed)
        score = 0.0
        minimum_upright = 1.0
        for step in range(500):
            action, _ = policy.predict(observation, deterministic=True)
            observation, reward, terminated, truncated, info = env.step(action)
            score += reward
            minimum_upright = min(minimum_upright, info['upright'])
            if terminated or truncated:
                break
        results.append({'seed': seed, 'steps': step + 1, 'terminated': bool(terminated),
                        'distance_x_m': info['x'], 'final_height_m': info['z'],
                        'max_tilt_rad': math.acos(max(-1., min(1., minimum_upright))),
                        'return': float(score)})
    return results


def run(urdf, output, steps, seed, limits, algorithm):
    torch.set_num_threads(1)
    torch.set_num_interop_threads(1)
    output.mkdir(parents=True, exist_ok=False)
    model_path = output / 'aliengo.xml'
    convert(urdf, model_path)
    env = AliengoEnv(model_path)
    # No subprocess vector environments; neural network updates run on CPU.
    parameters = {
        'ppo': dict(n_steps=256, batch_size=128, n_epochs=2,
                    policy_kwargs={'net_arch': dict(pi=[64, 64], vf=[64, 64])}),
        'a2c': dict(n_steps=128,
                    policy_kwargs={'net_arch': dict(pi=[64, 64], vf=[64, 64])}),
        'sac': dict(buffer_size=10000, learning_starts=256, batch_size=128,
                    train_freq=32, gradient_steps=8,
                    policy_kwargs={'net_arch': dict(pi=[64, 64], qf=[64, 64])}),
    }
    policy = {'ppo': PPO, 'a2c': A2C, 'sac': SAC}[algorithm](
        'MlpPolicy', env, device='cpu', verbose=1, seed=seed,
        learning_rate=3e-4, **deepcopy(parameters[algorithm]))
    seeds = EVALUATION_SEEDS
    baseline = evaluate(policy, model_path, seeds)
    started = time.monotonic()
    policy.learn(total_timesteps=steps)
    elapsed = time.monotonic() - started
    after = evaluate(policy, model_path, seeds)
    policy.save(str(output / 'policy'))
    report = {'unvalidated_experiment': True, 'algorithm': algorithm, 'seed': seed,
              'requested_timesteps': steps, 'actual_timesteps': int(policy.num_timesteps),
              'algorithm_parameters': parameters[algorithm], 'learning_rate': 3e-4,
              'train_seconds': elapsed, 'limits': limits,
              'urdf_sha256': hashlib.sha256(urdf.read_bytes()).hexdigest(),
              'versions': {'torch': torch.__version__, 'numpy': np.__version__,
                           'mujoco': mujoco.__version__},
              'baseline': baseline, 'evaluation': after}
    (output / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2), flush=True)
