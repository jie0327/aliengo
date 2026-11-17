#!/usr/bin/env python3
"""Tabular velocity tracking: a small MDP, not a quadruped dynamics model.

Only Python's standard library is used. Evaluation never updates the table.
Velocity is an integer bin of 0.2 m/s; actions change one bin per 0.2 s.
"""
import argparse
import json
import math
from pathlib import Path
import random

VELOCITIES = tuple(range(-3, 4))
COMMANDS = (-2, -1, 1, 2)
ACTIONS = (0, -1, 1)  # Stable tie rule: an all-zero table chooses hold.
BIN_M_S = 0.2
EVAL_SEED_START = 10_000


def finite(value, name):
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def unit_interval(value, name):
    finite(value, name)
    if not 0 <= value <= 1:
        raise ValueError(f"{name} must be in [0, 1]")


def count(value, name, minimum, maximum):
    if type(value) is not int or not minimum <= value <= maximum:
        raise ValueError(f"{name} must be an integer in [{minimum}, {maximum}]")


def discounted_returns(rewards, gamma, bootstrap=0.0):
    """Return-to-go for a collected segment; bootstrap is its final V estimate."""
    unit_interval(gamma, "gamma")
    result = [0.0] * len(rewards)
    running = finite(bootstrap, "bootstrap")
    for index in reversed(range(len(rewards))):
        running = finite(finite(rewards[index], "reward") + gamma * running, "return")
        result[index] = running
    return result


def q_update(current, reward, next_max, alpha, gamma, terminated, truncated=False):
    """A timeout stops collection, but does not erase the next state's value."""
    for value, name in [(current, "current Q"), (reward, "reward"), (next_max, "next Q")]:
        finite(value, name)
    unit_interval(alpha, "alpha")
    unit_interval(gamma, "gamma")
    if alpha == 0:
        raise ValueError("alpha must be positive")
    if type(terminated) is not bool or type(truncated) is not bool:
        raise ValueError("termination and truncation flags must be bool")
    target = finite(reward + (0.0 if terminated else gamma * next_max), "Q target")
    return finite(current + alpha * finite(target - current, "TD error"), "updated Q")


def empty_q_table():
    return {(velocity, command): [0.0] * len(ACTIONS)
            for velocity in VELOCITIES for command in COMMANDS}


def greedy_action(table, observation):
    values = table[observation]
    return ACTIONS[max(range(len(ACTIONS)), key=lambda index: values[index])]


def transition(velocity, command, action, wind=0):
    """One MDP step. Speeds outside [-3, 3] are absorbing failure outcomes."""
    if (type(velocity) is not int or velocity not in VELOCITIES
            or type(command) is not int or command not in COMMANDS
            or type(action) is not int or action not in ACTIONS
            or type(wind) is not int or wind not in (-1, 0, 1)):
        raise ValueError("invalid velocity/command/action/wind bin")
    next_velocity = velocity + action + wind
    terminated = next_velocity not in VELOCITIES
    reward = 1.0 - 0.25 * (next_velocity - command) ** 2 - 0.05 * abs(action)
    if terminated:
        reward -= 5.0
    return next_velocity, reward, terminated


def sample_wind(rng):
    draw = rng.random()
    return -1 if draw < 0.1 else (1 if draw >= 0.9 else 0)


def validate_config(episodes, seed, eval_episodes, horizon, gamma, alpha, epsilon):
    count(episodes, "episodes", 0, 100_000)
    count(seed, "seed", 0, 9_999)
    count(eval_episodes, "eval_episodes", 1, 10_000)
    count(horizon, "horizon", 1, 1_000)
    if (episodes + 2 * eval_episodes) * horizon > 5_000_000:
        raise ValueError("experiment exceeds the teaching limit of 5,000,000 steps")
    unit_interval(gamma, "gamma")
    unit_interval(alpha, "alpha")
    unit_interval(epsilon, "epsilon")
    if alpha == 0:
        raise ValueError("alpha must be positive")


def evaluate(table, episodes=100, horizon=40, gamma=0.95):
    """Independent per-episode held-out seeds, shared by baseline and trained Q."""
    validate_config(0, 0, episodes, horizon, gamma, 0.15, 0.2)
    episode_returns = []
    errors = []
    lengths = []
    failures = 0
    for index in range(episodes):
        rng = random.Random(EVAL_SEED_START + index)
        velocity, command = rng.choice(VELOCITIES), rng.choice(COMMANDS)
        rewards = []
        for step in range(horizon):
            action = greedy_action(table, (velocity, command))
            velocity, reward, terminated = transition(velocity, command, action, sample_wind(rng))
            rewards.append(reward)
            errors.append(abs(velocity - command) * BIN_M_S)
            if terminated:
                failures += 1
                break
        episode_returns.append(discounted_returns(rewards, gamma)[0])
        lengths.append(len(rewards))
    return {
        "mean_discounted_return": sum(episode_returns) / episodes,
        "mae_m_s": sum(errors) / len(errors),
        "termination_rate": failures / episodes,
        "mean_episode_steps": sum(lengths) / episodes,
    }


def run_experiment(episodes=3000, seed=7, eval_episodes=100, horizon=40,
                   gamma=0.95, alpha=0.15, epsilon=0.2):
    validate_config(episodes, seed, eval_episodes, horizon, gamma, alpha, epsilon)
    table = empty_q_table()
    baseline = evaluate(table, eval_episodes, horizon, gamma)
    rng = random.Random(seed)
    for episode in range(episodes):
        velocity, command = rng.choice(VELOCITIES), rng.choice(COMMANDS)
        for step in range(horizon):
            observation = (velocity, command)
            action = rng.choice(ACTIONS) if rng.random() < epsilon else greedy_action(table, observation)
            next_velocity, reward, terminated = transition(velocity, command, action, sample_wind(rng))
            truncated = (step + 1 == horizon) and not terminated
            # Use the final observation before resetting, including on a timeout.
            next_max = 0.0 if terminated else max(table[(next_velocity, command)])
            action_index = ACTIONS.index(action)
            table[observation][action_index] = q_update(
                table[observation][action_index], reward, next_max,
                alpha, gamma, terminated, truncated)
            velocity = next_velocity
            if terminated or truncated:
                break
    return {
        "config": {"episodes": episodes, "seed": seed, "eval_episodes": eval_episodes,
                   "horizon": horizon, "gamma": gamma, "alpha": alpha, "epsilon": epsilon,
                   "eval_seed_start": EVAL_SEED_START},
        "baseline": baseline,
        "trained": evaluate(table, eval_episodes, horizon, gamma),
        "q_table": [{"velocity_bin": velocity, "command_bin": command,
                     "q_hold_minus_plus": values}
                    for (velocity, command), values in sorted(table.items())],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episodes", type=int, default=3000, help="training episodes; 0 keeps the baseline")
    parser.add_argument("--seed", type=int, default=7, help="training RNG seed in [0, 9999]")
    parser.add_argument("--eval-episodes", type=int, default=100)
    parser.add_argument("--horizon", type=int, default=40, help="external collection cutoff in steps")
    parser.add_argument("--gamma", type=float, default=0.95)
    parser.add_argument("--alpha", type=float, default=0.15)
    parser.add_argument("--epsilon", type=float, default=0.2, help="fixed training exploration probability")
    parser.add_argument("--json", type=Path, help="optional report and Q table; creates parent directories")
    args = parser.parse_args()
    try:
        report = run_experiment(args.episodes, args.seed, args.eval_episodes, args.horizon,
                                args.gamma, args.alpha, args.epsilon)
        if args.json:
            args.json.parent.mkdir(parents=True, exist_ok=True)
            args.json.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    except (ValueError, OSError) as error:
        parser.error(str(error))
    print(f"train seed={args.seed} episodes={args.episodes} horizon={args.horizon} "
          f"alpha={args.alpha} gamma={args.gamma} epsilon={args.epsilon}")
    print(f"held-out evaluation seeds={EVAL_SEED_START}..{EVAL_SEED_START + args.eval_episodes - 1} "
          "greedy=True updates=False")
    print("policy       mean_return    MAE(m/s)    terminal%    mean_steps")
    for name, key in [("untrained", "baseline"), ("trained", "trained")]:
        values = report[key]
        print(f"{name:<12} {values['mean_discounted_return']:11.5f} "
              f"{values['mae_m_s']:11.5f} {100 * values['termination_rate']:12.2f} "
              f"{values['mean_episode_steps']:13.2f}")
    print("return = sum(gamma**t * reward_t) over the collected episode; no evaluation bootstrap")
    if args.json:
        print(f"JSON: {args.json}")


if __name__ == "__main__":
    main()
