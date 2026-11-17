#!/usr/bin/env python3
"""Hand-checkable GAE and PPO-Clip arithmetic, NOT a PPO trainer.

No neural network, autodiff, simulator, optimizer or robot connection is used.
Each next_value belongs to the final next observation, before any reset.
"""
import argparse
import json
import math
from pathlib import Path


def finite(value, name):
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def unit_interval(value, name):
    finite(value, name)
    if not 0 <= value <= 1:
        raise ValueError(f"{name} must be in [0, 1]")


def gae(rewards, values, next_values, terminated, truncated, gamma=0.9, lam=0.8):
    """GAE over a segment, with separate bootstrap and episode-continuation masks.

    A segment may concatenate episodes. A truncation bootstraps its final
    observation, but neither terminal nor truncated transitions carry the
    next episode's advantage backwards. End of rollout initializes carry=0.
    """
    unit_interval(gamma, "gamma")
    unit_interval(lam, "lambda")
    size = len(rewards)
    if size == 0 or any(len(sequence) != size for sequence in
                        [values, next_values, terminated, truncated]):
        raise ValueError("GAE needs nonempty sequences of equal length")
    for sequence, name in [(rewards, "reward"), (values, "value"), (next_values, "next value")]:
        for value in sequence:
            finite(value, name)
    if any(type(flag) is not bool for flag in [*terminated, *truncated]):
        raise ValueError("termination and truncation flags must be bool")
    deltas, advantages, targets = [0.0] * size, [0.0] * size, [0.0] * size
    carry = 0.0
    for index in reversed(range(size)):
        bootstrap = 0.0 if terminated[index] else gamma * next_values[index]
        deltas[index] = finite(rewards[index] + bootstrap - values[index], "TD residual")
        continuation = 0.0 if terminated[index] or truncated[index] else carry
        carry = finite(deltas[index] + gamma * lam * continuation, "advantage")
        advantages[index] = carry
        targets[index] = finite(carry + values[index], "value target")
    return {"deltas": deltas, "advantages": advantages, "value_targets": targets}


def clipped_surrogate(ratio, advantage, clip=0.2):
    """One sample's objective to MAXIMIZE, for either sign of advantage."""
    finite(ratio, "ratio")
    finite(advantage, "advantage")
    finite(clip, "clip")
    if ratio < 0 or not 0 <= clip < 1:
        raise ValueError("ratio must be nonnegative and clip must be in [0, 1)")
    clipped_ratio = max(1.0 - clip, min(1.0 + clip, ratio))
    raw = finite(ratio * advantage, "unclipped objective")
    clipped = finite(clipped_ratio * advantage, "clipped objective")
    return min(raw, clipped)


def experiment(gamma=0.9, lam=0.8, clip=0.2):
    rewards, values, next_values = [1.0, 1.0, 1.0], [0.5, 0.6, 0.7], [0.6, 0.7, 0.8]
    result = gae(rewards, values, next_values, [False, False, True], [False] * 3, gamma, lam)
    clipping = []
    for advantage in (2.0, -2.0):
        for ratio in (0.6, 0.9, 1.1, 1.4):
            objective = clipped_surrogate(ratio, advantage, clip)
            clipping.append({"ratio": ratio, "advantage": advantage,
                             "raw": ratio * advantage, "objective": objective})
    terminal = gae([1.0], [0.5], [2.0], [True], [False], gamma, lam)
    timeout = gae([1.0], [0.5], [2.0], [False], [True], gamma, lam)
    return {"config": {"gamma": gamma, "lambda": lam, "clip": clip}, "gae": result,
            "clipping": clipping, "terminal_advantage": terminal["advantages"][0],
            "timeout_advantage": timeout["advantages"][0]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gamma", type=float, default=0.9)
    parser.add_argument("--lam", type=float, default=0.8, help="GAE lambda")
    parser.add_argument("--clip", type=float, default=0.2, help="PPO epsilon; objective is maximized")
    parser.add_argument("--json", type=Path, help="optional numerical report")
    args = parser.parse_args()
    try:
        report = experiment(args.gamma, args.lam, args.clip)
        if args.json:
            args.json.parent.mkdir(parents=True, exist_ok=True)
            args.json.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    except (ValueError, OSError) as error:
        parser.error(str(error))
    print(f"gamma={args.gamma} lambda={args.lam} clip={args.clip} (numerical lab, no training)")
    print("terminal rollout: rewards=[1,1,1] V=[0.5,0.6,0.7] V_next=[0.6,0.7,0.8]")
    print(" t       delta    advantage     V_target")
    for index in range(3):
        print(f"{index:2d} {report['gae']['deltas'][index]:11.6f} "
              f"{report['gae']['advantages'][index]:12.6f} {report['gae']['value_targets'][index]:12.6f}")
    print("single step: reward=1 V=0.5 V_next=2")
    print(f"terminal advantage={report['terminal_advantage']:.6f} "
          f"timeout advantage={report['timeout_advantage']:.6f}")
    print(" ratio   advantage      raw    PPO_objective(maximize)")
    for row in report["clipping"]:
        print(f"{row['ratio']:6.2f} {row['advantage']:11.2f} {row['raw']:8.2f} {row['objective']:27.2f}")
    if args.json:
        print(f"JSON: {args.json}")


if __name__ == "__main__":
    main()
