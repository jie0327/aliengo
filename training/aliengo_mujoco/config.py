"""Comparable small training budgets and held-out evaluation seeds."""
ALGORITHMS = ('ppo', 'a2c', 'sac')
EVALUATION_SEEDS = (1001, 1002, 1003)


def validate_steps(steps):
    # PPO collects 256 steps per update; the other algorithms also divide this
    # budget exactly, so every run sees the same number of environment steps.
    return steps > 0 and steps % 256 == 0
