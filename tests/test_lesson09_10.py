"""Behavioral numerical contracts for the standard-library RL teaching labs."""
import importlib.util
import math
from pathlib import Path
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]


def load_example(relative):
    path = ROOT / "examples" / relative
    if not path.exists():
        return None
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


q_lab = load_example("lesson09/q_learning_speed.py")
ppo = load_example("lesson10/ppo_numbers.py")


class QLearningTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(q_lab, "lesson09 Q-learning lab must exist")

    def test_discounted_returns_match_three_step_hand_calculation(self):
        self.assertEqual(q_lab.discounted_returns([1, 2, 3], 0.5), [2.75, 3.5, 3.0])
        self.assertEqual(q_lab.discounted_returns([1, 2], 0.5, bootstrap=4), [3.0, 4.0])

    def test_q_update_bootstraps_timeout_but_not_terminal(self):
        # target is 1 + .9 * 4 = 4.6, or just 1 for a terminal transition.
        self.assertAlmostEqual(q_lab.q_update(2, 1, 4, 0.5, 0.9, False, True), 3.3)
        self.assertAlmostEqual(q_lab.q_update(2, 1, 4, 0.5, 0.9, True, False), 1.5)
        self.assertAlmostEqual(q_lab.q_update(2, 1, 4, 0.5, 0.9, True, True), 1.5)

    def test_zero_action_baseline_and_transition_are_explicit(self):
        table = q_lab.empty_q_table()
        self.assertEqual(q_lab.greedy_action(table, (0, 2)), 0)
        velocity, reward, terminated = q_lab.transition(0, 2, 1, 0)
        self.assertEqual(velocity, 1)
        self.assertAlmostEqual(reward, 0.70)
        self.assertFalse(terminated)

    def test_out_of_bounds_transition_is_a_real_terminal_state(self):
        velocity, reward, terminated = q_lab.transition(3, 2, 1, 0)
        self.assertEqual(velocity, 4)
        self.assertLess(reward, -4)
        self.assertTrue(terminated)

    def test_evaluation_is_repeatable_and_does_not_update_q_table(self):
        table = q_lab.empty_q_table()
        before = {key: values[:] for key, values in table.items()}
        first = q_lab.evaluate(table, episodes=12, horizon=30, gamma=0.95)
        self.assertEqual(first, q_lab.evaluate(table, episodes=12, horizon=30, gamma=0.95))
        self.assertEqual(table, before)

    def test_fixed_seed_learning_improves_held_out_return_and_error(self):
        first = q_lab.run_experiment(episodes=1200, seed=7, eval_episodes=60)
        second = q_lab.run_experiment(episodes=1200, seed=7, eval_episodes=60)
        self.assertEqual(first, second)
        self.assertGreater(first["trained"]["mean_discounted_return"],
                           first["baseline"]["mean_discounted_return"] + 5)
        self.assertLess(first["trained"]["mae_m_s"], first["baseline"]["mae_m_s"])
        self.assertLessEqual(first["trained"]["termination_rate"],
                             first["baseline"]["termination_rate"])

    def test_zero_training_episodes_preserve_untrained_baseline(self):
        result = q_lab.run_experiment(episodes=0, eval_episodes=10)
        self.assertEqual(result["baseline"], result["trained"])

    def test_invalid_parameters_and_nonfinite_arithmetic_are_rejected(self):
        for config in [{"episodes": -1}, {"episodes": 1.5}, {"eval_episodes": 0},
                       {"horizon": 0}, {"gamma": math.nan}, {"gamma": 1.1},
                       {"alpha": 0}, {"epsilon": -0.1}]:
            with self.subTest(config=config), self.assertRaises(ValueError):
                q_lab.run_experiment(**config)
        for values in [([math.nan], 0.9, 0), ([1], math.inf, 0),
                       ([1e308, 1e308], 1, 0)]:
            with self.subTest(values=values), self.assertRaises(ValueError):
                q_lab.discounted_returns(values[0], values[1], bootstrap=values[2])
        with self.assertRaises(ValueError):
            q_lab.q_update(0, 1e308, 1e308, 0.5, 1, False)


class PPONumberTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(ppo, "lesson10 PPO numerical lab must exist")

    def test_gae_matches_backwards_three_step_calculation(self):
        result = ppo.gae([1, 1, 1], [0.5, 0.6, 0.7], [0.6, 0.7, 0.8],
                         [False, False, True], [False, False, False], gamma=0.9, lam=0.8)
        for got, expected in zip(result["deltas"], [1.04, 1.03, 0.3]):
            self.assertAlmostEqual(got, expected)
        for got, expected in zip(result["advantages"], [1.93712, 1.246, 0.3]):
            self.assertAlmostEqual(got, expected)
        for got, expected in zip(result["value_targets"], [2.43712, 1.846, 1]):
            self.assertAlmostEqual(got, expected)

    def test_lambda_zero_uses_only_one_step_td_residual(self):
        result = ppo.gae([1, 1], [0.5, 0.6], [0.6, 0.7], [False, True],
                         [False, False], lam=0)
        self.assertEqual(result["advantages"], result["deltas"])

    def test_terminal_and_timeout_have_different_bootstrap_values(self):
        terminal = ppo.gae([1], [0.5], [2], [True], [False], gamma=0.9)
        timeout = ppo.gae([1], [0.5], [2], [False], [True], gamma=0.9)
        self.assertAlmostEqual(terminal["advantages"][0], 0.5)
        self.assertAlmostEqual(timeout["advantages"][0], 2.3)

    def test_timeout_bootstraps_final_observation_without_crossing_reset(self):
        result = ppo.gae([1, 100], [0.5, 0], [2, 0], [False, True],
                         [True, False], gamma=0.9, lam=1)
        self.assertAlmostEqual(result["advantages"][0], 2.3)
        self.assertAlmostEqual(result["advantages"][1], 100)

    def test_rollout_end_bootstraps_even_without_episode_end(self):
        result = ppo.gae([1], [0.5], [2], [False], [False], gamma=0.9)
        self.assertAlmostEqual(result["advantages"][0], 2.3)

    def test_positive_advantage_caps_only_probability_increase(self):
        self.assertAlmostEqual(ppo.clipped_surrogate(1.4, 2, 0.2), 2.4)
        self.assertAlmostEqual(ppo.clipped_surrogate(0.6, 2, 0.2), 1.2)

    def test_negative_advantage_caps_only_probability_decrease(self):
        self.assertAlmostEqual(ppo.clipped_surrogate(0.6, -2, 0.2), -1.6)
        self.assertAlmostEqual(ppo.clipped_surrogate(1.4, -2, 0.2), -2.8)

    def test_ratio_inside_interval_and_zero_advantage_are_unchanged(self):
        self.assertAlmostEqual(ppo.clipped_surrogate(1.1, -2, 0.2), -2.2)
        self.assertEqual(ppo.clipped_surrogate(1.4, 0, 0.2), 0)

    def test_invalid_sequences_flags_parameters_and_overflow_are_rejected(self):
        for rewards, values, next_values, terminated, truncated in [
            ([], [], [], [], []), ([1], [], [0], [False], [False]),
            ([math.nan], [0], [0], [False], [False]),
            ([1], [0], [0], [1], [False]),
            ([1e308], [0], [1e308], [False], [False]),
        ]:
            with self.subTest(rewards=rewards), self.assertRaises(ValueError):
                ppo.gae(rewards, values, next_values, terminated, truncated, gamma=1)
        for ratio, advantage, clip in [(-1, 1, 0.2), (math.inf, 1, 0.2),
                                       (1, math.nan, 0.2), (1, 1, -0.1),
                                       (1, 1, 1), (1e308, 2, 0.2)]:
            with self.subTest(ratio=ratio), self.assertRaises(ValueError):
                ppo.clipped_surrogate(ratio, advantage, clip)
        for params in [{"gamma": -0.1}, {"gamma": math.inf}, {"lam": 1.1}]:
            with self.subTest(params=params), self.assertRaises(ValueError):
                ppo.gae([1], [0], [0], [True], [False], **params)


class RLCLITests(unittest.TestCase):
    def test_nonfinite_cli_parameter_is_reported_without_traceback(self):
        for relative, args in [("lesson09/q_learning_speed.py", ["--gamma", "nan"]),
                               ("lesson10/ppo_numbers.py", ["--clip", "nan"])]:
            with self.subTest(relative=relative):
                self.assertTrue((ROOT / "examples" / relative).is_file())
                result = subprocess.run([sys.executable, "-S", str(ROOT / "examples" / relative),
                                         *args], text=True, capture_output=True)
                self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
                self.assertNotIn("Traceback", result.stderr)
                self.assertIn("error:", result.stderr)


if __name__ == "__main__":
    unittest.main()
