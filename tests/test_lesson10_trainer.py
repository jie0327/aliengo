"""PPO 的数值边界与真实训练行为；无 PyTorch 时跳过 GPU/神经网络测试。"""
import importlib
import importlib.util
import math
import os
from pathlib import Path
import sys
import tempfile
import subprocess
import unittest

EXAMPLES = Path(__file__).resolve().parents[1] / "examples" / "lesson10"
sys.path.insert(0, str(EXAMPLES))
os.environ.setdefault("CUBLAS_WORKSPACE_CONFIG", ":4096:8")
try:
    import torch
except ImportError:
    torch = None

core = importlib.import_module("ppo_trainer") if torch is not None and importlib.util.find_spec("ppo_trainer") else None
environment = importlib.import_module("speed_env") if torch is not None and importlib.util.find_spec("speed_env") else None
runner = importlib.import_module("train_ppo") if importlib.util.find_spec("train_ppo") else None


@unittest.skipIf(torch is None, "需要独立 PPO 环境中的 PyTorch")
class PPOCoreTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(core, "需要新增真实 PPO 训练实现")
        self.assertIsNotNone(environment, "需要连续速度交互环境")
        torch.set_num_threads(1)

    def make_trainer(self, **kwargs):
        config = dict(num_envs=4, rollout_steps=8, minibatch_size=16, epochs=2,
                      horizon=10, seed=7)
        config.update(kwargs)
        return core.PPOTrainer(core.PPOConfig(**config), "cpu")

    def test_gae_matches_hand_calculation(self):
        result = core.compute_gae(
            torch.tensor([[1.], [1.], [1.]]), torch.tensor([[.5], [.6], [.7]]),
            torch.tensor([[.6], [.7], [.8]]), torch.tensor([[False], [False], [True]]),
            torch.zeros(3, 1, dtype=torch.bool), gamma=.9, lam=.8)
        torch.testing.assert_close(result[0].squeeze(), torch.tensor([1.93712, 1.246, .3]))
        torch.testing.assert_close(result[1].squeeze(), torch.tensor([2.43712, 1.846, 1.]))

    def test_gae_timeout_bootstraps_final_state_without_crossing_reset(self):
        advantages, _ = core.compute_gae(
            torch.tensor([[1., 1., 1.], [100., 100., 100.]]),
            torch.tensor([[.5, .5, .5], [0., 0., 0.]]),
            torch.tensor([[2., 2., 2.], [0., 0., 0.]]),
            torch.tensor([[True, False, False], [True, True, True]]),
            torch.tensor([[False, True, False], [False, False, False]]), gamma=.9, lam=1)
        torch.testing.assert_close(advantages[0], torch.tensor([.5, 2.3, 92.3]))
        tail, _ = core.compute_gae(torch.tensor([[1.]]), torch.tensor([[.5]]),
                                  torch.tensor([[2.]]), torch.tensor([[False]]),
                                  torch.tensor([[False]]), gamma=.9, lam=.8)
        self.assertAlmostEqual(tail.item(), 2.3, places=5)

    def test_clipping_gradient_is_correct_for_both_advantage_signs(self):
        new_logs = torch.tensor([1.4, .6, .6, 1.4]).log().requires_grad_()
        loss = core.clipped_policy_loss(new_logs, torch.zeros(4),
                                        torch.tensor([2., 2., -2., -2.]), .2)
        self.assertAlmostEqual(loss.item(), .2, places=6)
        loss.backward()
        torch.testing.assert_close(new_logs.grad, torch.tensor([0., -.3, 0., .7]))

    def test_tanh_log_density_and_large_latent_actions_are_finite(self):
        trainer = self.make_trainer()
        observations = torch.zeros(3, 2)
        latent = torch.tensor([[0.], [.7], [30.]])
        distribution = trainer.policy.distribution(observations[:2])
        actual = trainer.policy.log_prob(observations, latent)
        expected = distribution.log_prob(latent[:2]).sum(-1)
        expected -= torch.log(1 - latent[:2].tanh().square()).sum(-1)
        torch.testing.assert_close(actual[:2], expected)
        self.assertTrue(torch.isfinite(actual).all().item())

    def test_rollout_is_frozen_and_update_changes_both_networks(self):
        trainer = self.make_trainer()
        batch = trainer.collect_rollout()
        saved = {key: value.clone() for key, value in batch.items()}
        actor_before = [p.detach().clone() for p in trainer.policy.actor.parameters()]
        critic_before = [p.detach().clone() for p in trainer.policy.critic.parameters()]
        self.assertTrue(all(not tensor.requires_grad for tensor in batch.values()))
        metrics = trainer.update(batch)
        for key in batch:
            torch.testing.assert_close(batch[key], saved[key], rtol=0, atol=0)
        self.assertTrue(any(not torch.equal(p, q) for p, q in zip(actor_before, trainer.policy.actor.parameters())))
        self.assertTrue(any(not torch.equal(p, q) for p, q in zip(critic_before, trainer.policy.critic.parameters())))
        self.assertTrue(all(math.isfinite(value) for value in metrics.values()))
        self.assertEqual(trainer.global_step, 32)
        self.assertEqual(trainer.update_count, 1)

    def test_collection_keeps_final_observation_before_timeout_reset(self):
        trainer = self.make_trainer(horizon=1)
        batch = trainer.collect_rollout()
        self.assertTrue(batch["truncated"].any().item())
        with torch.no_grad():
            expected = trainer.policy.value(batch["next_observations"])
        torch.testing.assert_close(batch["next_values"], expected)
        self.assertTrue((trainer.env.steps == 0).all().item())

    def test_environment_termination_and_truncation_are_distinct(self):
        env = environment.SpeedEnv(2, horizon=1, seed=7, device="cpu")
        env.velocity[:] = torch.tensor([.79, 0.])
        _, _, terminated, truncated = env.step(torch.tensor([[1.], [0.]]))
        self.assertEqual(terminated.tolist(), [True, False])
        self.assertEqual(truncated.tolist(), [False, True])
        with self.assertRaises(ValueError):
            env.step(torch.zeros(2, 1))

    def test_constant_advantages_and_partial_minibatch_stay_finite(self):
        trainer = self.make_trainer(minibatch_size=13)
        batch = trainer.collect_rollout()
        batch["advantages"].fill_(1.)
        metrics = trainer.update(batch)
        self.assertTrue(all(math.isfinite(value) for value in metrics.values()))
        self.assertGreater(metrics["optimizer_steps"], 0)

    def test_invalid_config_is_rejected_before_training(self):
        for kwargs in [{"gamma": float("nan")}, {"lam": 1.1}, {"learning_rate": 0},
                       {"clip": 1}, {"num_envs": 0}, {"horizon": 0},
                       {"epochs": 0}, {"minibatch_size": 1}, {"target_kl": -1}]:
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                self.make_trainer(**kwargs)


@unittest.skipIf(torch is None, "需要独立 PPO 环境中的 PyTorch")
class PPOLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(runner, "需要完整训练、评估和保存入口")
        torch.set_num_threads(1)

    def make_trainer(self, device="cpu"):
        return core.PPOTrainer(core.PPOConfig(num_envs=4, rollout_steps=8, horizon=10,
                                             minibatch_size=16, epochs=2), device)

    def test_evaluation_is_repeatable_and_preserves_training_rng_and_weights(self):
        trainer = self.make_trainer()
        parameters = {name: value.clone() for name, value in trainer.policy.state_dict().items()}
        rng_before = torch.get_rng_state().clone()
        first = runner.evaluate(trainer.policy, horizon=10, episodes=6)
        self.assertEqual(first, runner.evaluate(trainer.policy, horizon=10, episodes=6))
        torch.testing.assert_close(torch.get_rng_state(), rng_before, rtol=0, atol=0)
        for name, value in trainer.policy.state_dict().items():
            torch.testing.assert_close(value, parameters[name], rtol=0, atol=0)
        self.assertTrue(first["deterministic"])
        self.assertLessEqual(first["mean_episode_steps"], 10)

    def test_checkpoint_resume_matches_uninterrupted_cpu_training(self):
        self.check_resume("cpu")

    @unittest.skipIf(torch is None or not torch.cuda.is_available(), "需要 CUDA PyTorch 和 GPU")
    def test_cuda_rollout_and_checkpoint_resume_match_uninterrupted_training(self):
        self.check_resume("cuda")

    def check_resume(self, device):
        trainer = self.make_trainer(device)
        self.assertEqual(trainer.env.velocity.device.type, device)
        self.assertEqual(next(trainer.policy.parameters()).device.type, device)
        trainer.update(trainer.collect_rollout())
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "last.pt"
            runner.save_checkpoint(trainer, path)
            expected_metrics = trainer.update(trainer.collect_rollout())
            parameters = {name: value.clone() for name, value in trainer.policy.state_dict().items()}
            resumed = runner.load_checkpoint(path, device)
            actual_metrics = resumed.update(resumed.collect_rollout())
        self.assertEqual(expected_metrics, actual_metrics)
        self.assertEqual(resumed.global_step, 64)
        self.assertEqual(resumed.update_count, 2)
        self.assertTrue(resumed.optimizer.state)
        for name, value in resumed.policy.state_dict().items():
            torch.testing.assert_close(value, parameters[name], rtol=0, atol=0)

    def test_cli_writes_reports_checkpoints_and_rejects_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "run"
            command = [sys.executable, str(EXAMPLES / "train_ppo.py"), "--device", "cpu",
                       "--output", str(output), "--updates", "1", "--num-envs", "2",
                       "--rollout-steps", "4", "--horizon", "5", "--minibatch-size", "4",
                       "--epochs", "1", "--eval-episodes", "2"]
            result = subprocess.run(command, text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            for name in ("last.pt", "metrics.jsonl", "report.json"):
                self.assertTrue((output / name).is_file(), name)
            result = subprocess.run(command, text=True, capture_output=True)
            self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
            self.assertNotIn("Traceback", result.stderr)
            result = subprocess.run([sys.executable, str(EXAMPLES / "train_ppo.py"),
                                     "--evaluate", str(output / "last.pt"), "--device", "cpu",
                                     "--eval-episodes", "2"], text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_invalid_cli_options_report_error_without_traceback(self):
        with tempfile.TemporaryDirectory() as directory:
            for arguments in [["--updates", "0"], ["--gamma", "nan"], ["--eval-episodes", "0"],
                              ["--resume", str(Path(directory) / "missing.pt")]]:
                result = subprocess.run([sys.executable, str(EXAMPLES / "train_ppo.py"),
                                         "--output", str(Path(directory) / "run"), *arguments],
                                        text=True, capture_output=True)
                self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
                self.assertNotIn("Traceback", result.stderr)

    def test_corrupt_checkpoints_report_a_cli_error_without_traceback(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "corrupt.pt"
            for payload in (b"", b"not a checkpoint", []):
                if isinstance(payload, bytes):
                    path.write_bytes(payload)
                else:
                    torch.save(payload, path)
                result = subprocess.run([sys.executable, str(EXAMPLES / "train_ppo.py"),
                                         "--device", "cpu", "--evaluate", str(path)],
                                        text=True, capture_output=True)
                self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
                self.assertNotIn("Traceback", result.stderr)


if __name__ == "__main__":
    unittest.main()
