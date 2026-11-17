#!/usr/bin/env python3
"""完整 PPO 速度跟踪训练：采样、优化、评估、保存和续训。使用 --help 查看参数。"""
import argparse
from dataclasses import asdict
import json
import os
from pathlib import Path
import pickle
import time

EVAL_SEED = 10000  # 评估环境使用独立随机流；不参与策略更新或选择最佳权重。


def resolve_device(requested):
    import torch
    if requested == "cuda" and not torch.cuda.is_available():
        raise ValueError("请求了 CUDA，但当前 PyTorch 无法使用 GPU；请检查 .venv-ppo 安装")
    return torch.device("cuda" if requested != "cpu" and torch.cuda.is_available() else "cpu")


def evaluate(policy, horizon=80, episodes=32, gamma=.95):
    """使用确定性动作，评估一批独立回合，不改变训练参数或策略采样 RNG。"""
    import torch
    from speed_env import SpeedEnv
    if type(episodes) is not int or episodes < 1:
        raise ValueError("eval_episodes 必须是正整数")
    device = next(policy.parameters()).device
    env = SpeedEnv(episodes, horizon, EVAL_SEED, device)
    returns = torch.zeros(episodes, device=device)  # 每回合未折扣的奖励总和。
    discounted = torch.zeros_like(returns)  # 每回合的折扣回报。
    lengths = torch.zeros_like(returns)  # 每回合实际执行步数。
    alive = torch.ones(episodes, dtype=torch.bool, device=device)  # 尚未结束的评估回合。
    failures = torch.zeros_like(alive)  # 因越界结束的回合。
    error_sum = torch.zeros((), device=device)  # 逐步绝对速度误差总和。
    training_mode = policy.training
    policy.eval()
    try:
        with torch.no_grad():
            for step in range(horizon):
                action = policy.deterministic_action(env.observation())
                _, reward, terminated, truncated = env.step(action, active=alive)
                returns += reward * alive
                discounted += gamma ** step * reward * alive
                lengths += alive
                error_sum += ((env.velocity - env.command).abs() * alive).sum()
                failures |= terminated
                alive &= ~(terminated | truncated)
                if not alive.any().item():
                    break
    finally:
        policy.train(training_mode)
    return {"mean_return": returns.mean().item(), "mean_discounted_return": discounted.mean().item(),
            "mae_m_s": (error_sum / lengths.sum()).item(),
            "termination_rate": failures.float().mean().item(),
            "mean_episode_steps": lengths.mean().item(), "eval_seed": EVAL_SEED,
            "episodes": episodes, "deterministic": True}


def save_checkpoint(trainer, path):
    import torch
    # 只在一轮 PPO 更新结束后保存；当前 rollout 已消耗完，无需保存陈旧采样。
    state = {"format_version": 1, "config": asdict(trainer.config),
             "policy": trainer.policy.state_dict(), "optimizer": trainer.optimizer.state_dict(),
             "environment": trainer.env.state_dict(), "global_step": trainer.global_step,
             "update_count": trainer.update_count, "cpu_rng": torch.get_rng_state(),
             "device_type": trainer.device.type,
             "cuda_rng": torch.cuda.get_rng_state(trainer.device) if trainer.device.type == "cuda" else None}
    path = Path(path)
    temporary = path.with_suffix(path.suffix + ".tmp")
    torch.save(state, temporary)
    temporary.replace(path)  # 完整写入后原子替换，避免留下半个 last.pt。


def load_checkpoint(path, device="cpu", restore_training=True):
    import torch
    from ppo_trainer import PPOConfig, PPOTrainer
    # weights_only 限制反序列化内容；CPU 读取后再将网络/优化器移动到所选设备。
    try:
        state = torch.load(path, map_location="cpu", weights_only=True)
    except (EOFError, pickle.UnpicklingError, RuntimeError) as error:
        raise ValueError(f"无效或损坏的 PPO checkpoint：{path}") from error
    required = {"config", "policy", "optimizer", "environment", "global_step", "update_count",
                "cpu_rng", "device_type", "cuda_rng"}
    if not isinstance(state, dict) or state.get("format_version") != 1 or not required <= state.keys():
        raise ValueError("无效的 PPO checkpoint 格式或缺少必要字段")
    device = torch.device(device)
    if restore_training and state["device_type"] != device.type:
        raise ValueError("续训须使用与 checkpoint 相同类型的设备；跨设备可使用 --evaluate")
    trainer = PPOTrainer(PPOConfig(**state["config"]), device)
    trainer.policy.load_state_dict(state["policy"])
    trainer.global_step = state["global_step"]
    trainer.update_count = state["update_count"]
    if restore_training:
        trainer.optimizer.load_state_dict(state["optimizer"])
        trainer.env.load_state_dict(state["environment"])
        torch.set_rng_state(state["cpu_rng"])
        if device.type == "cuda":
            torch.cuda.set_rng_state(state["cuda_rng"], device)
    return trainer


def write_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
                          encoding="utf-8")


def print_evaluation(label, metrics):
    print(f"{label}: return={metrics['mean_return']:.3f} "
          f"discounted_return={metrics['mean_discounted_return']:.3f} "
          f"MAE={metrics['mae_m_s']:.5f}m/s "
          f"terminal={100 * metrics['termination_rate']:.1f}% "
          f"steps={metrics['mean_episode_steps']:.1f}", flush=True)


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", choices=("auto", "cpu", "cuda"), default="auto", help="计算设备，默认自动选择")
    parser.add_argument("--output", type=Path, help="新的训练输出目录，不能已存在")
    parser.add_argument("--updates", type=int, default=80, help="本次额外执行的 PPO 轮数，默认 80")
    sources = parser.add_mutually_exclusive_group()
    sources.add_argument("--resume", type=Path, help="加载 checkpoint，恢复参数、优化器、环境与 RNG 后继续训练")
    sources.add_argument("--evaluate", type=Path, help="仅评估 checkpoint，不更新参数")
    parser.add_argument("--eval-episodes", type=int, default=32, help="独立评估回合数，默认 32")
    parser.add_argument("--eval-every", type=int, default=20, help="每隔多少轮记录一次评估，默认 20")
    parser.add_argument("--save-every", type=int, default=20, help="每隔多少轮保存 checkpoint，默认 20")
    # None 区分用户显式设置和默认值；续训完全沿用 checkpoint 的配置。
    descriptions = {
        "num_envs": (int, "同时采样的环境数，默认 8"),
        "rollout_steps": (int, "每轮每个环境采样的步数，默认 64"),
        "horizon": (int, "每回合最大步数，默认 80"),
        "epochs": (int, "同一批采样优化的遍数，默认 4"),
        "minibatch_size": (int, "梯度更新小批量大小，默认 128"),
        "learning_rate": (float, "Adam 学习率，默认 0.0003"),
        "gamma": (float, "奖励折扣因子，默认 0.95"),
        "lam": (float, "GAE lambda，默认 0.95"),
        "clip": (float, "PPO 概率比裁剪宽度，默认 0.2"),
        "value_coef": (float, "价值损失权重，默认 0.5"),
        "entropy_coef": (float, "tanh 前高斯熵奖励权重，默认 0.001"),
        "max_grad_norm": (float, "梯度总范数上限，默认 0.5"),
        "target_kl": (float, "策略更新近似 KL 目标，默认 0.03"),
        "seed": (int, "训练种子 0～9999，默认 7"),
    }
    for name, (kind, help_text) in descriptions.items():
        parser.add_argument("--" + name.replace("_", "-"), type=kind, help=help_text)
    return parser, descriptions


def main(argv=None):
    parser, descriptions = build_parser()
    args = parser.parse_args(argv)
    if min(args.updates, args.eval_episodes, args.eval_every, args.save_every) < 1:
        parser.error("updates、eval-episodes、eval-every、save-every 必须为正整数")
    overrides = {name: getattr(args, name) for name in descriptions if getattr(args, name) is not None}
    if (args.resume or args.evaluate) and overrides:
        parser.error("加载 checkpoint 时自动使用保存的训练配置，请去掉额外的训练参数")
    if args.evaluate and args.output:
        parser.error("仅评估时不需要 --output")
    if not args.evaluate and (args.output is None or args.output.exists()):
        parser.error("训练需要 --output 指定一个尚不存在的新目录")
    # 必须在导入 torch、创建 CUDA 上下文之前设置。
    os.environ.setdefault("CUBLAS_WORKSPACE_CONFIG", ":4096:8")
    try:
        import torch
        from ppo_trainer import PPOConfig, PPOTrainer
    except ImportError:
        parser.error("缺少 PyTorch，请先按第 10 课安装 .venv-ppo 环境，再使用 scripts/lesson10_ppo.sh")
    torch.set_num_threads(1)
    torch.use_deterministic_algorithms(True)
    try:
        device = resolve_device(args.device)
        if args.resume or args.evaluate:
            trainer = load_checkpoint(args.resume or args.evaluate, device,
                                      restore_training=not bool(args.evaluate))
        else:
            trainer = PPOTrainer(PPOConfig(**overrides), device)
        hardware = torch.cuda.get_device_name(device) if device.type == "cuda" else "CPU"
        print(f"device={device} hardware={hardware} torch={torch.__version__}", flush=True)
        if args.evaluate:
            print_evaluation("evaluation", evaluate(trainer.policy, trainer.config.horizon,
                                                   args.eval_episodes, trainer.config.gamma))
            return
        args.output.mkdir(parents=True, exist_ok=False)
        baseline = evaluate(trainer.policy, trainer.config.horizon, args.eval_episodes, trainer.config.gamma)
        print_evaluation("before", baseline)
        started = time.monotonic()
        with (args.output / "metrics.jsonl").open("w", encoding="utf-8") as log:
            for iteration in range(1, args.updates + 1):
                metrics = trainer.update(trainer.collect_rollout())
                record = {"update": trainer.update_count, "global_step": trainer.global_step, **metrics}
                if iteration % args.eval_every == 0 or iteration == args.updates:
                    record["evaluation"] = evaluate(trainer.policy, trainer.config.horizon,
                                                     args.eval_episodes, trainer.config.gamma)
                    print_evaluation(f"eval update={trainer.update_count}", record["evaluation"])
                log.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
                log.flush()
                if iteration == 1 or iteration % 10 == 0 or iteration == args.updates:
                    print(f"update={trainer.update_count} steps={trainer.global_step} "
                          f"policy_loss={metrics['policy_loss']:.4f} value_loss={metrics['value_loss']:.4f} "
                          f"approx_kl={metrics['approx_kl']:.5f} clip_fraction={metrics['clip_fraction']:.3f}",
                          flush=True)
                if iteration % args.save_every == 0 or iteration == args.updates:
                    save_checkpoint(trainer, args.output / f"checkpoint_{trainer.update_count:05d}.pt")
                    save_checkpoint(trainer, args.output / "last.pt")
        if device.type == "cuda":
            torch.cuda.synchronize(device)
        trained = record["evaluation"]
        report = {"algorithm": "PPO-Clip", "task": "continuous_speed_tracking", "config": asdict(trainer.config),
                  "device": str(device), "hardware": hardware, "torch_version": str(torch.__version__),
                  "cuda_version": torch.version.cuda, "global_step": trainer.global_step,
                  "update_count": trainer.update_count, "additional_updates": args.updates,
                  "train_seconds": time.monotonic() - started,
                  "resumed_from": str(args.resume.resolve()) if args.resume else None,
                  "baseline": baseline, "trained": trained}
        write_json(args.output / "report.json", report)
        print_evaluation("after", trained)
        print(f"checkpoint: {args.output / 'last.pt'}\nreport: {args.output / 'report.json'}", flush=True)
    except (ValueError, OSError, RuntimeError, KeyError, TypeError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
