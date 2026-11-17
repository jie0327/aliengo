"""可学习的 PPO-Clip：高斯策略、两个 GAE mask 和多轮小批量梯度更新。"""
from dataclasses import dataclass
import math

import torch
from torch import nn
from torch.distributions import Normal
from torch.nn import functional as F

from speed_env import SpeedEnv


@dataclass
class PPOConfig:
    num_envs: int = 8  # 批量环境数，环境和网络共用所选设备。
    rollout_steps: int = 64  # 每轮每个环境采集的步数。
    horizon: int = 80  # 每个回合的步数上限，与 rollout 长度不同。
    epochs: int = 4  # 同一批采样重复优化的遍数。
    minibatch_size: int = 128  # 每次梯度更新使用的样本数。
    learning_rate: float = 3e-4  # Adam 学习率。
    gamma: float = .95  # 未来奖励的折扣因子。
    lam: float = .95  # GAE 多步残差混合系数。
    clip: float = .2  # PPO 概率比裁剪宽度，与 Q-learning 的探索率不同。
    value_coef: float = .5  # critic 损失在总损失中的权重。
    entropy_coef: float = .001  # 高斯分布熵奖励的权重。
    max_grad_norm: float = .5  # 所有参数梯度的总范数上限。
    target_kl: float = .03  # 近似 KL 超过 1.5 倍此值时停止本轮后续优化。
    seed: int = 7  # 训练随机种子，评估种子从 10000 开始。

    def validate(self):
        for name in ("num_envs", "rollout_steps", "horizon", "epochs", "minibatch_size"):
            value = getattr(self, name)
            if type(value) is not int or value < 1:
                raise ValueError(f"{name} 必须是正整数")
        if not 2 <= self.minibatch_size <= self.num_envs * self.rollout_steps:
            raise ValueError("minibatch_size 必须在 2 与 rollout 总样本数之间")
        if type(self.seed) is not int or not 0 <= self.seed < 10000:
            raise ValueError("seed 必须是 [0, 9999] 内的整数")
        for name in ("gamma", "lam", "clip", "learning_rate", "value_coef", "entropy_coef",
                     "max_grad_norm", "target_kl"):
            if not math.isfinite(getattr(self, name)):
                raise ValueError(f"{name} 必须是有限值")
        if not 0 <= self.gamma <= 1 or not 0 <= self.lam <= 1 or not 0 <= self.clip < 1:
            raise ValueError("gamma、lam 必须在 [0, 1]，clip 必须在 [0, 1)")
        if min(self.learning_rate, self.max_grad_norm, self.target_kl) <= 0:
            raise ValueError("learning_rate、max_grad_norm、target_kl 必须为正数")
        if min(self.value_coef, self.entropy_coef) < 0:
            raise ValueError("value_coef、entropy_coef 不能为负数")


class ActorCritic(nn.Module):
    def __init__(self):
        super().__init__()
        # actor：观测 → 未经过 tanh 的高斯均值；critic：观测 → 状态价值。
        self.actor = self.network()
        self.critic = self.network()
        self.log_std = nn.Parameter(torch.tensor([-.7]))  # 可训练的高斯标准差对数。
        for network, scale in ((self.actor, .01), (self.critic, 1.)):
            for layer in network:
                if isinstance(layer, nn.Linear):
                    nn.init.orthogonal_(layer.weight, math.sqrt(2))
                    nn.init.zeros_(layer.bias)
            nn.init.orthogonal_(network[-1].weight, scale)

    @staticmethod
    def network():
        return nn.Sequential(nn.Linear(2, 64), nn.Tanh(), nn.Linear(64, 64),
                             nn.Tanh(), nn.Linear(64, 1))

    def distribution(self, observations):
        # observations：[..., 2] 的观测；返回单维高斯分布。
        mean = self.actor(observations)
        std = self.log_std.clamp(-5, 1).exp().expand_as(mean)
        return Normal(mean, std)

    def value(self, observations):
        return self.critic(observations).squeeze(-1)

    def log_prob(self, observations, latent_actions):
        # latent_actions：采样得到的原始高斯动作 u；环境实际执行 tanh(u)。
        # 稳定计算 log(1 - tanh(u)^2)，避免大动作时 log(0)。
        log_jacobian = 2 * (math.log(2) - latent_actions - F.softplus(-2 * latent_actions))
        return (self.distribution(observations).log_prob(latent_actions) - log_jacobian).sum(-1)

    def sample(self, observations):
        latent_actions = self.distribution(observations).sample()
        return latent_actions, self.log_prob(observations, latent_actions), self.value(observations)

    def deterministic_action(self, observations):
        # 评估使用 tanh(高斯均值)，随观测变化，不进行随机采样。
        return self.actor(observations).tanh()


@torch.no_grad()
def compute_gae(rewards, values, next_values, terminated, truncated, gamma=.95, lam=.95):
    """张量形状均为 [时间步 T, 环境数 N]，每个环境分别向后递推。"""
    advantages = torch.zeros_like(rewards)  # 动作优势估计。
    carry = torch.zeros_like(rewards[0])  # 下一时间步的优势，rollout 末尾从 0 开始。
    for step in reversed(range(rewards.shape[0])):
        bootstrap = (~terminated[step]).float()  # 只有真实终止屏蔽下一价值。
        continuation = (~(terminated[step] | truncated[step])).float()  # 两种结束均阻断跨回合递推。
        delta = rewards[step] + gamma * bootstrap * next_values[step] - values[step]
        carry = delta + gamma * lam * continuation * carry
        advantages[step] = carry
    targets = advantages + values  # critic 拟合的 λ-return 目标。
    return advantages, targets


def clipped_policy_loss(new_log_probs, old_log_probs, advantages, clip):
    # 两次概率都评价 rollout 中同一个旧动作；old_log_probs 和 advantages 固定。
    ratio = (new_log_probs - old_log_probs).exp()  # 新旧策略概率密度之比。
    raw = ratio * advantages
    clipped = ratio.clamp(1 - clip, 1 + clip) * advantages
    return -torch.minimum(raw, clipped).mean()  # 优化器最小化，因此取负号。


class PPOTrainer:
    def __init__(self, config, device="cpu"):
        config.validate()
        self.config = config
        self.device = torch.device(device)
        torch.manual_seed(config.seed)
        self.policy = ActorCritic().to(self.device)
        self.optimizer = torch.optim.Adam(self.policy.parameters(), lr=config.learning_rate, eps=1e-5)
        self.env = SpeedEnv(config.num_envs, config.horizon, config.seed, self.device)
        self.global_step = 0  # 累计环境转移数，一步批量 step 贡献 num_envs 条。
        self.update_count = 0  # 已完成的 PPO 采样/优化轮数。

    @torch.no_grad()
    def collect_rollout(self):
        names = ("observations", "latent_actions", "old_log_probs", "old_values", "rewards",
                 "next_observations", "next_values", "terminated", "truncated")
        data = {name: [] for name in names}
        for _ in range(self.config.rollout_steps):
            observation = self.env.observation()
            latent_action, log_prob, value = self.policy.sample(observation)
            next_observation, reward, terminated, truncated = self.env.step(latent_action.tanh())
            next_value = self.policy.value(next_observation)  # 必须在 reset 前计算。
            for name, value in zip(names, (observation, latent_action, log_prob, value, reward,
                                           next_observation, next_value, terminated, truncated)):
                data[name].append(value)
            self.env.reset(terminated | truncated)  # 仅重置已经结束的环境。
        batch = {name: torch.stack(sequence) for name, sequence in data.items()}
        batch["advantages"], batch["targets"] = compute_gae(
            batch["rewards"], batch["old_values"], batch["next_values"],
            batch["terminated"], batch["truncated"], self.config.gamma, self.config.lam)
        self.global_step += self.config.num_envs * self.config.rollout_steps
        return batch

    def update(self, batch):
        config = self.config
        # flatten：把 [T, N, ...] 展为 [T*N, ...]，使各环境样本一起打乱。
        flat = {name: tensor.flatten(0, 1).detach() for name, tensor in batch.items()}
        advantages = flat["advantages"]
        # 整批标准化，unbiased=False 也适用于最后一个只有 1 条样本的 minibatch。
        advantages = (advantages - advantages.mean()) / (advantages.std(unbiased=False) + 1e-8)
        size = advantages.numel()
        totals = {name: 0. for name in ("policy_loss", "value_loss", "base_entropy",
                                      "approx_kl", "clip_fraction", "grad_norm")}
        samples = 0  # 已执行优化的小批量样本数，含多 epoch 的重复样本。
        optimizer_steps = 0
        early_stop = False
        for _ in range(config.epochs):
            indices = torch.randperm(size, device=self.device)
            for start in range(0, size, config.minibatch_size):
                selection = indices[start:start + config.minibatch_size]
                observations = flat["observations"][selection]
                new_log_probs = self.policy.log_prob(observations, flat["latent_actions"][selection])
                log_ratio = new_log_probs - flat["old_log_probs"][selection]
                ratio = log_ratio.exp()
                approx_kl = ((ratio - 1) - log_ratio).mean()  # 非负近似 KL 的样本估计。
                if not torch.isfinite(approx_kl).item():
                    raise ValueError("策略概率出现非有限值，请降低学习率")
                if approx_kl.item() > 1.5 * config.target_kl:
                    early_stop = True
                    break
                policy_loss = clipped_policy_loss(new_log_probs, flat["old_log_probs"][selection],
                                                  advantages[selection], config.clip)
                value_loss = .5 * (self.policy.value(observations) - flat["targets"][selection]).square().mean()
                # 这里的熵是 tanh 前高斯的解析熵，日志明确命名 base_entropy。
                base_entropy = self.policy.distribution(observations).entropy().sum(-1).mean()
                loss = policy_loss + config.value_coef * value_loss - config.entropy_coef * base_entropy
                if not torch.isfinite(loss).item():
                    raise ValueError("PPO 损失出现非有限值")
                self.optimizer.zero_grad(set_to_none=True)
                loss.backward()
                grad_norm = nn.utils.clip_grad_norm_(self.policy.parameters(), config.max_grad_norm,
                                                     error_if_nonfinite=True)
                self.optimizer.step()
                metrics = (policy_loss, value_loss, base_entropy, approx_kl,
                           ((ratio - 1).abs() > config.clip).float().mean(), grad_norm)
                for name, value in zip(totals, metrics):
                    totals[name] += value.detach().item() * selection.numel()
                samples += selection.numel()
                optimizer_steps += 1
            if early_stop:
                break
        self.update_count += 1
        return {**{name: value / max(1, samples) for name, value in totals.items()},
                "optimizer_steps": optimizer_steps, "early_stop": early_stop}
