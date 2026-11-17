"""用于 PPO 教学的连续速度跟踪环境；没有机器人动力学或 ROS 依赖。"""
import torch


class SpeedEnv:
    """单进程批量环境；每个元素是一条独立回合，支持 CPU 和 CUDA 张量。"""

    def __init__(self, num_envs=8, horizon=80, seed=7, device="cpu"):
        # num_envs：同时采样的环境数；horizon：每回合最大步数；seed：环境随机种子。
        if type(num_envs) is not int or num_envs < 1 or type(horizon) is not int or horizon < 1:
            raise ValueError("num_envs 和 horizon 必须是正整数")
        self.num_envs = num_envs
        self.horizon = horizon
        self.device = torch.device(device)  # 环境状态所在设备。
        self.rng = torch.Generator(device=self.device).manual_seed(seed)  # 独立于策略采样的 RNG。
        self.velocity = torch.zeros(num_envs, device=self.device)  # 当前速度，单位 m/s。
        self.command = torch.zeros_like(self.velocity)  # 目标速度，单位 m/s。
        self.steps = torch.zeros(num_envs, dtype=torch.long, device=self.device)  # 回合内步数。
        self.done = torch.ones(num_envs, dtype=torch.bool, device=self.device)  # 是否等待 reset。
        self.reset()

    def observation(self):
        # 观测最后一维：[归一化当前速度, 归一化目标速度]。
        return torch.stack((self.velocity / .8, self.command / .4), dim=-1)

    @torch.no_grad()
    def reset(self, mask=None):
        # mask：需要重置的环境；未指定时全部重置，不影响其他仍在进行的回合。
        if mask is None:
            mask = torch.ones_like(self.done)
        size = int(mask.sum().item())
        if size:
            self.velocity[mask] = torch.rand(size, generator=self.rng, device=self.device) * .8 - .4
            choices = torch.tensor([-.4, -.2, .2, .4], device=self.device)  # 目标速度集合。
            indices = torch.randint(4, (size,), generator=self.rng, device=self.device)
            self.command[mask] = choices[indices]
            self.steps[mask] = 0
            self.done[mask] = False
        return self.observation()

    @torch.no_grad()
    def step(self, action, active=None):
        # action：[环境数, 1] 的连续动作，范围 [-1, 1]。
        # active：评估时冻结已结束的槽位；训练时默认所有环境都运行。
        if active is None:
            active = torch.ones_like(self.done)
        if (self.done & active).any().item():
            raise ValueError("结束的环境必须先 reset 再 step")
        if action.shape != (self.num_envs, 1) or not torch.isfinite(action).all().item():
            raise ValueError("action 必须是 [num_envs, 1] 的有限数值")
        if (action.abs() > 1).any().item():
            raise ValueError("action 必须位于 [-1, 1]，请使用策略的 tanh 输出")
        wind = .01 * torch.randn(self.num_envs, generator=self.rng, device=self.device)
        # 每步 0.2 秒；最大动作改变 0.1 m/s，wind 是随机速度扰动，单位 m/s。
        self.velocity += (.1 * action.squeeze(-1) + wind) * active
        self.steps += active.long()
        terminated = (self.velocity.abs() > .8) & active  # 速度越界表示任务失败。
        truncated = (self.steps >= self.horizon) & ~terminated & active  # 时间上限不等于真实终止。
        error = self.velocity - self.command  # 有符号速度误差，单位 m/s。
        reward = 1 - (error / .2).square() - .01 * action.squeeze(-1).square()
        reward -= 5 * terminated.float()  # 越界附加惩罚。
        self.done |= terminated | truncated
        # 显式返回 reset 前的最终观测，由采样器先计算 next_value 再 reset。
        return self.observation(), reward, terminated, truncated

    def state_dict(self):
        # 保存回合状态和环境 RNG，使断点续训不丢失未完成的回合。
        return {"velocity": self.velocity.clone(), "command": self.command.clone(),
                "steps": self.steps.clone(), "done": self.done.clone(),
                "rng": self.rng.get_state()}

    def load_state_dict(self, state):
        for name in ("velocity", "command", "steps", "done"):
            value = state[name].to(self.device)
            if value.shape != (self.num_envs,):
                raise ValueError("checkpoint 环境数量与配置不一致")
            setattr(self, name, value.clone())
        self.rng.set_state(state["rng"].cpu())
