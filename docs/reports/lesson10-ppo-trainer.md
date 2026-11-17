# 第 10 课 PPO 训练器验证

验证日期：2026-11-16。对象为本课程新增的连续速度跟踪环境和 PyTorch PPO-Clip 实现。

## 代码与训练闭环

- `examples/lesson10/speed_env.py`：批量速度环境，独立 RNG，真实终止与时间截断。
- `examples/lesson10/ppo_trainer.py`：actor/critic、tanh 高斯密度、rollout、GAE、多 epoch/minibatch、Adam、梯度裁剪和 KL 提前停止。
- `examples/lesson10/train_ppo.py`：CPU/CUDA 选择、独立确定性评估、JSONL、checkpoint 与续训。
- `scripts/lesson10_ppo.sh`：使用独立 `.venv-ppo`，沿用项目计算互斥锁。

模型契约为两维归一化速度观测、一维动作；任务和物理机器人训练入口分开。

## CPU 实跑

运行于已有 `aliengo-learning:ppo-cpu` 容器的 PyTorch 2.4.1+cpu，1 个 PyTorch 线程。训练配置为种子 7、8 环境、64 步 rollout、80 轮更新，共 40960 条转移。其他参数使用默认值。每次在独立随机流上确定性评估 32 回合，各回合最多 80 步。

| 指标 | 训练前 | 训练后 |
|---|---:|---:|
| 平均未折扣回报 | -263.30316 | 73.40430 |
| 平均折扣回报 | -61.69098 | 13.66299 |
| 速度 MAE，m/s | 0.32772 | 0.01862 |
| 越界失败比例 | 0% | 0% |
| 平均回合步数 | 80 | 80 |

采样/优化循环含周期评估及保存约 2.89 秒。训练前确定性均值动作接近零，环境的微小扰动不一定使速度越界，因此失败率同为 0% 不能单独证明跟踪有效；误差和回报体现了本次学习改善。

结果保存在 [CPU 报告](../../results/lesson10_ppo_cpu_check/report.json)、[逐轮日志](../../results/lesson10_ppo_cpu_check/metrics.jsonl) 和同目录 `last.pt`。

## 自动检查与审查

- 基线项目测试：207 项，24 项因缺少对应环境跳过，其余通过。
- CPU PyTorch 容器中的 14 项训练器测试通过；随后增加 CUDA 续训回归测试，最终 15 项在 `.venv-ppo` 的 PyTorch 2.7.1+cu128 中全部通过，包含真实 GPU 采样和恢复验证。
- 最终宿主全套：222 项，39 项跳过，其余通过；新增 15 项在没有 PyTorch 的宿主解释器下跳过，实际计算验证在上述独立 PyTorch 环境中执行。
- 无第三方依赖的 `--help` 正常；缺少 PyTorch 时输出明确的安装提示。
- 同设备 CPU checkpoint 恢复后，下一轮指标和所有网络参数与不中断训练逐位一致；网络和优化器状态、未结束回合及 RNG 均恢复。
- 独立只读代码审查复核 GAE、tanh 密度、最后一个小批量只有 1 条样本、评估状态与 RNG。发现并修复了无 PyTorch 时的测试导入问题，以及损坏 checkpoint 的 CLI traceback；均先复现失败，再运行回归验证。

## GPU 实跑

本机实际使用 NVIDIA GeForce RTX 5060 Laptop GPU（8151 MiB，计算能力 12.0），驱动 595.99.02，独立 `.venv-ppo` 内 PyTorch 2.7.1+cu128 / CUDA 12.8、NumPy 2.2.6。安装包按官方 PyTorch/PyPI 的 SHA256 核验；忽略终端 ROS 的 Python 路径后，`pip check` 返回 `No broken requirements found`。

已执行真实 CUDA 张量前向和反向传播，以及默认规模的完整训练：

```bash
./scripts/lesson10_ppo.sh --device cuda \
  --output results/lesson10_ppo_gpu_check --updates 80
./scripts/lesson10_ppo.sh --device cuda \
  --evaluate results/lesson10_ppo_gpu_check/last.pt
```

| 指标 | 训练前 | 训练后 |
|---|---:|---:|
| 平均未折扣回报 | -195.57803 | 75.35838 |
| 平均折扣回报 | -46.11018 | 15.40393 |
| 速度 MAE，m/s | 0.28408 | 0.01455 |
| 越界失败比例 | 0% | 0% |
| 平均回合步数 | 80 | 80 |

训练 80 轮，共 40960 条转移，循环含周期评估与保存约 9.51 秒。GPU 和 CPU 的环境 RNG 实现不同，同一 seed 的具体采样不同；各自的训练前后对比使用同一设备和相同评估协议。这两次短运行的耗时也不构成严格的设备性能比较。

GPU 保存权重的独立加载评估复现上述训练后结果。另以短批次对照确认环境/rollout/网络都在 CUDA 上，GPU checkpoint 恢复后的下一轮指标与网络参数和不中断训练逐位相同；跨设备加载用于独立评估成功，跨设备续训清楚拒绝。

命令行续训也实际运行：从第 80 轮权重额外更新 2 轮，输出计数正确恢复为第 82 轮、41984 条转移，见 [续训报告](../../results/lesson10_ppo_gpu_resume_check/report.json)。最终 `.venv-ppo` 的 [依赖版本清单](../../results/lesson10_ppo_gpu_check/runtime-requirements.txt) 已归档；启动脚本忽略 ROS 可能注入的 `PYTHONPATH/PYTHONHOME`，并通过注入同名模块路径的行为检查。

原始数据见 [GPU 报告](../../results/lesson10_ppo_gpu_check/report.json)、[逐轮日志](../../results/lesson10_ppo_gpu_check/metrics.jsonl)、[CUDA 与 checkpoint 检查](../../results/lesson10_ppo_gpu_check/gpu_checks.json)。保存的 `last.pt` 可按课程命令加载。这些结果验证了连续速度任务的 PPO 训练闭环。
