# 第 10 课完整 PPO 训练器

用户要求将第 10 课的 PPO 数值教学补成完整训练器，并允许使用本机 GPU。

## 目标与实现选择

保留 `ppo_numbers.py` 的手算实验，新增可读的 PyTorch PPO-Clip 实现。使用连续速度跟踪任务衔接第 9 课；它有真实环境交互、可训练 actor/critic 和优化器，但不是 AlienGo 动力学模型。现有 `training/aliengo_mujoco` 已调用 Stable-Baselines3 实现机器人实验，继续作为独立路线。

考虑的方式：直接链接已有 SB3 入口最省代码，但看不到本课公式如何变成梯度更新；直接实现四足任务还会混入模型、接触和 PD 调试。选择小型连续任务上的完整 PPO 实现，使学习效果和算法边界可以分别验证。

## 行为与接口

- `speed_env.py`：批量连续速度环境。观测为当前速度和目标速度；动作经 tanh 限幅后改变速度。有真实越界终止和时间截断，step 返回 reset 前观测。
- `ppo_trainer.py`：独立 actor/critic、带 tanh 变换的高斯策略、采样旧 log 概率、两个 mask 的 GAE、全批优势标准化、多 epoch 和打乱的 minibatch、梯度裁剪、熵奖励及 KL 提前停止。
- `train_ppo.py`：命令行训练、确定性独立种子评估、JSONL 日志、定期与最终 checkpoint、恢复优化器及随机状态、单独评估。
- 支持 `--device auto|cpu|cuda`，显式请求 cuda 但不可用时清晰报错。环境与网络均放在所选设备上。
- checkpoint 保存网络、优化器、环境状态、CPU/CUDA/环境 RNG、生效配置及更新/交互计数。同设备续训可与不中断训练对照；跨 CPU/CUDA 类型仅支持独立评估，续训拒绝，以免错误恢复不兼容的环境 RNG。
- 新训练目录拒绝覆盖；续训写入新的输出目录。保留原报告和权重。
- 默认小规模验证；不启动 ROS、Gazebo、MuJoCo 或真机。

## 验收

数值测试核对 GAE 的真实终止/截断/rollout 边界、连续动作概率及正负优势裁剪。集成测试核对网络实际更新、冻结 rollout 数据、评估不学习、保存加载及续训。固定训练种子实跑训练前后对比回报、速度 MAE 和终止率，记录实际设备。原有课程测试继续通过。

## 环境

本机 RTX 5060 Laptop、驱动 595.99.02 已由 nvidia-smi 确认。独立 `.venv-ppo` 使用官方 CUDA 12.8 PyTorch 2.7.1 包；不更改 ROS 的 CPU 运行时。源码兼容 CPU，依赖为 PyTorch。现有仓库没有 HEAD，所有项目文件尚未跟踪，因此直接在用户指定工作目录增加文件，不创建空 worktree 或提交整个项目。
