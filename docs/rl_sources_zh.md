# 强化学习课程的一手资料与固定源码

核查日期：2026-11-11。配合[第9课](lesson_09_zh.md)、[第10课](lesson_10_zh.md)、[第11课](lesson_11_zh.md)与[第12课](lesson_12_zh.md)阅读。下面区分算法资料、训练环境、部署接口；论文结果不代表本工作区已训练成功。

## 算法与并行四足训练

| 一手来源 | 阅读重点 |
|---|---|
| [Proximal Policy Optimization Algorithms，Schulman等](https://arxiv.org/abs/1707.06347) | PPO概率比、裁剪目标与实验；第10课数值实验用来解释其中的计算 |
| [High-Dimensional Continuous Control Using Generalized Advantage Estimation](https://arxiv.org/abs/1506.02438) | GAE如何组合TD残差，偏差与方差的关系 |
| [Learning to Walk in Minutes Using Massively Parallel Deep Reinforcement Learning](https://arxiv.org/abs/2109.11978) | 大规模并行环境、课程学习与四足训练的原始研究 |
| [该研究项目页](https://leggedrobotics.github.io/legged_gym/) | 论文、演示与代码入口；演示属于作者实验 |

## 本地已下载的训练参考

下载的是小型源码与作者分支的AlienGo模型资源，没有安装Isaac Gym、PyTorch、CUDA训练运行时，也没有下载训练checkpoint。完整清单见[training-repositories.json](reports/rl-training-repositories.json)。

| 来源与固定提交 | 本地入口 | 已核查的边界 |
|---|---|---|
| [leggedrobotics/legged_gym，8fa29ac](https://github.com/leggedrobotics/legged_gym/tree/8fa29acc6fd1910c3d9659eef6310bdd301cde0a) | [注册表](../references/legged_gym/legged_gym/envs/__init__.py)、[README](../references/legged_gym/README.md) | 原始框架该提交没有AlienGo任务；需要自行移植或使用下面作者分支 |
| [legubiao/legged_gym，e91e7b3](https://github.com/legubiao/legged_gym/tree/e91e7b346cfcf6b3da12b02a58c0bdf0e6f482c5) | [AlienGo配置](../references/legubiao_legged_gym/legged_gym/envs/unitree/aliengo/aliengo_config.py)、[平地配置](../references/legubiao_legged_gym/legged_gym/envs/unitree/aliengo/flat/aliengo_flat_config.py) | 注册 `aliengo`（235观测）和 `aliengo_flat`（48观测）；提供真正的AlienGo训练入口 |
| [leggedrobotics/rsl_rl，v1.0.2，2ad79cf](https://github.com/leggedrobotics/rsl_rl/tree/2ad79cf0caa85b91721abfe358105f869a784121) | [PPO](../references/rsl_rl_v1.0.2/rsl_rl/algorithms/ppo.py)、[runner](../references/rsl_rl_v1.0.2/rsl_rl/runners/on_policy_runner.py)、[rollout storage](../references/rsl_rl_v1.0.2/rsl_rl/storage/rollout_storage.py) | 与上述旧legged_gym相配；未作为Python包安装 |

三者源码的根LICENSE均为BSD-3-Clause：[原始legged_gym](../references/legged_gym/LICENSE)、[作者分支](../references/legubiao_legged_gym/LICENSE)、[rsl_rl](../references/rsl_rl_v1.0.2/LICENSE)。模型资产需要另外查看资产许可；作者分支没有单独的AlienGo资产许可证文件，因此这部分许可来源仍有待补充。

## 一次训练应从哪些文件追到哪些结论

| 问题 | 固定本地源码 |
|---|---|
| 命令到底接受哪些参数？ | [helpers.py的get_args()](../references/legubiao_legged_gym/legged_gym/utils/helpers.py) |
| 环境、runner和日志路径怎样创建？ | [train.py](../references/legubiao_legged_gym/legged_gym/scripts/train.py)、[task_registry.py](../references/legubiao_legged_gym/legged_gym/utils/task_registry.py) |
| 48维怎样排列？动作怎样变成力矩？ | [legged_robot.py](../references/legubiao_legged_gym/legged_gym/envs/base/legged_robot.py)的compute_observations、step、_compute_torques |
| 哪些随机化、奖励和PPO参数真正开启？ | [base配置](../references/legubiao_legged_gym/legged_gym/envs/base/legged_robot_config.py)，再叠加AlienGo与flat覆盖项 |
| play为何表现比训练干净？怎样导出？ | [play.py](../references/legubiao_legged_gym/legged_gym/scripts/play.py)、helpers.py的export_policy_as_jit；默认开关为EXPORT_POLICY=True，无CLI export开关 |
| 实际机型与关节模型是什么？ | [AlienGo URDF](../references/legubiao_legged_gym/resources/unitree/aliengo/urdf/robot.urdf)，运行时还须记录导入后的DOF名字 |

v1.0.2与2026-11-11核查的[当前rsl_rl runner，8945bdc](https://github.com/leggedrobotics/rsl_rl/blob/8945bdcbc364f388e14caebf547dbf302d89e491/rsl_rl/runners/on_policy_runner.py)有配置、算法对象与环境返回值契约差别。该当前版本只作在线API对照，没有下载或混入旧环境。作者分支还有seed覆盖晚于环境创建的问题，第11课给出了启动顺序解释；不能把CLI上不同的seed数字直接当成独立实验的证据。

另外，v1.0.2的超时reward补偿使用动作前的当前状态价值，区别于第10课使用reset前最终下一观测的bootstrap；第11课给出了数值反例。旧实现用于源码学习与复现，不作为这些细节的标准答案。

## 控制器与 SDK 资料

当前仿真使用宇树 Guide 控制器，配合本项目的 AlienGo 仿真参数；未运行 RL 推理。阅读 [unitree_guide](../references/unitree_guide/README.md) 和 [SDK v3.3.1](../references/unitree_legged_sdk/include/unitree_legged_sdk/comm.h) 时，应将控制器、训练器和真机协议区分开。

训练源码只作为固定参考，未证明已有策略已经训练完成或能在 ROS 1 仿真与真机运行。

## 新硬件环境的后续入口

- [NVIDIA Isaac Gym页面](https://developer.nvidia.com/isaac-gym)：官方已声明旧软件不再支持。旧参考适合学习和在相容环境复现，不能直接当成新GPU的安装指南。
- [PyTorch2.7官方发布说明](https://pytorch.org/blog/pytorch-2-7/)：记录Blackwell与CUDA12.8支持引入；这是一项历史版本证据，不是建议在所有框架中统一安装2.7。
- [Isaac Lab v3.0.0安装与要求](https://isaac-sim.github.io/IsaacLab/v3.0.0/source/isaaclab/installation.html)：本次在线核查的版本默认Newton、可选Isaac Sim。现代AlienGo任务需要另行适配与验证，不能复用旧任务名假装已经注册。

当前课程的CPU教学实验、旧训练源码、现代GPU环境移植属于不同验证层次。阅读范围与本次实际运行的检查见[训练报告](reports/rl-training.md)。

[回到课程目录](course_index_zh.md) · [第11课训练路线](lesson_11_zh.md)
