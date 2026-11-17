# 第11课训练参考核查与验证记录

日期：2026-11-11。工作目录 `/home/jack/aliengo_learning`。对应[第11课](../../docs/lesson_11_zh.md)、[一手资料索引](../../docs/rl_sources_zh.md)和[来源锁定记录](rl-training-repositories.json)。

## 实际完成与验证边界

- 下载3份小型Git参考，全部固定detached HEAD；原始legged_gym仅源码和许可证，作者分支另含AlienGo URDF/mesh/texture，rsl_rl为v1.0.2。
- 工作树磁盘占用（含各自Git元数据，约数）：原始legged_gym 596 KiB、作者分支8.3 MiB、rsl_rl 328 KiB。未下载Isaac Gym、PyTorch、CUDA训练运行时和训练checkpoint；未执行Python包安装。
- 实跑教材中的CPU源码检查；72个参考Python文件通过AST语法解析；7个Bash命令块通过 `bash -n`。语法检查不代表训练环境可运行。
- 在线通过web检索与原作者/官方网页交叉核查机型、版本、算法入口、导出和现代环境边界；未依赖二手安装教程得出相容性结论。
- 未执行完整四足环境、PPO训练、policy评估、JIT导出数值比对、Sim2Sim或真机操作。没有向机器人发送报文，没有启动RL控制器，没有更改已有源码或学习者注释。

主机[PCI记录](../driver_installation_2026-11-12_zh.md)可见NVIDIA设备 `10de:2d19`，root硬件核查将其识别为RTX 5060 Laptop。初始 `nvidia-smi` 与torch环境缺失，仅说明驱动/训练软件尚未就绪；本报告不声称“没有GPU硬件”。旧Python3.8/PyTorch1.13.1/CUDA11.7/Isaac Gym Preview4路线未在本机Blackwell上验证，也不作为它的直接安装建议。驱动下载/权限核查属于root的独立记录，本子任务未安装驱动。

## 固定源码与机型结论

| 参考 | SHA | 核查结果 |
|---|---|---|
| leggedrobotics/legged_gym master | `8fa29acc6fd1910c3d9659eef6310bdd301cde0a` | 注册ANYmal、A1、Cassie，没有AlienGo；不能直接给它传aliengo任务名 |
| legubiao/legged_gym master | `e91e7b346cfcf6b3da12b02a58c0bdf0e6f482c5` | 注册aliengo与aliengo_flat，前者235观测含187高度、后者48；动作12维，AlienGo URDF在resources/unitree/aliengo |
| leggedrobotics/rsl_rl v1.0.2 | `2ad79cf0caa85b91721abfe358105f869a784121` | 与作者README要求一致，runner/ActorCritic/五项step接口与旧环境相配 |

三份根LICENSE均为BSD-3-Clause，原始声明完整保留。legged_gym根LICENSE另要求查看资产许可证；作者分支未提供单独AlienGo资产许可证文件，课程明确记录该缺口，没有推定所有资产为BSD。URL、请求ref、稀疏路径和许可证文件均写入JSON，供root合并根锁文件。

当前rsl_rl在线比较固定到 `8945bdcbc364f388e14caebf547dbf302d89e491`：runner建立算法方式、配置布局与四项step契约已不同，因此不能把当前rsl-rl-lib与固定旧legged_gym直接组合。该当前版本未下载为第四个本地仓库。

## 教材CPU命令的实际输出

执行方式：提取 `docs/lesson_11_zh.md` 第一个Bash代码块，交由Bash完整执行；它只使用Python标准库AST/XML解析，不导入训练包。退出码0。

```text
obs=48 actions=12 movable_joints=12
physics_hz=200.0 policy_hz=50.0
rollout_s=0.48 batch=98304
urdf_xml_order=FR_hip_joint,FL_hip_joint,RR_hip_joint,RL_hip_joint
```

最后一行是URDF XML排列，不能证明仿真导入后的DOF排列。完整环境首次创建时仍须记录gym.get_asset_dof_names()并按名字建立部署映射。

同次检查结果：

```text
Bash syntax: 7 blocks passed (conditional commands not executed)
references/legged_gym: clean=True
references/legubiao_legged_gym: clean=True
references/rsl_rl_v1.0.2: clean=True
Reference Python syntax: 72 files parsed; no packages imported
Repository SHA and license paths: 3/3 passed
```

随后单独解析并执行作者helpers.py的 `update_cfg_from_args()` 函数，使用标准库SimpleNamespace代替配置对象，验证seed陷阱（没有导入完整helpers模块）。结果：

```text
seed audit: CLI seed=2 leaves env_cfg.seed=1, changes train_cfg.seed=2
Markdown fences: balanced in both documents
```

源码调用顺序同时确认 `make_env()` 先按env_cfg.seed调用set_seed，再创建runner时才覆盖train_cfg.seed。课程的多seed条件片段因此在make_env前同步两个seed字段；该片段只完成语法检查，未在Isaac Gym环境中执行。

## 训练、评估与导出命令的来源

教材所有完整环境命令都标注“条件步骤、未实跑”：

- 零动作环境检查来自作者 `legged_gym/tests/test_env.py`，不能视为训练。
- 小规模CPU PPO与4096环境完整训练来自作者 `legged_gym/scripts/train.py`，CLI参数核对helpers.py；完整示例选择 `aliengo_flat`。
- 运行目录与checkpoint选择核对task_registry.py、helpers.py以及rsl_rl v1.0.2的保存逻辑，1500轮参考对应最终model_1500.pt。
- 播放与默认导出核对play.py。该提交没有export CLI flag，代码内EXPORT_POLICY=True；非循环策略保存到 `logs/flat_aliengo/exported/policies/policy_1.pt`。训练checkpoint字典与JIT actor不是同一格式。
- stock play关闭噪声、摩擦随机化与push，未实现完整held-out评估器。课程提供多seed/独立测试集/物理量指标的协议，明确这些指标尚无结果。

## 与现有ROS控制链的差异

主仓RL控制器和作者训练分支同属legubiao；缺少本地已有policy.pt对应唯一训练run/config的来源清单，因此不能归因于本课固定配置。源码可直接核实的差异是：

- 训练Kp/Kd为60/3，ROS策略YAML为20/0.75。
- 训练默认hip为0 rad，ROS stand_pos为±0.1 rad。
- 训练顺序来自gym导入DOF名字，URDF XML为FR/FL/RR/RL；ROS RL YAML为FL/FR/RL/RR。
- 训练含机身线速度真值，部署需匹配估计器与坐标系，不能直接用IMU加速度替代。

这些差异没有通过替换现有配置“修正”；它们是第12课离线契约审查的输入。

## 一手在线核查入口

- [原始legged_gym固定注册表](https://github.com/leggedrobotics/legged_gym/blob/8fa29acc6fd1910c3d9659eef6310bdd301cde0a/legged_gym/envs/__init__.py)、[作者分支固定README](https://github.com/legubiao/legged_gym/blob/e91e7b346cfcf6b3da12b02a58c0bdf0e6f482c5/README.md)、[AlienGo配置](https://github.com/legubiao/legged_gym/blob/e91e7b346cfcf6b3da12b02a58c0bdf0e6f482c5/legged_gym/envs/unitree/aliengo/aliengo_config.py)。
- [rsl_rl v1.0.2 runner](https://github.com/leggedrobotics/rsl_rl/blob/v1.0.2/rsl_rl/runners/on_policy_runner.py)、[当前API比较的固定runner](https://github.com/leggedrobotics/rsl_rl/blob/8945bdcbc364f388e14caebf547dbf302d89e491/rsl_rl/runners/on_policy_runner.py)。
- [NVIDIA Isaac Gym停止支持说明](https://developer.nvidia.com/isaac-gym)、[PyTorch2.7引入Blackwell支持](https://pytorch.org/blog/pytorch-2-7/)、[Isaac Lab v3.0.0安装与要求](https://isaac-sim.github.io/IsaacLab/v3.0.0/source/isaaclab/installation.html)。现代AlienGo移植尚未完成。
- PPO、GAE与Learning to Walk in Minutes三篇原论文入口已在线核实，链接见[资料索引](../../docs/rl_sources_zh.md)。
