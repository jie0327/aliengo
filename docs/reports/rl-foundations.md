# 第9～10课离线实验验证记录

日期：2026-11-11。范围：标准库教学模型与数值实验；没有四足 PPO 训练或硬件操作。

## TDD 记录

先创建 `tests/test_lesson09_10.py`，再运行：

```text
python3 -m unittest discover -s tests -p test_lesson09_10.py -v
Ran 18 tests in 0.001s
FAILED (failures=19)
```

失败来自两个课程实现文件尚不存在，`setUp` 明确断言 “lab must exist”，不是导入拼写错误。数值测试覆盖手算回报、Q 更新、GAE、终止/超时 bootstrap、跨 reset 的 GAE 隔离、正负优势裁剪、留出评估改进与非有限值。随后才创建实现。

## 执行决定

- 计划的实际位置为 `docs/superpowers/plans/2026-11-11-rl-courses.md`。
- 项目根目录无 `.git`，无法创建 worktree 或提交；按已分配的独占文件范围保存。
- 第9课使用 28 个非终止状态、3 个动作的离散速度模型，不替代机器人物理仿真。
- 第10课实现 GAE/PPO-Clip 的数值运算，不声称实现完整 PPO 训练器。


## GREEN 与运行环境

实现完成后的针对性验证：

```text
python3 -m unittest discover -s tests -p test_lesson09_10.py -v
Ran 18 tests in 0.155s
OK
```

以下是历史 Python 3.12 网络隔离容器中的输出记录。命令已更新为现行 ROS 1 离线入口；2026-11-12 在 Noetic 镜像的 Python 3.8 中验证了第9课短训练与第10课数值实验。无四足训练或硬件操作。

### 命令 1

```bash
./scripts/ros1_course.sh offline lesson09/q_learning_speed.py --help
```

```text
usage: q_learning_speed.py [-h] [--episodes EPISODES] [--seed SEED]
                           [--eval-episodes EVAL_EPISODES] [--horizon HORIZON]
                           [--gamma GAMMA] [--alpha ALPHA] [--epsilon EPSILON]
                           [--json JSON]

Tabular velocity tracking: a small MDP, not a quadruped dynamics model. Only
Python's standard library is used. Evaluation never updates the table.
Velocity is an integer bin of 0.2 m/s; actions change one bin per 0.2 s.

options:
  -h, --help            show this help message and exit
  --episodes EPISODES   training episodes; 0 keeps the baseline
  --seed SEED           training RNG seed in [0, 9999]
  --eval-episodes EVAL_EPISODES
  --horizon HORIZON     external collection cutoff in steps
  --gamma GAMMA
  --alpha ALPHA
  --epsilon EPSILON     fixed training exploration probability
  --json JSON           optional report and Q table; creates parent
                        directories
```

### 命令 2

```bash
./scripts/ros1_course.sh offline lesson09/q_learning_speed.py
```

```text
train seed=7 episodes=3000 horizon=40 alpha=0.15 gamma=0.95 epsilon=0.2
held-out evaluation seeds=10000..10099 greedy=True updates=False
policy       mean_return    MAE(m/s)    terminal%    mean_steps
untrained      -10.15959     0.41141        50.00         28.22
trained         14.77693     0.05245         0.00         40.00
return = sum(gamma**t * reward_t) over the collected episode; no evaluation bootstrap
```

### 命令 3

```bash
./scripts/ros1_course.sh offline lesson09/q_learning_speed.py --episodes 100
```

```text
train seed=7 episodes=100 horizon=40 alpha=0.15 gamma=0.95 epsilon=0.2
held-out evaluation seeds=10000..10099 greedy=True updates=False
policy       mean_return    MAE(m/s)    terminal%    mean_steps
untrained      -10.15959     0.41141        50.00         28.22
trained         13.00786     0.12140         0.00         40.00
return = sum(gamma**t * reward_t) over the collected episode; no evaluation bootstrap
```

### 命令 4

```bash
./scripts/ros1_course.sh offline lesson09/q_learning_speed.py --episodes 0
```

```text
train seed=7 episodes=0 horizon=40 alpha=0.15 gamma=0.95 epsilon=0.2
held-out evaluation seeds=10000..10099 greedy=True updates=False
policy       mean_return    MAE(m/s)    terminal%    mean_steps
untrained      -10.15959     0.41141        50.00         28.22
trained        -10.15959     0.41141        50.00         28.22
return = sum(gamma**t * reward_t) over the collected episode; no evaluation bootstrap
```

### 命令 5

```bash
./scripts/ros1_course.sh offline lesson10/ppo_numbers.py --help
```

```text
usage: ppo_numbers.py [-h] [--gamma GAMMA] [--lam LAM] [--clip CLIP]
                      [--json JSON]

Hand-checkable GAE and PPO-Clip arithmetic, NOT a PPO trainer. No neural
network, autodiff, simulator, optimizer or robot connection is used. Each
next_value belongs to the final next observation, before any reset.

options:
  -h, --help     show this help message and exit
  --gamma GAMMA
  --lam LAM      GAE lambda
  --clip CLIP    PPO epsilon; objective is maximized
  --json JSON    optional numerical report
```

### 命令 6

```bash
./scripts/ros1_course.sh offline lesson10/ppo_numbers.py
```

```text
gamma=0.9 lambda=0.8 clip=0.2 (numerical lab, no training)
terminal rollout: rewards=[1,1,1] V=[0.5,0.6,0.7] V_next=[0.6,0.7,0.8]
 t       delta    advantage     V_target
 0    1.040000     1.937120     2.437120
 1    1.030000     1.246000     1.846000
 2    0.300000     0.300000     1.000000
single step: reward=1 V=0.5 V_next=2
terminal advantage=0.500000 timeout advantage=2.300000
 ratio   advantage      raw    PPO_objective(maximize)
  0.60        2.00     1.20                        1.20
  0.90        2.00     1.80                        1.80
  1.10        2.00     2.20                        2.20
  1.40        2.00     2.80                        2.40
  0.60       -2.00    -1.20                       -1.60
  0.90       -2.00    -1.80                       -1.80
  1.10       -2.00    -2.20                       -2.20
  1.40       -2.00    -2.80                       -2.80
```

### 命令 7

```bash
./scripts/ros1_course.sh offline lesson10/ppo_numbers.py --lam 0
```

```text
gamma=0.9 lambda=0.0 clip=0.2 (numerical lab, no training)
terminal rollout: rewards=[1,1,1] V=[0.5,0.6,0.7] V_next=[0.6,0.7,0.8]
 t       delta    advantage     V_target
 0    1.040000     1.040000     1.540000
 1    1.030000     1.030000     1.630000
 2    0.300000     0.300000     1.000000
single step: reward=1 V=0.5 V_next=2
terminal advantage=0.500000 timeout advantage=2.300000
 ratio   advantage      raw    PPO_objective(maximize)
  0.60        2.00     1.20                        1.20
  0.90        2.00     1.80                        1.80
  1.10        2.00     2.20                        2.20
  1.40        2.00     2.80                        2.40
  0.60       -2.00    -1.20                       -1.60
  0.90       -2.00    -1.80                       -1.80
  1.10       -2.00    -2.20                       -2.20
  1.40       -2.00    -2.80                       -2.80
```

### 命令 8

```bash
./scripts/ros1_course.sh offline lesson10/ppo_numbers.py --clip 0.1
```

```text
gamma=0.9 lambda=0.8 clip=0.1 (numerical lab, no training)
terminal rollout: rewards=[1,1,1] V=[0.5,0.6,0.7] V_next=[0.6,0.7,0.8]
 t       delta    advantage     V_target
 0    1.040000     1.937120     2.437120
 1    1.030000     1.246000     1.846000
 2    0.300000     0.300000     1.000000
single step: reward=1 V=0.5 V_next=2
terminal advantage=0.500000 timeout advantage=2.300000
 ratio   advantage      raw    PPO_objective(maximize)
  0.60        2.00     1.20                        1.20
  0.90        2.00     1.80                        1.80
  1.10        2.00     2.20                        2.20
  1.40        2.00     2.80                        2.20
  0.60       -2.00    -1.20                       -1.80
  0.90       -2.00    -1.80                       -1.80
  1.10       -2.00    -2.20                       -2.20
  1.40       -2.00    -2.80                       -2.80
```

## 学习效果与解释边界

- 固定seed=7训练3000回合；评估种子10000～10099不参与训练，策略使用贪心动作，不更新Q表。
- 未训练全零Q基线的贪心平局规则为保持动作。相同留出条件下，平均采集段折扣回报从−10.15959提高到14.77693，MAE从.41141降到.05245 m/s，越界回合由50%降到0%。这是实际计算结果，不是手写目标输出。
- 只训练100回合时回报13.00786、MAE .12140 m/s；训练0回合时两份评估完全一致。表明这个任务上学习更新有作用，但不承诺所有seed/回合数单调进步。
- 留出的是同一分布的随机种子，不是新地形或动力学。MAE按实际采集步计算，必须连同提前终止率与回合长度解释。
- 第10课手算GAE为[1.93712,1.246,.30]，终止/超时单步优势为.5/2.3；正优势上侧与负优势下侧的裁剪均通过测试。没有构建策略网络、优化器或四足训练环境。

## 一手资料核查

2026-11-11通过联网浏览阅读以下作者论文与官方文档，并在教材靠近相关结论处链接：

- Watkins/Dayan [Q-learning, 1992](https://www.gatsby.ucl.ac.uk/~dayan/papers/cjch.pdf)：动作价值、折扣与TD更新。
- Schulman等 [PPO论文v2](https://arxiv.org/abs/1707.06347v2)：式7的min、正负优势以及多epoch/minibatch流程。
- Schulman等 [GAE论文v6](https://arxiv.org/html/1506.02438v6)：式16～18、λ的偏差/方差权衡。
- [Gymnasium Handling Time Limits](https://gymnasium.farama.org/tutorials/gymnasium_basics/handling_time_limits/) 与 [Env API](https://gymnasium.farama.org/api/env/)：真正终止不bootstrap、外部截断仍bootstrap，有限任务时限与剩余时间观测。
- [OpenAI Spinning Up PPO](https://spinningup.openai.com/en/latest/algorithms/ppo.html)：on-policy、目标符号及裁剪不是硬KL约束。

PPO对照图由集成代理调用本课函数生成，已人工查看并嵌入第10课，显示正优势1.2上侧平台与负优势.8下侧平台。

## 全测试首次集成检查

```text
python3 -m unittest discover -s tests -v
Ran 125 tests in 0.739s
FAILED (failures=2)
```

当时第12课代理正在补充边界测试，失败仅为 `test_lesson12.PolicyContractTests.test_metadata_bounds_history_and_decimation_before_allocating` 的 `history_steps` 与 `decimation` 两个子用例（期望ValueError未抛出）。第9～10课18项与其余课程当时均通过。已向集成代理报告，不能将这一轮称为全套通过；最终集成结果由主报告记录。

## 最终复验

第12课完成相应边界实现后，本代理再次实际运行全套课程测试：

```text
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -q
Ran 125 tests in 0.701s
OK
```

最后一次第9～10课针对性复验为18项通过（0.147s）。两课 Markdown 的所有本地链接目标均存在；前后导航为8→9→10→11；PPO图已查看。删除了本代理运行时在两份新示例目录内产生的 `__pycache__`。未修改既有控制源码或第8课文件，未安装依赖、未连接硬件、未执行关机。

