**配对代码：** [gait_table.py](../examples/lesson05/gait_table.py) → [State_Trotting.cpp](../references/unitree_guide/unitree_guide/src/FSM/State_Trotting.cpp) → [WaveGenerator.cpp](../references/unitree_guide/unitree_guide/src/Gait/WaveGenerator.cpp)。

# 第 5 课：从站立到迈步——状态机与步态

[上一课：运动学](lesson_04_zh.md) · [学习路线](learning_path_zh.md) · [下一课：状态估计](lesson_06_zh.md)

本课把“何时切换控制方法”和“何时轮到哪条腿抬脚”分开理解。前者是状态机，后者是步态相位。建议先用 45 分钟完成源码追踪与离线 CSV；站立条件确认后，再由你自主进行短时仿真实验。

**前置：** 完成第 1 课的消息与固定站立观察；理解 PD 与第 4 课坐标系/FK 的用途。若当前只能显示模型、还没有固定站立记录，先完成第 1 课。本课提供小跑实验步骤；现有 [验证记录](verification.md) 只支持短时收腿、固定站立和反馈检查，不能当作小跑验收。

**完成后你应能：** 根据实际源码画状态图；解释 `command` 与连续方向输入的区别；手算 FR/RL、FL/RR 的同相关系；用观察证据区分“没切换状态”“没进入周期步态”和“输出异常”。本课离线程序不会发布任何控制消息。

## 1. 键盘、状态与速度意图

Guide 的 [KeyBoard.cpp](../references/unitree_guide/unitree_guide/src/interface/KeyBoard.cpp) 直接读取终端。`2` 触发站立，`4` 请求小跑，`w/s`、`a/d`、`j/l` 分别调整前后、侧移和转向，空格把速度意图清零。速度输入累加并保持；状态切换和速度意图是不同的量。

## 2. 状态机

从 Passive 通过 `2` 进入 FixedStand；等站稳后按 `4` 进入 Trotting。状态的进入、运行和退出分别由 [FSM.cpp](../references/unitree_guide/unitree_guide/src/FSM/FSM.cpp) 和各状态实现处理；进入状态不等于已经稳定迈步。

## 3. 找到控制链

从 [IOROS.cpp](../references/unitree_guide/unitree_guide/src/interface/IOROS.cpp) 追踪反馈和电机目标，再阅读 [State_Trotting.cpp](../references/unitree_guide/unitree_guide/src/FSM/State_Trotting.cpp) 的轨迹、平衡力和关节目标计算。仿真中实际构建的是针对 AlienGo 参数的适配版本，位于 `.ros1/src/unitree_guide`。

## 4. 支撑、摆动与“相位”的量纲

**支撑期**：计划让脚支撑机身；**摆动期**：让脚离地，从起点移到下一落脚点。这里先学习计划的接触标记，不把它当成传感器测得的真实接触。

设周期 `T=0.45 s`，支撑占比 `r=0.5`，则每条腿每周期支撑 `0.225 s`、摆动 `0.225 s`。周期相位 `u` 和阶段内部相位 `p` 都无单位，但不是同一个量：

```text
u_i = (t/T - bias_i) mod 1       一个整周期在 [0,1) 的进度
u_i < r:  支撑，contact=1，p=u_i/r
否则:     摆动，contact=0，p=(u_i-r)/(1-r)
```

阶段内部的 `p` 每进入一次支撑或摆动都会重新从 0 开始，到下一切换前接近 1。不要把 CSV 里的 `phase=0.5` 永远解释成“整个周期过了一半”。

Guide 在 `UnitreeGuideController.cpp` 中使用：

```cpp
WaveGenerator(0.45, 0.5, Vec4(0, 0.5, 0.5, 0));
```

此教学相位表使用 `FR FL RR RL` 顺序。所以 FR/RL 的偏置都是 0，FL/RR 都是 0.5，构成两组对角同相。

为方便手算，下面临时用 `T=0.5 s`：

| 时间（s） | FR / RL | FL / RR | 阶段内部 p |
|---|---|---|---|
| 0.000 | 支撑开始 | 摆动开始 | 都是 0 |
| 0.125 | 支撑中点 | 摆动中点 | 都是 0.5 |
| 0.250 | 摆动开始 | 支撑开始 | 都是 0 |
| 0.375 | 摆动中点 | 支撑中点 | 都是 0.5 |
| 0.500 | 下一周期支撑开始 | 下一周期摆动开始 | 都是 0 |

边界采用半开区间：`u=r` 已进入摆动；`u=1` 回到下一周期的 0。若把 `r` 改为 `0.6`，同一对角关系仍然存在，但可能出现两组同时支撑的区间；所以不要硬背“一直恰好两条腿支撑”。

## 5. 离线生成相位与抬脚 CSV

先预测第一组四行的 `contact`，再运行：

```bash
./scripts/ros1_course.sh offline lesson05/gait_table.py --period 0.5 --duration 0.5 --dt 0.125
```

输出列含时间 `time_s`、腿名、整周期进度 `cycle`、计划接触 `contact`、阶段进度 `phase`，以及摆动局部偏移和速度。首四行腿序 FR/FL/RR/RL 的接触值应是 `1,0,0,1`；`0.25 s` 时交换为 `0,1,1,0`；`0.5 s` 回到第一组。

保存默认周期的两周期样本；重定向发生在主机，CSV 留在你的实验目录：

```bash
mkdir -p artifacts/lesson05
./scripts/ros1_course.sh offline lesson05/gait_table.py --period 0.45 --duration 0.9 --dt 0.05625 > artifacts/lesson05/gait.csv
head -n 9 artifacts/lesson05/gait.csv
```

只改变支撑占比后再预测：

```bash
./scripts/ros1_course.sh offline lesson05/gait_table.py --period 0.5 --stance-ratio 0.6 --duration 0.5 --dt 0.05
```

`sample` 实现相位表；`swing` 用摆线让抬脚两端速度接近零：

```text
x(p) = D [p - sin(2πp)/(2π)]
z(p) = H [1 - cos(2πp)] / 2
```

`D` 是本次摆动的水平行程，`H` 是最高抬脚高度，单位均为米；`p` 无单位。速度需要对实际时间求导，所以 `vx/vz` 还要除以摆动时间 `T(1-r)`，单位 `m/s`。

例如 `D=0.1 m, H=0.04 m, 摆动时间=0.5 s`：起点偏移 `(0,0)`，中点 `(0.05,0.04)`，终点 `(0.1,0)`；中点水平速度为 `0.4 m/s`，竖直速度为 0。**平均速度与峰值速度不同**，不能用 `D/时间` 当作全过程速度。

注意 CSV 中 `swing_x_m/swing_z_m` 是每次摆动相对起点的局部示意量；支撑行用 0 标记“不处在摆动段”，于是列值在阶段交界可能重置。它不是脚在世界中的连续轨迹，不能拿整列差分当作真实足速度，也不能把 CSV 直接当关节目标播放器。采样时间是 `0,dt,2dt,...`，只输出不超过 `duration` 的等间隔样本。

打开 [gait_table.py](../examples/lesson05/gait_table.py)，从 `LEGS/BIASES → sample → swing → csv.DictWriter` 阅读。`zip` 将每条腿与偏置配对；`% 1` 让进度周而复始；`contact` 决定当前应使用哪一个阶段公式。参数若出现零周期、非法支撑占比或非有限值，程序会退出并报告错误。每次最多输出 100000 个时间样本（每个样本四条腿），避免误设极小 `dt` 产生无界 CSV。

## 6. 教学表与 Guide 的区别

离线相位表使用固定参数和简化轨迹，[WaveGenerator.cpp](../references/unitree_guide/unitree_guide/src/Gait/WaveGenerator.cpp) 和 [GaitGenerator.cpp](../references/unitree_guide/unitree_guide/src/Gait/GaitGenerator.cpp) 还处理状态切换与足端目标；不能用离线输出预测现场的全部接触。

## 7. 仿真实验

按 [ROS 1 仿真说明](ros1_sim_zh.md) 站稳、进入小跑并轻按两次 `w`，用空格清零，观察机器人和反馈。停止或出现异常时记录按键、经过的仿真时间和姿态，避免同时运行多个控制器。

## 8. 排错

检查终端焦点、状态切换、关节与 IMU 反馈、模型参数以及步态接触假设。已通过的速度和时间范围见仿真说明，不能推广到所有地形和命令。

## 9. 三个验收问题

1. 状态切换与速度意图分别是什么？空格会清除什么？
2. 在对角小跑中，哪两只脚按同一相位运动？相位从 0 到 1 表示什么？
3. 为什么离线相位表不能证明机器人在仿真中一定不会摔倒？
