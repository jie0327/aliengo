**配对代码：** [drift_demo.py](../examples/lesson06/drift_demo.py)、[ROS 1 观测节点](../examples/lesson06/ros1_observe_state.py) → [Estimator.cpp](../references/unitree_guide/unitree_guide/src/control/Estimator.cpp)。

# 第 6 课：状态估计——测到了什么，又推算了什么

[上一课：状态机与步态](lesson_05_zh.md) · [学习路线](learning_path_zh.md) · [下一课：受力与 MPC](lesson_07_zh.md)

建议 45～60 分钟。前置：能按名字读取关节角；知道足端位置必须带坐标系；能区分支撑期和摆动期。读本课不用安装滤波库，也不用切换机器人状态。

本课结束后，你应能解释四类数据、算出小偏置怎样积成大漂移，并用一个有次数和超时限制的只读程序检查 IMU 与关节反馈。本课的离线数字是可重复教学实验；当前仿真的实际验收范围仍以[验证记录](verification.md)为准。

## 1. 先给每个数字贴标签

| 数据 | 从哪里来 | 它能说明什么；还缺什么 |
|---|---|---|
| IMU 角速度、加速度 | `/imu_sensor_broadcaster/imu` 的传感器接口 | rad/s、m/s²；有噪声、偏置、安装方向与时间戳；仿真里由传感器模型生成 |
| IMU 消息中的 `orientation` | 仿真姿态或设备内部姿态融合 | 是四元数形式的姿态信息，并非加速度计直接测出的“原始角度”；真机是否提供、参考系是什么须核对设备 |
| 关节 `position`、`velocity`、`effort` | `/joint_states` | 角度 rad、角速度 rad/s、力矩 N·m；速度可能由差分得到，effort 的物理来源取决于硬件接口 |
| 机身位置、速度、世界中的脚位置 | Guide 的 `Estimator` 内部状态 | 结合 IMU、关节运动学、支撑假设和模型推算；不是 `/joint_states` 直接给出的量 |
| 仿真真值 ground truth | 仿真引擎的模型位置和速度 | 用来评价估计误差；真机没有一条天然准确的“世界位置真值”反馈 |

“消息里有数字”与“有独立传感器直接测到”要分开。SDK 的 `tauEst` 也是估计输出力矩，不能自动视作独立力矩传感器。第 8 课再建立 ROS / SDK 字段映射。

**先预测：** 只读 12 个关节角，能否知道整台机器人在房间中向前移动了几米？考虑整台机器人连同四脚一起平移，而关节角不变的情况。

## 2. 一个四元数为什么要写四个数

本课约定世界系 W 的 z 向上，传感器系 S 固连 IMU；`R_WS` 把 S 中表达的向量转到 W。机身系 B 与 S 未必重合，实际应用还需要安装变换。四元数表示旋转，不是四个欧拉角。

| 接口 | 分量顺序 | 无旋转 |
|---|---|---|
| ROS `geometry_msgs/Quaternion`、本课 Python 函数 | `(x, y, z, w)` | `(0, 0, 0, 1)` |
| Unitree SDK v3.3.1 `IMU.quaternion[4]` | `(w, x, y, z)` | `(1, 0, 0, 0)` |
| 本项目 Guide 的 `Quat` 向量和 `quatToRotMat()` | `(w, x, y, z)` | `(1, 0, 0, 0)` |

ROS 使用带名字的四元数字段 `x/y/z/w`；SDK 数组可能使用不同顺序。不要把数组下标当作所有接口的统一约定。[官方 SDK 结构定义](../references/unitree_legged_sdk/include/unitree_legged_sdk/comm.h)。

手算例子：绕世界 z 正向旋转 90°，ROS 四元数为 `(0, 0, √0.5, √0.5)`。S 的向前向量 `(1,0,0)` 旋转到 W 后是 `(0,1,0)`。SDK 中同一旋转写成 `(√0.5,0,0,√0.5)`。两种写法只是重排，物理旋转没有变。`q` 和 `−q` 也表示同一旋转，不能只靠分量相减衡量姿态误差。

打开 [drift_demo.py](../examples/lesson06/drift_demo.py)，看 `wxyz_to_xyzw()` 的这两行：

```python
w, x, y, z = finite_values(quaternion, 4)
return x, y, z, w
```

`rotate_xyzw()` 会检查维度、非有限值和零四元数，然后归一化。归一化可以修正轻微数值误差，不能修正分量顺序、反向变换或错误安装轴。

## 3. 静止时读到约 9.81，为什么没有飞起来

按 ROS IMU 约定，加速度字段通常是传感器系的比力 `f_S`。静止、传感器 z 朝上时，它约为 `(0,0,+9.81)`，并不是世界坐标中的运动加速度。[REP 145 官方约定](https://github.com/ros-infrastructure/rep/blob/master/rep-0145.rst)也说明：姿态可能由设备融合提供，传感器参考世界系并非总能从 TF 中读出。

先明确比力和重力，再做运算：

```text
f_S = R_SW (a_W − g_W)
a_W = R_WS f_S + g_W
g_W = (0, 0, −9.81) m/s²
```

水平静止时 `R_WS=I`，于是 `a_W=(0,0,9.81)+(0,0,−9.81)=0`。若 IMU 翻转 180°，S 的读数可成为 `(0,0,−9.81)`，必须先旋转回 W，再加 `g_W`。直接对 S 的 z 分量减 9.81 会算错。

本课的 `world_acceleration()` 对应这个算式，但只适用于你已确认“该 quaternion 表示 S→W，W 为 z-up，而且字段是比力”的输入。有的驱动已经移除了重力；再减一次会引入错误。IMU 不在机身原点时，转动还会带来杆臂的切向、向心加速度。不要把本课函数直接当作 AlienGo 的完整机身估计器。

## 4. 离线实验：0.02 m/s² 的偏置能有多大影响

假设机器人实际静止、姿态与重力补偿都正确，但世界 x 方向还剩常值加速度误差 `b=0.02 m/s²`，初始速度和位置误差均为零。

先积分一次，速度误差 `Δv(t)=bt`；再积分一次，位置误差 `Δp(t)=½bt²`。10 秒后是 `0.2 m/s` 和 `1 m`；20 秒后是 `0.4 m/s` 和 `4 m`。时间加倍，位置误差变为四倍。

本课使用每一步内加速度恒定的精确积分：

```python
position += velocity * step + 0.5 * bias * step * step
velocity += bias * step
```

第二行用来更新下一步速度；第一行的 `velocity` 是本步开始时的值。单位检查：`m/s × s` 和 `m/s² × s²` 都是米。

从项目根目录运行：

```bash
cd /home/jack/aliengo_learning
./scripts/ros1_course.sh offline lesson06/drift_demo.py --bias 0.02 --duration 10 --dt 0.01 --csv logs/course/lesson06_drift_10s.csv
./scripts/ros1_course.sh offline lesson06/drift_demo.py --bias 0.02 --duration 20 --dt 0.01 --csv logs/course/lesson06_drift_20s.csv
./scripts/ros1_course.sh offline lesson06/drift_demo.py --bias 0 --duration 10 --dt 0.01
```

**预测：** 先写出三次运行的最终位置误差，再运行。**动手：** 把偏置换成 `-0.02`，然后只把步长换成 `0.001`。**预期：** 负偏置改变漂移方向；减小步长仍得到约 `1 m`。这是偏置造成的物理积分误差，增加采样率无法消除它。本例未加入随机噪声、姿态误差或滤波，不能用其结果预测真机准确度。

CSV 三列是时间、速度误差、位置误差，首行为零；最后一步不足 `dt` 时会缩短，避免多积分一段时间。可用表格软件画 `time_s` 对 `position_error_m`。

## 5. 时间戳是计算的一部分

ROS stamp 换成秒的方法是 `sec + nanosec × 10⁻⁹`。同一个流中相邻测量的步长是 `dt=t_new−t_old`，不能拿两个不同时间起点的时钟直接相减。

例如 `t_old=1.00 s`、`t_new=1.01 s`，得到 `dt=0.01 s`；若 Gazebo 重置后时间变成 `0.00 s`，则是倒退，需要清空或重新初始化积分状态。实际仿真的高频 broadcaster 可能连续发布相同时间戳；观测器先校验字段，再将这种消息作为重复样本跳过，计入 `duplicate_stamps`，不增加有效样本数、不更新时间基准，也不进入积分。真正小于上次时间戳才属于倒退。丢失一段数据后，也不应无条件把很大的 `dt` 送进估计器。

`checked_dt()` 检查进入积分的 `0 < dt ≤ max_dt` 和有限数值；观测入口已先去重，重复消息的零步长不会交给它。下面的故意错误命令应返回非零并解释原因：

```bash
./scripts/ros1_course.sh offline lesson06/drift_demo.py --dt 0
./scripts/ros1_course.sh offline lesson06/drift_demo.py --bias nan
```

仿真 `/clock`、ROS 消息时间、主机墙钟和 SDK 计时不是同一个概念。程序用主机 `time.monotonic()` 控制等待上限，所以仿真暂停也会按时退出；运动积分用测量时间戳。SDK 设备时间的换算与时钟对齐留到第 8 课。

## 6. 有限次数地观察 IMU 与关节

启动 ROS 1 仿真后另开终端：

```bash
./scripts/ros1_course.sh sim lesson06/ros1_observe_state.py --samples 30 --timeout 8
```

程序只订阅 `/trunk_imu` 和 `/aliengo_gazebo/joint_states`；从每路接收指定条数后退出。消息戳使用仿真发布者的时间，超时使用墙钟；两路样本没有同步。四元数、角速度和加速度遵守消息可用性标志，全零协方差表示未知。

## 7. 回到 Guide：估计器信了哪些假设

阅读 [Estimator.cpp](../references/unitree_guide/unitree_guide/src/control/Estimator.cpp) 中利用 IMU 预测、用支撑脚约束修正的过程。计划支撑不自动等于真实接触；脚滑或模型不准会影响估计。

## 8. 排错与验收

| 现象 | 先保留哪条证据、怎样解释 |
|---|---|
| `complete=false`，某路计数为0 | 保存话题列表、控制器列表、输出全文；确认 topic、容器、domain、QoS 和仿真是否暂停 |
| `stamp moved backwards` | 查看是否重置仿真或收到乱序数据；重新开始一段观察，不能取绝对值伪造正步长 |
| `duplicate_stamps` 增大 | 同一时间戳的重复发布已跳过；它们不增加有效计数，持续重复直到期限仍未收满时返回超时 |
| 四元数为0或范数异常 | 查看 covariance 不可用标记、字段顺序和上游接口；不要直接当作单位姿态 |
| 静止角速度异常大 | 本地 launch 曾修正独立 `imu_link` 固定物理链接的数值噪声，见验证页；确认仍使用本地启动器，再保留原始IMU、姿态、时间证据，不能直接加大滤波或改控制增益 |
| 比力接近9.81但积分速度越来越大 | 核查重力与坐标变换、姿态误差、偏置及时间戳；先别把它解释成真实运动 |

不连接 ROS 的边界测试：

```bash
python3 -m unittest discover -s tests -p test_lesson06_07.py -v
```

本课三个问题，先自己回答再展开：

1. 静止水平时 IMU 的 z 比力为 `+9.81 m/s²`，为什么不能直接当作机身向上加速？请同时写出坐标变换与重力项。
2. 残余偏置 `0.02 m/s²` 连续积20秒会产生多大的速度和位置误差？减小 `dt` 为什么不能把它们消掉？
3. 支撑脚打滑、时间戳倒退、用仿真真值替代估计输入，各破坏了哪一个假设？请附本次只读观察的两个 `frame_id` 与最终计数，没运行仿真就明确记录“待补做”。

<details>
<summary>讲评与继续条件</summary>

1. 比力在传感器系表达；已知方向后用 `a_W=R_WS f_S+g_W`。水平静止时两个 z 项相消；姿态四元数本身也可能是融合输出。
2. `Δv=0.4 m/s`，`Δp=4 m`。常值偏置仍在，采得更密只会更准确地积分这个错误。
3. 打滑破坏脚固定假设；倒退破坏正时间推进和状态连续性；真值替代掩盖真机估计问题。能区别这些问题，并解释观测记录中的坐标、时间和来源，就可以进入第7课。

</details>

[上一课：状态机与步态](lesson_05_zh.md) · [下一课：站稳需要多少力](lesson_07_zh.md)
