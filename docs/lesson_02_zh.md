**配对代码：** [ROS 1 只读节点](../examples/lesson02/ros1_joint_state_reader.py) 与 [关节名称处理](../examples/lesson02/joint_state_reader.py)。

# 第 2 课：写一个只读 ROS 1 节点

[上一课：环境、话题与站立](lesson_01_zh.md) · [八课路线](learning_path_zh.md) · [下一课：单关节 PD](lesson_03_zh.md)

这一课把第一课的手工查表，变成一个会自己接收消息的小程序。完成后，你应该能说明：**哪个函数收到消息，如何用名字找角度，为什么终端一秒只打印两行。** 预计 45～60 分钟，可以把 Python 部分和仿真观测分开学。

前置条件是第一课已经完成：认得右前腿 `FR`，知道关节状态话题的用途，会从项目根目录运行脚本。在线部分需要第一课的仿真仍在运行；本课不需要改变机器人姿态。当前教学环境是 ROS 1 Noetic / Gazebo 11，范围见 [仿真说明](ros1_sim_zh.md)。离线数组练习不需要 Gazebo 或 ROS。

## 1. 先把一条消息当作两列表格

`JointState` 中 `name[i]` 与 `position[i]` 是同一个关节。`i` 叫“索引”，Python 从 **0** 开始计数。考虑这条练习消息：

```python
names = ["FR_calf_joint", "FR_hip_joint", "FR_thigh_joint"]
positions = [-1.4, 0.1, 0.7]
```

| 索引 `i` | `names[i]` | `positions[i]` |
|---|---|---:|
| 0 | `FR_calf_joint` | −1.4 rad |
| 1 | `FR_hip_joint` | 0.1 rad |
| 2 | `FR_thigh_joint` | 0.7 rad |

这里 `positions[0]` 是小腿角度。如果下一条消息把髋关节放在第一项，它就变成髋关节角度。**同一条消息按相同索引对应；不同消息不靠记住索引来认关节。** `JointState` 的定义还允许 `position`、`velocity`、`effort` 为空；非空数组应与名字数组等长，所以程序必须检查字段是否存在。[官方消息定义](https://github.com/ros/common_msgs/blob/noetic-devel/sensor_msgs/msg/JointState.msg)

| 本课变量或字段 | 含义 | 单位 |
|---|---|---|
| `name` / 示例中的 `names` | 关节名字列表 | 无 |
| `position` / `positions` / `angle` | 转动关节角度 | rad |
| `velocity` | 转动关节角速度；本课不依赖它存在 | rad/s |
| `effort` | 转动关节力矩字段；本课不解释其测量来源 | N·m |
| `now`、`last_print`、`timeout` | 电脑上的单调墙钟时间或时长 | s |
| `rate` | 最大终端打印频率 | Hz，即每秒次数 |

弧度与角度的换算是 `角度 = 弧度 × 180 / π`。例如 `0.1 rad ≈ 5.73°`；程序统一输出 rad，避免把 `0.7` 当成 `0.7°`。

**先预测：** 上表中，`FR_thigh_joint` 的角度是多少？如果只交换 `names` 的前两项、不交换 `positions`，消息还能正确描述同一组状态吗？先写下答案，再看后面的练习。

## 2. 这一课需要的 Python，只有几个积木

打开完整程序 [joint_state_reader.py](../examples/lesson02/joint_state_reader.py)。暂时跳过 `main()` 里的参数解析，先认这几个写法：

```python
FR_JOINTS = ("FR_hip_joint", "FR_thigh_joint", "FR_calf_joint")
values = {}
values["FR_hip_joint"] = 0.1
angle = values["FR_hip_joint"]
```

第一行的圆括号装着固定的三个名字，称为元组；本课可以把它看成不修改的列表。`{}` 是字典，可以“用名字取值”。第三行把键 `FR_hip_joint` 对应到 `0.1`，第四行取出这个数；这比在脑中记住“髋关节永远是第几项”清楚。

提取函数的关键几行如下，与文件中的写法一致：

```python
for joint in FR_JOINTS:
    indices = [index for index, name in enumerate(names) if name == joint]
    values[joint] = None
    if not indices:
        issues.append(f"缺少关节名 {joint}")
    elif len(indices) > 1:
        issues.append(f"关节名重复 {joint}")
    elif indices[0] >= len(positions):
        issues.append(f"缺少位置 {joint}")
    else:
        angle = float(positions[indices[0]])
```

| 代码 | 用普通话读出来 |
|---|---|
| `for joint in FR_JOINTS:` | 依次拿出三个要找的关节名；冒号后缩进的行属于循环 |
| `enumerate(names)` | 同时拿到每项的索引和名字，例如 `(0, "FR_calf_joint")` |
| `[index ... if name == joint]` | 收集所有“名字相等”的索引；`==` 是比较，`=` 是赋值 |
| `values[joint] = None` | 先标记为不可用；`None` 不是数字 0 |
| `if not indices:` | 没有找到名字，记录原因 |
| `elif len(indices) > 1:` | 同名出现多次，无法确定应该取哪一项 |
| `indices[0] >= len(positions)` | 索引超出了位置数组范围，不能硬读 |
| `float(positions[indices[0]])` | 读取同一索引的角度并转为浮点数 |

程序随后用 `math.isfinite(angle)` 排除 `NaN` 与无穷大，最后 `return values, issues` 把“查到的值”和“问题列表”一起交还给调用者。`return` 表示这次函数执行结束；定义函数 `def ...` 本身不会立即去订阅消息。

完整函数也检查 `name` 与 `position` 的长度。不完整消息会显示可读的部分和 `N/A`，并说明问题；不会把缺失角度填成零，因为“没测到”和“确实为零”含义不同。

## 3. 离线先试数组顺序

在项目根目录运行：

```bash
cd /home/jack/aliengo_learning
./scripts/ros1_course.sh offline lesson02/joint_state_reader.py --help
./scripts/ros1_course.sh offline lesson02/joint_state_reader.py --demo
```

`course.sh offline` 启动短暂独立容器，把当前目录映射成 `/work`；脚本名相对于 `examples/`。`--demo` 使用三条写好的数学数据，不加载 ROS，也不发送任何消息。

**先预测：** 原顺序与重排后的三关节读数应当相同还是不同？第三条消息只有髋关节时，缺的小腿应打印 `0`、旧值，还是不可用？

预期输出的关键部分是：

```text
原顺序: sample=1 received=1 rad | FR_hip_joint=0.1000 FR_thigh_joint=0.7000 FR_calf_joint=-1.4000
重排后: sample=2 received=2 rad | FR_hip_joint=0.1000 FR_thigh_joint=0.7000 FR_calf_joint=-1.4000
缺数据: sample=2 received=3 rad | FR_hip_joint=0.1000 FR_thigh_joint=N/A FR_calf_joint=N/A | ...
```

前两行顺序相同，是因为**输出**统一按髋、大腿、小腿排列，输入顺序可以变化。第三行 `received` 增加而 `sample` 没增加：收到消息不等于获得一条完整样本。离线程序已经结束，不需要按 `Ctrl+C`。

<details>
<summary>参考讲评</summary>

1. 用名字与数组下标对应，重排不会改变对应关系。
2. 输入可以比打印快；本例按墙钟限频打印。
3. 缺失或无消息不等于真实值为零，超时与字段错误需分别报告。

</details>
