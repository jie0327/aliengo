**配对代码：** [urdf_fk.py](../examples/lesson04/urdf_fk.py)、[planar_ik.py](../examples/lesson04/planar_ik.py) → [AlienGo leg.xacro](../references/unitree_ros/robots/aliengo_description/xacro/leg.xacro) 与 [unitreeLeg.cpp](../references/unitree_guide/unitree_guide/src/common/unitreeLeg.cpp)。

# 第 4 课：从关节角到脚的位置——运动学

[上一课：一个关节怎样追到目标](lesson_03_zh.md) · [学习路线](learning_path_zh.md) · [下一课：状态机与步态](lesson_05_zh.md)

上一课的 PD 回答“关节怎样追到目标角度”。这一课回答更靠前的问题：“这些角度对应脚在哪里？”给定角度算位置叫正运动学（FK）；给定位置寻找角度叫逆运动学（IK）。建议分两次各 40 分钟：先完成真实 URDF 的 FK，再完成二维教学 IK。

**前置：** 会按关节名提取弧度数值，理解位置误差；能运行第 2、3 课的离线例子。需要的 Python 新知识只有字典、函数、循环与 `math.sin/cos`。本课例子均只计算数字，不订阅、不发布，也不连接 SDK。

**完成后你应能：** 从 URDF 找到一条腿的坐标变换链；用手算参照判断 FK 的方向和单位；解释 IK 的多解、不可达、奇异与限位。真实 AlienGo 使用三维 URDF FK，逆解练习另外使用有明确约定的二维 2R 模型；本课不提供完整 AlienGo 三维逆解器。

## 1. 先把“在哪里”说完整

写“脚的位置是 `(0.24, -0.14, -0.35)`”仍缺三个说明：相对谁、轴朝哪里、单位是什么。本课真实 FK 固定使用：

| 项目 | 本课约定 |
|---|---|
| 参考系 | `trunk`，机身连杆坐标系，随机器人运动 |
| 轴方向 | `+x` 朝前，`+y` 朝机器人左侧，`+z` 朝上 |
| 位置 | 米 `m`，计算 `FR_foot` 等足端连杆原点 |
| 关节角 | 弧度 `rad`；正向按关节 `axis` 的右手规则 |
| 角度顺序 | `hip thigh calf`，共三个角度 |

坐标方向与 SI 单位遵循 [ROS REP-103](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst)。本课还会用当前模型的 `axis` 和 `origin` 核对具体实现，不能仅凭 ROS 的通用约定猜某个模型。

把右手拇指指向 `+y`，四指弯曲方向是大腿关节的正向。处于竖直向下的腿绕 `+y` 转正角，会先向 **负 x** 方向偏。这是常见的直觉误区：画面里“顺时针”会随相机位置改变，关节轴不会。

FK 输出的是脚的连杆原点，不是 Gazebo 世界中的坐标，也不是球形脚底的接触点。当前脚的碰撞球半径是 `0.0265 m`；不能把足端原点的高度直接当成离地间隙。若想得到世界坐标，还要乘上世界到机身的姿态与平移。

## 2. 从真实模型读一条腿

所有命令在项目根目录执行：

```bash
cd /home/jack/aliengo_learning
./scripts/ros1_course.sh offline lesson04/urdf_fk.py --leg FR --describe
```

程序读取预览阶段生成的 `.ros1/aliengo.urdf`。若缺失，它会明确提示：

```bash
./scripts/ros1_model.sh
```

该命令在 ROS 1 容器中展开官方 AlienGo 模型，不需要启动仿真；先完成 `./scripts/ros1_build.sh`。

**先预测：** 从 `trunk` 到 `FR_foot`，需要经过三个还是四个关节？先写答案，再看 `--describe`。

右前腿的真实链为：

```text
trunk
  └─ FR_hip_joint    转动：轴 (1,0,0)，原点 (0.2407,-0.051,0)
       FR_hip
         └─ FR_thigh_joint  转动：轴 (0,1,0)，原点 (0,-0.0868,0)
              FR_thigh
                └─ FR_calf_joint  转动：轴 (0,1,0)，原点 (0,0,-0.25)
                     FR_calf
                       └─ FR_foot_fixed 固定：原点 (0,0,-0.25)
                            FR_foot
```

这里有三个驱动关节，但有四个坐标变换。漏掉最后一个固定关节，就会把小腿关节位置当成脚的位置。`hip_rotor` 等转子是旁支，不在这条通向脚的链上。读 `joint/origin`，不要拿网格的 `visual/origin` 或质心的 `inertial/origin` 代替关节间距。

长度、偏置和限位来自宇树官方 AlienGo 模型的 [leg.xacro](../references/unitree_ros/robots/aliengo_description/xacro/leg.xacro) 和 [const.xacro](../references/unitree_ros/robots/aliengo_description/xacro/const.xacro)。

| 腿 | hip 相对 trunk 的原点 x / y（m） | thigh 侧向偏置（m） |
|---|---|---|
| FR | `+0.2407 / -0.051` | `-0.0868` |
| FL | `+0.2407 / +0.051` | `+0.0868` |
| RR | `-0.2407 / -0.051` | `-0.0868` |
| RL | `-0.2407 / +0.051` | `+0.0868` |

四条腿的大腿、小腿长度均为 `0.25 m`，三个关节轴均依次为 `+x、+y、+y`。ROS 1 模型将 thigh 声明为 continuous，尽管 const.xacro 中另有角度常量，展开的关节不含位置上下限。左右镜像的偏置不同，**关节轴本身在这个模型中没有简单地全部取反**。

| 关节 | URDF 角度范围（rad） | 换成度，帮助理解 |
|---|---|---|
| hip | `[-1.221730, +1.221730]` | `[-70°, +70°]` |
| thigh | `continuous`，展开后的 URDF 没有位置上下限 | 不能据此推断实机可以无限旋转 |
| calf | `[-2.775074, -0.645772]` | `[-159°, -37°]` |

程序中的三角函数吃弧度，例如 `90° = π/2 rad`，不能直接输入 `90`。URDF 限位是模型声明，不能替代实物的机械、固件和控制模式限位。

## 3. 手算一个能抓出错误的参照

暂时令三个角全为零，只是为了检查几何相加：

```text
FR: x = 0.2407
    y = -0.051 - 0.0868 = -0.1378
    z = -0.25 - 0.25  = -0.50 m
```

**这个零角姿态的小腿超出了 URDF 限位。** 因此程序默认拒绝它；只有明确要求数学参照时才关闭限位检查：

```bash
./scripts/ros1_course.sh offline lesson04/urdf_fk.py --all-legs --angles 0 0 0 --allow-outside-limits
```

预期得到四个点：

| FR | FL | RR | RL |
|---|---|---|---|
| `(0.2407,-0.1378,-0.5)` | `(0.2407,0.1378,-0.5)` | `(-0.2407,-0.1378,-0.5)` | `(-0.2407,0.1378,-0.5)` |

接着用一个在模型限位内、可手算的弯腿角：

```bash
./scripts/ros1_course.sh offline lesson04/urdf_fk.py --leg FR --angles 0 1.5707963267948966 -1.5707963267948966
```

大腿从向下转到负 x 方向，小腿的总转角又回到向下；预期 FR 足端为 `(-0.0093, -0.1378, -0.25) m`。这能检查关节转轴的符号；只有零角测试无法发现所有转轴符号错误。

现在先预测下面这组角的 x 是否会移动，再执行：

```bash
./scripts/ros1_course.sh offline lesson04/urdf_fk.py --all-legs --angles 0 0.8 -1.6
```

大腿角为 `+0.8`，小腿相对大腿再转 `-1.6`，所以两段相对机身的前后倾角为 `+0.8` 与 `-0.8`。前后偏移相消，FR 结果约为 `(0.2407,-0.1378,-0.348353) m`。

**再改变一个条件：** 把 hip 改为 `0.1`，其他两个角不动。记录四腿的 y、z 改变方向。相同 hip 正角会将四条腿都绕 `+x` 旋转，不会自动生成左右镜像的动作；若希望左右镜像，还要结合偏置与 hip 符号推导。

## 4. 从几何相加走到变换矩阵

二维旋转不能解释侧摆；三维里还需要知道后续位移是在谁的局部轴上写的。用齐次矩阵把“旋转 + 平移”放在一起：

```text
T = [ R  p ]       p_root = T_root_tip × [x_tip,y_tip,z_tip,1]^T
    [ 0  1 ]
```

脚原点在自己的坐标系是 `(0,0,0)`，所以最终位置就是 `T_root_tip` 最后一列的前三个数。旋转矩阵 `R` 仍有用，它说明脚的坐标轴相对机身如何转动。

对每个 URDF 转动关节，本例按下面顺序累乘：

```python
transform = multiply(transform, joint["origin"])
transform = multiply(transform, rotation(joint["axis"], angle))
```

第一行先到达关节安装位置和零角姿态，第二行再绕该关节的局部轴转动。`origin` 的 RPY 按 `Rz(yaw) Ry(pitch) Rx(roll)` 生成。矩阵乘法不交换：先移动 0.25 m 再转身，与先转身再沿新方向移动 0.25 m，终点不同。

打开 [urdf_fk.py](../examples/lesson04/urdf_fk.py)，按这个顺序读：

1. `UrdfModel.__init__`：XML 解析得到字典，以子连杆名索引关节。`float(...)` 将 XML 中的字符串转成可计算数值。
2. `chain(root, tip)`：从脚沿 `parent` 反向找到 `trunk`，再倒序得到正向链。它只选择这条路径，不把所有 XML 关节一起乘上。
3. `forward(...)`：检查每个活动关节的角度是否明确给出、有限、在模型限位内，然后组合变换。固定关节只乘安装变换。
4. `foot_position(...)`：将三个角与三个关节名配对，提取最终平移。

为了让你能用其他库独立核对，`forward` 返回普通 Python 的 4×4 列表。其接口为：

```python
model = UrdfModel(".ros1/aliengo.urdf")
T = model.forward("trunk", "FR_foot", {
    "FR_hip_joint": 0.0,
    "FR_thigh_joint": 0.8,
    "FR_calf_joint": -1.6,
})
```

这里的代码块用于理解接口；日常执行仍使用上面的 `course.sh offline` 命令。这个小解析器只处理本课所需的 fixed/revolute/continuous 关节，不是通用 URDF 计算库，遇到不支持的类型会报错。Guide 的 [unitreeLeg.cpp](../references/unitree_guide/unitree_guide/src/common/unitreeLeg.cpp) 处理单腿运动学、雅可比和力矩映射；本课解析器是独立教学代码。

## 5. 把 IK 缩成能画在纸上的二维问题

先画两根长度均为 `0.25 m` 的棍子，中间用铰链连接。**以下是另一个教学模型：** 原点在第一个关节；纸面 x 向右、y 向上；`q1=0` 时第一根棍子指向 `+x`；两个角都以逆时针为正，`q2` 相对第一根棍子。它没有 hip 侧摆、没有左右偏置；这里的角度不能直接填到 AlienGo 的三个电机里。

```text
x →，+y ↑
第一段方向 q1，第二段方向 q1+q2
x 分量相加：x = l1 cos(q1) + l2 cos(q1+q2)
+y 分量相加：y = l1 sin(q1) + l2 sin(q1+q2)
```

给定 `(x,y)` 后，把两段和目标连线构成的三角形代入余弦定理：

```text
c2 = (x²+y²-l1²-l2²)/(2l1l2)
q2 = ± acos(c2)
q1 = atan2(y,x) - atan2(l2 sin(q2), l1+l2 cos(q2))
```

`atan2(y,x)` 会考虑所在象限，不等同于只计算 `atan(y/x)`。`±` 是两种肘部弯曲方向，两个角都不一定相同，但脚可以在同一个位置。

先手算目标 `(0.25,0.25)`：一根向右、一根向上能到达；反过来先向上、再向右也能到达。预测两组角，再运行：

```bash
./scripts/ros1_course.sh offline lesson04/planar_ik.py --target 0.25 0.25
./scripts/ros1_course.sh offline lesson04/planar_ik.py --angles 0.6 -1.2
```

第一条预期输出约 `(0, π/2)` 与 `(π/2, -π/2)`；第二条先 FK，再对位置求两支 IK，再做 FK。`error_m` 应接近浮点舍入量，通常小于 `1e-12 m`。**往返验收比较位置，不要求最终角度必须等于起始角度**：因为另一支解也可能合法。

打开 [planar_ik.py](../examples/lesson04/planar_ik.py)：`planar_fk` 是上面的两行分量相加；`planar_ik` 先检查可达性，再枚举正负 `q2`，最后筛选限位。检查失败时返回非零退出状态并说明原因，不把一个错误目标悄悄改成其他位置。

## 6. 四个需要亲自试的边界

每次先写一句预测，然后只执行对应命令。

| 情形与命令 | 应观察到什么 | 原因与排错 |
|---|---|---|
| `./scripts/ros1_course.sh offline lesson04/planar_ik.py --target 0.51 0` | `unreachable`，退出码非零 | 最长只能伸到 `0.5 m`；先检查米与毫米 |
| `./scripts/ros1_course.sh offline lesson04/planar_ik.py --lengths 0.3 0.2 --target 0.05 0` | `unreachable` | 可达区域还有内孔：半径至少 `abs(l1-l2)=0.1 m` |
| `./scripts/ros1_course.sh offline lesson04/planar_ik.py --target 0.5 0` | 两支合并成一支，雅可比行列式为零 | 完全伸直是奇异构型 |
| `./scripts/ros1_course.sh offline lesson04/planar_ik.py --target 0 0` | `singular` | 等长两段完全折回时，第一关节方向不由位置唯一决定 |

再做限位筛选：

```bash
./scripts/ros1_course.sh offline lesson04/planar_ik.py --target 0.25 0.25 --limits -2 2 -2 -0.5
./scripts/ros1_course.sh offline lesson04/planar_ik.py --target 0.25 0.25 --limits -2 2 -0.4 0.4
```

第一条只保留负 `q2` 那支；第二条两支都不满足第二关节范围，报告限位失败。这些范围只是二维教学范围，与 AlienGo 的硬件限位无关。程序使用主值角度，不枚举加减 `2π` 后的所有表示。

不能先把逆解角度截到限位边界，再宣称脚仍然到达原目标。截角改变 FK 结果，必须重新检查位置误差。

为什么伸直附近敏感？二维雅可比的行列式为 `l1*l2*sin(q2)`，单位 `m²`。`q2→0` 时趋近零，某些方向的末端小位移需要明显的角度变化。例如目标从 `(0.500,0)` 缩回 `(0.499,0)`，只变化 1 mm，`|q2|` 已约为 `0.127 rad`。运行下面两条比较，而不要把“求出一组数字”当作“能平稳跟踪”：

```bash
./scripts/ros1_course.sh offline lesson04/planar_ik.py --target 0.499 0
./scripts/ros1_course.sh offline lesson04/planar_ik.py --target 0.5 0
```

## 7. 与仿真反馈对照：仍然只读

启动仿真后用 `./scripts/ros1_course.sh sim lesson02/ros1_joint_state_reader.py --samples 3` 观察右前腿三个弧度角，再通过 `./scripts/ros1_course.sh offline lesson04/urdf_fk.py --leg FR --angles HIP THIGH CALF` 计算；把 HIP、THIGH、CALF 换成实际数值。输出相对 `trunk`，并非世界坐标或球形脚底接触点。

## 8. 三个验收问题

1. `FR_foot` 的 `(0.2407,-0.1378,-0.348353)` 为什么不能直接当成 Gazebo 世界中的落脚点？请说出坐标系、单位与所计算的几何点。
2. 二维 IK 为同一个目标返回两支解，其中一支超限位，另一支接近奇异点。你会按什么顺序检查，为什么不能直接截角后使用？
3. 在本模型中，FR 和 FL 复制相同的 hip 正角是否会保持左右镜像？结合 `axis` 与 `thigh` 偏置解释；再说明二维 2R 解为什么不能直接作为 AlienGo 三维关节目标。

<details>
<summary>参考讲评：先完成预测、计算和自己的回答再展开</summary>

1. 这是以 `trunk` 为参考、米为单位的足端连杆原点坐标；需要世界到 `trunk` 的变换才能转换成世界点，且连杆原点不是碰撞球底部。
2. 先核对坐标系、单位、有限值和可达性，再检查解分支、关节限位、FK 重建误差、奇异程度与运动连续性。后续还要检查碰撞、速度、加速度、力矩和模式。截角会改变足端位置，必须重新做 FK，不能维持原来的到达结论。
3. 四腿 hip 都绕 `+x` 转动，左右的侧向安装偏置取反；相同正 hip 角会让竖直腿段在 y 上产生同向分量，通常破坏镜像。二维例子还有不同的零角、正向约定，且没有 hip 自由度和三维偏置；它只用来理解逆解现象。

</details>

记录 `预测 → FK 输出 → 手算差值 → 解释`。下一课会用相位安排四条腿何时支撑、何时抬起；一串足端点还要经过状态机、估计与控制，才会变成可执行的运动。

[上一课](lesson_03_zh.md) · [学习路线](learning_path_zh.md) · [下一课](lesson_05_zh.md)
