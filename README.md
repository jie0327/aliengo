# AlienGo 运动控制与强化学习：ROS 1 学习工作区

这套学习环境以照片中的 **AlienGo** 为目标，主线为 Docker 中的 ROS 1 Noetic、Gazebo 11 和基于宇树 `unitree_guide` 的 AlienGo 参数适配。目标是先学会观察与键盘控制仿真机器人，再学习运动控制、强化学习训练和部署接口。

## 启动 ROS 1 仿真

本机需要 Docker。首次使用先构建镜像和独立工作区：

```bash
cd /home/jack/aliengo_learning
./scripts/ros1_build_env.sh
./scripts/ros1_build.sh
python3 scripts/ros1_depot_setup.py
```

一个终端启动仿真，另一个终端启动键盘：

```bash
./scripts/ros1_sim.sh
```

```bash
./scripts/ros1_keyboard.sh
```

键盘终端获得焦点后，按 `2` 站立并等待站稳，按 `4` 进入小跑；轻按 `w` 两次给出低速前进意图，按空格停止。按键说明、运行方式、实际测试覆盖和限制见 [ROS 1 仿真说明](docs/ros1_sim_zh.md)。这些是仿真命令，不连接真机。Guide 直接读取控制终端的键盘。

镜像构建来源和验证范围见 [ROS 1 仿真说明](docs/ros1_sim_zh.md)。

默认世界为用户指定的 **Depot 仓库**，来自 `livox_gz_sim` 顶图使用的 OpenRobotics 场景。执行 `./scripts/ros1_sim.sh --rviz` 同时查看仓库和 MID-360 点云；首次准备 RViz 镜像用 `./scripts/ros1_viz_build.sh`。场景来源、适配和切换命令见 [Depot 与点云查看](docs/depot_world_zh.md)。

## MID-360 激光雷达

AlienGo 仿真使用 Livox 官方 MID-360 扫描表及近似外形，输出 `/livox/lidar` 点云和 `/livox/imu`；安装位姿需要按实物测量调整，仿真仍不能完全复现真实传感器。真机使用独立的 Livox 官方驱动工作区，并需要配置雷达和网卡的实际地址。参见 [仿真使用说明](docs/mid360_sim_zh.md)和[真机驱动说明](docs/mid360_zh.md)。

本机图形仿真默认使用 RTX 5060 Laptop GPU 渲染，并包含 MID-360 安装支架；本地 Guide 的仿真控制和步态统一跟随 `/clock`。ODE 物理与当前 Livox 射线插件仍使用 CPU。

GPU 接入、行走失稳对照及连续 60 秒验证见 [修复记录](docs/reports/mid360-walking-fix.md)。

## D435 彩色图像与点云

保留机身已有的相机外观，增加独立 RGB、深度和左右红外仿真。运行 `./scripts/ros1_build.sh` 构建相机插件，再用 `./scripts/ros1_sim.sh --rviz` 查看 RGB 与彩色点云；默认 RGB 为 1280×720、深度为 848×480、目标频率 30 Hz。采用固定版本社区 RealSense 插件和本地颜色投影修正，外参与成像参数仍需按实机标定调整。话题、参数和仿真范围见 [D435 使用说明](docs/d435_sim_zh.md)。

## Super-LIO 估计与点云地图

已接入官方 ROS1 Super-LIO，并适配当前 MID360 仿真的瞬时 XYZ 点云。先运行 `./scripts/super_lio_sim.sh`，再用键盘 `2` 站稳，然后运行 `./scripts/super_lio_start.sh`，看到 `Map init done` 后再进入运动。估计位姿、地图与传感器独立回放已验证；启动顺序、官方与本地改动及地图文件见 [Super-LIO说明](docs/super_lio_zh.md) 和 [验证记录](docs/reports/super-lio.md)。

## FAST-LIVO2 三传感器融合（独立入口）

FAST-LIVO2 使用独立工作区和镜像：运行 `./scripts/fast_livo_sim.sh`，用原有 `ros1_keyboard.sh` 按 `2` 站稳，再运行 `./scripts/fast_livo_start.sh` 启动融合与 RViz。原有仿真、Super-LIO 和 GBPlanner 的启动方式保持不变。构建、地图输出和验证方法见 [FAST-LIVO2 说明](docs/fast_livo_zh.md)。

## GBPlanner 低速自主探索

当前提供 GBPlanner3、共享 Voxblox 与实测地面图的探索入口，保留水平 MID-360 和现有下排 D435 安装配置。先执行 `./scripts/ros1_build.sh` 与 `./scripts/gbplanner_build.sh`；规划器构建要求本机已有固定 ID 的缓存依赖镜像，具体来源与限制见 [GBPlanner 操作说明](docs/gbplanner_zh.md)。

依次运行 `gbplanner_sim.sh`、`ros1_keyboard.sh`，按 `2` 站稳后运行 `gbplanner_start.sh`。等待 `Map init done` 且融合已运行，再按 `4`、轻按两次 `w` 低速前进约 1.5–2m 采图；空格停止手动速度、按 `2` 站稳，最后运行 `gbplanner_enable.sh`。停止自主探索用 `gbplanner_stop.sh` 或键盘 `2`/`1`；空格只停止手动速度。当前仅修补官方碰撞检查的朝向、完整体积、端点和输出拒绝问题；沿用原机器人尺寸、跟随参数、起点清理与地图重置，不增加独立安全校验服务。探索默认使用较低 D435 分辨率，传感器安装位姿保持不变，相机实机外参仍待标定。

RViz 包含 RGB、彩色点云、历史实测地面和当前执行路径；地面脚印状态仅供诊断。启动与地图诊断见 [操作说明](docs/gbplanner_zh.md)。当前修复见 [转弯修复记录](docs/reports/gbplanner-turn-clearance.md)，早期版本验收见 [验证记录](docs/reports/gbplanner.md)。当前不声明楼梯、完整仓库覆盖或实机部署通过。

## AlienGo 实机部署

已准备独立实机入口、用户提供的原厂 v3.8.4 高层 SDK 桥接、真实 MID-360 数据适配和机载 D435 的 ROS 网络接入脚本。笔记本网线连接机器人的操作顺序见 [实机部署速记](docs/gbplanner_real_zh.md)。机载相机驱动、外参和电机固件仍须现场确认；软件回环通过不代表实机运动已验收。

## VS Code 阅读 C/C++ 源码

用 VS Code 打开整个 `aliengo_learning` 目录。完成 ROS 1 工作区构建后，执行一次：

```bash
./scripts/ros1_editor_setup.sh
```

脚本从 Noetic 镜像导出 ROS/Gazebo 头文件到 `.ros1/editor/sysroot`，并把真实编译参数转换成宿主机可用的 `.ros1/editor/compile_commands.json`。配置覆盖 `.ros1/src`、对应的 `references` 副本和独立 SDK 示例，各目标保留自己的编译标准和宏；宿主机无需安装 ROS 1。普通构建会更新编辑器编译数据库，重建镜像后重新执行上述脚本更新头文件。

项目已配置 Microsoft C/C++ 扩展的头文件索引和缓存上限。若编辑器仍显示旧的缺失头文件标记，执行命令面板中的 **Developer: Reload Window（开发人员：重新加载窗口）**。这套配置保留代码诊断。

## 课程与强化学习

从 [课程目录](docs/course_index_zh.md) 查看迁移状态。第 9～12 课保留强化学习学习路线：

1. [强化学习与速度跟踪](docs/lesson_09_zh.md)：CPU 表格学习实验。
2. [PPO、GAE 与策略梯度](docs/lesson_10_zh.md)：数值计算和算法概念。
3. [AlienGo 训练任务](docs/lesson_11_zh.md)：观测、动作、奖励和固定训练源码。
4. [策略部署接口](docs/lesson_12_zh.md)：关节顺序、观测/动作契约和离线异常检查。

这几课的离线实验无需启动 Gazebo、GPU 或连接机器人。示例在无网络的 ROS 1 镜像中运行：

```bash
./scripts/ros1_course.sh offline lesson09/q_learning_speed.py
./scripts/ros1_course.sh offline lesson10/ppo_numbers.py
./scripts/ros1_course.sh offline lesson12/offline_policy_contract.py --demo
```

第 9、10 课提供速度跟踪与公式实验；第 10 课还新增 [完整 PyTorch PPO 训练器](docs/lesson_10_zh.md#9-运行完整-ppo-训练器)，支持 CPU/GPU、独立评估、保存权重和断点续训：

```bash
./scripts/lesson10_ppo.sh --device cuda --output results/lesson10_ppo --updates 80
```

运行前按第 10 课准备独立 `.venv-ppo`。生成的是小型连续速度任务的策略。第 12 课离线练习使用明确给出的假动作；`legged_gym` 的 AlienGo 训练参考需要另行准备与其版本相容的训练环境。现有 [真实策略只读推理预览](docs/rl_policy_preview_zh.md) 已收到仿真反馈并输出 12 维动作；它不控制关节。策略行走闭环尚未稳定，完整四足训练和真机控制尚未完成。详见 [训练资料](docs/rl_sources_zh.md)、[训练环境准备](docs/rl_environment_zh.md)和 [强化学习历史验证记录](docs/rl_verification.md)。

还可以在独立的 MuJoCo 环境里学习和比较 PPO、A2C、SAC。先按 [训练实验说明](training/aliengo_mujoco/README.md) 准备环境，再依次运行：

```bash
./scripts/rl_train.sh ppo my-ppo 4096
./scripts/rl_train.sh a2c my-a2c 4096
./scripts/rl_train.sh sac my-sac 4096
```

结果保存在 `.ppo-work/`。实验使用 CPU，得到的是 MuJoCo 策略文件；它们尚未接入 Gazebo 关节控制，也不能直接用于真机。

## 源码与边界

- `references/unitree_ros`：宇树官方 ROS 1 AlienGo 描述、Gazebo 和关节控制源码。
- `references/unitree_guide`：宇树官方 Guide 教学控制器；上游原始代码只列 Go1/A1，当前仿真需要项目内的 AlienGo 适配，详见仿真说明。
- `references/legubiao_legged_gym` 与 `references/rsl_rl_v1.0.2`：固定版本的 AlienGo 训练任务和 PPO 源码参考；训练依赖与 ROS 1 仿真分开。
- `references/unitree_legged_sdk`：旧版 SDK 参考；真实设备的固件、模式、接口和关节映射需另行核实。

固定源码版本见 [repositories.lock.json](repositories.lock.json)。仿真键盘行走、策略训练、Sim2Sim 和真实设备控制分别验证，不能相互替代。
