# AlienGo 策略推理预览（实验中）

该入口读取 Gazebo 的 IMU 与关节反馈，调用 UniO4-Aliengo 的预训练策略，并在 `/rl_policy/action_preview` 发布 12 维动作。**它不发布电机或关节控制指令**，也不接入真实机器人。一次隔离仿真中已收到 5 组有限动作并正常退出；这不代表策略可以稳定行走。

## 策略与观测

候选来源是 [UniO4-Aliengo](https://github.com/anubhav1772/UniO4-Aliengo)，固定提交 `00fcfc088811f7629fea3561f16cbfd3294214c7`，使用其中 `pretraining/runs/gait-conditioned-agility/recent/225743.070104` 的两个 JIT 文件。它们来自 AlienGo 项目，但本环境尚未复现其训练。原仓库根目录未找到明确的许可证；文件由准备脚本直接从上游下载，不随项目分发。模型与 PyTorch CPU wheel 下载后均检查固定 SHA256。

这份策略使用 58 维观测和 5 帧历史，网络输入共 290 维：重力方向、15 项速度与步态意图、12 关节相对默认位置、12 关节速度、上一次动作和 4 项步态时钟。关节顺序是 `FL, FR, RL, RR`，每腿为 `hip, thigh, calf`；周期为 0.02 秒。默认预览意图是零速度。`--vx`、`--vy`、`--yaw` 只改变策略观察到的意图，**不会让机器人移动**；它们也不会自动读取键盘控制器的输入。

预览采用策略上一次**建议的**动作作为历史，实际机器人仍由 Guide 控制，所以它不是执行策略后的真实闭环轨迹。旧的 48 维学习示例与该策略的 58 维契约不同，不能互换。

## 准备与运行

当机器资源允许且 ROS 1 仿真可用时，可运行以下命令进行后续验证：

```bash
cd /home/jack/aliengo_learning
python3 scripts/ros1_rl_setup.py
./scripts/ros1_rl_preview.sh --samples 10
```

准备脚本需要联网，下载约 200 MB，解压后本地 `.ros1_rl` 约占 1 GB；提供的 PyTorch wheel 适用于 x86_64、Noetic 容器 Python 3.8。第二条命令需要 `./scripts/ros1_sim.sh` 启动的仿真正在运行，仿真容器仍保持网络隔离。节点会检查模型哈希、反馈是否完整及时、时钟是否更新，并在结束时退出。`--samples 0` 表示持续观察；`Ctrl+C` 可结束预览。

观察消息可在仿真容器内执行 `rostopic echo /rl_policy/action_preview`。前几帧历史以零填充；输出是建议动作而不是已执行动作。2026-11-12 在一个限内存 4 GB、4 核的隔离无界面仿真容器中，通过 `./scripts/ros1_rl_preview.sh --samples 5 --timeout 30` 收到 5 组有限动作，程序退出码为 0。这是一次短时只读测试，不是长期实时性或闭环稳定测试。

## 物理控制状态

一次隔离仿真实验从 Guide 站姿切换到此策略后，零速度输入时约 1.6 秒机身高度由 0.419 m 降至 0.230 m，实验按阈值停止。上游训练采用 actuator network，而当前仿真用即时 PD；模型质量、初始姿态和步态时序也有差异。原因尚未确定，策略闭环、持续行走和真机均未验收。
