# ROS 1 AlienGo 仿真与键盘操作

这是 Gazebo 11 中的 AlienGo 仿真，使用 ROS Noetic、宇树 `unitree_ros` 的 AlienGo 模型和 `unitree_guide` 控制器。官方 Guide 原本只提供 A1/Go1 控制参数；本环境在工作区副本上应用 [AlienGo 几何和动力学近似补丁](../ros1/aliengo-guide.patch)，沿用 Go1 控制增益。它是本地适配实验，不是宇树官方提供的原生 AlienGo 行走控制器，也不连接真机。

## 首次构建

本项目镜像名为 `aliengo-learning:noetic`，由 [Dockerfile.ros1](../docker/Dockerfile.ros1) 从 `ros:noetic-ros-base-focal` 和 ROS Noetic final 软件源构建。此次验证的基础镜像 ID 是 `sha256:72b8bc59035dc0a5b8e07aae28c16caa84192971d72d207c72ed734fb1d5e97d`；Ubuntu 软件包未逐个固定版本，重新构建需要软件源可用，不保证逐字节相同。Noetic 和 Gazebo Classic 已停止官方维护。

在项目根目录执行：

```bash
cd /home/jack/aliengo_learning
./scripts/ros1_build_env.sh
./scripts/ros1_build.sh
python3 scripts/ros1_depot_setup.py
```

脚本从 `references/unitree_ros` 和 `references/unitree_guide` 复制源码到独立的 `.ros1` 目录，仅在副本中打补丁并在无网络容器里编译。再次构建会保留 `.ros1/src` 中已有内容；如果想重新应用更新后的补丁，应先检查并备份该目录。原始参考源码不被修改。

## 启动与操作

打开一个终端，运行图形仿真：

```bash
cd /home/jack/aliengo_learning
./scripts/ros1_sim.sh
```

等 Gazebo 出现 AlienGo 后，另开一个终端运行键盘控制器：

```bash
cd /home/jack/aliengo_learning
./scripts/ros1_keyboard.sh
```

键盘窗口应保持焦点，切换到英文输入法，不用按回车：

1. 按一次 `2`，等待固定站立完成。
2. 按一次 `4`，进入运动模式，先观察站稳。
3. 轻按 `W` 两次前进；`S` 后退，`A/D` 左右平移，`J/L` 左右转向。
4. 按空格清零方向输入。输入会累加，松开按键不会自动清零；不要长按。

键盘终端退出前先按空格，再用 `Ctrl+C` 退出控制器；在仿真终端按 `Ctrl+C` 关闭仿真。控制器退出后不会继续维持站立。无图形界面可运行 `./scripts/ros1_sim.sh --headless`。容器名是 `aliengo-ros1-sim`，仅使用容器内网络，与真实机器人隔离。

当前图形入口默认接入本机 NVIDIA GPU；MID-360 的安装支架、点云和图形设置见 [MID-360 仿真说明](mid360_sim_zh.md)。本地 Guide 已通过补丁将仿真控制节拍和步态相位改为 ROS `/clock`，控制器会跟随物理时间变慢、暂停和恢复。

机身下排 D435 默认输出 RGB、深度和彩色点云，`--rviz` 已提供相应显示，见 [D435 说明](d435_sim_zh.md)。相机在 `--headless` 下仍需渲染；只运行雷达可使用 `d435_enabled:=false`。

当前默认世界是 [Depot 仓库](depot_world_zh.md)。加 `--rviz` 可同时打开点云查看器；用 `--world earth` 切回原平地，用 `--world city` 选择此前准备的静态城市。世界切换不改变键盘按键。

## 实测范围

2026-11-12 在本机旧实验镜像的独立无界面仿真中，实际向伪终端发送按键：两次 `W` 前进约 60 秒仿真时间，位移约 2.9 米；再增加两次 `W` 前进约 30 秒，位移约 2.9 米；空格后 5 秒漂移约 0.004 米。另测了侧移及转向。测试记录与采样见 [ROS 1 仿真记录](reports/ros1-2026-11-12/check.log) 和 [采样数据](reports/ros1-2026-11-12/samples.json.gz)。

同一天使用新构建的 `aliengo-learning:noetic` 镜像，从零编译并在无界面实际按键测试：两次 `W` 前进 20.49 秒，最大绝对横滚约 0.0075 rad；停车与转向均通过，见[测试记录](reports/ros1-2026-11-12/new-image-check.log)。正式 GUI 入口中另一次连续前进采样约 9.1 秒仿真时间，x 增加约 0.47 米、最大绝对横滚约 0.0074 rad。界面启动时出现 Intel 图形驱动回退警告；这次仿真时间约为墙钟时间的 0.60 倍。这些历史测试使用旧版墙钟控制；当前仿真分支已改为 ROS 仿真时间，避免负载变化引起步态与物理时间失配。以上只是本机平地有限测试，不保证其他负载、速度、地形或真机上的稳定性。

状态观测可在另一个终端中执行，例如：

```bash
docker exec aliengo-ros1-sim bash -c 'source /opt/ros/noetic/setup.bash; source /work/devel/setup.bash; rostopic echo -n 1 /aliengo_gazebo/joint_states'
```

机身 IMU 为 `/trunk_imu`，关节状态为 `/aliengo_gazebo/joint_states`；项目目录在容器中以 `/course` 只读挂载，控制源码和构建产物位于 `/work`。
