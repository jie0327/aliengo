# AlienGo 仿真中的 Super-LIO

本项目使用 [Super-LIO 官方 ROS1 分支](https://github.com/Liansheng-Wang/Super-LIO/tree/ros1)，固定提交 `60b57aaac8dc397f80c56364e7ccb008c300cc29`。官方已经提供真实 MID360 的 [启动文件与配置](https://github.com/Liansheng-Wang/Super-LIO/blob/ros1/src/super_lio/launch/Livox_mid360.launch)。这里接入的是已有 AlienGo + MID360 + Depot 仿真。

## 启动顺序

需要先站稳，再启动 LIO：上游用最初 50 个 IMU 样本初始化重力和陀螺偏置，不能把落地或起身阶段当作静止状态。

终端一启动仿真：

```bash
cd /home/jack/aliengo_learning
./scripts/super_lio_sim.sh
```

终端二启动键盘，按 `2` 后等待站稳：

```bash
cd /home/jack/aliengo_learning
./scripts/ros1_keyboard.sh
```

终端三启动 Super-LIO 与地图查看：

```bash
cd /home/jack/aliengo_learning
./scripts/super_lio_start.sh
```

看到 `Map init done` 后，再回键盘终端按 `4`，轻按两次 `w` 前进。`j/l` 转向，空格停车。关闭前先停车，在终端三按 `Ctrl+C` 等待保存地图，再关闭控制器和仿真。

地图保存于 `.super_lio/maps/日期时间-PID/depot.pcd`；每次启动使用新目录。纯计算可用 `super_lio_sim.sh --headless` 和 `super_lio_start.sh --headless`，它们不打开图形窗口。

`./scripts/super_lio_start.sh` 打开的 RViz 现在同时显示 **D435 彩色图像**（`/camera/color/image_raw`）和 **D435 彩色点云**（`/camera/depth/color/points`，RGB8 着色）。彩色点云通过相机 TF 显示在 `lio_map` 中，与 LIO 地图和轨迹一起查看；需使用默认启用 D435 的仿真。分辨率、相机参数和渲染依赖见 [D435 说明](d435_sim_zh.md)。

图像显示刚启动或重置时，黄色 `No Image received` 表示还在等首帧，收到图像后恢复正常。彩色点云还需要等待其时间戳对应的 LIO 位姿：显示队列设为 10 帧，为估计 TF 的处理延迟留出缓冲。相机时间戳和估计器保持原始时间语义。

## 官方支持与本地适配

官方 `lidar_type=1` 接收 Livox `CustomMsg`，其中包含逐点纳秒偏移。当前仿真插件每次在一个场景快照中计算射线，只发布 XYZ。为保留真实时间语义，本地兼容补丁增加 `SIM_SNAPSHOT=8`：

- 直接接收原 `/livox/lidar` 的 XYZ `PointCloud2`。
- 帧起点、终点均为采样时间，每点偏移为零，不制造逐点运动畸变。
- 保留原有 IMU 传播、滤波、点面匹配与地图更新；瞬时扫描仅进行雷达到 IMU 的外参变换。
- IMU 仍为 m/s²、rad/s，保留重力响应；使用 `gravity_norm=9.81`。
- 雷达时间戳使用射线查询结束处的世界仿真时钟，消除 ROS `/clock` 传播延迟。点云仍是一次快照，线程调度会带来采样时间误差。

仿真没有真实反射强度、逐点采样时间和硬件噪声；本次不验证真实 MID360 的去畸变效果。官方 MID360 接收分支仍保留。原始源码在 `references/Super-LIO`，实际运行的补丁副本在 `.super_lio/src`，改动见 `ros1/patches/super_lio_snapshot.patch`。

## 估计位姿与地图

Super-LIO 只接收点云和 IMU。它的局部估计坐标系命名为 `lio_map`，不直接等同于 Gazebo 出生坐标。

- `/lio/odom`：IMU 估计位姿。
- `/lio/cloud_world`：配准后的当前扫描，坐标系为 `lio_map`。
- `/lio/path`：估计轨迹。
- `lio_map → base`：由估计的 IMU 位姿与 URDF 静态安装外参计算。

此模式关闭此前用于显示的 Gazebo 真值 `world → base`。真值只在验证脚本中做一次初始对齐和误差评估，不参与估计器输入。RViz 按高度显示配准点云并累积最近 30 秒；PCD 文件由算法累积扫描后保存。本次未接入闭环、GBPlanner 或占据栅格转换。

## 构建与验证入口

本机已准备镜像与工作区。重新准备时：

```bash
./scripts/super_lio_build_env.sh
./scripts/super_lio_build.sh
./scripts/ros1_build.sh
```

镜像为 `aliengo-learning:noetic-lio`，保留原来的 Noetic/Gazebo/RViz 环境，增加 PCL、glog、gflags、TBB。编译使用单任务、O2 和 3GiB 上限；算法 TBB 限为两个工作线程。再次构建保留已有 `.super_lio/src` 修改。

完整验证由单个有界容器运行，完成后关闭：

```bash
./scripts/verification/super_lio_trial.sh --seconds 20 --record-bag
./scripts/verification/super_lio_replay.sh logs/super-lio-trial-XXXXXX/sensors.bag
```

第二条只重放传感器和时钟，不启动 Gazebo，验证算法确实能独立产生估计和地图。实测结果、地图文件与适用范围见 [接入验证记录](reports/super-lio.md)。

## 学习入口

按以下顺序阅读实际工作区副本，并与 `references/Super-LIO` 对照本地补丁：

1. `src/super_lio/src/apps/super_lio_node.cpp`：参数、循环、处理入口与退出保存。
2. `src/super_lio/src/ros/ROSWrapper.cpp`：传感器消息、时间同步与输出接口。
3. `src/super_lio/src/lio/super_lio.cpp`：静止初始化、传播、匹配、地图更新。
4. `src/super_lio/src/lio/ESKF.cpp`：误差状态滤波。
5. `src/super_lio/include/OctVoxMap/OctVoxMap.hpp`：体素地图与近邻搜索。
