# Depot 仓库与 MID360 点云

默认世界采用用户指定的 [livox_gz_sim 顶图](https://github.com/mathaimp/livox_gz_sim/blob/main/docs/hap_fastlio.webp)中的 **OpenRobotics Depot** 仓库。来源是 [Fuel Depot](https://app.gazebosim.org/OpenRobotics/fuel/models/Depot) 的固定 version 6；不是该代码仓库自带的 12×8 米 demo 房间。

## 启动

本机资源与 RViz 镜像已准备；从项目根目录运行：

```bash
./scripts/ros1_sim.sh --rviz
```

另开终端运行键盘：

```bash
./scripts/ros1_keyboard.sh
```

按 `2` 并等待站稳，按 `4` 进入小跑，再轻按 `w` 两次低速前进；按空格停车。键盘终端需要获得焦点，松开按键不会自动停车。

RViz 显示 `/livox/lidar`，按高度着色。固定坐标系为 `world`，动态 `world → base` 来自 Gazebo 的实际位姿，所以机器人在 RViz 中也会平移、转向和倾斜。相机只在启动时对准出生点，参考网格固定在世界中；当前显示实时点云，没有累积 SLAM 地图。可以拖动旋转视角、滚轮缩放。图片里的 FAST-LIO 建图是原作者另行运行的算法，本次没有接入。

之前固定在 `base` 时，机身始终位于显示坐标原点，只能看见四肢动作，导致“Gazebo 在走、RViz 原地踏步”。只把 Fixed Frame 改成 world 还不够，必须同时提供真实的动态位姿变换。

修复后已实测 Gazebo 和 TF 同步前进约 0.96 米，位姿与视角验证见 [RViz 运动同步记录](reports/rviz-world-motion.md)。

场景选项：

```bash
./scripts/ros1_sim.sh --world depot --rviz  # 默认仓库
./scripts/ros1_sim.sh --world earth        # 原平地
./scripts/ros1_sim.sh --world city --rviz   # 可选静态城市
./scripts/ros1_sim.sh --headless --rviz     # 只开 RViz，省去 Gazebo 窗口
```

## 从原始资源准备

已有 ROS1 工作区后执行一次：

```bash
python3 scripts/ros1_depot_setup.py
./scripts/ros1_viz_build.sh
```

准备脚本下载约 78 MiB 的固定版本 ZIP、核对 SHA256，再生成本地 Classic 模型。来源与哈希记录在 [depot_source.json](../ros1/worlds/depot_source.json)，原始资产保存在 `references/fuel_models/Depot`，本地派生模型位于 `ros1/worlds/models/Depot`。离线启动使用这些已准备的资源。

可选城市资源用 `python3 scripts/ros1_city_setup.py` 准备，来源固定在 [city_sources.json](../ros1/worlds/city_sources.json)。

## ROS1 适配

项目继续使用 ROS Noetic / Gazebo Classic 11。`livox_gz_sim` 的运行代码使用 ROS2 / Gazebo Harmonic；这里只复用图片中的 Depot 场景资源。

适配保留原模型网格、0.6 的原始网格比例和布局。仓库外墙约 30.6×15.6 米，出生点为 `(-5, 0, 0.6)`，面向 +x 的底层开阔通道。模型改为静态，去除风扇的现代 Gazebo 插件，风扇保留初始姿态；Classic 直接读取 DAE 内嵌材质和 UV，观感会与原图的 PBR 渲染有所差异。

原 Depot 只有地面平面碰撞，墙和箱子只有可视网格。当前 MID360 的 CPU ODE Ray 读取碰撞几何，所以适配从原可视子网格生成墙、柱、屋顶、货箱和完整 FLOOR 的碰撞；FLOOR 同时包含底层地板和约 2 米高的夹层。删除与底层地板重合的原平面，避免重复接触；可承重地面随实际仓库地板范围结束。ODE 步长和控制参数沿用原平地配置。

原模型两盏 `fan_light` 的衰减常数、一次项和二次项均为 0，Classic/OGRE 的倒数光照计算会除以零，造成整个室内过曝发白。适配设置了有限的衰减参数，保留室内灯光及真实贴图。

本机已通过 30 秒低速行走、停车与 RViz 实测；截图、频率、性能和具体范围见 [验证记录](reports/depot-world.md)。

GPU 用于 Gazebo 和 RViz 渲染；ODE 物理与当前 MID360 射线检测仍由 CPU 计算。运行容器设置内存、CPU 和线程数限制。点云字段与传感器近似范围见 [MID360 说明](mid360_sim_zh.md)。

## 仿真位姿与后续 SLAM

[gazebo_pose_tf.py](../ros1/mid360/scripts/gazebo_pose_tf.py) 按仿真时间 30Hz 查询一个模型，使用 Gazebo 应答的时间戳发布 `world → base` 和 `/sim/ground_truth/odom`。里程计 pose 表达在 world，twist 表达在 base；高度使用绝对世界高度，城市中的 5.01m 地面偏移也会保留。仿真暂停时发布一起暂停。

这是明确标注的**仿真真值**，用于显示和对照。以后接入运动估计、SLAM 或 GBPlanner 时，若由其他节点发布到 base 的 TF，应运行 `./scripts/ros1_sim.sh sim_ground_truth_tf:=false` 关闭真值 TF，并将 RViz 固定系选择为该系统提供的 odom/map 等坐标系。当前没有新增 map 坐标系或定位算法。

## 来源及署名

Depot 作者为 **Rafi Abdullah、Louise Poubel**，发布者 OpenRobotics，许可证为 [CC-BY-4.0](https://creativecommons.org/licenses/by/4.0/)。原 `model.config` 作者信息保留。本项目的修改包括静态化、Classic 材质兼容、碰撞几何和世界配置；原资产没有整体缩放或重新设计布局。
