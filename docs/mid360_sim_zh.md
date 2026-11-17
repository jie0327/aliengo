# AlienGo MID-360 仿真

仿真使用 [Livox 官方仿真插件与 MID-360 扫描表](https://github.com/Livox-SDK/livox_laser_simulation/tree/1cce1073633a062b92e30243a4c2920e45551bb5)，在 Gazebo 中对非重复扫描方向进行射线检测。ROS 1 Noetic / Gazebo 11 的适配、坐标系及无命中处理通过项目补丁完成；固定版本和文件校验值见 `references/livox_laser_simulation/README.aliengo.md`。外形使用带许可的第三方近似模型，不是官方 CAD。

首次使用或升级旧工作区后先运行 `./scripts/ros1_build.sh`，然后启动：

```bash
./scripts/ros1_sim.sh --headless
```

当前入口也默认启用机身下排 D435；无窗口时仍需 X11/GPU 或虚拟显示来渲染相机。仅运行雷达可加 `d435_enabled:=false`，相机话题和显示依赖见 [D435 说明](d435_sim_zh.md)。

需要图形界面时运行 `./scripts/ros1_sim.sh`。当前这台 Linux/NVIDIA 电脑默认使用 RTX 5060 Laptop GPU 渲染：启动脚本只读挂载宿主 NVIDIA 图形库及设备，通过 [PRIME render offload](https://download.nvidia.com/XFree86/Linux-x86_64/595.58.03/README/primerenderoffload.html) 接入现有 X11 显示。无需在 Noetic 镜像里安装另一套显卡驱动；本机没有 NVIDIA Container Toolkit，因此这里采用项目内的显式设备和库挂载。`--software-rendering` 可显式选择软件渲染作对照，软件渲染限制 Mesa 工作线程数为 2。GPU 加速的是 Gazebo 的画面渲染，当前 ODE 物理和 Livox 射线检测仍在 CPU 上。

本机连续 60 秒键盘行走、停车、暂停退出及修复前后的实时因子对照见 [验证记录](reports/mid360-walking-fix.md)。

默认场景现为 Depot 仓库。查看点云可运行 `./scripts/ros1_sim.sh --rviz`，首次需执行 `./scripts/ros1_viz_build.sh` 准备查看器镜像。具体资源准备和场景说明见 [Depot 与点云查看](depot_world_zh.md)。

RViz 使用 world 固定坐标系。独立的 Gazebo 真值节点发布动态 world→base，使机器人在 RViz 中按实际位姿移动；四肢和雷达的相对变换仍由 robot_state_publisher 提供。`/sim/ground_truth/odom` 是仿真真值，不是控制器或 SLAM 的估计结果。

安装位置可通过六个参数调整，例如：

```bash
./scripts/ros1_sim.sh --headless mid360_x:=0.30 mid360_y:=0 mid360_z:=0.103 mid360_roll:=0 mid360_pitch:=0 mid360_yaw:=0
```

平移单位为米、角度为弧度；参数定义 `trunk` 到 `mid360_link` 的固定变换。默认值为 `(0.25, 0, 0.103)` 米、姿态 `(0, 0, 0)` 弧度，是直接贴机背水平安装的仿真参考，实际使用必须测量标定。GBPlanner 的默认雷达位置同步为该值。

默认已去掉原来的 42 mm 支架：雷达底面直接贴合机身顶面 `z=0.056 m`，扫描原点在雷达底面上方 47 mm，因此 `mid360_z=0.103 m`。实机固定材料的厚度需计入实测外参。若手动调高 `mid360_z`，模型仍会生成相应高度的两柱顶板支架作为水平安装示意；若改变雷达倾角，需要按实际结构修改支架。它不是实物制造图纸。

点云为 `/livox/lidar`、`sensor_msgs/PointCloud2`，坐标系为 `mid360_link`。官方表有 80 万个方向，每个仿真更新推进 24000 个，默认每隔 4 条取 1 条，也就是每次检测 6000 条射线；未命中不会产生点，因此点云数目取决于场景。扫描表会循环。可通过 `mid360_downsample:=1` 使用全部射线，或增大该值减轻计算负担；允许 1～24000，数值越大点越稀疏。参数示例：

```bash
./scripts/ros1_sim.sh --headless mid360_downsample:=1
```

设置每秒 10 次是仿真时间的目标，实际墙钟频率和实时因子受计算负载影响。距离检测范围设为 0.1～100 m。内部话题 `/mid360_sim/points_raw` 是官方插件的旧版 `sensor_msgs/PointCloud`，转换节点将其转为 PointCloud2。

本地 Guide 仿真时钟补丁使控制周期、步态相位和物理仿真统一使用 ROS `/clock`。目标控制周期仍是 0.002 仿真秒；仿真变慢或暂停时，控制也一起等待，不再按墙钟提前推进步态。真机编译分支保留原计时方式。补丁位于 `ros1/patches/guide_sim_clock.patch`，首次创建工作区时自动应用；本机已有的 `.ros1/src/unitree_guide` 已直接修复，构建脚本不会覆盖其他已有工作区的源码修改。

仿真还发布 `/livox/imu`（`sensor_msgs/Imu`），目标频率为仿真时间 200 Hz，坐标系为 `mid360_imu_link`，角速度单位 rad/s、线加速度单位 m/s²。相对 `mid360_link` 的 IMU 位置 `(0.011, 0.02329, -0.04412)` 米取自 MID-360 手册；仿真不输出真实硬件无法提供的姿态，姿态协方差首项为 -1。噪声目前设为零，不能反映真实 IMU 的偏置、漂移或噪声。当前固定版本真机驱动的 IMU frame 名称与仿真不同，见[真机说明](mid360_zh.md)。

另开终端检查：

```bash
docker exec aliengo-ros1-sim bash -c 'source /opt/ros/noetic/setup.bash; rostopic info /livox/lidar; rostopic echo -n 1 /livox/lidar/header; rostopic echo -n 1 /livox/imu/header'
```

这是更接近设备扫描方向的仿真，仍非硬件数据：每次更新按当前场景状态检测，不能复现逐点时间及运动畸变；点云只含 XYZ，没有真实反射强度、逐点时间或 Livox 自定义消息，也未模拟反射率、距离相关失效、天气及真实噪声。扫描表循环、射线抽样、场景和动力学的简化都可能影响算法表现；不能用仿真代替实物标定与验证。
