# 大场景与 MID360 点云查看实施计划

> **For agentic workers:** 使用 executing-plans 执行以下步骤；模型调查与显示配置可独立委派，所有构建和仿真由主代理串行运行。

**Goal:** 在用户最终指定的 Depot 仓库中用 AlienGo 的 MID360 观察实时点云。

**Architecture:** 将 OpenRobotics Fuel Depot v6 适配为 Gazebo Classic 静态仓库，保留此前完成的 CitySim 作为可选场景。场景切换和 RViz 通过现有 ROS1 仿真入口开启。初版使用 base 固定系；后续运动同步修复改为 world 固定系，并通过 Gazebo 真值发布动态 world→base。

**Tech Stack:** ROS Noetic、Gazebo Classic 11、NVIDIA OpenGL、RViz、SDF、Python。

**Spec:** 用户最初选择大厂区／城市街区，之后明确指定 mathaimp/livox_gz_sim 的 docs/hap_fastlio.webp 顶图中的 Depot 仓库。该选择覆盖此前的城市默认场景，保留已有键盘控制和 GPU 渲染。

## 最新场景决定

- 使用该图片中的 Depot 仓库资源；不采用仓库自带的 12×8 米 demo 房间。
- 保留 ROS Noetic / Gazebo Classic 11，不移植原仓库的 ROS2/Harmonic 插件。
- 下载固定 Fuel version 6，保存原始资源、哈希和 CC-BY-4.0 作者信息。
- 原 Depot 只有地面碰撞；从原可视网格补齐墙、柱、箱子和完整 FLOOR 的碰撞，供当前 CPU Ray 扫描。完整 FLOOR 替换原平面，避免脚下重复接触并保留夹层。
- 世界坐标、模型尺度、网格布局保持原值。风扇作为静态网格保留初始姿态，移除现代 Gazebo 专用风扇插件。
- 新默认场景为 Depot，earth 和 city 仍可显式选择。

## 约束与方案

- 大场景不能只把小仓库整体缩放；道路、建筑和雷达量程使用真实米制尺寸。
- 不引入 ROS2，不改控制器和雷达扫描算法；物理参数沿用当前已验证配置。
- 停用 CitySim 中与当前目标无关的动态车辆、行人控制和特效插件；保留场景建筑、道路与静态障碍物。
- 模型与许可证固定保存，运行容器继续使用 network none。
- RViz 只展示实时 XYZ 点云，不伪装 SLAM；后续动态 world→base 使用明确标注的 Gazebo 仿真真值，不发布虚假的 map 定位。
- 构建、仿真分别串行限资源运行；不安装宿主驱动。

## Review Focus

- 递归模型与材质 URI 必须全部离线可解析。
- 出生位置必须位于可行走道路上，不能穿地或在建筑内。
- RViz 必须收到点云并正确解析 base 到 mid360_link 的 TF。
- --headless --rviz 也应获得 X11/GPU 资源。
- 更换环境不能破坏已有的站立、低速前进和停车。

## 执行步骤

- [x] 固定 CitySim 提交，检查资源规模、许可和实际坐标，准备静态 SDF 及所有模型依赖。
- [x] 在派生 Noetic 镜像安装 RViz，增加 base 固定坐标系、按高度着色的 PointCloud2 配置。
- [x] 扩展 scripts/ros1_sim.sh 与 ros1/mid360/gazebo.launch 的场景、出生位姿和 RViz 参数。
- [x] 在 CitySim 实测键盘行走并截取 RViz 点云。
- [x] 下载并适配 Depot v6，固定来源哈希并加入默认场景。
- [x] 在 Depot 实测键盘低速前进、停车、雷达数据和实际 RViz 订阅与显示。
- [x] 保存实际点云／显示与性能证据、来源说明及简明启动方法，完成独立审查。结果见 docs/reports/depot-world.md。
