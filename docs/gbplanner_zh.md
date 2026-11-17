# GBPlanner 低速探索：仿真操作说明

当前入口把 AlienGo、Super-LIO、下排 D435 与水平 MID-360 接到 GBPlanner3。针对转弯前端碰货架立柱的问题，仅修补规划器内部的碰撞几何和路径输出拒绝，不增加独立安全服务。沿用原始 .75×.45×.34m 尺寸、原跟随前瞻 .25m 和偏差门限 .30m，不设置额外安全余量。先遥控采集约 1.5–2m 初始地图，再显式使能探索。最新结果见 [转弯修复记录](reports/gbplanner-turn-clearance.md)，早期验收见 [验证记录](reports/gbplanner.md)。

MID-360 保持现有水平安装；保留用户指定的原厂下排 D435 对应的现有仿真位置和朝向。相机外参是当前模型的名义值，尚未按实机标定，不能称为原厂测量外参。此入口不连接真机，也不表示楼梯、完整仓库覆盖或实机部署已经验证。

## 先构建，再启动

在工程根目录执行。已有基础环境时，重新运行控制器构建以应用自主速度接口，然后构建规划器：

```bash
cd /home/jack/aliengo_learning
./scripts/ros1_build.sh
./scripts/gbplanner_build.sh
```

尚未准备 ROS1、Depot 或 LIO 工作区时，先按 [ROS1 仿真说明](ros1_sim_zh.md) 和 [Super-LIO 说明](super_lio_zh.md) 完成基础构建。相关入口是 `ros1_build_env.sh`、`ros1_depot_setup.py`、`super_lio_build_env.sh` 和 `super_lio_build.sh`。

规划器构建依赖本机缓存镜像 `aliengo-exploration:simulation`，脚本要求其 ID 精确等于 `sha256:1874ecd35a34f6dd47d32becda8fb85b2c67f273a400413d95dfd297c8e1c5d8`。镜像缺失或 ID 不同会退出；该脚本不从零安装这套缓存依赖。它在无网络模式下生成当前运行镜像 `aliengo-learning:exploration`，从固定原始源码和本地补丁重建 `.gbplanner` 覆盖工作区。来源、补丁校验和与镜像 ID 保存在 `.gbplanner/provenance/`。缓存只提供依赖，启动节点、机器人、LIO 和控制器均来自当前工程。

## 操作顺序

终端一启动仿真：

```bash
cd /home/jack/aliengo_learning
./scripts/gbplanner_sim.sh
```

默认探索配置把 D435 调为 RGB 640×360、深度 424×240、15Hz，减少渲染和传输负担；安装位姿与视场保持现有配置。融合前端分别以约 10Hz 雷达、5Hz 相机处理数据。此模式要求启用 D435，关闭它会被入口拒绝。仿真启动时关闭 Gazebo 真值 TF，导航使用 Super-LIO 估计位姿。

终端二打开原有键盘控制器：

```bash
cd /home/jack/aliengo_learning
./scripts/ros1_keyboard.sh
```

让键盘终端获得焦点，按 `2` 并等待机器人站稳。终端三启动 LIO、双传感器融合、规划器、默认禁用的探索协调器和 RViz：

```bash
cd /home/jack/aliengo_learning
./scripts/gbplanner_start.sh
```

等待输出 `Map init done`，确认建图融合已运行、两路输入统计在更新，再回键盘终端按 `4`，轻按两次 `w` 慢速前进约 **1.5–2m**。距离是采图目标，不能仅按固定等待秒数判断。采图期间 `gbplanner_start.sh` 必须持续运行，否则走过的区域不会进入这次融合地图。然后按空格停止手动速度，再按 `2` 站稳。

终端四显式使能：

```bash
cd /home/jack/aliengo_learning
./scripts/gbplanner_enable.sh
```

使能检查 LIO、时钟、两路融合数据是否新鲜，以及控制器模式是否正确；通过后切换自主模式，请求 GBPlanner 路径并跟随。障碍、地形和路径的空间判断由带碰撞几何修补的 GBPlanner 完成；协调器不根据 `footprint_known` 拒绝使能，也没有额外的起点或路径空间校验服务。一次跟随约 0.6m 短段后重新规划。默认禁用和输入过期停车仍保留；不能把默认禁用理解为“未知起点必定拒绝使能”。请按上述流程先完成手动采图。

停止自主探索：

```bash
./scripts/gbplanner_stop.sh
```

键盘 `2`（站立）或 `1`（被动模式）也会退出自主模式。**空格仅用于手动模式停车，不能作为自主探索停止键。** 停止后需重新执行 `gbplanner_enable.sh` 才能恢复；输入恢复不会自动重放已取消路径。退出时先停止探索，再在启动建图的终端按 `Ctrl+C` 并等待地图保存，最后关闭键盘与仿真。LIO 地图保存到 `.super_lio/maps/exploration-日期时间-PID/`。

只需后台运行时，可分别使用 `gbplanner_sim.sh --headless` 与 `gbplanner_start.sh --headless`；后者只关闭探索 RViz。

## 在 RViz 里看什么

固定坐标系为 `lio_map`。默认显示同时提供：

| 显示 | 话题与含义 |
|---|---|
| D435 彩色图像 | `/camera/color/image_raw`，检查下排相机画面 |
| D435 彩色点云 | `/camera/depth/color/points`，RGB8 着色，显示当前深度观测 |
| Super-LIO 配准点云与轨迹 | `/lio/cloud_world`、`/lio/path`，点云累积最近 30 秒 |
| 已观测地面 | `/exploration/terrain_points`，保留实际回波支持的历史地面格 |
| GBPlanner 当前执行路径 | `/exploration/path`，橙色短段，表示协调器当前跟随路径 |

参考网格只是显示辅助。地面未知格保留 NaN，融合不生成无限平地，也不输入 Gazebo 真值导航。历史观测地面可在当前静态 Depot 中保留，但不代表相机此刻能看到脚下。看到点云或地面颜色并不等于可走；碰撞几何使用原尺寸并按实际路径方向检查完整机身体积；转身只检查实际转角，不使用全方向膨胀包络。原未知空间政策和地形逻辑沿用，协调器没有额外空间校验。

## 地图与停步诊断

左侧中文显示成方框时，先检查容器能否读取中文字库。当前启动器会把宿主的 `/usr/share/fonts/opentype/noto` 只读挂载到容器；GBPlanner 验证入口也已补齐该挂载。字体挂载在创建容器时生效，旧容器需停止并通过相应脚本重新启动，再打开 RViz；无需重新编译算法或修改中文显示名称。字体检查和实际 RViz 窗口验证记录见 [字体修复记录](../logs/rviz-fonts/report.md)。

在仿真运行时，可另开一个容器终端查看状态：

```bash
docker exec -it aliengo-ros1-sim bash
source /opt/ros/noetic/setup.bash
rostopic echo -n 1 /exploration/terrain_status
rostopic echo -n 1 /exploration/map_status
rostopic echo -n 1 /exploration/status
rostopic echo -n 1 /aliengo/controller/status
```

`terrain_status` 的 `footprint_known` 和 `reason` 仅作为地面观测诊断，不作为协调器使能或路径放行条件。`sources` 给出前端两源处理状态；`map_status.sources` 中 `mid360_link` 与 `d435_depth_optical_frame` 各有采集时间戳和处理计数。重复查看时，两源时间戳与计数应继续前进。`exploration/status` 给出 `enabled`、`state`、`reason` 和完成路径数。

| 返回原因 | 检查与处理 |
|---|---|
| `waiting for LIO and mapping status` | 确认先站稳再启动建图，并已看到 `Map init done` |
| 地面诊断 `unknown footprint cell`、`obstacle or hole in footprint` | 查看实测地面与采图过程；这些字段不再触发额外的协调器空间门控 |
| 官方规划器无有效路径 | 核对两源地图、估计位姿与官方规划器日志；当前版本没有自定义 root/check_path 拒绝原因 |
| `… stopped or stale`、`controller status stopped` | 检查 Gazebo 是否暂停、对应传感器/LIO/控制器是否仍更新；恢复数据后重新检查并显式使能 |
| `controller left autonomy mode` | 检查键盘模式切换；站稳后按需重新显式使能 |

保持水平 MID-360 和下排 D435 时，初始视野可能覆盖不到脚下；手动前进让此前前方的真实观测落入后续脚印，是该流程采用的初始化方式。传感器位姿保持不变；不推断无限平地，不给规划器输入 Gazebo 真值。官方自身的 Rrg root clearing 与 map reset 按上游行为运行。

架构、源码版本、已执行测试与仍待验收项见 [验证记录](reports/gbplanner.md)。
