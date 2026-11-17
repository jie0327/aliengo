# GBPlanner3 接入与验证记录

**状态更新（2026-11-14）：当前已应用转弯撞柱修复补丁，最新实现和验证见 [转弯修复记录](gbplanner-turn-clearance.md)。下文保留修复前的官方版本接入与验收快照，其中“未修改官方空间逻辑”和 N3ynQv“最终轮”均指当时版本。**

记录日期：2026-11-14。按用户最新要求“就先用官方的检查逻辑，不要新加安全校验”，当前空间检查采用未修改的官方 GBPlanner3 逻辑。当前版本已完成构建、后端验证和两轮 Gazebo/RViz 自主闭环；最终轮使用全部最新代码。此前自定义检查版本的失败记录单独保留。使用步骤见 [操作说明](../gbplanner_zh.md)。

## 架构与来源

GBPlanner3 固定于 `f9904bb69392d2d46b2cbe939762f967a65daa14`；Voxblox 固定于 `d1db0e19049fb441655eddefcdba9c9af596ead7`。完整依赖锁见 [gbplanner-sources.lock.json](../../references/gbplanner-sources.lock.json)。`references/` 保留上游原始源码，实际构建副本位于 `.gbplanner/src`，应用 [规划后端补丁](../../ros1/patches/gbplanner_backend.patch)、[Voxblox 原生点云补丁](../../ros1/patches/gbplanner_voxblox_native.patch) 和 [轴向射线修复](../../ros1/patches/gbplanner_voxblox_raycast.patch)。最后一项修复上游将零分量判定写成 `abs(component)<0`、导致射线只更新起点的问题。

构建使用缓存依赖镜像 `aliengo-exploration:simulation`，严格校验 ID `sha256:1874ecd35a34f6dd47d32becda8fb85b2c67f273a400413d95dfd297c8e1c5d8`，生成当前运行镜像 `aliengo-learning:exploration`。该缓存不是当前工程可从零重建的完整依赖安装流程；缺少它时构建会失败。运行入口覆盖缓存中的旧入口，加载当前 `/work`、`/lio` 和 `/gbp` 覆盖工作区，禁止运行旧工程导航节点。实际构建产物的镜像 ID、源码锁与补丁校验和记录在 `.gbplanner/provenance/`。

| 组件 | 当前实现 |
|---|---|
| 定位 | 当前 Super-LIO，`lio_map` 为估计坐标系；导航使用估计位姿，Gazebo 真值仅允许测试评分 |
| 点云融合 | `/livox/lidar` 与 `/camera/depth/color/points` 经采集时刻 TF 处理；发布的 `/exploration/cloud` 保持原传感器坐标系、原时间戳和独立射线原点 |
| 三维空间 | 两源进入共享 Voxblox TSDF；按源等待 TF，保留跨传感器同戳、交错/延迟到达数据；空间可通行判断采用官方逻辑 |
| 地面 | `/exploration/elevation_map` 为 0.10m GridMap；仅保留真实回波支持的地面格，未知为 NaN；`terrain_points` 显示历史实测地面 |
| base 位姿 | `/exploration/base_odom` 由 LIO IMU 位姿和当前 URDF 外参换算，不引入第二个 base TF 父节点 |
| 规划与检查 | `/exploration/plan_raw` 请求真实 GBPlanner；保留官方 Rrg root clearing、map reset 及官方空间检查行为；不增加自定义空间检查 |
| 跟随 | 默认禁用；显式使能后约 20Hz 跟随约 0.6m 短段，再请求新路径；跟随默认线速度上限 0.10m/s，底层接口约束为 0.12m/s、0.30rad/s |
| 控制与取消 | Guide 接收 `base` 系 `TwistStamped`；缺命令约 0.4s 归零；协调器检查 ROS/墙钟进展和输入新鲜度，取消代次屏蔽迟到路径 |

已删除自定义 `/exploration/check_path` 服务、root 拒绝、完整脚印检查、低障碍带检查、规划后路径检查及前缀空间校验。协调器使能检查输入新鲜度和控制模式，不追加障碍或地形准入判断。`terrain_status.footprint_known` 与 `reason` 仅用于观测诊断，不作为使能或跟随的额外空间门控。默认禁用、命令超时、取消、操作者停止仍保留。

原生双传感器融合、实测 elevation 地图、轴向射线缺陷修复、MID-360 增益几何适配、单次规划行为树和短段跟随/重规划保留。未知地面格保持 NaN，不推断无限平地，不以 Gazebo 真值导航。地图融合使用两路传感器；探索增益模型使用 MID-360，未启用相机增益或语义标注。机器人包络配置为 0.75×0.45×0.34m、名义站立 base 高度约 0.43m。这些是当前仿真参数，并非实机标定。

## 安装位姿与初始化依据

保留水平 MID-360 和用户指定的原厂下排 D435 对应的现有模型安装位置、朝向；当前相机外参仍是名义仿真值。不得把它称为实机原厂测量结果。

此前覆盖实验 `logs/super-lio-trial-a7abqC` 的结果已记入 [设计](../superpowers/specs/2026-11-14-gbplanner-design.md)：初始 D435 地面最近约 1.06m、MID-360 约 4.4m，脚印 24 格均未命中；走约 1.67m 后保留的 D435 历史回波覆盖 24/24 格。它说明初始化路线的观测依据，**回波命中本身不是自由空间或通行证明，也不是本次 GBPlanner 闭环通过记录**。

用户接受融合运行后先遥控约 1.5–2m，此初始化仍是当前操作流程。实际顺序是构建、`gbplanner_sim.sh`、键盘 `2` 站稳、`gbplanner_start.sh`、等待 `Map init done`、键盘 `4` 与两次 `w` 低速采图、空格停手动速度、`2` 站稳，最后 `gbplanner_enable.sh`。当前验证器检查启动时默认禁用，不再要求未知起点拒绝使能；官方起点清理和地图重置行为已经恢复。

## 当前版本的构建与代码核对

[源码核对记录](../../logs/gbplanner-integration/official-source-verification.json) 确认实际工作区的 `rrg.cpp`、`rrg.h`、`gbplanner.h`、`gbplanner_ros.cpp` 和 `planner_msgs/CMakeLists.txt` 与固定上游逐字节一致。[构建记录](../../logs/gbplanner-integration/official-build.log) 为 6 个包全部通过，耗时约 1 分 37 秒。

[后端 ROS 记录](../../logs/gbplanner-integration/official-ros-check.log) 验证了独立原点、90°旋转相机射线、同戳/延迟输入与独立 TF 等待重试，以及六个正负轴向射线；确认加载当前 `/gbp` 的 Voxblox 库、没有 `/exploration/check_path` 服务。直接调用官方规划服务连续得到两条 5 位姿路径，长度分别 2.538m、2.330m。来源、接口与构建信息见 [后端记录](../../logs/gbplanner-integration/backend-report.json)。

运行环境已实际核对：`rospack find super_lio` 为 `/lio/src/super_lio`，控制器为 `/work/src/unitree_guide`，规划器为 `/gbp/src/gbplanner/gbplanner`；`liblio.so` 来自 `/lio/devel/lib`。先加载缓存依赖/规划器，再加载当前机器人/LIO，以免旧缓存中的 SLAM 覆盖当前工程。

## 整机仿真验收

执行入口：

```bash
./scripts/verification/gbplanner_trial.sh --rviz
```

最终轮 [N3ynQv 原始报告](../../logs/gbplanner-trial-N3ynQv/summary.json) 为 **PASS，13 项检查全部通过**，墙钟约 **152.85s**。该轮在最后一处命令计算时效修复后重新启动，使用最终代码；[精简指标](../../logs/gbplanner-trial-N3ynQv/acceptance-metrics.json) 与 [轨迹图](../../logs/gbplanner-trial-N3ynQv/evaluation-trajectory.png) 可直接核对。Gazebo 位姿仅由评分器读取，没有输入导航。

| 项目 | 最终轮实测 |
|---|---|
| 初始状态 | 默认禁用，先手动采图；没有额外空间使能门槛 |
| 手动初始化 | 1.683m，融合全程运行 |
| 自主执行 | 完成 3 段，至少两次重新规划；净位移 1.590m |
| 实测地面格 | 10,238 → 11,898 |
| TSDF 相机/雷达处理计数 | 相机 197 → 283；雷达 388 → 556 |
| 命令发布进程暂停 | Guide 约 0.406s 后归零；恢复不续走旧路径 |
| 暂停物理与 ROS 时钟 | 约 0.554s 后停止；恢复时钟不自动续走 |
| 主动停止正在执行的路径 | 约 0.044s 得到停用/站立状态 |
| 显式重新使能 | 会话代次 6 → 10，新路径消息序号 3 → 5 |
| 姿态 | 站稳后最低 base 高度 0.421m，最大绝对 roll 0.0088rad、pitch 0.0038rad |
| RViz | 实际订阅 RGB、彩色点云、实测地面与执行路径 |
| 退出 | LIO 保存非空 PCD，控制器和仿真均正常退出 |

13 项覆盖默认禁用、初始化距离、重复路径、自主运动、两源持续融合、地图增长、命令中断停车/不重放、新代次、暂停时钟停车/不重放、主动停止及地图保存。姿态指标证明本轮未跌倒；本测试没有单独的全机身接触/碰撞检测结论。

此前官方版本轮 [bZtKLJ](../../logs/gbplanner-trial-bZtKLJ/summary.json) 也为 PASS：手动 1.685m、自主 1.572m、完成 3 段；它早于最后的命令计算时效修复，因此以 N3ynQv 为最终验收依据。

## 组件回归与审查

- 当前宿主完整测试：`python3 -m pytest -q tests`，**189 passed、22 skipped，11.97s**。跳过项需要 Noetic，不能当作已经通过。
- 最终 Noetic 协调器测试：[11 项实际 ROS 边界回归](../../logs/gbplanner-integration/coordinator-final-ros.log)，覆盖旧命令年龄、迟到模式 RPC、取消、键盘接管和畸形消息；未知脚印、空 floor 不增加空间门槛。
- 融合适配器此前 4 项实际 ROS 检查与 C++ GridMap 往返通过；Guide [最终构建](../../logs/gbplanner-integration/guide-build-final.log) 通过，控制接口和原有时钟回归通过。
- Python 编译、相关 Shell 语法及文档链接检查通过；[独立审查](../../logs/gbplanner-integration/final-review.md) 的当前范围结论已确认官方空间逻辑恢复，最后的命令发布时效问题已修复。

命令时效检查只处理发布循环停顿/消息过期，未增加空间可通行性规则。精确的“计算后、发布前暂停”竞态由实际 ROS 回归覆盖；整机轮另外暂停命令发布进程验证停车和恢复行为。

## 保留的失败与范围

旧自定义检查轮 [OmDA1F](../../logs/gbplanner-trial-OmDA1F/summary.json) 为 FAIL：手动约 1.685m 后，官方生成的路径在新增复核层被清空，未执行自主运动。该复核层已按用户要求删除，旧记录保留供追溯。

当前验证范围为 Depot 平地、固定传感器安装和低速仿真。实测地面图默认固定 16×16m；楼梯、完整仓库覆盖、动态障碍能力和硬件执行未由本次验收证明。相机外参仍需实机标定，仿真未完整复现真实深度噪声和运动畸变。
