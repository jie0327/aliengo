# GBPlanner + 下排 D435 自主探索设计

## 当前约束（按用户最新指令修订）

用户要求 AlienGo/Gazebo 接入 GBPlanner。MID-360 保持水平安装；D435 对应照片橙框的原厂下排相机，保持当前模型位置和朝向。当前相机外参是仿真名义值，不能冒充实机原厂标定。用户接受在融合已运行时先遥控约 1.5–2m，再使能自主探索。

**最新明确指令：先使用官方的检查逻辑，不新加安全校验。** 此指令覆盖旧版设计中的额外起点、完整脚印、低位机身体积、整条返回路径和路径前段复核要求。使用固定 GBPlanner3 原始 `Rrg` 与规划回调的可通行性判断，包括官方的起点初始化/清空处理；不增加 `/exploration/check_path` 或其他空间复核服务，不自行清空或裁剪官方路径来复核可通行性。下文为当前实现依据，旧设计留存在 `logs/gbplanner-integration/before/design-custom-checks.md`。

首轮范围为 Depot 平地低速仿真，不包含楼梯、完整仓库覆盖、实机标定或硬件执行。旧 `/home/jack/ros2_ws` 只作已审查的依赖缓存，不修改、不启动旧工程导航节点。

## 数据与运行环境

GBPlanner3 固定于 `f9904bb69392d2d46b2cbe939762f967a65daa14`，Voxblox 固定于 `d1db0e19049fb441655eddefcdba9c9af596ead7`。`references/` 保存上游原始源码，本地补丁在 `.gbplanner` 中应用。保留的补丁只处理双源原生点云接入、轴向射线缺陷、当前 MID-360 非对称视场模型和单次规划行为树等接入需求。

依赖缓存镜像只供应依赖。先加载缓存依赖和当前规划器，再加载当前 `/work` 机器人和 `/lio` Super-LIO，防止缓存中的旧 SLAM 覆盖当前工程。以 `rospack` 和动态库路径核实实际来源，记录源码锁、补丁和镜像 ID。

Super-LIO 是导航估计位姿来源，`lio_map` 为地图坐标系。Gazebo 真值只用于验证评分，不向定位、建图、规划或控制提供真值导航数据。

## 融合与地面输入

- 输入 `/livox/lidar`、`/camera/depth/color/points`、`/lio/odom`、`/livox/imu` 与采集时刻 TF。
- `/exploration/cloud` 保留各传感器原坐标、原时间戳和独立射线原点。雷达约 10Hz、相机约 5Hz，过滤非有限点、自身回波和距离异常并降采样。
- Voxblox 按源分别排队、等待 TF 和计数，接受跨源同戳或交错到达的观测。轴向射线的零分量缺陷有单独补丁及回归。
- `/exploration/elevation_map` 为 0.10m GridMap，只使用实测地面回波，未知格为 NaN；不填充无限平面或未观测地面。官方 GBPlanner 自己读取该高程图并作地面投影与碰撞判断。
- `/exploration/terrain_status` 的 `footprint_known`、`floor_z`、`reason` 等只用于覆盖诊断，不作为额外使能或路径门槛。
- `/exploration/base_odom` 由 LIO IMU 位姿和 URDF 安装外参换算，保留时间戳，不增加第二个 base TF。匹配实际 `/livox/imu` 角速度换算杆臂速度；缺角速度时保留位姿、标记 twist 不可信。
- 历史实测地面在当前静态 Depot 中保留，实时输入仍须持续更新。

此前覆盖实验表明 D435 初始地面最近约 1.06m，MID-360 约 4.4m；遥控约 1.67m 后历史 D435 回波可覆盖脚下区域。该结果用于选择初始化流程，不当作官方规划或通行结果。

## 规划与跟随

原始服务 `/exploration/plan_raw` 使用官方 `planner_msgs/planner_srv`。`Rrg::reset`、`Rrg::setState` 和规划服务回调恢复固定上游原样；不添加占据/未知 root 复核，不添加返回路径碰撞复核，不抽取重新验证的路径前段。

配置按当前机器人尺寸与传感器安装设置官方参数，保守包络约 0.75×0.45×0.34m、名义 base 高度约 0.43m。可通行性由官方算法判断。

协调器默认禁用。显式 `/exploration/enable` 后确认时钟、估计位姿、两路融合输入和控制接口持续更新，站立完成后进入自主模式。接收官方路径后仅检查消息格式、有限数值和跟随进度，按约 0.6m 执行段跟随并重新请求规划；这是执行长度管理，不是另一套空间可通行性判断。

保留取消代次、迟到结果丢弃、命令超时、暂停时钟和输入中断处理。停止后的旧路径不能自动重放。规划 RPC 在工作线程执行，跟随命令约 20Hz，不被规划服务阻塞。

## 控制与启动

Guide 接收 `base` 系 `/exploration/cmd_vel`（TwistStamped），默认零，接口速度上限约 0.12m/s、0.30rad/s，命令约 0.4s 过期。模式切换在控制线程执行，服务为 `/aliengo/controller/stand`、`/aliengo/controller/autonomy`，状态为 `/aliengo/controller/status`。保持原有键盘和 TF 行为。

启动顺序：`gbplanner_sim.sh` → 原键盘 `2` 站稳 → `gbplanner_start.sh` 启动 LIO/融合/规划/RViz → 遥控约 1.5–2m → 空格停止手动速度、`2` 站稳 → `gbplanner_enable.sh`。自主停止用 `gbplanner_stop.sh` 或键盘 `2`/`1`；空格仅用于手动速度。

## 验收

核实官方检查代码原样、没有额外空间服务/门槛，双源按正确原点与时间进入地图，默认禁用且先遥控初始化。实际 Gazebo 执行至少两次重新规划和约 1m 自主运动，记录路径、地图增长和姿态；命令中断、暂停/恢复时钟和显式停止不能重放旧路径。保留失败轮次与当前版本的独立审查结果。
