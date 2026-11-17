# GBPlanner 实机准备与验证

日期：2026-11-15。操作入口：[实机部署速记](../gbplanner_real_zh.md)。目标接法为笔记本通过网线连接 AlienGo，使用机载头部下排 D435、MID-360 和原厂高层 SDK。

## 已准备

- `scripts/gbplanner_real.sh`：独立的 master、雷达、机载相机接入、建图、SDK、只读检查、录包、RViz、显式使能/停止入口。无需 Gazebo、仿真 Guide 或 `/clock`。
- 已构建 `aliengo-learning:gbplanner-real`，安装官方 ROS RealSense 2.3.2、librealsense 2.50.0、depth_image_proc。镜像及 SDK 哈希见 [.hardware/build.json](../../.hardware/build.json)。当前镜像 ID 为 `sha256:b1d2a2d1c8fbc1ed5966c52979b28faf90e9a28013588e9848f3494d136b6b14`。
- 使用用户指定 `unitree_legged_sdk_v3.8.4`，提交 `1c6226d3a6608432b43d1e488c5816ba134a354e`。实际桥接链接这份头文件/动态库，检查 HighCmd=113、HighState=244 字节；只发送高层命令。未使用缓存镜像中的旧控制栈。
- [.hardware/robot.yaml](../../.hardware/robot.yaml) 模板保留待测位置为 `null`。根据之前记录生成 [.hardware/MID360.json](../../.hardware/MID360.json)：主机 `.1.5`、雷达 `.1.158`。底盘 `.123.220`、感知电脑 `.123.12`、笔记本 `.123.100`，仍须现场确认。
- 机载相机探测、暂存、启动和一次性系统时钟同步脚本；官方 ROS wrapper 2.3.2 源码已保存在 `.hardware/onboard-kit/`，附下载地址与 SHA256。归档为 wrapper 源码，未假设它包含机载系统依赖。
- 每轮保存配置、构建来源、启动日志、bag 与地图到 `.hardware/sessions/`；录入原始传感器、相机深度/内参、TF、LIO、执行路径和控制状态，便于排查现场碰撞。

## 数据与控制

MID-360 使用真实 CustomMsg 和逐点时间，保留时间戳、timebase、tag/line 与逐点 offset，交给官方 LIO type=1。Driver2 与固定 LIO 内嵌 Driver1 头文件的 CustomMsg MD5 相同，已用实际 C++ 订阅链路验证；未使用仿真 type=8。原始 IMU 加速度从 g 转为 m/s²，协方差同步转换，保留时间戳，明确物理 IMU frame 与芯片偏移。

导航使用完整去畸变回波，避免把注册 `.25m` 体素滤波后的点集直接当作障碍输入。注册原有 `1m` blind 不变；导航补回原融合范围 `.2–1m` 内的原始有效回波，按其采样时间与 LIO TF 转入同一参考雷达坐标系。近点时间分组跨度小于1ms，不外推缺失 TF，不生成地面或障碍观测。

机载 D435 发布424×240、6Hz的深度与 RGB，笔记本使用官方 depth_image_proc 从深度和内参生成 XYZ。避免录包、融合和 RViz 各自从机上接收整幅密集点云。相机内部 TF 由官方驱动发布，机身安装外参由实测配置发布；实机 RViz 显示 XYZ 高度颜色与 RGB 图像，不依赖仿真关节模型。

高层桥接复用原 CommandGate：命令时效 `.4s`、速度上限 `vx=.12m/s`、`vy=.08m/s`、`wz=.30rad/s`；跟随器保持 `.10m/s`、前瞻 `.25m`、偏差门限 `.30m`。新鲜 HighState 用于站立握手；命令/反馈过期使速度归零，遥控按键或明显摇杆输入停止 SDK 发送。未实现低层关节控制、自动起身或恢复动作。物理接管行为仍需现场验证。

GBPlanner 原 `.75×.45×.34m` 尺寸、零额外余量、未知空间政策、地形逻辑和已有碰撞修补保持不变。协调器只增加系统时间支持；仿真仍检测 `/clock` 停顿。未新增起点、脚印或活动路径空间校验层。

## 已运行的验证

所有 SDK/ROS 集成测试在 `--network none` 容器内进行，SDK 对端仅为回环地址；未连接机器人。源数据与外部 SDK 对端由测试提供，SDK 序列化、ROS、LIO、depth_image_proc、融合和规划节点使用真实实现。

| 检查 | 结果与证据 |
|---|---|
| 镜像、桥接编译与动态库 | 通过；[构建日志](../../logs/hardware-prep/bridge-build.log) |
| 完整项目测试 `python3 -m pytest -q tests` | **202 passed, 24 skipped**；[输出](../../logs/hardware-prep/host-tests.log)，跳过项主要需要 Noetic 环境 |
| 真实 Noetic 协调器边界测试 | **13/13**，包含无 `/clock` 运行及原有停止/过期/未知脚印测试；[输出](../../logs/hardware-prep/backend.log) |
| 原规划器几何、最终路径拒绝及真实规划服务 | **32/32 + 2/2**，多次有效规划通过；同一 backend 日志 |
| 高层桥接状态核心 | **8/8**；[集成日志](../../logs/hardware-prep/integration.log) |
| 真实 SDK UDP 回环 | ABI、启动零速、速度下发、命令过期、反馈中断与遥控接管均通过；同一集成日志 |
| 真实传感器格式整条链路 | CustomMsg→官方 LIO→完整/近场去畸变→原融合→GBPlanner 运行；深度 Image+CameraInfo→官方 XYZ nodelet→相机融合通过 |
| `check --motion` | **ok=true**；[只读检查结果](../../.hardware/test-output/readiness.log)，未知脚印保持诊断，不构成额外拒绝 |
| PCD 持久化 | 验证新生成 [map.pcd](../../.hardware/test-output/maps/map.pcd)；[运行统计](../../.hardware/test-output/pipeline-result.json) |
| 干净机载环境 | 捕获真实 ROS setup 后的启动环境，master为`.123.100`、IP为`.123.12`、无冲突ROS_HOSTNAME；[输出](../../logs/hardware-prep/onboard-master-green.log) |
| 对时工具 | 回环 HTTP 测得正确的时差符号；测试未调用 `--set`、未调整任何系统时钟 |

测试发现并修正：回调后使用旧时间采样导致新速度被拒绝；上游 LIO 将绝对保存路径拼到源码目录下，启动器改用其支持的相对路径；完整近点被注册 voxel/blind 滤掉；ROS setup 以 localhost 填充默认 master；干净 shell 的 nounset 与冲突 ROS_HOSTNAME。相关 RED 日志保存在 `logs/hardware-prep/`。

裸 `pytest` 会扫描工作区内第三方 `.ros1_rl/site/sympy`，并因缺少其测试依赖 `hypothesis` 出现 collection error。项目的完整测试范围为 `tests/`，上表记录该命令结果；未修改第三方测试或安装无关依赖。Noetic 旧服务测试仍会打印上游 socket ResourceWarning，13项测试结果为 OK。

## 审查与现场剩余项

已完成独立只读代码审查并修正所有 Critical/Important 项，包括机载 master 初始化、探测 shell、对时操作、机载依赖说明及近回波处理；没有剩余 Critical/Important 发现。审查未替代实际设备验证。

尚需现场确认：机载感知电脑的登录方式、ROS/RealSense 与相机占用状态；目标下排 D435 序列号；实际双网段可达性；两机时间；机身—传感器实测外参；电机固件是否满足这份 SDK；物理速度方向、停车与遥控接管。若 TX2 没有配套 ROS 相机环境，仍需按其系统准备依赖。

本次未更改笔记本网卡配置、机载系统、固件或原厂开机服务，未发送实际机器人命令。当前证明的是软件准备和接口联调；首轮实机仍按部署速记进行空旷平地短距离验收。
