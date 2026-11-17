# AlienGo 实机部署速记

适用：**笔记本网线接 AlienGo，使用机载头部 D435、MID-360 和原厂高层 SDK**。准备日期：2026-11-15，计划接机：11月16日。所有笔记本命令均在 `/home/jack/aliengo_learning` 执行。

笔记本镜像和高层桥接已构建，使用你提供的 `unitree_legged_sdk_v3.8.4`。保留原规划尺寸、余量、未知空间政策和跟随参数；没有新增空间校验层。**今天通过的是软件和回环验证，机载相机、实测外参和实机运动尚待现场确认。**

## 1. 接线、网络和现场必填项

带上笔记本电源、网线、机器人遥控器，确认雷达供电和网线已接入机器人交换机。以下为之前的实测记录，地址变化时以现场为准：

| 设备 | 地址 / 条件 |
|---|---|
| 笔记本 | `192.168.123.100/24` 与 `192.168.1.5/24` |
| AlienGo 运动电脑 | `192.168.123.220:8082/UDP`；本地 SDK 端口 `8081` |
| 机载感知电脑 / D435 | `192.168.123.12`；需要其 SSH 用户名和登录方式 |
| MID-360 | `192.168.1.158`；已生成 [.hardware/MID360.json](../.hardware/MID360.json) |
| 运动程序 / SDK | 先前查到 `Legged_sport 1.0.20`；SDK v3.8.4 |
| 电机固件 | 仍待核实；所用 SDK README 要求 H0.1.7/H0.1.9 的固件至少 `0.1.35` |

同一网口要**同时保留两个网段**，不要来回切换已有的 `aliengo-direct` / `mid360-direct` 配置。可创建独立配置，Wi-Fi 上网不变；已有同名配置时只执行最后一行。若现场网口不是 `enp2s0`，先替换：

```bash
ip -br addr
sudo nmcli con add type ethernet con-name aliengo-gbplanner ifname enp2s0 \
  ipv4.method manual ipv4.addresses '192.168.123.100/24,192.168.1.5/24' \
  ipv4.never-default yes ipv6.method disabled connection.autoconnect no
sudo nmcli con up aliengo-gbplanner
ping -c 2 192.168.123.220
ping -c 2 192.168.123.12
ping -c 2 192.168.1.158
```

编辑 [.hardware/robot.yaml](../.hardware/robot.yaml)：填写 `base_to_mid360`、`base_to_d435` 的实测 `xyz/rpy`，记录实际固件。获取和核对步骤见 [现场传感器采参流程](gbplanner_sensor_parameters_zh.md)。`base` 取机身 trunk 原点，X 前、Y 左、Z 上；位置用米、角度用弧度，D435 外参量到驱动的 `d435_link`。模板中的 `null` 不是零外参。MID-360 内部 IMU 偏移已处理，驱动 JSON 外参保持零。现有机器人尺寸为 `.75×.45×.34m`，须核对实际载荷是否仍在这个包络内。

已查到第三方 AlienGo 工程的相机安装模型，可供核对；其数值尚未确认对应原厂下排相机，坐标原点也需区分。见 [D435 外参查询记录](reports/aliengo-d435-extrinsics-search.md)。

## 2. 先接通机载 D435

D435 在机器人上，**不用笔记本的 `camera` 子命令**。宇树原厂 `192.168.123.12:8080` 提供的是查看图像的网页流；当前融合需要 ROS 深度点云，网页能看到画面还不够。[宇树相机说明](https://github.com/UnitreeSupport/Unitree_Docs/blob/master/docs/Aliengo/AlienGo.md)

先在终端 A 启动笔记本 ROS master，再检查机载环境。下面的 `USER` 替换为感知电脑实际用户名，不要求在聊天中提供密码：

```bash
./scripts/gbplanner_real.sh master
```

```bash
./scripts/gbplanner_real.sh camera-probe USER@192.168.123.12
./scripts/gbplanner_real.sh camera-stage USER@192.168.123.12
```

**先对时，再启动相机与 LIO。** 已有 NTP/chrony 时沿用；否则另开终端提供笔记本时钟参考，再执行一次机载对时（会在机载终端请求 sudo 密码）：

```bash
./scripts/gbplanner_real.sh time-server
```

```bash
./scripts/gbplanner_real.sh camera-sync USER@192.168.123.12
./scripts/gbplanner_real.sh camera-onboard USER@192.168.123.12
```

`camera-sync` 通过18080端口读取笔记本时钟，设置感知电脑的**系统时钟**，然后输出复测偏差；`offset_seconds` 应接近零。不改传感器消息时间戳。每轮开始前同步，长时间实验使用持续 NTP。对时结束可退出 `time-server`。

`probe` 只读检查系统、ROS、RealSense、占用相机的进程和时间；`stage` 只复制启动文件和离线源码。机上已有兼容 `realsense2_camera` 时，`camera-onboard` 直接启动并发到笔记本 master。机载节点使用 `ROS_MASTER_URI=http://192.168.123.100:11311`、`ROS_IP=192.168.123.12`；入口会清除冲突的 `ROS_HOSTNAME`。

如 `RobotVisionSystem` 正占用相机，先记录其原启动方式，再在机载终端停止**实际查到的视觉进程**。这会暂停原厂视觉功能；保留 `Legged_sport`。脚本不自动结束进程、不修改开机服务、不升级固件。若机上没有 ROS 相机驱动，见文末“相机缺依赖”；这是现场最需要先排除的接入项。

机上应发布 `/camera/depth/image_rect_raw`、`/camera/depth/camera_info` 和相机内部 TF；笔记本启动 `map` 后由官方 `depth_image_proc` 生成 `/camera/depth/color/points`，frame 为 `d435_depth_optical_frame`。机载深度与 RGB 默认424×240、6Hz，减少有线传输；点云在笔记本按高度着色，RGB图像仍可查看。多相机时务必选对下排D435：填写 `robot.yaml` 的 `camera_serial`，或在 `camera-onboard` 命令末尾加序列号。[D400分辨率与帧率](https://www.realsenseai.com/wp-content/uploads/2019/07/Intel-RealSense-D400-Series-Datasheet-Jun-2019.pdf)

## 3. 建图、采图、探索

以下长时间运行的命令**各占一个终端**。关闭其他工程的 SDK/控制进程，避免争用 `8081`；本流程使用原厂运动控制，**不启动仿真 Guide 或厂商自动行走示例**。

| 顺序 | 操作 |
|---|---|
| ① | 保持终端 A 的 `master` 和机载相机运行 |
| ② | 终端 B：`./scripts/gbplanner_real.sh lidar` |
| ③ | 用遥控器进入原厂运动模式，站稳并保持静止 |
| ④ | 终端 C：`./scripts/gbplanner_real.sh map`；等待 `Map init done` |
| ⑤ | 另开终端：`./scripts/gbplanner_real.sh record`，保存本轮传感器、路径、速度和状态 |
| ⑥ | 用原厂遥控器低速前进约 **1.5–2m** 采图，然后零速站稳、松开摇杆和按键 |
| ⑦ | 终端 D：`./scripts/gbplanner_real.sh sdk`；此时开始高层 UDP 通信，启动不发送行走速度 |
| ⑧ | 执行下面的检查和显式使能 |

```bash
./scripts/gbplanner_real.sh check --motion
./scripts/gbplanner_real.sh rviz       # 可选，单独终端
./scripts/gbplanner_real.sh enable
```

`check` 只读，不发站立或速度命令。确认结果 `ok: true`，两源持续更新，IMU 静止加速度模长约 `9.8m/s²`，点云与地面、机身朝向一致。首次在空旷平地短距离验证转向和遥控接管，再进入货架区域。使能会请求高层站立，并等待**新收到的 HighState** 确认后跟随规划路径。

所有笔记本子命令默认使用 `.123.100` 的 ROS 地址；地址变更时每个终端统一设置 `ROS_MASTER_URI`、`ROS_IP`，并修改机载启动脚本对应默认值。**实机不发布或依赖 `/clock`**。

## 4. 停止与保存

```bash
./scripts/gbplanner_real.sh stop
```

原厂遥控器按键或明显摇杆输入会使桥接停止发送 SDK 指令、交还遥控；实际接管效果须首次短距离实测。接管后若要继续，先停掉并重启 `sdk`，站稳后重新 `enable`。命令或反馈中断会归零，恢复数据不会自动继续旧路径。

正常收工：先 `stop`，再退出 `sdk`；在 `record` 终端按 `Ctrl+C` 完成 bag，随后退出 `map` 并等地图保存，最后关闭相机、雷达和 master。退出自定义相机后，按先前记录恢复原厂视觉程序；未修改自启动，正常重启也可恢复。软件停止受进程和网络影响，紧急情况用遥控器原厂停机操作。

本轮配置、启动日志、bag 和地图在 `.hardware/latest-session`，实际目录为 `.hardware/sessions/日期时间-PID/`。bag 按约1GB分卷。地图在 `maps/map.pcd`，PCD 分片在 `maps/PCD/`。

## 常见现场问题

| 现象 | 优先检查 |
|---|---|
| 能看网页图像，没有深度点云 | 需在 `.123.12` 启动 ROS RealSense 驱动；不能用网页流替代深度 |
| 机载有话题、笔记本收不到 | 两端 ROS_IP/master、双向 TCP 可达、机载时间；ROS 不只使用11311端口 |
| D435 打不开 | 相机是否被原厂视觉进程占用、序列号、驱动与相机固件配套 |
| `timestamp stale` / 相机 TF 一直等待 | 同步两机系统时钟，再重启本轮建图；不把旧消息改成当前时间 |
| 无雷达 / LIO 不初始化 | `.1.5` 与 `.1.158` 路由、UDP端口、供电；静止等待初始化 |
| `no fresh vendor HighState` | `.123.220:8082`、运动模式、SDK/固件、8081端口及其他控制进程 |
| 官方规划器无路径 | 看两源地图与外参，补足实际采图；沿用官方空间判断，不追加起点门控 |

**相机缺依赖：** 离线归档只有官方 ROS wrapper 2.3.2 源码。机上还需编译工具、ROS1、兼容的 librealsense ≥2.50 开发包，以及 `cv_bridge`、`image_transport`、`nodelet`、`tf`、`ddynamic_reconfigure`、`diagnostic_updater` 等 ROS 依赖。脚本会先检查主要依赖；准备好后在机载终端运行 `bash ~/aliengo-gbplanner-camera/onboard_camera.sh --build`，只构建独立工作区；再运行 `camera-onboard`。若 TX2 环境不满足要求，需按其 JetPack 与相机固件配套准备，不能直接运行笔记本的 x86 Noetic 镜像。此分支尚未在机载电脑构建。[官方 ROS1 2.3.2](https://github.com/realsenseai/realsense-ros/tree/2.3.2)

**需要重建时：** `./scripts/gbplanner_real.sh build`。本机已构建，不必在现场重复下载；源码或 SDK 改变后才重建。离线验证记录见 [准备与验证报告](reports/gbplanner-hardware-prep.md)。
