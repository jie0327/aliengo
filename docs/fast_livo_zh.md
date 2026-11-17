# AlienGo 仿真中的 FAST-LIVO2

FAST-LIVO2 使用独立启动入口、镜像 `aliengo-learning:noetic-fast-livo` 和工作区 `.fast_livo`。原来的 `ros1_sim.sh`、`super_lio_sim.sh`、GBPlanner 命令继续按原方式使用。FAST-LIVO2 没有替换现有定位器，也没有修改机器人模型、雷达或相机插件。

## 启动

在项目目录分别打开三个终端。先启动 FAST-LIVO2 专用仿真：

```bash
./scripts/fast_livo_sim.sh
```

然后启动原有键盘控制，按 `2` 起身并等待站稳：

```bash
./scripts/ros1_keyboard.sh
```

最后启动三传感器融合与 RViz：

```bash
./scripts/fast_livo_start.sh
```

看到 `IMU Initials`，随后出现 `FAST-LIVO2 visual update: tracked_points=...` 且特征数大于零后，再按 `4` 进入小跑，轻按两次 `w` 低速前进。空格停车。结束时先停车，在估计器终端按 `Ctrl+C` 等待地图保存，再关闭控制和仿真。

无图形窗口可分别使用 `fast_livo_sim.sh --headless`、`fast_livo_start.sh --headless`。相机仍需要渲染环境，脚本沿用现有 GPU/Xvfb 处理。FAST-LIVO2 专用入口默认 RGB 640×360、深度 424×240、15 Hz，只影响这一入口；原仿真默认分辨率和传感器安装位置保持不变。允许通过已有 `d435_*:=...`、`mid360_*:=...` 参数覆盖。

与原有仿真共用计算互斥锁。已有仿真运行时，新入口会退出提示，不会结束或覆盖已有容器。请先自行退出当前仿真再切换。

## 输入、输出与标定

| 项目 | 接口 |
| --- | --- |
| 激光 | `/livox/lidar`，原始 XYZ `PointCloud2` |
| IMU | `/livox/imu`，m/s²、rad/s |
| 视觉 | `/camera/color/image_raw` |
| 启动标定 | `/camera/color/camera_info` 与静态传感器 TF |
| 融合里程计 | `/fast_livo/odom`，IMU 位姿 |
| 彩色地图扫描 | `/fast_livo/cloud_world`，`livo_map` 坐标系 |
| 轨迹 | `/fast_livo/path` |
| 跟踪图像 | `/fast_livo/tracked_image` |
| 机器人 TF | `livo_map → base`，由融合估计及静态安装外参计算 |

启动前自动读取匹配的 RGB 图像和 CameraInfo，并从静态 TF 求得 LiDAR→IMU、LiDAR→彩色光学相机外参。标定缺失、过期或不支持的相机模型会令启动失败；实际加载值保存在 `/fast_livo/calibration/loaded`。因此修改专用仿真的相机尺寸或安装位置后，无需手工抄写仿真 YAML 中的标定。

地图保存在 `.fast_livo/maps/日期时间-PID/`，包括 `all_raw_points.pcd`、降采样后的 `all_downsampled_points.pcd` 和 `lidar_poses.txt`。上游的彩色地图只保留相机可见并成功投影的点，不等于完整 360° 激光占据地图。默认退出时保存累积地图；长时间录制会增加内存占用。

FAST-LIVO2 与 RViz 的 ROS 日志保存在同名的 `.fast_livo/logs/日期时间-PID/`，关闭容器后仍保留。报告运动中出现的问题时可以直接检查这些日志。专用镜像内含 Noto 中文字体，RViz 的中文显示无需依赖宿主机字体目录；旧镜像需运行 `./scripts/fast_livo_build_env.sh` 更新，算法工作区无需重新编译。

## 构建和源码

```bash
./scripts/fast_livo_build_env.sh
./scripts/fast_livo_build.sh
```

镜像以已有 `aliengo-learning:noetic-lio` 为基础。若在新机器从头准备，先按项目说明构建 ROS1、RViz、Super-LIO 依赖镜像；算法不要求运行 Super-LIO。已有 `.ros1` 传感器工作区无需因本次接入重新构建。

上游为 [hku-mars/FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2)，固定提交 `0d2c0346107b75b59934975adec9a6eeeb913c64`。Sophus 与 Vikit 使用上游指定的版本系列，完整提交与许可记录在 `ros1/fast_livo/source.json`。

参考源码位于 `references/FAST-LIVO2`、`references/rpg_vikit` 和对应 Sophus 目录，保持原样；补丁只应用到 `.fast_livo/src`。setup 再次运行保留工作区修改，并检查版本来源。若源版本或补丁改变，脚本会拒绝覆盖旧工作区；应先保存并移走旧工作区，再重新准备。

本地补丁 `ros1/patches/fast_livo_sim.patch` 包含：

- `SIM_SNAPSHOT=8` 接收当前瞬时 XYZ 点云，所有点的帧内采样偏移为零。异步图像之间仍按 IMU 对整帧快照作刚体时间对齐。
- 修正 IMU 初始化时的传播起点、空区间历史和点云边界处理，包含实际 C++ 回归测试。
- 统一消息到测量时刻，输出视觉更新之后的融合位姿；关闭上游独立 IMU TF，通过现有数学转换发布 base TF。
- 可配置输出坐标系和地图目录、限制消息缓冲、采用单任务 O2 构建。

## 验证

```bash
./scripts/verification/fast_livo_snapshot_test.sh
./scripts/verification/fast_livo_overflow_test.sh
./scripts/verification/fast_livo_trial.sh --seconds 20 --record-bag
./scripts/verification/fast_livo_replay.sh logs/fast-livo-trial-XXXXXX/sensors.bag
```

试验有墙钟时限并自动清理自己启动的容器。在线验证检查站立、前进、停止和转向，真值只用于一次初始对齐及误差统计；独立回放只输入传感器、CameraInfo 和静态安装 TF，不启动 Gazebo。检查包括实际视觉特征、彩色点云、测量时间戳、估计 TF 和退出保存。

当前接入面向仿真学习，不包含闭环、GBPlanner 切换或实机部署。仿真外参不能作为实机标定，Gazebo 图像与雷达时间戳也不能替代硬件同步验证。实测结果见 [验证记录](reports/fast-livo2.md)。
