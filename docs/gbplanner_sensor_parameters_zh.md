# 实机传感器参数：现场获取与核对

用于2026-11-16接机。先做感知采参与核对，探索保持默认关闭。网络与驱动启动命令见 [部署速记](gbplanner_real_zh.md)。

## 1. 先区分要取得的参数

| 参数 | 获取方式 |
|---|---|
| D435内参：K、D、P、分辨率 | 由实际相机、当前流配置生成的 `camera_info` 读取 |
| D435内部深度/彩色坐标关系 | 由RealSense驱动读取设备标定并发布，独立于机身安装外参 |
| MID-360内部雷达—IMU关系、原始IMU单位 | 当前固定驱动和配置已经适配；现场检查输出、避免重复转换 |
| 机身→MID-360、机身→下排D435安装位姿 | 优先查本台原厂配置/原有TF；缺少可信数据时量测并校正 |

设备内参、相机内部外参与机身安装外参分别记录。RealSense的 `depth_to_color` 不能用来填写 `base_to_d435`。官方ROS1话题和参数说明见 [RealSense 2.3.2](https://github.com/realsenseai/realsense-ros/tree/2.3.2#published-topics)。

## 2. 确认设备身份与原厂安装关系

固定雷达支架、相机和线缆；机器人站稳。先执行：

```bash
./scripts/gbplanner_real.sh camera-probe USER@192.168.123.12
```

确认相机序列号、机载ROS/驱动和固件。通过实际画面对应下排相机，把序列号填入 `.hardware/robot.yaml` 的 `camera_serial`，避免下次启动选到另一台相机。

在启动我们自己的 `map` 前，查找机载原有配置和TF是否给出机身—相机关系。原有TF的frame名须现场确认；若给的是光学frame，要换算到当前需要的 `d435_link`。记录参数来自哪个文件/程序、对应哪个序列号。我们后来发布的TF是配置结果，不是独立测量证据。

雷达安装位姿通常随本次支架确定，也需查安装资料或量测。相机的 [公开工程模型参考值](reports/aliengo-d435-extrinsics-search.md)尚未证实对应本台原厂下排，保持“待核对”标签。

## 3. 对时、固定模式并读取真实内参

按部署速记启动master、雷达、机载相机。`camera-stage` 后先完成 `time-server` / `camera-sync`，再启动相机和LIO；长时间实验使用持续NTP。当前目标配置为雷达10Hz、相机424×240且6Hz，实际分辨率以消息和驱动日志为准。

在笔记本新终端进入已准备的ROS环境：

```bash
./scripts/gbplanner_real.sh shell
```

下面命令均在这个容器终端执行，文件会落到笔记本 `.hardware/calibration/`：

```bash
calibration_dir="/hardware/calibration/$(date +%Y%m%d-%H%M%S)"
mkdir -p "$calibration_dir"
timeout 10s rostopic echo -n 1 /camera/depth/camera_info > "$calibration_dir/depth-camera-info.yaml"
timeout 10s rostopic echo -n 1 /camera/color/camera_info > "$calibration_dir/color-camera-info.yaml"
rosparam dump "$calibration_dir/ros-parameters.yaml"
```

确认读取命令收到消息后正常结束，文件里有实际宽高和K/P矩阵。相机分辨率变化后重新留档，使用当前消息里的内参，不手工沿用其他分辨率的数值。参数dump用于留档，内参以camera_info文件为准。

## 4. 先检查连续数据，再录原始包

```bash
timeout 10s rostopic hz /livox/raw_lidar
timeout 10s rostopic hz /livox/raw_imu
timeout 10s rostopic hz /camera/depth/image_rect_raw
```

观察是否持续更新、频率接近配置、是否反复断连或出现大段延迟。静止IMU的原始加速度模长在当前驱动下约为1g；运行适配器后的 `/livox/imu` 模长约9.8m/s²。检查模长，不要求某个轴固定等于9.8。

录一分钟静态数据用于检查和后续校正。该命令不依赖已填写机身外参，也不启动运动控制：

```bash
rosbag record --duration=60 --lz4 -O "$calibration_dir/static-sensors.bag" \
  /livox/raw_lidar /livox/raw_imu \
  /camera/depth/image_rect_raw /camera/depth/camera_info \
  /camera/color/image_raw /camera/color/camera_info /tf /tf_static
```

选择两传感器能共同看到的地面、墙面/墙角。若之后需要完整外参校正，再补充多个视角或标定板数据；仅有一块地面不能完整确定前后、左右与偏航等六自由度。

## 5. 核对安装外参，固化配置

把有来源的安装位姿填入 `.hardware/robot.yaml` 的 `base_to_mid360` 和 `base_to_d435`。位置单位米、姿态单位弧度，参考原点为本项目 `base`；传感器原点按对应frame定义，量外壳时要换算。

先以原厂/量测结果或明确标记的初值运行感知：站稳后启动 `map`，等待 `Map init done`，另开终端启动 `rviz` 和 `check`。此阶段无需启动SDK桥接。

- 静止观察：两源看到的同一地面/墙面位置是否一致，有无明显倾斜、整体跳动或双层。
- 用原厂遥控器慢速前进约1–2m，再小角度转向、停止：检查同一面墙在历史观测中是否明显分裂，地面和障碍是否仍对齐。
- 检查实际前进方向与估计机身X轴一致、左右转方向一致，核对机身参考轴。
- 有偏差时结合原始包检查时间、坐标轴、单位和安装位姿，确认来源后再校正。现有 `check` 检查数据新鲜与TF连通，不自动估计或证明外参精度。

每次修改安装外参后重新启动本轮 `map`、重新采图并清除RViz历史点云；保留旧轮记录供比较，避免混用不同外参下的历史地图。

最终保留序列号、固件/驱动版本、实际分辨率、对时结果、两套安装外参及其来源、原始bag。确认这组配置后再按部署速记接入SDK、检查并使能探索。
