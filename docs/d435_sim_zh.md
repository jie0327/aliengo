# AlienGo 下排 D435 仿真

保留 AlienGo 原有的机身和相机外观，只添加不可见的相机坐标系和传感器，不增加外壳、质量或碰撞。彩色和深度分别渲染，以各自内参和相机间外参生成彩色点云。

## 构建和查看

```bash
./scripts/ros1_build.sh
./scripts/ros1_sim.sh --rviz
```

首次使用 RViz 需运行 `./scripts/ros1_viz_build.sh`。显示列表中新增 **D435 彩色图像** 和 **D435 彩色点云**；点云采用 RGB8 着色。原有键盘控制方法不变，无需启动真机 RealSense 驱动。

| 输出 | ROS 话题 | 格式及坐标系 |
| --- | --- | --- |
| 彩色 | `/camera/color/image_raw` | `rgb8`，`d435_color_optical_frame` |
| 彩色内参 | `/camera/color/camera_info` | 独立 RGB 内参 |
| 深度 | `/camera/depth/image_rect_raw` | `16UC1`，毫米，零为无效 |
| 深度内参 | `/camera/depth/camera_info` | 独立深度内参 |
| 彩色点云 | `/camera/depth/color/points` | 有序 `PointCloud2`，XYZ 米、RGB，`d435_depth_optical_frame` |
| 左/右红外 | `/camera/infra1/image_rect_raw`、`/camera/infra2/image_rect_raw` | 单通道图，各有 `camera_info` |

深度图没有预先对齐到 RGB。点云保留深度图的行列和完整深度视场，每个点转换到 RGB 相机后投影取色。超出彩色视场的点保留 XYZ，RGB 为黑；无效深度对应 NaN 坐标。

## 默认参数与调整

| 参数 | 默认值 |
| --- | --- |
| `d435_enabled` | `true` |
| `d435_x/y/z` | `0.32 / 0 / -0.03` 米 |
| `d435_roll/pitch/yaw` | `0 / 0 / 0` 弧度 |
| `d435_color_width/height` | `1280 / 720` |
| `d435_depth_width/height` | `848 / 480` |
| `d435_rate` | `30` Hz，仿真时间目标值 |
| `d435_color_hfov` | 约 `1.204277184` 弧度（69°） |
| `d435_depth_hfov` | 约 `1.518436449` 弧度（87°） |
| `d435_min_depth/max_depth` | `0.2 / 10` 米 |
| `d435_color_offset_y` | `0.015` 米 |
| `d435_stereo_baseline` | `0.050` 米 |

六个安装参数定义 `trunk` 到深度物理原点的固定变换。默认位置是模型前部下排估计值，尚未测量本机。相对深度相机，RGB 物理原点在 Y 正方向 15 mm，右红外在 Y 负方向 50 mm，取自[官方 D435 名义模型](https://github.com/IntelRealSense/realsense-ros/blob/debadfaaa4a21c667c076ad2abacf69f9427c825/realsense2_description/urdf/_d435.urdf.xacro)。光学帧均为 X 向右、Y 向下、Z 向前，物理帧为 X 向前、Y 向左、Z 向上。

调整安装位置，例如：

```bash
./scripts/ros1_sim.sh --rviz d435_x:=0.33 d435_z:=-0.025 d435_pitch:=0.1
```

降低负载，例如：

```bash
./scripts/ros1_sim.sh --rviz d435_color_width:=640 d435_color_height:=360 d435_depth_width:=424 d435_depth_height:=240 d435_rate:=15
```

宽高和视场决定理想针孔内参；调整宽高不会自动复现真实设备该分辨率的裁切模式。当前假定方形像素、主点在图像中心、畸变为零。若有实机 `CameraInfo`，应按该分辨率校准视场；型号相同不代表标定参数相同。

只运行原有雷达模式：

```bash
./scripts/ros1_sim.sh --headless d435_enabled:=false
```

`./scripts/super_lio_sim.sh` 同样接受这些参数；相机不会自动接入 Super-LIO。

## 无界面运行

`--headless` 关闭 Gazebo 窗口，相机仍需渲染。脚本优先复用宿主 X11/GPU；完全没有显示服务时使用 `xvfb-run` 与 Mesa 软件渲染。旧镜像缺少 `xvfb/xauth`，需运行 `./scripts/ros1_build_env.sh` 并重建所用 RViz/Super-LIO 派生镜像。缺依赖时会明确报错。

## 来源与仿真范围

采用 [PAL Robotics ROS1 插件](https://github.com/pal-robotics/realsense_gazebo_plugin/tree/49ab134fa517280bb8393e715173dc631ccc2557)，属于社区项目。固定原始源码保存在 `references/realsense_gazebo_plugin`，本地修改为 `ros1/patches/realsense_rgbd.patch`，构建时应用。修改修正红外覆盖彩色缓存、不同视场/尺寸下点云取色错误、内参和点云主点不一致等问题。

输出为场景渲染 RGB 与几何深度，深度经过毫米量化。左右红外为对应位置的灰度渲染，深度不是由这些图像做双目匹配得到。曝光、运动模糊、主动红外图案、距离噪声、双目孔洞、反光/透明材料失效均未复现。默认内外参为公开名义值，不能代替该台实机的标定。进一步逼近实机需要同一分辨率/帧率下的标定、深度误差和失效样本。

验收命令为 `./scripts/verification/d435_trial.sh`，用已知距离墙面和偏置彩色标记检查几何、颜色投影、光学 TF 及雷达/IMU 输出。日志位于 `logs/d435-trial-*`；结果见 [验收记录](reports/d435.md)。
