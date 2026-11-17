# MID-360 真机驱动（ROS 1）

这里使用 Livox 官方 `livox_ros_driver2` 和 Livox-SDK2 接收 MID-360 点云和 IMU。驱动只负责传感器，不向 AlienGo 发送运动命令。真机、网卡、供电、IP 和安装外参尚未验证；下面的 IP 和位姿仅为示例，必须依据实际设备修改。仿真传感器与此驱动是独立链路。

## 构建

需有 Docker、Git、网络和项目已有的 `aliengo-learning:noetic` 镜像。首次运行：

```bash
./scripts/ros1_build_env.sh  # 若基础镜像还不存在
./scripts/mid360_build.sh
```

脚本将 [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2) 固定在 `c0796f04c143143899c87a773d9f6b7136453c0b`，将 [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) 固定在 `21445540f0d100dc86a7e6df312dd70bbdb4afdf`，构建独立的 `.mid360` 工作区和 `aliengo-learning:mid360` 镜像。脚本不会调用官方会清理工作区的 `build.sh`，不会安装到宿主机 `/usr/local`，也不改动 `.ros1`；已有 `.mid360` 时拒绝覆盖。构建使用项目共享互斥锁，先关闭由项目脚本启动的仿真或训练。重新构建前请自行备份或移走旧工作区。

## 配置网络

按设备手册确认雷达供电、线缆、实际 IP 和主机网卡地址。主机地址必须已配置在连接雷达的网卡上，与雷达网络可达且不同；不要直接把下面的示例当作设备实际地址。容器运行在宿主机网络中，使用官方协议 UDP 端口：雷达端 56100、56200、56300、56400、56500，主机端分别为 56101、56201、56301、56401、56501；确保对应网络和防火墙允许通信。

例如实际主机地址为 `192.168.1.5`、雷达地址为 `192.168.1.12` 时，生成配置：

```bash
python3 scripts/mid360_config.py .mid360/config.json 192.168.1.5 192.168.1.12
```

生成器不会覆盖已有文件；地址变更时使用新文件名，或备份后自行编辑。JSON 的坐标外参保留为零，防止与 ROS TF 重复应用安装外参。相关格式和端口见[官方配置](https://github.com/Livox-SDK/livox_ros_driver2/blob/21445540f0d100dc86a7e6df312dd70bbdb4afdf/config/MID360_config.json)。

## 启动与检查

```bash
./scripts/mid360_run.sh .mid360/config.json
```

默认将 `sensor_msgs/PointCloud2` 发布到 `/livox/lidar`，frame 为 `mid360_link`；IMU 发布到 `/livox/imu`。固定版本的官方驱动将 IMU 消息的 frame 硬编码为 `livox_frame`，不会使用点云的 `frame_id` 参数；因此需要单独处理该 frame，才能在 TF 中正确使用。点云默认频率 10 Hz。连接成功后，在同一 ROS master 的 ROS 1 终端中执行 `rostopic hz /livox/lidar`、`rostopic echo -n 1 /livox/imu`；也可在 RViz 添加 PointCloud2 并把 Fixed Frame 设置为 `mid360_link`。如果宿主机没有 ROS 1，可另开终端执行 `docker exec -it aliengo-mid360-driver bash`，在容器内 `source /opt/ros/noetic/setup.bash` 后运行上述 `rostopic` 命令。若终端运行在其他容器或机器上，须自行配置同一个 `ROS_MASTER_URI` 及可互相访问的 ROS 网络地址；已有 master 时，启动容器会使用传入的宿主环境默认网络配置，必要时以 Docker 的 ROS 网络变量配置启动。

该固定版本驱动的源码将 IMU 原始加速度按 **g** 直接复制到 `sensor_msgs/Imu.linear_acceleration`，而 ROS 消息约定该字段以 m/s² 表示；陀螺仪单位为 rad/s。将真机数据交给按照 ROS 单位读取的算法前，须先将加速度乘以标准重力加速度（约 9.80665），并正确处理 IMU 与点云坐标系和芯片位置。按 MID-360 产品手册，IMU 芯片相对点云原点为 `(0.011, 0.02329, -0.04412)` 米，坐标轴方向相同。当前驱动未自动执行单位转换；仿真 IMU 采用 ROS 的 m/s²，使用包含此偏移的 `mid360_imu_link`。实际设备仍需验证。源码见 [IMU 数据类型](https://github.com/Livox-SDK/livox_ros_driver2/blob/21445540f0d100dc86a7e6df312dd70bbdb4afdf/src/comm/lidar_imu_data_queue.h)及[消息赋值](https://github.com/Livox-SDK/livox_ros_driver2/blob/21445540f0d100dc86a7e6df312dd70bbdb4afdf/src/lddc.cpp)。

点云位于雷达坐标系中。需要与机器人坐标系关联时，优先由包含 MID-360 安装关节的机器人模型及 `robot_state_publisher` 发布 `trunk -> mid360_link`。若没有该 TF，可让此驱动临时发布：

```bash
./scripts/mid360_run.sh .mid360/config.json publish_static_tf:=true x:=0 y:=0 z:=0 yaw:=0 pitch:=0 roll:=0
```

上面全零只是占位；应测量雷达坐标系相对 `trunk` 的平移（米）与姿态（弧度），替换所有数值。照片无法精确确定外参或传感器轴向。TF 发布默认关闭，已有相同父子关系时不要启用，以免重复发布。`parent_frame:=...` 可改父坐标系；`publish_freq:=...` 可调整点云发布频率。驱动容器使用宿主网络；如没有发现设备，先核实实际 IP、网卡、供电、端口与连接状态，再查驱动日志。此流程未经本台雷达实测，不能据此推断已完成硬件连接或外参标定。

## 当前验证范围

已在本项目 Noetic 镜像上完成固定版本 SDK2 和驱动编译，并在独立网络容器中用回环测试配置启动驱动，确认 SDK 初始化和配置解析。测试不含实体雷达，没有验证收到点云、IMU、外参或真机网络；离线测试中的中断退出未及时完成，最终停止了测试容器，真机退出行为仍需验证。
