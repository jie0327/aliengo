# RViz 世界坐标运动同步

## 原因与改动

修复前 RViz 使用 `base` 固定系，现场 TF 只有 `base → trunk → 雷达/四肢`，没有 `world → base`。Gazebo 中模型虽已移动，RViz 仍把机身当作显示原点，因而只看到腿在动。把 Fixed Frame 字符串换成 world 也无法单独解决缺少动态位姿的问题。

新增 [Gazebo 真值节点](../../ros1/mid360/scripts/gazebo_pose_tf.py)，以仿真时间 30Hz 查询单个模型。保留服务应答时间戳和绝对世界高度，发布动态 `world → base` 及 `/sim/ground_truth/odom`。odom 的 pose 在 world，twist 用姿态逆旋转到 base。没有修改键盘、步态控制或雷达扫描器。

现场比对已确认 Gazebo 模型原点就是 URDF 的 base，base→trunk 为单位变换。RViz 使用 world 固定系，Orbit 的 Target Frame 保持 `<Fixed Frame>`；[启动前缀](../../ros1/mid360/rviz_spawn_view.py) 只在启动时将焦点和固定网格放到出生点，保留 Depot/City 不同位置与高度，并完整保留 ROS remap。

## 完整运行验证

串行运行一个 4CPU / 4GiB 容器，包含 Gazebo GUI、RViz、MID360 与真实键盘控制。按 2、4、w、w、空格完成前进停车，并测试暂停退出。记录目录为 `logs/keyboard-trial-8vc6uP`；[完整结果](rviz-world-motion.json) 和 [从运行中 RViz 导出的配置](rviz-world-runtime.rviz)。

| 检查 | 结果 |
|---|---|
| 结果 | PASS |
| 前进时间 | 20.027 仿真秒 |
| Gazebo 前进位移 | 0.962645 m |
| world→base TF 前进位移 | 0.962609 m |
| 两者位移差 | 0.036 mm |
| 243 个行走样本最大三维位置差 | 0.162 mm |
| RViz 实际 Fixed Frame | world |
| RViz 实际视角 | Orbit / Target Frame=<Fixed Frame> |
| 点云时刻的 world←mid360_link | 查询成功，点云与变换时间均为 36.2058s |
| 停车后漂移 | 4.56 mm / 5 仿真秒 |
| 实时因子 | 0.786 |
| 雷达 / IMU | 10.04 / 199.78 Hz，按仿真时间 |

测试按每次 Gazebo 测量的时间戳查询 TF，并对比同一测量的世界坐标；点云查询使用原始采集时间，没有用 Time(0) 替代。还通过 RViz 的 save_config 服务检查了运行中的视角，而非只读模板。

真实 ROS 消息测试覆盖时间戳、平移/姿态、城市绝对高度、偏航/横滚后的线速度和角速度、原消息不被修改。宿主测试中这 3 项因没有 ROS 而跳过，已在 Noetic 容器内单独运行通过；另有 4 项出生点视角与真实 exec 参数传递回归。新增代码经过独立静态审查。

## 使用范围

当前 world→base 来自明确标注的 Gazebo 仿真真值，用于可视化和算法对照。以后由估计器或 SLAM 发布到 base 的 TF 时，用 `sim_ground_truth_tf:=false` 关闭该真值通路，并选择系统实际提供的 odom/map 固定系。没有实现 SLAM、FAST-LIO 或 GBPlanner。
