# RViz 世界坐标运动同步修复

**Goal:** Gazebo 中机器人平移、转向和机身姿态在 RViz 中一致显示，保持键盘控制和 MID360 数据。

**Evidence:** 当前 RViz 固定系为 base，TF 仅有 base→trunk→传感器/四肢。现场查询 world→base 不存在；Gazebo 模型位姿与 base link 位姿完全一致。Guide 的 odom 发布分支未编译。

**Design:** 独立 ROS1 节点按仿真时间 30Hz 查询一个 Gazebo 模型，使用响应时间戳发布动态 world→base 和明确命名的 /sim/ground_truth/odom。RViz 固定世界系，相机只在启动时对准出生点。真值通路可关闭，供将来估计器/SLAM 接管；不改控制器，不伪造 map TF。

- [x] 写真实 ROS 消息回归，验证位姿、时间戳、绝对高度和速度的坐标变换；先确认失败。
- [x] 接入 Gazebo 真值节点和 RViz 出生点视角，默认 Fixed Frame=world。
- [x] 实时加载到正在运行的容器，读取 TF 与 Gazebo 位姿进行对比。
- [x] 扩展键盘回归，验证 world→base 实际位移、点云时间戳变换及 RViz 固定系；串行限资源实测。
- [x] 更新说明、记录结果并完成独立审查。日志 logs/keyboard-trial-8vc6uP，结果 docs/reports/rviz-world-motion.md。
