# AlienGo + ROS 1 课程目录

主线切换为 ROS 1 Noetic 与 Gazebo 11。先看 [ROS 1 仿真说明](ros1_sim_zh.md)，按独立仿真入口构建、站立和操作键盘。各课的操作入口和只读观测已对应 ROS 1 环境。

| 课次 | 内容 | 当前状态 |
|---|---|---|
| 1 | [启动、观察与站立](lesson_01_zh.md) | ROS 1 仿真与键盘入口 |
| 2 | [关节状态与节点](lesson_02_zh.md) | ROS 1 只读观测节点 |
| 3 | [关节 PD 与离散步长](lesson_03_zh.md) | CPU 离线实验与 ROS 1 控制源码 |
| 4 | [坐标系、URDF FK 与教学 IK](lesson_04_zh.md) | ROS 1 官方模型与离线 FK/IK |
| 5 | [状态机与步态](lesson_05_zh.md) | ROS 1 Guide 与离线步态实验 |
| 6 | [IMU、时间与状态估计](lesson_06_zh.md) | ROS 1 IMU/关节只读观测与离线实验 |
| 7 | [支撑力、力矩与预测控制](lesson_07_zh.md) | CPU 离线实验与 Guide 源码 |
| 8 | [SDK 映射与离线接口](lesson_08_zh.md) | 离线接口练习与 SDK 参考 |
| 9 | [强化学习与速度跟踪](lesson_09_zh.md) | CPU 离线学习实验，可用 ROS 1 教学镜像 |
| 10 | [GAE、PPO 与策略梯度](lesson_10_zh.md) | CPU 离线数值实验，可用 ROS 1 教学镜像 |
| 11 | [AlienGo 训练任务](lesson_11_zh.md) | 阅读训练源码；完整训练环境需另行准备 |
| 12 | [策略部署接口](lesson_12_zh.md) | 离线契约实验与只读策略推理；策略闭环尚未稳定 |

第 9、10、12 课先在工作区执行：

```bash
cd /home/jack/aliengo_learning
./scripts/ros1_course.sh offline lesson09/q_learning_speed.py
./scripts/ros1_course.sh offline lesson10/ppo_numbers.py
./scripts/ros1_course.sh offline lesson12/offline_policy_contract.py --demo
```

这些程序运行在网络隔离的容器内，只使用 Python 标准库；具体实验和预期输出见各课正文。训练参考版本见 [强化学习资料](rl_sources_zh.md)。
