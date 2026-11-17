# 强化学习课程验证记录

第9～12课保留 CPU 教学实验、训练源码阅读与部署契约。2026-11-12，使用 ROS 1 Noetic 教学镜像验证了第9课短训练、第10课 PPO 数值实验，以及第12课假动作契约演示；第12课输出 `demo_passed=true`，含45维观测和135维三帧历史。

2026-11-11 原教学模型实验曾记录默认 Q-learning 训练与留出评估的平均速度绝对误差从 0.41141 降到 0.05245 m/s；这些数据来自离散教学环境，不包含机器人刚体与足地接触。第10课的 GAE/PPO 是公式实验，没有神经网络或优化器。详细历史输出见 [基础实验](reports/rl-foundations.md) 和 [部署契约实验](reports/rl-deployment.md)。

训练参考保存在 [repositories.lock.json](../repositories.lock.json) 指定版本；AlienGo 平地任务包含48维观测、12维动作，粗糙地形任务为235维。课程没有运行完整 PPO 训练，也没有验证策略权重来源、策略在 Gazebo 的闭环表现或真实设备控制。默认仿真运行 Guide，不会自动启用强化学习。

GPU 驱动安装和环境证据见 [驱动安装验收](driver_installation_2026-11-12_zh.md) 与 [训练环境准备](rl_environment_zh.md)。训练框架需要独立准备并核验兼容性，不能把驱动安装当成训练成功。
