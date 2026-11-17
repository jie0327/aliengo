# 第 1 课：启动 AlienGo 仿真、观察反馈与站立

[课程目录](course_index_zh.md) · [下一课：只读关节节点](lesson_02_zh.md)

本课的目标是认出 12 个关节，观察仿真反馈，理解键盘操作与控制器的关系。详细启动、键盘说明和当前实验范围见 [ROS 1 仿真说明](ros1_sim_zh.md)。只操作仿真；此课程不连接真实机器。

## 1. 启动、站立、尝试移动

在项目目录构建一次：

```bash
cd /home/jack/aliengo_learning
./scripts/ros1_build.sh
```

在第一个终端运行 `./scripts/ros1_sim.sh`，另开终端运行 `./scripts/ros1_keyboard.sh` 并使其获得键盘焦点。按 `2` 站立，等待动作完成，再按 `4`；轻按两次 `w`，用空格停止。按键输入会累加并保持。动作异常时先停止实验并记录现象。

## 2. 从反馈认识四条腿

另开终端运行只读程序：

```bash
./scripts/ros1_course.sh sim lesson02/ros1_joint_state_reader.py --samples 3 --timeout 10
./scripts/ros1_course.sh sim lesson06/ros1_observe_state.py --samples 5 --timeout 10
```

每条腿包括 hip、thigh、calf 三个关节；FR、FL、RR、RL 分别是右前、左前、右后、左后。关节位置单位是弧度，角速度是 rad/s，转动关节 effort 的单位为 N·m。`JointState` 中名字与数值按索引对应，别假设不同消息的数组顺序相同。

ROS 1 仿真中的关节状态话题为 `/aliengo_gazebo/joint_states`，IMU 为 `/trunk_imu`。键盘由 Guide 进程直接读取，并由控制器生成关节指令。画面上有模型不等于控制器已工作；要同时观察状态与反馈。

## 3. 一条关节怎样跟随目标

位置 PD 和前馈力矩的一个常见形式是 `tau=Kp*(q_des-q)+Kd*(dq_des-dq)+tau_ff`。模型和控制器组织可以从 [joint_controller.cpp](../references/unitree_ros/unitree_legged_control/src/joint_controller.cpp) 与 [IOROS.cpp](../references/unitree_guide/unitree_guide/src/interface/IOROS.cpp) 找起。模型声明的限制不自动证明整个仿真或真实设备都按它执行保护。

## 验收问题

1. 四条腿如何用名称区分？一条腿的三个关节分别叫什么？
2. 为什么要依据关节名而不是数组位置提取角度？
3. 怎样区分模型已经显示、反馈正在更新以及控制器能稳定行走？
