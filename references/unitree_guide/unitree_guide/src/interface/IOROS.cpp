/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
// 仅当构建启用 ROS 时编译这套接口；算法内部仍使用 LowlevelCmd/LowlevelState。
#ifdef COMPILE_WITH_ROS

#include "interface/IOROS.h"
#include "interface/KeyBoard.h"
#include <iostream>
#include <unistd.h>
#include <csignal>

// ROS 接口的 SIGINT 回调；main 在接口构造后会再次注册自己的 ShutDown 处理函数。
void RosShutDown(int sig){
	ROS_INFO("ROS interface shutting down!");
	ros::shutdown();
}

// 构造 ROS 通信层：读取机型话题前缀、建立反馈订阅/命令发布，并启动键盘输入面板。
// :IOInterface() 调用基类构造函数；_robot_name 取自 /robot_name，例如 aliengo。
IOROS::IOROS():IOInterface(){
    std::cout << "The control interface for ROS Gazebo simulation" << std::endl;
    ros::param::get("/robot_name", _robot_name);
    std::cout << "robot_name: " << _robot_name << std::endl;

    // start subscriber
    initRecv();
    // 临时开启一个 ROS 回调线程等待首批状态；subSpinner 是局部对象，构造函数结束时销毁。
    // 后续控制周期还会在 sendCmd 的 ros::spinOnce() 中处理收到的回调。
    ros::AsyncSpinner subSpinner(1); // one threads
    subSpinner.start();
    usleep(300000);     //wait for subscribers start
    // initialize publisher
    initSend();   

    signal(SIGINT, RosShutDown);

    cmdPanel = new KeyBoard();
}

// 析构函数负责结束键盘输入，并请求关闭 ROS 通信。
IOROS::~IOROS(){
    delete cmdPanel;
    ros::shutdown();
}

// 桥接一次控制通信：先发 cmd，再把回调缓存复制到 state，最后附加键盘输入。
// const LowlevelCmd* 表示本函数不能通过 cmd 修改源命令；state 指向可写的共享反馈对象。
void IOROS::sendRecv(const LowlevelCmd *cmd, LowlevelState *state){
    sendCmd(cmd);
    recvState(state);

    state->userCmd = cmdPanel->getUserCmd();
    state->userValue = cmdPanel->getUserValue();
}

// 把算法内部的 12 个关节命令逐字段拷贝到 ROS 消息，再分别发布到 12 个关节话题。
// q: rad；dq: rad/s；tau: N·m 前馈项；Kp: N·m/rad；Kd: N·m·s/rad；mode 为控制模式码。
// 此处仅传输这些字段；位置/速度误差与 PD、前馈叠加由下游关节控制器完成。
void IOROS::sendCmd(const LowlevelCmd *lowCmd){
    // int i(0) 是把整数 i 初始化为 0 的另一种 C++ 写法；++i 让索引每轮增加 1。
    for(int i(0); i < 12; ++i){
        _lowCmd.motorCmd[i].mode = lowCmd->motorCmd[i].mode;
        _lowCmd.motorCmd[i].q = lowCmd->motorCmd[i].q;
        _lowCmd.motorCmd[i].dq = lowCmd->motorCmd[i].dq;
        _lowCmd.motorCmd[i].tau = lowCmd->motorCmd[i].tau;
        _lowCmd.motorCmd[i].Kd = lowCmd->motorCmd[i].Kd;
        _lowCmd.motorCmd[i].Kp = lowCmd->motorCmd[i].Kp;
    }
    // _lowCmd 虽是 LowCmd 容器，这里发布的是每关节 MotorCmd，而非整包 LowCmd 话题。
    for(int m(0); m < 12; ++m){
        _servo_pub[m].publish(_lowCmd.motorCmd[m]);
    }
    // 处理已排队的 ROS 回调以更新 _lowState；不会等待 12 个关节与 IMU 的时间戳对齐。
    ros::spinOnce();
}

// 将最近一次收到的缓存值复制给算法。缓存由各话题独立更新，不代表同一时刻的同步快照。
// q/dq/ddq 分别为 rad、rad/s、rad/s²；tauEst 为 N·m，是反馈力矩字段，不是命令 tau。
void IOROS::recvState(LowlevelState *state){
    for(int i(0); i < 12; ++i){
        state->motorState[i].q = _lowState.motorState[i].q;
        state->motorState[i].dq = _lowState.motorState[i].dq;
        state->motorState[i].ddq = _lowState.motorState[i].ddq;
        state->motorState[i].tauEst = _lowState.motorState[i].tauEst;
    }
    // 三轴加速度单位 m/s²、角速度单位 rad/s；四元数无量纲，需要复制四个元素。
    // 本接口只搬运 IMU 数值，不在这里旋转坐标或扣除重力；这些计算需看估计器。
    for(int i(0); i < 3; ++i){
        state->imu.quaternion[i] = _lowState.imu.quaternion[i];
        state->imu.accelerometer[i] = _lowState.imu.accelerometer[i];
        state->imu.gyroscope[i] = _lowState.imu.gyroscope[i];
    }
    // 上面的循环只覆盖下标 0..2，因此单独补上四元数第 4 个分量 z。
    state->imu.quaternion[3] = _lowState.imu.quaternion[3];
}

// 建立发布者：advertise<消息类型>(话题名,队列长度)；队列长度 1，避免长期积压旧命令。
// 固定电机顺序：FR[0..2]、FL[3..5]、RR[6..8]、RL[9..11]，每腿 hip/thigh/calf。
// FR/FL/RR/RL 分别是右前/左前/右后/左后；字符串拼接得到所选机器人命名空间下的话题。
void IOROS::initSend(){
    _servo_pub[0] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/FR_hip_controller/command", 1);
    _servo_pub[1] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/FR_thigh_controller/command", 1);
    _servo_pub[2] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/FR_calf_controller/command", 1);
    _servo_pub[3] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/FL_hip_controller/command", 1);
    _servo_pub[4] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/FL_thigh_controller/command", 1);
    _servo_pub[5] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/FL_calf_controller/command", 1);
    _servo_pub[6] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/RR_hip_controller/command", 1);
    _servo_pub[7] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/RR_thigh_controller/command", 1);
    _servo_pub[8] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/RR_calf_controller/command", 1);
    _servo_pub[9] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/RL_hip_controller/command", 1);
    _servo_pub[10] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/RL_thigh_controller/command", 1);
    _servo_pub[11] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/RL_calf_controller/command", 1);
}

// 建立状态订阅，与发布端保持相同电机索引；IMU 单独订阅 /trunk_imu。
// &IOROS::函数名 是成员函数指针，this 告诉 ROS 在当前对象上调用它，消息到来时填缓存。
void IOROS::initRecv(){
    _imu_sub = _nm.subscribe("/trunk_imu", 1, &IOROS::imuCallback, this);
    _servo_sub[0] = _nm.subscribe("/" + _robot_name + "_gazebo/FR_hip_controller/state", 1, &IOROS::FRhipCallback, this);
    _servo_sub[1] = _nm.subscribe("/" + _robot_name + "_gazebo/FR_thigh_controller/state", 1, &IOROS::FRthighCallback, this);
    _servo_sub[2] = _nm.subscribe("/" + _robot_name + "_gazebo/FR_calf_controller/state", 1, &IOROS::FRcalfCallback, this);
    _servo_sub[3] = _nm.subscribe("/" + _robot_name + "_gazebo/FL_hip_controller/state", 1, &IOROS::FLhipCallback, this);
    _servo_sub[4] = _nm.subscribe("/" + _robot_name + "_gazebo/FL_thigh_controller/state", 1, &IOROS::FLthighCallback, this);
    _servo_sub[5] = _nm.subscribe("/" + _robot_name + "_gazebo/FL_calf_controller/state", 1, &IOROS::FLcalfCallback, this);
    _servo_sub[6] = _nm.subscribe("/" + _robot_name + "_gazebo/RR_hip_controller/state", 1, &IOROS::RRhipCallback, this);
    _servo_sub[7] = _nm.subscribe("/" + _robot_name + "_gazebo/RR_thigh_controller/state", 1, &IOROS::RRthighCallback, this);
    _servo_sub[8] = _nm.subscribe("/" + _robot_name + "_gazebo/RR_calf_controller/state", 1, &IOROS::RRcalfCallback, this);
    _servo_sub[9] = _nm.subscribe("/" + _robot_name + "_gazebo/RL_hip_controller/state", 1, &IOROS::RLhipCallback, this);
    _servo_sub[10] = _nm.subscribe("/" + _robot_name + "_gazebo/RL_thigh_controller/state", 1, &IOROS::RLthighCallback, this);
    _servo_sub[11] = _nm.subscribe("/" + _robot_name + "_gazebo/RL_calf_controller/state", 1, &IOROS::RLcalfCallback, this);
}

// const T& 是只读引用：直接查看 ROS 传来的消息，避免按值复制整个消息对象。
// ROS orientation 的字段习惯按 x/y/z/w 排列；Guide 数组明确要求 w/x/y/z，须按名重排。
void IOROS::imuCallback(const sensor_msgs::Imu & msg)
{ 
    _lowState.imu.quaternion[0] = msg.orientation.w;
    _lowState.imu.quaternion[1] = msg.orientation.x;
    _lowState.imu.quaternion[2] = msg.orientation.y;
    _lowState.imu.quaternion[3] = msg.orientation.z;

    // 保留 IMU 的 x/y/z 轴顺序：角速度直接取 angular_velocity，单位 rad/s。
    _lowState.imu.gyroscope[0] = msg.angular_velocity.x;
    _lowState.imu.gyroscope[1] = msg.angular_velocity.y;
    _lowState.imu.gyroscope[2] = msg.angular_velocity.z;
    
    // 加速度直接取 linear_acceleration，单位 m/s²；是否含重力由上游传感器定义决定。
    _lowState.imu.accelerometer[0] = msg.linear_acceleration.x;
    _lowState.imu.accelerometer[1] = msg.linear_acceleration.y;
    _lowState.imu.accelerometer[2] = msg.linear_acceleration.z;
}

// 下列 12 个回调采用相同字段映射，只是写入的电机索引不同；本层不估算 ddq。
// 每个回调保存 mode/q/dq/tauEst，供下一次 recvState 读取，接收消息本身不会直接控制关节。
// 电机 0：右前腿 hip（髋部外展/内收关节）。
void IOROS::FRhipCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[0].mode = msg.mode;
    _lowState.motorState[0].q = msg.q;
    _lowState.motorState[0].dq = msg.dq;
    _lowState.motorState[0].tauEst = msg.tauEst;
}

// 电机 1：右前腿 thigh（大腿关节）。
void IOROS::FRthighCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[1].mode = msg.mode;
    _lowState.motorState[1].q = msg.q;
    _lowState.motorState[1].dq = msg.dq;
    _lowState.motorState[1].tauEst = msg.tauEst;
}

// 电机 2：右前腿 calf（小腿/膝关节）。
void IOROS::FRcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[2].mode = msg.mode;
    _lowState.motorState[2].q = msg.q;
    _lowState.motorState[2].dq = msg.dq;
    _lowState.motorState[2].tauEst = msg.tauEst;
}

// 电机 3：左前腿 hip。
void IOROS::FLhipCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[3].mode = msg.mode;
    _lowState.motorState[3].q = msg.q;
    _lowState.motorState[3].dq = msg.dq;
    _lowState.motorState[3].tauEst = msg.tauEst;
}

// 电机 4：左前腿 thigh。
void IOROS::FLthighCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[4].mode = msg.mode;
    _lowState.motorState[4].q = msg.q;
    _lowState.motorState[4].dq = msg.dq;
    _lowState.motorState[4].tauEst = msg.tauEst;
}

// 电机 5：左前腿 calf。
void IOROS::FLcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[5].mode = msg.mode;
    _lowState.motorState[5].q = msg.q;
    _lowState.motorState[5].dq = msg.dq;
    _lowState.motorState[5].tauEst = msg.tauEst;
}

// 电机 6：右后腿 hip。
void IOROS::RRhipCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[6].mode = msg.mode;
    _lowState.motorState[6].q = msg.q;
    _lowState.motorState[6].dq = msg.dq;
    _lowState.motorState[6].tauEst = msg.tauEst;
}

// 电机 7：右后腿 thigh。
void IOROS::RRthighCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[7].mode = msg.mode;
    _lowState.motorState[7].q = msg.q;
    _lowState.motorState[7].dq = msg.dq;
    _lowState.motorState[7].tauEst = msg.tauEst;
}

// 电机 8：右后腿 calf。
void IOROS::RRcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[8].mode = msg.mode;
    _lowState.motorState[8].q = msg.q;
    _lowState.motorState[8].dq = msg.dq;
    _lowState.motorState[8].tauEst = msg.tauEst;
}

// 电机 9：左后腿 hip。
void IOROS::RLhipCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[9].mode = msg.mode;
    _lowState.motorState[9].q = msg.q;
    _lowState.motorState[9].dq = msg.dq;
    _lowState.motorState[9].tauEst = msg.tauEst;
}

// 电机 10：左后腿 thigh。
void IOROS::RLthighCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[10].mode = msg.mode;
    _lowState.motorState[10].q = msg.q;
    _lowState.motorState[10].dq = msg.dq;
    _lowState.motorState[10].tauEst = msg.tauEst;
}

// 电机 11：左后腿 calf。
void IOROS::RLcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[11].mode = msg.mode;
    _lowState.motorState[11].q = msg.q;
    _lowState.motorState[11].dq = msg.dq;
    _lowState.motorState[11].tauEst = msg.tauEst;
}

#endif  // COMPILE_WITH_ROS