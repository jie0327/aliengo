/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sched.h>

#include "control/ControlFrame.h"
#include "control/CtrlComponents.h"
#include "Gait/WaveGenerator.h"
#include "control/BalanceCtrl.h"

// #ifdef 是编译期选择：只有启用相应宏，预处理器才把其中的代码交给 C++ 编译器。
// 因而仿真的 ROS 接口与真机的 SDK 接口由构建配置选择，不是在运行时自动识别机器人。
#ifdef COMPILE_WITH_REAL_ROBOT
#include "interface/IOSDK.h"
#endif // COMPILE_WITH_REAL_ROBOT

#ifdef COMPILE_WITH_ROS
#include "interface/KeyBoard.h"
#include "interface/IOROS.h"
#endif // COMPILE_WITH_ROS

// 主循环的退出标志；CtrlComponents 保存它的地址，因此其他组件也能访问同一变量。
bool running = true;

// over watch the ctrl+c command
// SIGINT 通常来自终端 Ctrl+C；回调把标志改为 false，让 main 的 while 循环结束。
void ShutDown(int sig)
{
    std::cout << "stop the controller" << std::endl;
    running = false;
}

// 尝试为当前进程设置 FIFO 实时调度和该策略的最高优先级，以减少被其他进程打断的机会。
// 这需要系统授权；失败只打印原有错误信息并继续，成功也不等于保证每轮准时完成。
void setProcessScheduler()
{
    pid_t pid = getpid();
    sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1)
    {
        std::cout << "[ERROR] Function setProcessScheduler failed." << std::endl;
    }
}

// 程序入口：argc/argv 接收命令行参数；依次组装通信、模型、步态、估计和状态机。
int main(int argc, char **argv)
{
    /* set real-time process */
    setProcessScheduler();
    /* set the print format */
    std::cout << std::fixed << std::setprecision(3);

#ifdef RUN_ROS
    // 初始化 ROS 节点；节点名用于 ROS 通信标识，关节话题中的机型来自 IOROS 读取的参数。
    ros::init(argc, argv, "unitree_gazebo_servo");
#endif // RUN_ROS

    // * 声明指针，保存对象地址；用共同的 IOInterface 接口调用各通信实现的 sendRecv。
    IOInterface *ioInter;
    CtrlPlatform ctrlPlat;

#ifdef COMPILE_WITH_SIMULATION
    // new 在堆上构造对象，返回地址；IOROS 负责 Gazebo 的 ROS 关节命令和反馈。
    ioInter = new IOROS();
    ctrlPlat = CtrlPlatform::GAZEBO;
#endif // COMPILE_WITH_SIMULATION

#ifdef COMPILE_WITH_REAL_ROBOT
    ioInter = new IOSDK();
    ctrlPlat = CtrlPlatform::REALROBOT;
#endif // COMPILE_WITH_REAL_ROBOT

    // CtrlComponents 集中保存各组件和共享的 lowCmd/lowState，状态机通过它们交换数据。
    // -> 表示通过指针访问成员；例如 ctrlComp->dt 是这个组件对象的 dt 字段。
    CtrlComponents *ctrlComp = new CtrlComponents(ioInter);
    ctrlComp->ctrlPlatform = ctrlPlat;
    // dt 单位为秒，0.002 s 对应目标墙钟循环频率 1/dt = 500 Hz，也供估计和控制计算使用。
    // FSM 用系统时间等待补足周期；它没有在此锁定 Gazebo 仿真步长，实际频率会受耗时影响。
    ctrlComp->dt = 0.002; // run at 500hz
    // & 取得 running 的地址，使组件持有的是同一退出标志，而非 bool 值的副本。
    ctrlComp->running = &running;

#ifdef ROBOT_TYPE_A1
    ctrlComp->robotModel = new A1Robot();
#endif
#ifdef ROBOT_TYPE_Go1
    // 上游这里创建 Go1Robot，原类本身不是原生 AlienGo 控制模型。
    // 本项目在 .ros1 的实际工作副本中用 ros1/aliengo-guide.patch 替换其几何/惯性参数适配 AlienGo。
    ctrlComp->robotModel = new Go1Robot();
#endif

    // 步态参数依次为周期 0.45 s、支撑时间占比 0.5、四腿的归一化相位偏置。
    // 腿序 FR/FL/RR/RL = 右前/左前/右后/左后；相同偏置的对角腿同步，形成 Trot。
    ctrlComp->waveGen = new WaveGenerator(0.45, 0.5, Vec4(0, 0.5, 0.5, 0)); // Trot
    // ctrlComp->waveGen = new WaveGenerator(1.1, 0.75, Vec4(0, 0.25, 0.5, 0.75));  //Crawl, only for sim
    // ctrlComp->waveGen = new WaveGenerator(0.4, 0.6, Vec4(0, 0.5, 0.5, 0));  //Walking Trot, only for sim
    // ctrlComp->waveGen = new WaveGenerator(0.4, 0.35, Vec4(0, 0.5, 0.5, 0));  //Running Trot, only for sim
    // ctrlComp->waveGen = new WaveGenerator(0.4, 0.7, Vec4(0, 0, 0, 0));  //Pronk, only for sim

    // 此时模型、dt、步态等已就绪，geneObj 再创建依赖它们的状态估计器和力分配控制器。
    ctrlComp->geneObj();

    // 普通局部对象 ctrlFrame 在栈上构造；其内部建立 FSM，并从 passive 状态开始。
    ControlFrame ctrlFrame(ctrlComp);

    // 注册 main 的 Ctrl+C 处理函数；它会替换此前 IOROS 对同一 SIGINT 的注册。
    signal(SIGINT, ShutDown);

    // 一次 run 对应一轮 FSM：发已有命令、收反馈、更新步态/估计、计算下一轮命令、等待。
    while (running)
    {
        ctrlFrame.run();
    }

    // delete 触发 CtrlComponents 析构，释放它所管理的通信、模型、估计等堆对象。
    delete ctrlComp;
    return 0;
}
