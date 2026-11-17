/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/FSM.h"
#include <iostream>

// FSM 是有限状态机：每个 State 封装一种控制行为，共用 ctrlComp 中的命令和反馈。
// FSM:: 是类作用域；冒号后的 _ctrlComp(ctrlComp) 是构造函数成员初始化列表。
FSM::FSM(CtrlComponents *ctrlComp)
    :_ctrlComp(ctrlComp){

    // nullptr 代表空指针；其他状态只在这里各创建一次，切换时改指针而非重新分配对象。
    _stateList.invalid = nullptr;
    _stateList.passive = new State_Passive(_ctrlComp);
    _stateList.fixedStand = new State_FixedStand(_ctrlComp);
    _stateList.freeStand = new State_FreeStand(_ctrlComp);
    _stateList.trotting = new State_Trotting(_ctrlComp);
    _stateList.balanceTest = new State_BalanceTest(_ctrlComp);
    _stateList.swingTest = new State_SwingTest(_ctrlComp);
    _stateList.stepTest = new State_StepTest(_ctrlComp);
#ifdef COMPILE_WITH_MOVE_BASE
    _stateList.moveBase = new State_move_base(_ctrlComp);
#endif  // COMPILE_WITH_MOVE_BASE
    initialize();
}

// ~FSM 是析构函数；销毁 FSM 时释放状态列表中由 new 创建的对象。
FSM::~FSM(){
    _stateList.deletePtr();
}

// 初始化就调用 passive.enter()，先准备阻尼模式的初始 lowCmd，再进入正常循环。
void FSM::initialize(){
    _currentState = _stateList.passive;
    _currentState -> enter();
    _nextState = _currentState;
    _mode = FSMMode::NORMAL;
}

// 每次调用处理一个控制周期。lowState 是本轮输入，lowCmd 是留给下一次 sendRecv 的输出。
void FSM::run(){
    // getSystemTime 返回系统墙钟的微秒时间戳；包含后续通信、估计、计算的全部耗时。
    _startTime = getSystemTime();
    // 顺序很关键：先发送上一轮状态计算好的 lowCmd（首轮是 enter 准备的命令），再收反馈。
    _ctrlComp->sendRecv();
    // 根据此前设置的步态模式生成计划接触 contact 和相位 phase；不是直接测得的触地状态。
    _ctrlComp->runWaveGen();
    // 用刚收到的关节/IMU 数据以及计划接触更新机身位置、速度等估计，供下方状态使用。
    _ctrlComp->estimator->run();
    if(!checkSafty()){
        // setPassive 改的是输入面板命令；不会在这里直接重写已发送的电机命令。
        // ROS 接口下一次 sendRecv 才把面板命令复制到 lowState，再由状态切换逻辑处理。
        _ctrlComp->ioInter->setPassive();
    }

    if(_mode == FSMMode::NORMAL){
        // 先让当前状态写出 lowCmd，再检查用户命令是否请求切换，因此命令在下一轮才发送。
        _currentState->run();
        _nextStateName = _currentState->checkChange();
        if(_nextStateName != _currentState->_stateName){
            // 本轮只记录目标并打印信息；exit/enter 实际要到下一轮 CHANGE 分支执行。
            _mode = FSMMode::CHANGE;
            _nextState = getNextState(_nextStateName);
            std::cout << "Switched from " << _currentState->_stateNameString
                      << " to " << _nextState->_stateNameString << std::endl;
        }
    }
    else if(_mode == FSMMode::CHANGE){
        // 生命周期顺序：旧状态退出 -> 更新当前指针 -> 新状态初始化 -> 新状态运行一次。
        // 本轮顶部仍已发送旧命令；此处生成的新命令会由再下一次 sendRecv 发送。
        _currentState->exit();
        _currentState = _nextState;
        _currentState->enter();
        _mode = FSMMode::NORMAL;
        _currentState->run();
    }

    // dt 的秒乘 10^6 转为微秒，(long long) 显式转换为整数时间长度。
    // 等到距本轮起点达到 dt；若工作已超时就不再补等，所以 500 Hz 是目标墙钟频率。
    absoluteWait(_startTime, (long long)(_ctrlComp->dt * 1000000));
}

// 把状态名枚举映射为已创建的状态对象地址；switch/case 按枚举值选择分支。
// return 会立即离开函数；default/INVALID 返回空指针表示没有有效状态对象。
FSMState* FSM::getNextState(FSMStateName stateName){
    switch (stateName)
    {
    case FSMStateName::INVALID:
        return _stateList.invalid;
        break;
    case FSMStateName::PASSIVE:
        return _stateList.passive;
        break;
    case FSMStateName::FIXEDSTAND:
        return _stateList.fixedStand;
        break;
    case FSMStateName::FREESTAND:
        return _stateList.freeStand;
        break;
    case FSMStateName::TROTTING:
        return _stateList.trotting;
        break;
    case FSMStateName::BALANCETEST:
        return _stateList.balanceTest;
        break;
    case FSMStateName::SWINGTEST:
        return _stateList.swingTest;
        break;
    case FSMStateName::STEPTEST:
        return _stateList.stepTest;
        break;
#ifdef COMPILE_WITH_MOVE_BASE
    case FSMStateName::MOVE_BASE:
        return _stateList.moveBase;
        break;
#endif  // COMPILE_WITH_MOVE_BASE
    default:
        return _stateList.invalid;
        break;
    }
}

// 检查机身是否倾斜过大；保留上游函数名 checkSafty，返回 true 表示通过此项检查。
bool FSM::checkSafty(){
    // The angle with z axis less than 60 degree
    // R 把机身坐标转到世界坐标，R(2,2) 是两个 z 轴夹角的余弦；矩阵下标从 0 开始。
    // cos(60°)=0.5，低于该值说明夹角超过 60°；这里检查总倾斜，不能等同于单独的俯仰角。
    if(_ctrlComp->lowState->getRotMat()(2,2) < 0.5 ){
        return false;
    }else{
        return true;
    }
}