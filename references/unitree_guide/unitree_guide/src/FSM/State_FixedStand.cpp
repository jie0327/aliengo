/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include <iostream>
#include "FSM/State_FixedStand.h"

// 固定站立通过关节位置 PD，把进入状态时的实测姿态逐渐插值到头文件中的 _targetPos。
// 冒号后调用基类 FSMState 的构造函数，登记共享组件、状态枚举和显示名称。
State_FixedStand::State_FixedStand(CtrlComponents *ctrlComp)
                :FSMState(ctrlComp, FSMStateName::FIXEDSTAND, "fixed stand"){}

// enter 只在进入状态时执行：设置增益、清除旧前馈项，并记录本次插值的起点。
void State_FixedStand::enter(){
    // i 是腿号 0..3；每腿三关节的电机索引是 3*i+0/1/2，分别为 hip/thigh/calf。
    for(int i=0; i<4; i++){
        // 仿真与真机使用各自的站立 Kp/Kd；位置增益单位 N·m/rad，速度增益单位 N·m·s/rad。
        if(_ctrlComp->ctrlPlatform == CtrlPlatform::GAZEBO){
            _lowCmd->setSimStanceGain(i);
        }
        else if(_ctrlComp->ctrlPlatform == CtrlPlatform::REALROBOT){
            _lowCmd->setRealStanceGain(i);
        }
        // dq_des=0 rad/s，tau_ff=0 N·m；清除此前状态残留的速度/前馈力矩命令。
        // tau 字段置零仍允许 PD 产生力矩：tau_PD=Kp*(q_des-q)+Kd*(dq_des-dq)。
        _lowCmd->setZeroDq(i);
        _lowCmd->setZeroTau(i);
    }
    // 关节角 q 单位为 rad；从实测值起步，使切入站立时位置目标与当前关节位置衔接。
    for(int i=0; i<12; i++){
        _lowCmd->motorCmd[i].q = _lowState->motorState[i].q;
        _startPos[i] = _lowState->motorState[i].q;
    }
    // 请求步态发生器把四腿都标为计划支撑，下一轮 runWaveGen 再更新 contact/phase。
    _ctrlComp->setAllStance();
}

// run 每个控制周期调用一次，仅推进位置目标，沿用 enter 设置的零速度、零前馈和增益。
void State_FixedStand::run(){
    // _duration 在头文件中为 1000 个调用步数；每轮增加 1/1000，目标 500 Hz 时约需 2 s。
    // (float)1 强制使用浮点值；这里的进度按调用次数推进，实际时间随循环耗时变化。
    _percent += (float)1/_duration;
    // ?: 是三目运算符，把进度限制到最大 1；_percent 是 0..1 的比例而非 0..100。
    _percent = _percent > 1 ? 1 : _percent;
    for(int j=0; j<12; j++){
        // 线性插值 q_des=(1-p)*q_start+p*q_target；p=0 为起点，p=1 后保持目标站立角。
        _lowCmd->motorCmd[j].q = (1 - _percent)*_startPos[j] + _percent*_targetPos[j]; 
    }
}

// 离开时重置进度；下次进入会重新测量起点，再从 0 开始插值。
void State_FixedStand::exit(){
    _percent = 0;
}

// 根据离散用户命令返回下一状态名；此函数只提出切换请求，FSM 负责调用 exit/enter。
// 仿真键盘对应：1 被动、3 自由站立、4 小跑、0 平衡测试、9 摆腿测试、8 踏步测试。
FSMStateName State_FixedStand::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::L2_X){
        return FSMStateName::FREESTAND;
    }
    else if(_lowState->userCmd == UserCommand::START){
        return FSMStateName::TROTTING;
    }
    else if(_lowState->userCmd == UserCommand::L1_X){
        return FSMStateName::BALANCETEST;
    }
    else if(_lowState->userCmd == UserCommand::L1_A){
        return FSMStateName::SWINGTEST;
    }
    else if(_lowState->userCmd == UserCommand::L1_Y){
        return FSMStateName::STEPTEST;
    }
#ifdef COMPILE_WITH_MOVE_BASE
    else if(_lowState->userCmd == UserCommand::L2_Y){
        return FSMStateName::MOVE_BASE;
    }
#endif  // COMPILE_WITH_MOVE_BASE
    else{
        return FSMStateName::FIXEDSTAND;
    }
}