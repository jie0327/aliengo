/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "Gait/WaveGenerator.h"
#include <iostream>
#include <sys/time.h>
#include <math.h>

// 波形器只按时间安排4条腿何时支撑/摆动，不读取足底力、碰撞或触地传感器。
// contact为4x1整数计划标志（1支撑、0摆动）；phase为4x1无量纲相位，表示当前阶段的进度。
// 腿索引0/1/2/3依次为FR右前、FL左前、RR右后、RL左后。
// period为完整步态周期s，stancePhaseRatio为支撑时间占比，bias为各腿相对周期的延迟比例。
// 例如main中的bias=(0,0.5,0.5,0)，使FR/RL同相、FL/RR同相，两组对角腿相差半周期。
// 冒号后是成员初始化列表；构造时把传入的周期、支撑占比和4维偏移保存到对象中。
WaveGenerator::WaveGenerator(double period, double stancePhaseRatio, Vec4 bias)
    : _period(period), _stRatio(stancePhaseRatio), _bias(bias)
{

    // 支撑/摆动都要有非零持续时间，且后面相位计算会分别除以_stRatio与1-_stRatio。
    if ((_stRatio >= 1) || (_stRatio <= 0))
    {
        std::cout << "[ERROR] The stancePhaseRatio of WaveGenerator should between (0, 1)" << std::endl;
        exit(-1);
    }

    // bias.rows()是Eigen列向量的行数，此处等于4；bias(i)取第i条腿的周期偏移。
    for (int i(0); i < bias.rows(); ++i)
    {
        if ((bias(i) > 1) || (bias(i) < 0))
        {
            std::cout << "[ERROR] The bias of WaveGenerator should between [0, 1]" << std::endl;
            exit(-1);
        }
    }

    // getSystemTime返回微秒；此起点供后续把实际流逝时间换算为周期相位。
    // setZero把每项置0；<<是Eigen按顺序填充向量的语法，不是这里的输出流操作。
    // 初始历史模式为全摆动，固定phase=0.5；模式切换还需经过calcContactPhase的交接逻辑。
    _startT = getSystemTime();
    _contactPast.setZero();
    _phasePast << 0.5, 0.5, 0.5, 0.5;
    _statusPast = WaveStatus::SWING_ALL;
}

// 对象不在此析构函数中执行额外的资源释放。
WaveGenerator::~WaveGenerator()
{
}

// 计算当前模式的候选波形，并在模式切换时按每条腿的支撑标记匹配情况交接。
// Vec4&/VecInt4&为C++引用参数：写入phaseResult/contactResult会直接更新调用者的向量。
void WaveGenerator::calcContactPhase(Vec4 &phaseResult, VecInt4 &contactResult, WaveStatus status)
{

    // 先算新模式候选值；这时还未必会把候选值直接输出。
    calcWave(_phase, _contact, status);

    if (status != _statusPast)
    {
        // _switchStatus每项1表示正在交接；sum统计未完成标志，用setOnes开启一轮切换。
        if (_switchStatus.sum() == 0)
        {
            _switchStatus.setOnes();
        }
        // 用同一套时间规则计算旧模式，作为交接期间要继续保持的计划。
        calcWave(_phasePast, _contactPast, _statusPast);
        // two special case
        // 全支撑与全摆动的标记永不相等，故这两种直接互切时先把旧标记改成目标值。
        if ((status == WaveStatus::STANCE_ALL) && (_statusPast == WaveStatus::SWING_ALL))
        {
            _contactPast.setOnes();
        }
        else if ((status == WaveStatus::SWING_ALL) && (_statusPast == WaveStatus::STANCE_ALL))
        {
            _contactPast.setZero();
        }
    }

    if (_switchStatus.sum() != 0)
    {
        // 新旧计划都把该腿列为支撑或都列为摆动时，允许使用新相位并清除此腿切换标志；
        // 标记不同则继续输出旧模式的contact和phase，避免切换模式时立即打断该腿阶段。
        for (int i(0); i < 4; ++i)
        {
            if (_contact(i) == _contactPast(i))
            {
                _switchStatus(i) = 0;
            }
            else
            {
                _contact(i) = _contactPast(i);
                _phase(i) = _phasePast(i);
            }
        }
        // 4条腿都完成交接后，记下新模式，后续控制周期就直接沿用此模式。
        if (_switchStatus.sum() == 0)
        {
            _statusPast = status;
        }
    }

    // 复制给外部共享向量；Estimator和GaitGenerator读取的就是这套计划标记及相位。
    phaseResult = _phase;
    contactResult = _contact;
}

// 返回单腿一个完整周期中支撑阶段的持续时间，单位s。
float WaveGenerator::getTstance()
{
    return _period * _stRatio;
}

// 返回摆动阶段持续时间s，轨迹生成器用它把“相位导数”换成真实时间速度。
float WaveGenerator::getTswing()
{
    return _period * (1 - _stRatio);
}

// 返回完整步态周期s，而不是控制主循环周期dt。
float WaveGenerator::getT()
{
    return _period;
}

// 三种模式共用的波形计算：周期交替、全摆动、全支撑。
// 这里的phase在支撑段和摆动段各自从0走到1，并不是整周期不重置的相位。
void WaveGenerator::calcWave(Vec4 &phase, VecInt4 &contact, WaveStatus status)
{
    if (status == WaveStatus::WAVE_ALL)
    {
        // 先计算距构造时刻的实际时间s；(double)显式转为浮点，1e-6将微秒换成秒。
        _passT = (double)(getSystemTime() - _startT) * 1e-6;
        for (int i(0); i < 4; ++i)
        {
            // fmod(a,T)取浮点余数，除以T得到[0,1)整周期进度；减T*bias让各腿错开。
            // 加一个T可使启动时bias在[0,1]范围内的被取模量非负。
            _normalT(i) = fmod(_passT + _period - _period * _bias(i), _period) / _period;
            if (_normalT(i) < _stRatio)
            {
                // 周期前_stRatio为支撑段；再除以占比得到该支撑段内部0~1进度。
                contact(i) = 1;
                phase(i) = _normalT(i) / _stRatio;
            }
            else
            {
                // 余下时间为摆动段，先扣去支撑部分，再除以摆动时间占比归一化。
                contact(i) = 0;
                phase(i) = (_normalT(i) - _stRatio) / (1 - _stRatio);
            }
        }
    }
    else if (status == WaveStatus::SWING_ALL)
    {
        // 固定计划标记，不再随时间轮换；0.5只表示本模式提供的固定阶段相位。
        contact.setZero();
        phase << 0.5, 0.5, 0.5, 0.5;
    }
    else if (status == WaveStatus::STANCE_ALL)
    {
        // 全支撑同样输出固定相位0.5；这不表示脚的位置固定在摆动轨迹的中点。
        contact.setOnes();
        phase << 0.5, 0.5, 0.5, 0.5;
    }
}