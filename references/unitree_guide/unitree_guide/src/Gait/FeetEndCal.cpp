/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "Gait/FeetEndCal.h"

// 落脚点预测器：把机身运动的前瞻位移、速度反馈修正和各腿理想站姿偏置相加。
// 输出为世界坐标足端位置m；计算假设平地，并最终强制落脚点的世界z=0。
// 初始化列表保存共享估计器、底层状态和机器人模型指针，->用于访问它们的成员。
FeetEndCal::FeetEndCal(CtrlComponents *ctrlComp)
           : _est(ctrlComp->estimator), _lowState(ctrlComp->lowState),
             _robModel(ctrlComp->robotModel){
    // 保存单腿计划支撑/摆动时长s；它们来自完整周期乘对应时间占比。
    _Tstance  = ctrlComp->waveGen->getTstance();
    _Tswing   = ctrlComp->waveGen->getTswing();

    // 反馈系数乘线速度差/角速度差后得到位移/角度修正，按公式量纲相当于时间s。
    // 这些是本实现已有参数，并非从机器人几何尺寸自动计算。
    _kx = 0.005;
    _ky = 0.005;
    _kyaw = 0.005;

    // 理想站姿为机身坐标3x4矩阵，每列一条腿：FR右前、FL左前、RR右后、RL左后。
    // 提取每条腿在机身xy平面的极坐标：半径m、初始方向角rad；z不参与平面落点布局。
    Vec34 feetPosBody = _robModel->getFeetPosIdeal();
    for(int i(0); i<4; ++i){
        // pow(x,2)为平方；atan2(y,x)按象限返回朝向，能区分前后和左右。
        _feetRadius(i)    = sqrt( pow(feetPosBody(0, i), 2) + pow(feetPosBody(1, i), 2) );
        _feetInitAngle(i) = atan2(feetPosBody(1, i), feetPosBody(0, i));
    }
}

// 当前析构函数体为空；共享组件的生命周期由外层控制组件管理。
FeetEndCal::~FeetEndCal(){

}

// legID指定腿，vxyGoalGlobal=[vx,vy]为世界速度目标m/s，dYawGoal为偏航速度目标rad/s。
// phase是该腿摆动阶段的0~1进度；返回值Vec3是这次预测的世界坐标落点。
Vec3 FeetEndCal::calFootPos(int legID, Vec2 vxyGoalGlobal, float dYawGoal, float phase){
    // 当前机身线速度、角速度都沿世界轴表达；本函数实际偏航预测使用下方getDYaw()。
    _bodyVelGlobal = _est->getVelocity();
    _bodyWGlobal = _lowState->getGyroGlobal();

    // 前两项：当前线速度*(剩余摆动时间+半个支撑时间)，即向下一支撑阶段中点前瞻。
    // 最后一项为速度误差反馈，按源码为k*(当前速度-目标速度)，不能把符号看反。
    // _nextStep先表示世界轴中的平面位移；此处置z=0，后面再叠加理想足端布局。
    _nextStep(0) = _bodyVelGlobal(0)*(1-phase)*_Tswing + _bodyVelGlobal(0)*_Tstance/2 + _kx*(_bodyVelGlobal(0) - vxyGoalGlobal(0));
    _nextStep(1) = _bodyVelGlobal(1)*(1-phase)*_Tswing + _bodyVelGlobal(1)*_Tstance/2 + _ky*(_bodyVelGlobal(1) - vxyGoalGlobal(1));
    _nextStep(2) = 0;

    _yaw = _lowState->getYaw();
    _dYaw = _lowState->getDYaw();
    // 同样前瞻剩余摆动加半支撑时长，得到将要转过的偏航角rad。
    // 偏航反馈在这里采用kyaw*(目标偏航速度-当前偏航速度)，顺序与上面平移反馈不同。
    _nextYaw = _dYaw*(1-phase)*_Tswing + _dYaw*_Tstance/2 + _kyaw*(dYawGoal - _dYaw);

    // 理想站姿偏置从机身平面旋到预计世界朝向：r*[cos(yaw+初始角+前瞻角), sin(...)]。
    // 这里只利用偏航布置落点，没有用完整3D旋转去补偿机身横滚或俯仰。
    _nextStep(0) += _feetRadius(legID) * cos(_yaw + _feetInitAngle(legID) + _nextYaw);
    _nextStep(1) += _feetRadius(legID) * sin(_yaw + _feetInitAngle(legID) + _nextYaw);

    // 世界落点=当前机身世界位置+预测相对位移；最后覆盖z，明确采用世界平地z=0模型。
    // 该值是规划目标，不是根据碰撞、测高或真实触地事件检测出来的落脚位置。
    _footPos = _est->getPosition() + _nextStep;
    _footPos(2) = 0.0;

    return _footPos;
}