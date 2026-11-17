/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "Gait/GaitGenerator.h"

// 足端轨迹层：支撑腿保持世界位置，摆动腿从离地起点沿摆线移动到预测落点。
// Vec34为3x4，列0/1/2/3依次是FR右前、FL左前、RR右后、RL左后，每列按xyz排列。
// 本文件_startP/_endP及输出feetPos都在世界坐标中，单位m；feetVel为世界速度m/s。
// 初始化列表保存共享组件的指针；_phase/_contact与波形器、估计器读取同一组计划数据。
GaitGenerator::GaitGenerator(CtrlComponents *ctrlComp)
              : _waveG(ctrlComp->waveGen), _est(ctrlComp->estimator), 
                _phase(ctrlComp->phase), _contact(ctrlComp->contact), 
                _robModel(ctrlComp->robotModel), _state(ctrlComp->lowState){
    // FeetEndCal负责预测每条摆动腿的着地点，本类负责从起点到该落点的连续轨迹。
    _feetCal = new FeetEndCal(ctrlComp);
    _firstRun = true;
}

// 当前实现的析构函数体为空，没有在此处执行额外清理。
GaitGenerator::~GaitGenerator(){
}

// 保存世界坐标水平速度目标[x,y]（m/s）、偏航角速度目标（rad/s）及抬脚高度（m）。
// 这里只更新轨迹参数，实际轨迹在run时根据各腿当前计划相位计算。
void GaitGenerator::setGait(Vec2 vxyGoalGlobal, float dYawGoal, float gaitHeight){
    _vxyGoal = vxyGoalGlobal;
    _dYawGoal = dYawGoal;
    _gaitHeight = gaitHeight;
}

// 标记下次run需要重读足端起点，并清零水平目标；不会重置WaveGenerator的计时起点。
void GaitGenerator::restart(){
    _firstRun = true;
    _vxyGoal.setZero();
}

// 两个&引用是输出参数：直接把4足世界位置/速度目标写进调用者传来的矩阵。
void GaitGenerator::run(Vec34 &feetPos, Vec34 &feetVel){
    if(_firstRun){
        // 进入步态后的第一次调用，用估计器计算的当前4足世界位置初始化轨迹起点。
        _startP = _est->getFeetPos();
        _firstRun = false;
    }

    // 每条腿独立选择支撑保持或摆动轨迹；(*指针)(i)先取共享向量，再读第i项。
    // contact=1/0来自时间步态计划，不能当成真实触地传感器的判断结果。
    for(int i(0); i<4; ++i){
        if((*_contact)(i) == 1){
            if((*_phase)(i) < 0.5){
                // 支撑阶段前半段持续刷新起点；后半段保留该值，随后作为摆动离地起点。
                // _startP.col(i)是Eigen第i列的可写视图，赋值只更新这一条腿的xyz。
                _startP.col(i) = _est->getFootPos(i);
            }
            // 支撑轨迹目标取缓存世界位置、速度为0；更新起点时这个位置目标也会跟着更新。
            feetPos.col(i) = _startP.col(i);
            feetVel.col(i).setZero();
        }
        else{
            // 每个摆动控制周期都用最新机身速度和当前摆动相位重算落点，不是离地后固定一次。
            // phase在摆动阶段内从0走到1；calFootPos输出世界坐标的目标落点。
            _endP.col(i) = _feetCal->calFootPos(i, _vxyGoal, _dYawGoal, (*_phase)(i));

            feetPos.col(i) = getFootPos(i);
            feetVel.col(i) = getFootVel(i);
        }
    }
    // 缓存本次位置和相位，作为本对象的历史量；本文件当前轨迹公式没有使用这两个缓存。
    _pastP = feetPos;
    _phasePast = *_phase;
}

// 根据第i条腿当前摆动相位求世界位置：x/y由起点平滑移到预测落点，z在起点高度上抬升。
Vec3 GaitGenerator::getFootPos(int i){
    Vec3 footPos;

    footPos(0) = cycloidXYPosition(_startP.col(i)(0), _endP.col(i)(0), (*_phase)(i));
    footPos(1) = cycloidXYPosition(_startP.col(i)(1), _endP.col(i)(1), (*_phase)(i));
    // 高度曲线不使用_endP.z；假定落地高度与起点相同，不能直接表达跨台阶的高度变化。
    footPos(2) =  cycloidZPosition(_startP.col(i)(2), _gaitHeight, (*_phase)(i));
    
    return footPos;
}

// 给出摆线关于时间的解析速度（m/s）；它与上面的足端位置一起供控制器追踪。
// 公式把当前起终点当作常量，未包含run中实时重算_endP产生的额外时间导数。
Vec3 GaitGenerator::getFootVel(int i){
    Vec3 footVel;

    footVel(0) = cycloidXYVelocity(_startP.col(i)(0), _endP.col(i)(0), (*_phase)(i));
    footVel(1) = cycloidXYVelocity(_startP.col(i)(1), _endP.col(i)(1), (*_phase)(i));
    footVel(2) =  cycloidZVelocity(_gaitHeight, (*_phase)(i));

    return footVel;
}

// 水平摆线：令s=phase，u=2πs，则p(s)=start+(end-start)*(u-sin(u))/(2π)。
// s=0/1时恰好到起点/终点，且两端关于相位的速度为0，避免起落脚时水平速度突变。
// float是单精度浮点形参/返回值；M_PI为π常量，sin接收弧度。
float GaitGenerator::cycloidXYPosition(float start, float end, float phase){
    float phasePI = 2 * M_PI * phase;
    return (end - start)*(phasePI - sin(phasePI))/(2*M_PI) + start;
}

// 由dp/dt=(dp/ds)*(ds/dt)，并假设摆动相位匀速ds/dt=1/Tswing，得到水平速度。
// v=(end-start)*(1-cos(2πs))/Tswing，两端为0，中间最大。
float GaitGenerator::cycloidXYVelocity(float start, float end, float phase){
    float phasePI = 2 * M_PI * phase;
    return (end - start)*(1 - cos(phasePI)) / _waveG->getTswing();
}

// 竖直抬脚：z=start+h*(1-cos(2πs))/2；s=0/1回到起点高度，s=0.5达到start+h。
// 因此h表示最大抬升高度，不是整个周期一直保持的足端高度。
float GaitGenerator::cycloidZPosition(float start, float h, float phase){
    float phasePI = 2 * M_PI * phase;
    return h*(1 - cos(phasePI))/2 + start;
}

// 对竖直位置求时间导数：vz=h*π*sin(2πs)/Tswing；前半段抬升、后半段下降。
// 起点、最高点和终点的竖直速度均为0。
float GaitGenerator::cycloidZVelocity(float h, float phase){
    float phasePI = 2 * M_PI * phase;
    return h*M_PI * sin(phasePI) / _waveG->getTswing();
}