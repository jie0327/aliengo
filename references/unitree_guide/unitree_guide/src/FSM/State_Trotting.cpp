/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/State_Trotting.h"
#include <iomanip>

// 小跑状态把用户速度依次变成：机身运动目标 -> 四足轨迹 -> 足端力/关节力矩及关节位置、速度。
// 本文件的 Global/G 表示世界坐标，Body/B 表示随机身转动的坐标（x前、y左、z上）。
// Vec3为3x1，Vec34为3x4且每列对应一条腿；腿序为FR右前、FL左前、RR右后、RL左后。
// 位置/速度/角度/角速度/力/力矩分别用m、m/s、rad、rad/s、N、N·m。
// 构造函数冒号后的初始化列表先构造FSMState基类，再保存公共控制组件的指针；->访问指针指向的对象。
State_Trotting::State_Trotting(CtrlComponents *ctrlComp)
             :FSMState(ctrlComp, FSMStateName::TROTTING, "trotting"), 
              _est(ctrlComp->estimator), _phase(ctrlComp->phase), 
              _contact(ctrlComp->contact), _robModel(ctrlComp->robotModel), 
              _balCtrl(ctrlComp->balCtrl){
    // new创建本状态使用的步态轨迹生成器；估计器、波形器和机器人模型仍由CtrlComponents共享。
    _gait = new GaitGenerator(ctrlComp);

    // 摆动足相对起点的最大抬脚高度，0.08m即8cm。
    _gaitHeight = 0.08;

// #ifdef在编译时选择对应机型的增益，并非运行时的if判断。
// asDiagonal()把三个轴的增益放到3x3对角矩阵上，矩阵乘误差即可分别控制x/y/z。
// _Kpp/_Kdp把位置/速度误差变成线加速度；_kpw/_Kdw把姿态/角速度误差变成角加速度。
// _KpSwing/_KdSwing则把摆动足的位置/速度误差变成足端虚拟控制力。
#ifdef ROBOT_TYPE_Go1
    _Kpp = Vec3(70, 70, 70).asDiagonal();
    _Kdp = Vec3(10, 10, 10).asDiagonal();
    _kpw = 780; 
    _Kdw = Vec3(70, 70, 70).asDiagonal();
    _KpSwing = Vec3(400, 400, 400).asDiagonal();
    _KdSwing = Vec3(10, 10, 10).asDiagonal();
#endif

#ifdef ROBOT_TYPE_A1
    _Kpp = Vec3(20, 20, 100).asDiagonal();
    _Kdp = Vec3(20, 20, 20).asDiagonal();
    _kpw = 400;
    _Kdw = Vec3(50, 50, 50).asDiagonal();
    _KpSwing = Vec3(400, 400, 400).asDiagonal();
    _KdSwing = Vec3(10, 10, 10).asDiagonal();
#endif

    // 每个Vec2保存[下限, 上限]，前两项是机身平移速度m/s，最后一项是偏航角速度rad/s。
    _vxLim = _robModel->getRobVelLimitX();
    _vyLim = _robModel->getRobVelLimitY();
    _wyawLim = _robModel->getRobVelLimitYaw();

}

// 离开对象生命周期时释放这里new出来的轨迹生成器；~标记C++析构函数。
State_Trotting::~State_Trotting(){
    delete _gait;
}

// 每次切入小跑状态都重新建立参考，避免沿用上次小跑的位置、朝向和轨迹起点。
void State_Trotting::enter(){
    // _pcd是世界坐标机身位置目标；z取理想站姿足端负高度的相反数，隐含平地z=0。
    _pcd = _est->getPosition();
    _pcd(2) = -_robModel->getFeetPosIdeal()(2, 0);
    _vCmdBody.setZero();
    _yawCmd = _lowState->getYaw();
    // rotz仅绕世界z轴旋转，因此目标横滚和俯仰为0，偏航保持进入时的方向。
    _Rd = rotz(_yawCmd);
    _wCmdGlobal.setZero();

    // 清空输入面板命令，并让步态生成器在下次run时重新读取四足起点。
    _ctrlComp->ioInter->zeroCmdPanel();
    _gait->restart();
}

// FSM实际切出此状态时调用：清空用户输入并请求四足统一为摆动标记，供后继状态接管。
void State_Trotting::exit(){
    _ctrlComp->ioInter->zeroCmdPanel();
    _ctrlComp->setAllSwing();
}

// 这里只返回期望状态名；真正的exit/enter由外层FSM在状态切换流程中调用。
// L2_B进入被动状态，L2_A回到固定站立，其余命令继续留在小跑。
FSMStateName State_Trotting::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::L2_A){
        return FSMStateName::FIXEDSTAND;
    }
    else{
        return FSMStateName::TROTTING;
    }
}

// 每个控制周期调用一次；本函数写入lowCmd，外层FSM的后续sendRecv负责交给底层接口。
void State_Trotting::run(){
    // 名字含Body的_posBody/_velBody仍是世界坐标中的机身原点位置和线速度。
    // _posFeet2BGlobal是“机身原点指向足端”的相对向量，但分量沿世界轴表达，供计算力臂。
    // _posFeetGlobal/_velFeetGlobal则是足端世界绝对位置/速度，均为3x4。
    _posBody = _est->getPosition();
    _velBody = _est->getVelocity();
    _posFeet2BGlobal = _est->getPosFeet2BGlobal();
    _posFeetGlobal = _est->getFeetPos();
    _velFeetGlobal = _est->getFeetVel();
    _B2G_RotMat = _lowState->getRotMat();
    // 旋转矩阵正交，所以transpose()转置就是逆；R_B2G乘机身向量得到世界向量。
    _G2B_RotMat = _B2G_RotMat.transpose();
    _yaw = _lowState->getYaw();
    _dYaw = _lowState->getDYaw();

    _userValue = _lowState->userValue;

    getUserCmd();
    calcCmd();

    // segment(0,2)取世界速度向量的x、y两项；偏航速度与抬脚高度一起交给轨迹生成器。
    // run的两个引用输出分别接收4条腿的足端世界目标位置m和速度m/s。
    _gait->setGait(_vCmdGlobal.segment(0,2), _wCmdGlobal(2), _gaitHeight);
    _gait->run(_posFeetGlobalGoal, _velFeetGlobalGoal);

    calcTau();
    calcQQd();

    // 根据运动指令及机身跟踪误差决定继续迈步还是全支撑。
    // 外层FSM已在本轮run之前更新过phase/contact；这里修改的是之后一次波形更新所用状态。
    if(checkStepOrNot()){
        _ctrlComp->setStartWave();
    }else{
        _ctrlComp->setAllStance();
    }

    // 同时下发前馈力矩、关节位置和速度目标；vec34ToVec12按腿把每列3个关节展开为12x1。
    // 底层还会根据本轮设置的关节Kp/Kd叠加位置/速度反馈，并非只执行_tau。
    _lowCmd->setTau(_tau);
    _lowCmd->setQ(vec34ToVec12(_qGoal));
    _lowCmd->setQd(vec34ToVec12(_qdGoal));

    // int i(0)是以0直接初始化循环索引；++i每次走到下一条腿。
    // (*_contact)(i)先解引用共享4x1整数向量，再取第i项；1=计划支撑，0=计划摆动。
    // 这是步态计划标记，不是足底传感器检测到的真实触地，增益也按这个计划选择。
    for(int i(0); i<4; ++i){
        if((*_contact)(i) == 0){
            _lowCmd->setSwingGain(i);
        }else{
            _lowCmd->setStableGain(i);
        }
    }

}

// ||是逻辑“或”，任一条件越过阈值就迈步；fabs取绝对值以同时检查正负方向。
// 即使用户速度为0，位置偏差超过0.08m或速度偏差超过0.05m/s仍可能触发迈步。
// 平移命令阈值为0.03m/s，偏航命令阈值为0.20rad/s；这里未检查z和横滚/俯仰误差。
bool State_Trotting::checkStepOrNot(){
    if( (fabs(_vCmdBody(0)) > 0.03) ||
        (fabs(_vCmdBody(1)) > 0.03) ||
        (fabs(_posError(0)) > 0.08) ||
        (fabs(_posError(1)) > 0.08) ||
        (fabs(_velError(0)) > 0.05) ||
        (fabs(_velError(1)) > 0.05) ||
        (fabs(_dYawCmd) > 0.20) ){
        return true;
    }else{
        return false;
    }
}

// 为上层调用者提供直接设置机身x/y速度m/s与偏航速度rad/s的接口，禁止竖直速度。
// 本类run仍会调用getUserCmd；最终使用哪个输入还取决于调用顺序或派生类对getUserCmd的覆盖。
void State_Trotting::setHighCmd(double vx, double vy, double wz){
    _vCmdBody(0) = vx;
    _vCmdBody(1) = vy;
    _vCmdBody(2) = 0; 
    _dYawCmd = wz;
}

// 把摇杆归一化量映射到机器人允许的物理速度范围；横向和转向负号用于适配摇杆正方向。
void State_Trotting::getUserCmd(){
    /* Movement */
    _vCmdBody(0) =  invNormalize(_userValue.ly, _vxLim(0), _vxLim(1));
    _vCmdBody(1) = -invNormalize(_userValue.lx, _vyLim(0), _vyLim(1));
    _vCmdBody(2) = 0;

    /* Turning */
    _dYawCmd = -invNormalize(_userValue.rx, _wyawLim(0), _wyawLim(1));
    // 一阶低通：本次输出=0.9*上次输出+0.1*新输入，减小偏航速度命令突变。
    _dYawCmd = 0.9*_dYawCmdPast + (1-0.9) * _dYawCmd;
    _dYawCmdPast = _dYawCmd;
}

// 将机身坐标的用户速度变成世界坐标命令，并按dt（s）积分得到位置、偏航目标。
void State_Trotting::calcCmd(){
    /* Movement */
    _vCmdGlobal = _B2G_RotMat * _vCmdBody;

    // saturation(value,[min,max])把数值夹在上下界内；这里限制目标相对当前估计速度的偏差。
    // ±0.2的单位为m/s，并不是直接设置每周期的加速度上限。
    _vCmdGlobal(0) = saturation(_vCmdGlobal(0), Vec2(_velBody(0)-0.2, _velBody(0)+0.2));
    _vCmdGlobal(1) = saturation(_vCmdGlobal(1), Vec2(_velBody(1)-0.2, _velBody(1)+0.2));

    // 位置积分p_d+=v_d*dt后限制在当前机身位置±0.05m，避免积累过大的水平追踪误差。
    _pcd(0) = saturation(_pcd(0) + _vCmdGlobal(0) * _ctrlComp->dt, Vec2(_posBody(0) - 0.05, _posBody(0) + 0.05));
    _pcd(1) = saturation(_pcd(1) + _vCmdGlobal(1) * _ctrlComp->dt, Vec2(_posBody(1) - 0.05, _posBody(1) + 0.05));

    // 保持高度不随速度积分变化；_pcd.z仍使用enter设置的站姿高度。
    _vCmdGlobal(2) = 0;

    /* Turning */
    _yawCmd = _yawCmd + _dYawCmd * _ctrlComp->dt;

    // _Rd为期望机身到世界旋转矩阵，_wCmdGlobal为世界轴表达的期望角速度。
    _Rd = rotz(_yawCmd);
    _wCmdGlobal(2) = _dYawCmd;
}

// 机身PD先给出期望加速度，QP分配支撑反力，摆动足用足端PD，最后统一映射为关节力矩。
void State_Trotting::calcTau(){
    _posError = _pcd - _posBody;
    _velError = _vCmdGlobal - _velBody;

    // 平移PD：a_d=Kp*(p_d-p)+Kd*(v_d-v)，输出_ddPcd单位m/s²。
    // _Rd*R_G2B为世界轴表达的姿态误差旋转；rotMatToExp把它变成轴角误差向量rad。
    // 姿态比例项加角速度阻尼项形成_dWbd（rad/s²），不是直接得到关节力矩。
    _ddPcd = _Kpp * _posError + _Kdp * _velError;
    _dWbd  = _kpw*rotMatToExp(_Rd*_G2B_RotMat) + _Kdw * (_wCmdGlobal - _lowState->getGyroGlobal());

    // 对期望线加速度、角加速度逐轴限幅，防止机身误差瞬间提出过大的控制需求。
    _ddPcd(0) = saturation(_ddPcd(0), Vec2(-3, 3));
    _ddPcd(1) = saturation(_ddPcd(1), Vec2(-3, 3));
    _ddPcd(2) = saturation(_ddPcd(2), Vec2(-5, 5));

    _dWbd(0) = saturation(_dWbd(0), Vec2(-40, 40));
    _dWbd(1) = saturation(_dWbd(1), Vec2(-40, 40));
    _dWbd(2) = saturation(_dWbd(2), Vec2(-10, 10));

    // calF根据机身动力学、计划支撑腿及摩擦约束，用QP（二次规划）求地面对机器人的反力N。
    // 返回3x4世界坐标反力；前面的负号转成腿向地面施力的控制约定，再用于tau=J^T*f。
    // _posFeet2BGlobal提供每足相对机身的位置，QP据此计算足端反力对机身产生的力矩。
    _forceFeetGlobal = - _balCtrl->calF(_ddPcd, _dWbd, _B2G_RotMat, _posFeet2BGlobal, *_contact);

    // QP的非支撑腿反力被约束为0；对摆动腿改用世界坐标足端PD虚拟力：
    // f=KpSwing*(p_goal-p)+KdSwing*(v_goal-v)，让空中足端追踪抬脚轨迹。
    // col(i)访问3x4矩阵第i列，代表第i条腿的xyz；此力不表示摆动脚真的接触地面。
    for(int i(0); i<4; ++i){
        if((*_contact)(i) == 0){
            _forceFeetGlobal.col(i) = _KpSwing*(_posFeetGlobalGoal.col(i) - _posFeetGlobal.col(i)) + _KdSwing*(_velFeetGlobalGoal.col(i)-_velFeetGlobal.col(i));
        }
    }

    // 单腿雅可比J按机身轴定义，因此先把4列足端力从世界系旋转到机身系。
    // getTau内部逐腿执行tau_i=J_i(q_i).transpose()*f_i；q取当前关节角，结果为12x1 N·m。
    _forceFeetBody = _G2B_RotMat * _forceFeetGlobal;
    _q = vec34ToVec12(_lowState->getQ());
    _tau = _robModel->getTau(_q, _forceFeetBody);
}

// 将同一套足端世界轨迹变成关节位置q_goal（rad）和关节速度qd_goal（rad/s）。
void State_Trotting::calcQQd(){

    // 当前足端相对机身原点的位置，按机身轴表达；用于在当前构型求速度雅可比。
    Vec34 _posFeet2B;
    _posFeet2B = _robModel->getFeet2BPositions(*_lowState,FrameType::BODY);
    
    // 位置变换先减机身世界位置，再乘R_G2B：p_B=R_G2B*(p_G-p_body_G)。
    // 当前启用的速度变换只减机身平移速度；没有减旋转坐标系产生的omega×r项。
    // 下方原有注释行给出带该角速度项的形式，但本程序当前并不执行那一行。
    for(int i(0); i<4; ++i){
        _posFeet2BGoal.col(i) = _G2B_RotMat * (_posFeetGlobalGoal.col(i) - _posBody);
        _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody); 
        // _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody - _B2G_RotMat * (skew(_lowState->getGyro()) * _posFeet2B.col(i)) );  //  c.f formula (6.12) 
    }
    
    // getQ为足端位置逆运动学；FrameType::BODY说明输入原点在机身，而非单腿髋关节。
    // getQd内部由当前足端位置恢复构型，再算qd=J(q)^(-1)*v_goal；不是对_qGoal做差分。
    // vec12ToVec34把按腿排列的12个关节量恢复成每腿一列、每列3关节的3x4矩阵。
    _qGoal = vec12ToVec34(_robModel->getQ(_posFeet2BGoal, FrameType::BODY));
    _qdGoal = vec12ToVec34(_robModel->getQd(_posFeet2B, _velFeet2BGoal, FrameType::BODY));
}

