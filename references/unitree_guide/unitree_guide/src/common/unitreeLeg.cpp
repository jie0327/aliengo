/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "common/unitreeLeg.h"
#include <iostream>

// 本文件负责“一条三关节腿”的几何计算，不读取传感器，也不直接发送电机命令。
// q=[侧摆hip角, 大腿thigh角, 小腿calf角]，单位rad；qd为对应角速度，单位rad/s。
// Vec3、Mat3是Eigen的3维列向量与3×3矩阵；v(i)访问分量，M(i,j)访问第i行第j列。
// HIP表示以该腿髋部为原点，BODY表示以机身为原点；这里两者用相同的轴方向，位置相差_pHip2B。
// 本文件中的长度/位置用m，足端线速度用m/s，足端力用N，关节力矩用N·m。

/************************/
/*******QuadrupedLeg*****/
/************************/
// 构造一条腿：侧向髋偏置、大腿长、小腿长，以及“机身原点到髋部”的位置向量。
// 冒号后的列表直接初始化成员；const成员必须在此初始化，不能等到函数体内再赋值。
QuadrupedLeg::QuadrupedLeg(int legID, float abadLinkLength, float hipLinkLength, 
                           float kneeLinkLength, Vec3 pHip2B)
            :_abadLinkLength(abadLinkLength), 
             _hipLinkLength(hipLinkLength), 
             _kneeLinkLength(kneeLinkLength), 
             _pHip2B(pHip2B){
    // 官方编号为0=FR右前、1=FL左前、2=RR右后、3=RL左后；左右镜像由侧向长度的符号表达。
    if (legID == 0 || legID == 2)
        _sideSign = -1;
    else if (legID == 1 || legID == 3)
        _sideSign = 1;
    else{
        std::cout << "Leg ID incorrect!" << std::endl;
        exit(-1);
    }
}

// Forward Kinematics
// 正运动学FK：已知三个关节角，求足端相对髋部的位置pEe2H，结果仍用腿/机身轴方向表达。
Vec3 QuadrupedLeg::calcPEe2H(Vec3 q){
    // 髋偏置沿左右方向取正负；腿段零角时朝下，因此沿z轴的长度取负。
    float l1 = _sideSign * _abadLinkLength;
    float l2 = -_hipLinkLength;
    float l3 = -_kneeLinkLength;

    // 预先计算sin/cos，下面的几何公式会重复使用；q(0)是侧摆角而不是数组的第1个索引。
    float s1 = std::sin(q(0));
    float s2 = std::sin(q(1));
    float s3 = std::sin(q(2));

    float c1 = std::cos(q(0));
    float c2 = std::cos(q(1));
    float c3 = std::cos(q(2));

    // 和角公式：c23=cos(q1+q2)、s23=sin(q1+q2)，这里q1/q2指向量索引1/2的大腿与小腿角。
    float c23 = c2 * c3 - s2 * s3;
    float s23 = s2 * c3 + c2 * s3;

    Vec3 pEe2H;

    // 先把大小腿在前后/上下平面中的投影相加，再通过侧摆角旋转到三维；x前、y左、z上。
    pEe2H(0) = l3 * s23 + l2 * s2;
    pEe2H(1) = -l3 * s1 * c23 + l1 * c1 - l2 * c2 * s1;
    pEe2H(2) =  l3 * c1 * c23 + l1 * s1 + l2 * c1 * c2;

    return pEe2H;
}

// Forward Kinematics
// 将FK结果从髋原点平移到机身原点；这里不包含机身在世界中的位置或姿态旋转。
Vec3 QuadrupedLeg::calcPEe2B(Vec3 q){
    return _pHip2B + calcPEe2H(q);
}

// Derivative Forward Kinematics
// 速度正运动学：v_foot=J(q)*qd。机身暂作固定参考，不包含机身平移/旋转带来的世界速度。
Vec3 QuadrupedLeg::calcVEe(Vec3 q, Vec3 qd){
    return calcJaco(q) * qd;
}

// Inverse Kinematics
// 逆运动学IK：把目标足端位置变成三个关节目标角；采用该腿几何对应的解析解分支。
Vec3 QuadrupedLeg::calcQ(Vec3 pEe, FrameType frame){
    Vec3 pEe2H;
    // 无论输入以髋部还是机身为原点，都先转为髋部相对位置；这不是世界坐标到机身的旋转。
    if(frame == FrameType::HIP)
        pEe2H = pEe;
    else if(frame == FrameType::BODY)
        pEe2H = pEe - _pHip2B;
    else{
        std::cout << "[ERROR] The frame of QuadrupedLeg::calcQ can only be HIP or BODY!" << std::endl;
        exit(-1);
    }

    // 此处变量q1/q2/q3按数学编号，分别写回结果索引0/1/2，不能与上面的索引名混淆。
    float q1, q2, q3;
    Vec3 qResult;
    float px, py, pz;
    float b2y, b3z, b4z, a, b, c;

    // 从Eigen列向量取出目标的x/y/z坐标，便于代入解析公式。
    px = pEe2H(0);
    py = pEe2H(1);
    pz = pEe2H(2);

    // b2y是带左右符号的髋偏置；b3z/b4z为朝下的两段腿长。
    b2y = _abadLinkLength * _sideSign;
    b3z = -_hipLinkLength;
    b4z = -_kneeLinkLength;
    a = _abadLinkLength;
    // c为髋原点到足端的直线距离；减去侧向偏置后，b是大小腿所在平面中的合成距离。
    // pow(x,2)求平方、sqrt求平方根；当前实现没有在这里处理负根号或不可达目标。
    c = sqrt(pow(px, 2) + pow(py, 2) + pow(pz, 2)); // whole length
    b = sqrt(pow(c, 2) - pow(a, 2)); // distance between shoulder and footpoint

    // 先求侧摆，再用余弦定理求膝角，最后回代大腿角；解的正负分支由这些函数固定选择。
    q1 = q1_ik(py, pz, b2y);
    q3 = q3_ik(b3z, b4z, b);
    q2 = q2_ik(q1, q3, px, py, pz, b3z, b4z);

    qResult(0) = q1;
    qResult(1) = q2;
    qResult(2) = q3;

    return qResult;
}

// Derivative Inverse Kinematics
// 速度逆运动学：qd=J(q)^(-1)*v_foot；输入q已经是关节角。
// 这里直接求逆，腿接近伸直等奇异姿态时矩阵可能不可逆；原实现没有阻尼伪逆处理。
Vec3 QuadrupedLeg::calcQd(Vec3 q, Vec3 vEe){
    return calcJaco(q).inverse() * vEe;
}

// Derivative Inverse Kinematics
// 同名函数的另一种参数形式（重载）：先从足端位置求q，再求对应关节速度。
Vec3 QuadrupedLeg::calcQd(Vec3 pEe, Vec3 vEe, FrameType frame){
    Vec3 q = calcQ(pEe, frame);
    return calcJaco(q).inverse() * vEe;
}

// Inverse Dynamics
// 用虚功关系tau=J^T*f把足端力映射成三关节力矩；transpose()只转置矩阵，不求逆。
// force须与J所用轴方向一致；调用方负责地面反力的作用方向与世界/机身坐标转换。
// 原英文称Inverse Dynamics，但这里只做力到力矩映射，不包含完整惯性、科氏力或腿部重力模型。
Vec3 QuadrupedLeg::calcTau(Vec3 q, Vec3 force){
    return calcJaco(q).transpose() * force;
}

// Jacobian Matrix
// 雅可比矩阵J=∂p_foot/∂q：每列对应一个关节，每行对应足端x/y/z，描述“小转角造成的位移”。
Mat3 QuadrupedLeg::calcJaco(Vec3 q){
    Mat3 jaco;

    float l1 = _abadLinkLength * _sideSign;
    float l2 = -_hipLinkLength;
    float l3 = -_kneeLinkLength;

    float s1 = std::sin(q(0));
    float s2 = std::sin(q(1));
    float s3 = std::sin(q(2));

    float c1 = std::cos(q(0));
    float c2 = std::cos(q(1));
    float c3 = std::cos(q(2));

    float c23 = c2 * c3 - s2 * s3;
    float s23 = s2 * c3 + c2 * s3;
    // 第0列是FK对侧摆角的偏导；第1/2列分别对大腿/小腿角求偏导。
    // 侧摆绕x轴旋转，不改变足端x坐标，因此J(0,0)=0；其余元素可与calcPEe2H逐项求导对照。
    jaco(0, 0) = 0;
    jaco(1, 0) = -l3 * c1 * c23 - l2 * c1 * c2 - l1 * s1;
    jaco(2, 0) = -l3 * s1 * c23 - l2 * c2 * s1 + l1 * c1;
    jaco(0, 1) = l3 * c23 + l2 * c2;
    jaco(1, 1) = l3 * s1 * s23 + l2 * s1 * s2;
    jaco(2, 1) = -l3 * c1 * s23 - l2 * c1 * s2;
    jaco(0, 2) = l3 * c23;
    jaco(1, 2) = l3 * s1 * s23;
    jaco(2, 2) = -l3 * c1 * s23;

    return jaco;
}

// 由y/z平面投影与侧向髋偏置求侧摆角；atan2(y,x)保留象限，区别于只用atan(y/x)。
float QuadrupedLeg::q1_ik(float py, float pz, float l1){
    float q1;
    float L = sqrt(pow(py,2)+pow(pz,2)-pow(l1,2));
    q1 = atan2(pz*l1+py*L, py*l1-pz*L);
    return q1;
}

// 由三角形余弦定理求膝关节角：两段长度b3z/b4z与合成距离b决定夹角。
float QuadrupedLeg::q3_ik(float b3z, float b4z, float b){
    float q3, temp;
    temp = (pow(b3z, 2) + pow(b4z, 2) - pow(b, 2))/(2*fabs(b3z*b4z));
    // acos输入必须在[-1,1]；此处夹断余弦值，但这不等于完整检查目标是否可达。
    if(temp>1) temp = 1;
    if(temp<-1) temp = -1;
    q3 = acos(temp);
    // 将三角形内角转换到本模型膝关节的负角分支；实际运算单位仍是rad。
    q3 = -(M_PI - q3); //0~180
    return q3;
}

// 已知侧摆角和膝角后，投影回大小腿平面，用atan2消去组合项，求大腿关节角。
float QuadrupedLeg::q2_ik(float q1, float q3, float px, float py, float pz, float b3z, float b4z){
    float q2, a1, a2, m1, m2;
    
    // a1/a2为去掉侧摆影响后的投影；m1/m2描述膝角确定后两腿段的合成几何关系。
    a1 = py*sin(q1) - pz*cos(q1);
    a2 = px;
    m1 = b4z*sin(q3);
    m2 = b3z + b4z*cos(q3);
    q2 = atan2(m1*a1+m2*a2, m1*a2-m2*a1);
    return q2;
}
