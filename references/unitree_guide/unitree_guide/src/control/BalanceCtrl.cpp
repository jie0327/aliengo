/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "control/BalanceCtrl.h"
#include "common/mathTools.h"
#include "common/timeMarker.h"

// 平衡力分配器：用当前一帧的12个足端力分量组成二次规划（QP），使合力/合力矩跟踪目标。
// 它只优化当前时刻，上一周期的力仅用来平滑变化，没有预测时域或未来状态递推，因此不是MPC。
// 此构造重载接收质量mass（kg）、机体系惯量Ib（3×3，kg·m²）、合力/力矩误差权重S（6×6）及正则权重。
// 冒号后是C++成员初始化列表，在函数体之前保存传入的质量、惯量及权重。
BalanceCtrl::BalanceCtrl(double mass, Mat3 Ib, Mat6 S, double alpha, double beta)
            : _mass(mass), _Ib(Ib), _S(S), _alpha(alpha), _beta(beta){
    // Fprev为上一帧12维地面反力，初值0；世界系z向上，重力沿负z方向。
    _Fprev.setZero();
    _g << 0, 0, -9.81;
    _fricRatio = 0.3;
    // 每足5×3摩擦金字塔矩阵：乘力[f_x,f_y,f_z]ᵀ后要求每项>=0，mu为无量纲摩擦系数。
    // 展开为 |f_x|<=mu*f_z、|f_y|<=mu*f_z、f_z>=0，最后一项表示地面只能推足，不能向下拉足。
    _fricMat <<  1,  0, _fricRatio,
                -1,  0, _fricRatio,
                 0,  1, _fricRatio,
                 0, -1, _fricRatio,
                 0,  0, 1;
}

// 常用构造函数：从机器人模型读取质量、质心偏置和惯量，并设置力分配的各项权重。
// robModel是模型指针；->通过指针调用其成员函数，读取结果后保存在本控制器中。
BalanceCtrl::BalanceCtrl(QuadrupedRobot *robModel){
    // Vec6/Vec12分别是6×1/12×1的Eigen列向量，s对应合力/合力矩，w和u对应每足xyz力分量。
    Vec6 s;
    Vec12 w, u;

    _mass = robModel->getRobMass();
    // pcb为从基座原点指向质心的位移，在机体系表达（3×1，米）；Ib为质心处机体系惯量。
    _pcb = robModel->getPcb();
    _Ib = robModel->getRobInertial();
    _g << 0, 0, -9.81;

    // W惩罚力幅值，U惩罚相邻周期力变化；alpha/beta调节二者相对合力/力矩跟踪误差的比重。
    // <<是Eigen按顺序填值的语法；12项按每足[x,y,z]堆叠，四足顺序与运动学模型一致。
    w << 10, 10, 4, 10, 10, 4, 10, 10, 4, 10, 10, 4;
    u << 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3;
    _alpha = 0.001;
    _beta  = 0.1;
    _fricRatio = 0.4;

    // S前3项权衡世界系xyz合力误差，后3项权衡绕xyz轴的合力矩误差；数值越大，越倾向优先跟踪。
    // 力和力矩的单位不同，因此这些权重也承担不同物理量之间的尺度调整作用。
    s << 20, 20, 50, 450, 450, 450; 

    // asDiagonal()将向量各项放到方阵对角线上，其余元素为0：S为6×6，W/U为12×12。
    _S = s.asDiagonal();
    _W = w.asDiagonal();
    _U = u.asDiagonal();
    
    _Fprev.setZero();
    // 与上面重载相同的5条每足约束，此构造路径使用mu=0.4；世界系z轴作为平地法向。
    _fricMat <<  1,  0, _fricRatio,
                -1,  0, _fricRatio,
                 0,  1, _fricRatio,
                 0, -1, _fricRatio,
                 0,  0, 1;
}

// 输入期望线加速度ddPcd（3×1，米/秒²）、期望角加速度dWbd（3×1，弧度/秒²），均用世界系表达。
// rotM为机体系到世界系的3×3旋转矩阵；feetPos2B每列为基座指向一足的世界系位移（3×4，米）。
// contact为4维计划接触标志（1支撑、0摆动），用于施加约束，并不是足端接触传感器的检测结果。
// 返回3×4世界系地面反力（牛顿），每列一足，表示地面对机器人的力；状态机调用处再取负来生成足端施力命令。
Vec34 BalanceCtrl::calF(Vec3 ddPcd, Vec3 dWbd, RotMat rotM, Vec34 feetPos2B, VecInt4 contact){
    // A把12维足力映射成6维合力/合力矩，bd为目标，约束则取决于本帧哪些腿计划支撑。
    calMatrixA(feetPos2B, rotM, contact);
    calVectorBd(ddPcd, dWbd, rotM);
    calConstraints(contact);

    // 等价目标（省略与F无关的常数）：0.5*[(AF-bd)ᵀS(AF-bd)+alpha*FᵀWF+beta*(F-Fprev)ᵀU(F-Fprev)]。
    // 展开后得到求解器要求的0.5*FᵀG*F+g0ᵀF；G为12×12，线性项有12个分量。
    // transpose()表示转置；这里的乘法顺序保证维度匹配，不能把矩阵乘法当作逐元素乘法。
    _G = _A.transpose()*_S*_A + _alpha*_W + _beta*_U;
    _g0T = -_bd.transpose()*_S*_A - _beta*_Fprev.transpose()*_U;

    solveQP();

    // 保存当前解，作为下一帧的变化惩罚参考；vec12ToVec34按连续3项恢复每条腿的力向量。
    _Fprev = _F;
    return vec12ToVec34(_F);
}

// 建立6×12的力到合力/合力矩映射A：上3行求Σf_i，下3行求Σ(r_i×f_i)。
// r_i必须从质心出发，所以将基座到足的位移减去旋转到世界系的基座到质心偏置。
// 此处给所有腿都建立映射，contact参数未在函数体使用；摆动足的零力由calConstraints统一约束。
void BalanceCtrl::calMatrixA(Vec34 feetPos2B, RotMat rotM, VecInt4 contact){
    for(int i(0); i < 4; ++i){
        // block(起始行,起始列,行数,列数)选择可写子矩阵；I3为3×3单位阵。
        // skew(r)构造反对称矩阵，满足skew(r)*f=r×f；r为米、f为牛顿，所得力矩为牛顿·米。
        _A.block(0, 3*i, 3, 3) = I3;
        _A.block(3, 3*i, 3, 3) = skew(feetPos2B.col(i) - rotM*_pcb);
    }
}

// 把期望加速度换算成6维目标[合力;合力矩]：由质心平动方程m*a=Σf+m*g得到Σf=m*(a-g)。
// 转动采用简化关系Σtau=I_G*alpha，I_G=R*I_B*Rᵀ；本实现没有加入omega×(I_G*omega)陀螺项。
void BalanceCtrl::calVectorBd(Vec3 ddPcd, Vec3 dWbd, RotMat rotM){
    // head(3)/tail(3)分别选择向量前3项/后3项；前半单位为牛顿，后半为牛顿·米。
    _bd.head(3) = _mass * (ddPcd - _g);
    _bd.tail(3) = (rotM * _Ib * rotM.transpose()) * dWbd;
}

// 按本周期计划接触状态建立线性约束：支撑足满足摩擦金字塔，摆动足的xyz力全部等于0。
// 内部采用CI*F+ci0>=0、CE*F+ce0=0；CI/CE按“每行一条约束”存放，交给求解器时再转置。
void BalanceCtrl::calConstraints(VecInt4 contact){
    // 先数支撑腿，再确定约束条数：每条支撑腿5个不等式，每条摆动腿3个等式。
    int contactLegNum = 0;
    for(int i(0); i<4; ++i){
        if(contact(i) == 1){
            contactLegNum += 1;
        }
    }

    // 动态Eigen矩阵resize(行,列)改变大小；12列始终对应四条腿的xyz力，而不是只优化支撑腿。
    _CI.resize(5*contactLegNum, 12);
    _ci0.resize(5*contactLegNum);
    _CE.resize(3*(4 - contactLegNum), 12);
    _ce0.resize(3*(4 - contactLegNum));

    // setZero()使未参与某条约束的其他足力系数为0，偏置向量ci0/ce0也为0。
    _CI.setZero();
    _ci0.setZero();
    _CE.setZero();
    _ce0.setZero();

    // ceID/ciID是各类约束中的腿编号，与原始腿号i不同；++表示加1，以连续填入对应的行块。
    int ceID = 0;
    int ciID = 0;
    for(int i(0); i<4; ++i){
        if(contact(i) == 1){
            // 此5×3子块仅作用于第i条腿，得到±f_x+mu*f_z>=0、±f_y+mu*f_z>=0、f_z>=0。
            _CI.block(5*ciID, 3*i, 5, 3) = _fricMat;
            ++ciID;
        }else{
            // 此3×3单位阵给出f_x=f_y=f_z=0，保证摆动足不为机体提供虚构的支撑力。
            _CE.block(3*ceID, 3*i, 3, 3) = I3;
            ++ceID;
        }
    }
}

// 将Eigen矩阵复制成QuadProg++的数据结构，求解本帧QP，再把12维最优地面反力取回。
// QuadProg++接口定义见其头文件：min 0.5*xᵀGx+g0ᵀx，CEᵀx+ce0=0，CIᵀx+ci0>=0。
void BalanceCtrl::solveQP(){
    // n=12为优化变量数，m为等式条数，p为不等式条数；size()返回向量元素总数。
    int n = _F.size();
    int m = _ce0.size();
    int p = _ci0.size();

    // 求解器按“每列一条约束”存CE/CI，所以尺寸分别是n×m、n×p，与内部_CE/_CI相反。
    G.resize(n, n);
    CE.resize(n, m);
    CI.resize(n, p);
    g0.resize(n);
    ce0.resize(m);
    ci0.resize(p);
    x.resize(n);

    // Eigen用(i,j)取矩阵元素，QuadProg++用[i][j]取元素；双层循环只做容器之间的数据复制。
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            G[i][j] = _G(i, j);
        }
    }

    // 将内部按行存放的等式约束转成求解器要求的列；不等式在后面的循环作同样处理。
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            CE[i][j] = (_CE.transpose())(i, j);
        }
    }

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < p; ++j) {
            CI[i][j] = (_CI.transpose())(i, j);
        }
    }

    // 复制目标函数线性项及约束偏置；这里的[]为向量元素访问，不涉及矩阵转置。
    for (int i = 0; i < n; ++i) {
        g0[i] = _g0T[i];
    }

    for (int i = 0; i < m; ++i) {
        ce0[i] = _ce0[i];
    }

    for (int i = 0; i < p; ++i) {
        ci0[i] = _ci0[i];
    }

    // x由求解器写入最优力，返回值value为目标函数值；原函数在此处未额外处理返回状态。
    // 求解器会在内部改写G做分解，因此前面使用其专用矩阵副本，而不直接传入Eigen的_G。
    double value = solve_quadprog(G, g0, CE, ce0, CI, ci0, x);

    // 逐项拷回Eigen向量_F，分量顺序仍为[f0x,f0y,f0z,...,f3x,f3y,f3z]。
    for (int i = 0; i < n; ++i) {
        _F[i] = x[i];
    }
}