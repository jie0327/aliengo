/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

// #include "unitree_legged_control/joint_controller.h"
#include "joint_controller.h"
#include <pluginlib/class_list_macros.h>

// 本文件是ROS1 ros_control的“单关节力矩控制插件”；12个关节分别加载一个实例。
// 上层Guide通过MotorCmd发送q/dq/Kp/Kd/tau，本插件读反馈、计算PD+前馈力矩，再交给Gazebo。
// q为rad、dq为rad/s、tau为N·m；Kp为N·m/rad，Kd为N·m·s/rad。
// 这段是在仿真中执行关节控制，不是通过SDK向真实机器人发送UDP命令。
// #define rqtTune // use rqt or not

// namespace把插件类放到独立名字空间，避免与其他包的同名类冲突。
namespace unitree_legged_control 
{

    // 构造时将命令、状态和内部伺服参数清零；&取地址，sizeof给出整块结构体的字节数。
    UnitreeJointController::UnitreeJointController(){
        memset(&lastCmd, 0, sizeof(unitree_legged_msgs::MotorCmd));
        memset(&lastState, 0, sizeof(unitree_legged_msgs::MotorState));
        memset(&servoCmd, 0, sizeof(ServoCmd));
    }

    UnitreeJointController::~UnitreeJointController(){
        // 析构函数在实例销毁时运行，关闭这个关节的力传感器与命令订阅。
        sub_ft.shutdown();
        sub_cmd.shutdown();
    }

    // 力/力矩话题回调：按关节轴取相应力矩分量；这里只缓存sensor_torque，不在此执行PD。
    // ConstPtr是指向只读消息的智能指针，msg->字段用于访问消息内容。
    void UnitreeJointController::setTorqueCB(const geometry_msgs::WrenchStampedConstPtr& msg)
    {
        if(isHip) sensor_torque = msg->wrench.torque.x;
        else sensor_torque = msg->wrench.torque.y;
        // printf("sensor torque%f\n", sensor_torque);
    }

    // 普通ROS回调接收上层命令；这里只更新目标，不代表此刻电机就已经到达目标。
    void UnitreeJointController::setCommandCB(const unitree_legged_msgs::MotorCmdConstPtr& msg)
    {
        // mode选择模式；q/dq是目标位置/速度，tau是前馈力矩，不是已经施加的总力矩。
        lastCmd.mode = msg->mode;
        lastCmd.q = msg->q;
        lastCmd.Kp = msg->Kp;
        lastCmd.dq = msg->dq;
        lastCmd.Kd = msg->Kd;
        lastCmd.tau = msg->tau;
        // the writeFromNonRT can be used in RT, if you have the guarantee that
        //  * no non-rt thread is calling the same function (we're not subscribing to ros callbacks)
        //  * there is only one single rt thread
        // 将最新命令写入实时缓冲区；update()在控制线程读取，避免直接依赖回调执行时机。
        command.writeFromNonRT(lastCmd);
    }

    // Controller initialization in non-realtime
    // init在加载插件时调用：建立关节句柄、读取URDF和ROS接口，成功返回true。
    // EffortJointInterface表示最终向仿真关节写力矩；NodeHandle引用n提供当前控制器的命名空间。
    bool UnitreeJointController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
    {
        isHip = false;
        isThigh = false;
        isCalf = false;
        // rqtTune = false;
        sensor_torque = 0;
        // 例如/aliengo_gazebo/FR_hip_controller；参数joint给出其对应的URDF关节名称。
        name_space = n.getNamespace();
        if (!n.getParam("joint", joint_name)){
            ROS_ERROR("No joint given in namespace: '%s')", n.getNamespace().c_str());
            return false;
        }
        
        // load pid param from ymal only if rqt need 
        // if(rqtTune) {
#ifdef rqtTune
            // #ifdef是编译期开关：只有启用rqtTune，才编入从参数服务器读取PID的这段代码。
            // Load PID Controller using gains set on parameter server
            if (!pid_controller_.init(ros::NodeHandle(n, "pid")))
                return false;
#endif
        // }

        // 从robot_description解析模型；后面的限位函数读取此关节的类型与limits字段。
        urdf::Model urdf; // Get URDF info about joint
        if (!urdf.initParamWithNodeHandle("robot_description", n)){
            ROS_ERROR("Failed to parse urdf file");
            return false;
        }
        joint_urdf = urdf.getJoint(joint_name);
        if (!joint_urdf){
            ROS_ERROR("Could not find joint '%s' in urdf", joint_name.c_str());
            return false;
        }
        // 根据名字判断侧摆hip或小腿calf，以便setTorqueCB选择力矩测量轴。
        if(joint_name == "FR_hip_joint" || joint_name == "FL_hip_joint" || joint_name == "RR_hip_joint" || joint_name == "RL_hip_joint"){
            isHip = true;
        }
        if(joint_name == "FR_calf_joint" || joint_name == "FL_calf_joint" || joint_name == "RR_calf_joint" || joint_name == "RL_calf_joint"){
            isCalf = true;
        }        
        // 句柄不是机器人本体副本，而是读位置/力矩和写力矩命令的接口。
        joint = robot->getHandle(joint_name);

        // Start command subscriber
        sub_ft = n.subscribe(name_space + "/" +"joint_wrench", 1, &UnitreeJointController::setTorqueCB, this);
        // 相对话题command会加上当前命名空间；20是队列容量，&类名::方法绑定本实例回调。
        sub_cmd = n.subscribe("command", 20, &UnitreeJointController::setCommandCB, this);

        // pub_state = n.advertise<unitree_legged_msgs::MotorState>(name_space + "/state", 20); 
        // Start realtime state publisher
        // reset接管新建的发布器；RealtimePublisher支持update中尝试加锁，避免等待普通发布操作。
        controller_state_publisher_.reset(
            new realtime_tools::RealtimePublisher<unitree_legged_msgs::MotorState>(n, name_space + "/state", 1));        

        return true;
    }

    // 下列set/get函数操作pid_controller_；默认update的实际PD使用MotorCmd里的Kp/Kd。
    // const double &是只读引用；getGains的非常量引用参数则允许函数写回查询结果。
    void UnitreeJointController::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup)
    {
        pid_controller_.setGains(p,i,d,i_max,i_min,antiwindup);
    }

    void UnitreeJointController::getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup)
    {
        pid_controller_.getGains(p,i,d,i_max,i_min,antiwindup);
    }

    void UnitreeJointController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
    {
        bool dummy;
        pid_controller_.getGains(p,i,d,i_max,i_min,dummy);
    }

    // Controller startup in realtime
    // 控制器激活时以当前角度作为初始目标，速度和前馈力矩置零，再初始化实时缓冲区。
    void UnitreeJointController::starting(const ros::Time& time)
    {
        // lastCmd.Kp = 0;
        // lastCmd.Kd = 0;
        // 起点来自实际仿真关节，不能假设机器人启动时每个角度都是0。
        double init_pos = joint.getPosition();
        lastCmd.q = init_pos;
        lastState.q = init_pos;
        lastCmd.dq = 0;
        lastState.dq = 0;
        lastCmd.tau = 0;
        lastState.tauEst = 0;
        command.initRT(lastCmd);

        pid_controller_.reset();
    }

    // Controller update loop in realtime
    // ros_control周期调用：取最新目标→按模式准备参数→读反馈→算力矩→写关节→发布状态。
    // period.toSec()是本轮控制步长（秒），不是固定写死的0.002，也不是系统启动后的总时间。
    void UnitreeJointController::update(const ros::Time& time, const ros::Duration& period)
    {
        double currentPos, currentVel, calcTorque;
        // readFromRT返回缓冲区数据指针；*解引用后复制成这一轮使用的命令快照。
        lastCmd = *(command.readFromRT());

        // set command data
        // PMSM=0x0A：按目标角度、速度、PD增益和前馈力矩组合控制。
        if(lastCmd.mode == PMSM) {
            servoCmd.pos = lastCmd.q;
            positionLimits(servoCmd.pos);
            servoCmd.posStiffness = lastCmd.Kp;
            // PosStopF是特殊停用标记而非正常关节角；关闭Kp项后不再追踪位置目标。
            if(fabs(lastCmd.q - PosStopF) < 0.00001){
                servoCmd.posStiffness = 0;
            }
            servoCmd.vel = lastCmd.dq;
            velocityLimits(servoCmd.vel);
            servoCmd.velStiffness = lastCmd.Kd;
            // VelStopF同样表示停用速度反馈项；fabs取绝对值，用容差比较浮点数。
            if(fabs(lastCmd.dq - VelStopF) < 0.00001){
                servoCmd.velStiffness = 0;
            }
            servoCmd.torque = lastCmd.tau;
            effortLimits(servoCmd.torque);
        }
        // BRAKE=0x00：取消位置刚度与前馈，设目标速度0、阻尼20，形成速度阻尼力矩。
        if(lastCmd.mode == BRAKE) {
            servoCmd.posStiffness = 0;
            servoCmd.vel = 0;
            servoCmd.velStiffness = 20;
            servoCmd.torque = 0;
            effortLimits(servoCmd.torque);
        }

        // } else {
        //     servoCmd.posStiffness = 0;
        //     servoCmd.velStiffness = 5;
        //     servoCmd.torque = 0;
        // }
        
        // rqt set P D gains
        // if(rqtTune) {
#ifdef rqtTune
            double i, i_max, i_min;
            getGains(servoCmd.posStiffness,i,servoCmd.velStiffness,i_max,i_min);
#endif
        // } 

        // 反馈位置由关节句柄读取。computeVel采用位置差分，并混合上轮速度作平滑：
        // dq=0.35*dq_last+0.65*(q-q_last)/dt，而不是直接读取joint.getVelocity()。
        currentPos = joint.getPosition();
        currentVel = computeVel(currentPos, (double)lastState.q, (double)lastState.dq, period.toSec());
        // computeTorque位于unitree_joint_control_tool.cpp：
        // tau=Kp*(q_des-q)+Kd*(dq_des-dq)+tau_ff；servoCmd.torque对应tau_ff。
        calcTorque = computeTorque(currentPos, currentVel, servoCmd);      
        effortLimits(calcTorque);

        // 此接口写入的是最终力矩；Gazebo随后按动力学推进，角度不会立即跳到目标。
        joint.setCommand(calcTorque);

        lastState.q = currentPos;
        lastState.dq = currentVel;
        // lastState.tauEst = calcTorque;
        // lastState.tauEst = sensor_torque;
        // 发布字段名虽叫tauEst，这里实际取joint.getEffort()，不是上面calcTorque变量的直接复制。
        lastState.tauEst = joint.getEffort();

        // pub_state.publish(lastState);
        // publish state
        // &&短路求值：发布器存在才尝试加锁；本轮拿不到锁就跳过发布，不在控制循环等待。
        if (controller_state_publisher_ && controller_state_publisher_->trylock()) {
            controller_state_publisher_->msg_.q = lastState.q;
            controller_state_publisher_->msg_.dq = lastState.dq;
            controller_state_publisher_->msg_.tauEst = lastState.tauEst;
            controller_state_publisher_->unlockAndPublish();
        }

        // printf("sensor torque%f\n", sensor_torque);

        // if(joint_name == "wrist1_joint") printf("wrist1 setp:%f  getp:%f t:%f\n", servoCmd.pos, currentPos, calcTorque);
    }

    // Controller stopping in realtime
    // 原实现的停止回调为空；它本身没有额外执行“回固定站立”等状态机动作。
    void UnitreeJointController::stopping(){}

    // 引用参数会原地修改传入的数值，clamp把值夹在URDF指定范围内。
    // 注意三个限位函数都只处理REVOLUTE/PRISMATIC；CONTINUOUS关节不会进入下面的分支。
    void UnitreeJointController::positionLimits(double &position)
    {
        if (joint_urdf->type == urdf::Joint::REVOLUTE || joint_urdf->type == urdf::Joint::PRISMATIC)
            clamp(position, joint_urdf->limits->lower, joint_urdf->limits->upper);
    }

    // 速度按正负velocity限幅；这里是对目标速度处理，不是检测真实关节一定没有超速。
    void UnitreeJointController::velocityLimits(double &velocity)
    {
        if (joint_urdf->type == urdf::Joint::REVOLUTE || joint_urdf->type == urdf::Joint::PRISMATIC)
            clamp(velocity, -joint_urdf->limits->velocity, joint_urdf->limits->velocity);
    }

    // 力矩限幅既用于前馈，也用于PD相加后的最终力矩；是否生效仍取决于上述关节类型判断。
    void UnitreeJointController::effortLimits(double &effort)
    {
        if (joint_urdf->type == urdf::Joint::REVOLUTE || joint_urdf->type == urdf::Joint::PRISMATIC)
            clamp(effort, -joint_urdf->limits->effort, joint_urdf->limits->effort);
    }

} // namespace

// Register controller to pluginlib
// 注册插件类型，使controller_manager能根据配置里的类型名创建此类实例。
PLUGINLIB_EXPORT_CLASS(unitree_legged_control::UnitreeJointController, controller_interface::ControllerBase);
