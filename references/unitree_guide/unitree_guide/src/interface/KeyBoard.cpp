/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "interface/KeyBoard.h"
#include <iostream>

// 键盘把离散按键转换为两类输入：userCmd 用于状态切换，userValue 模拟手柄摇杆。
KeyBoard::KeyBoard(){
    userCmd = UserCommand::NONE;
    userValue.setZero();

    // 保存终端设置，再关闭规范输入 ICANON（无需回车）和 ECHO（不回显按键）。
    // &= 与按位取反 ~ 配合清除标志位；不会改变其余终端标志。
    tcgetattr( fileno( stdin ), &_oldSettings );
    _newSettings = _oldSettings;
    _newSettings.c_lflag &= (~ICANON & ~ECHO);
    tcsetattr( fileno( stdin ), TCSANOW, &_newSettings );

    // 单独线程阻塞等待键盘，避免控制循环等待输入；this 是当前 KeyBoard 对象的地址。
    pthread_create(&_tid, NULL, runKeyBoard, (void*)this);
}

// 析构时取消并等待键盘线程结束，再恢复终端原来的输入/回显设置。
KeyBoard::~KeyBoard(){
    pthread_cancel(_tid);
    pthread_join(_tid, NULL);
    tcsetattr( fileno( stdin ), TCSANOW, &_oldSettings );
}

// 数字键模拟手柄组合键；FSM 各状态决定是否接受对应命令，并非任何数字都能随时切换。
// 1=被动，2=固定站立，3=自由站立，4=小跑；0/9/8=平衡/摆腿/踏步测试。
// 启用 COMPILE_WITH_MOVE_BASE 时，5 才会产生 MOVE_BASE 使用的 L2_Y 命令。
UserCommand KeyBoard::checkCmd(){
    switch (_c){
    case '1':
        return UserCommand::L2_B;
    case '2':
        return UserCommand::L2_A;
    case '3':
        return UserCommand::L2_X;
    case '4':
        return UserCommand::START;
#ifdef COMPILE_WITH_MOVE_BASE
    case '5':
        return UserCommand::L2_Y;
#endif  // COMPILE_WITH_MOVE_BASE
    case '0':
        return UserCommand::L1_X;
    case '9':
        return UserCommand::L1_A;
    case '8':
        return UserCommand::L1_Y;
    case ' ':
        // 空格把所有摇杆值立即清零；返回 NONE 表示不发起状态切换请求。
        userValue.setZero();
        return UserCommand::NONE;
    default:
        return UserCommand::NONE;
    }
}

// 摇杆值无量纲，范围 [-1,1]；每读到一个字符累加/减 0.05，松键不会自动回零。
// 按住时终端重复字符会继续累加；min/max 限幅，空格或状态调用 zeroCmdPanel 才清零。
// w/s -> ly，d/a -> lx，i/k -> ry，l/j -> rx；冒号连写 case 使大小写走同一分支。
// 在 Trotting 中 ly 映射前后速度，-lx 映射左右速度，-rx 映射偏航角速度；实际单位由状态赋予。
// 在 FreeStand 中这些通道改为机身姿态/高度输入，因此本层不直接输出 m/s 或 rad/s。
void KeyBoard::changeValue(){
    switch (_c){
    case 'w':case 'W':
        // min<float> 显式指定模板类型为 float：正向增加，但不超过上界 1。
        userValue.ly = min<float>(userValue.ly+sensitivityLeft, 1.0);
        break;
    case 's':case 'S':
        userValue.ly = max<float>(userValue.ly-sensitivityLeft, -1.0);
        break;
    case 'd':case 'D':
        userValue.lx = min<float>(userValue.lx+sensitivityLeft, 1.0);
        break;
    case 'a':case 'A':
        userValue.lx = max<float>(userValue.lx-sensitivityLeft, -1.0);
        break;

    case 'i':case 'I':
        userValue.ry = min<float>(userValue.ry+sensitivityRight, 1.0);
        break;
    case 'k':case 'K':
        userValue.ry = max<float>(userValue.ry-sensitivityRight, -1.0);
        break;
    case 'l':case 'L':
        userValue.rx = min<float>(userValue.rx+sensitivityRight, 1.0);
        break;
    case 'j':case 'J':
        userValue.rx = max<float>(userValue.rx-sensitivityRight, -1.0);
        break;
    default:
        break;
    }
}

// pthread 需要普通/静态函数入口，不能直接传需要隐含 this 参数的非静态成员函数。
// void* 可传递对象地址；把 arg 转回 KeyBoard* 后，通过 -> 调用该对象的成员 run。
void* KeyBoard::runKeyBoard(void *arg){
    ((KeyBoard*)arg)->run(NULL);
    return NULL;
}

// 键盘线程主体；while(1) 持续工作，退出由析构函数的 pthread_cancel 触发。
void* KeyBoard::run(void *arg){
    while(1){
        // 每轮重建文件描述符集合，监听标准输入 stdin 对应的文件描述符。
        FD_ZERO(&set);
        FD_SET( fileno( stdin ), &set );

        // 最后一个参数 NULL 表示无超时：没有输入时阻塞在这里，控制线程仍可独立运行。
        res = select( fileno( stdin )+1, &set, NULL, NULL, NULL);

        if(res > 0){
            // 每次最多读取一个字节；先识别状态命令，没有状态命令时再尝试改摇杆值。
            // userCmd 会被本字符的结果覆盖；userValue 的累加结果则保留供控制循环读取。
            ret = read( fileno( stdin ), &_c, 1 );
            userCmd = checkCmd();
            if(userCmd == UserCommand::NONE)
                changeValue();
            _c = '\0';
        }
        // usleep 单位为微秒，此处为 1 ms；前面的 select 会阻塞，所以它不是 1 kHz 控制周期。
        usleep(1000);
    }
    return NULL;
}