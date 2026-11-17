# MID360 安装与键盘行走修复验证

所有对照在同一 ROS Noetic 镜像和本机上串行运行，容器限制 4 CPU、4 GiB 内存，无交换空间。带雷达的测试均保留默认 6000 条射线/帧、10 Hz 仿真目标频率；使用真实伪终端按键，顺序为站立、小跑、两次 W、停车。日常仿真入口的资源上限仍为 6 CPU、6 GiB。

| 场景 | 前进仿真秒 | 实时因子 | 最大横滚采样 | 结果 |
|---|---:|---:|---:|---|
| 裸机，无界面 | 30.01 | 0.978 | 0.63° | PASS |
| MID360，无界面，旧时钟 | 30.01 | 0.929 | 0.34° | PASS |
| MID360，GUI 未接通 GPU，旧时钟 | 9.33 | 0.097 | 44.79° | FAIL |
| MID360+支架，GPU GUI，旧时钟 | 30.00 | 0.841 | 0.76° | PASS |
| MID360+支架，GPU GUI，仿真时钟 | 60.00 | 0.855 | 0.38° | PASS |

未接通 GPU 的 GUI 组在前进约 9.33 个仿真秒后触发姿态终止条件，最终收到的横滚约 46°；不是完整跑完 30 秒。第一轮低 PID 上限导致 GUI 启动失败的试验已排除，不作为行走数据。

当前 GPU 渲染器由 GLX 实测及 Gazebo Ogre 日志双重确认：`NVIDIA GeForce RTX 5060 Laptop GPU/PCIe/SSE2`，OpenGL `4.6.0 NVIDIA 595.99.02`。原图形入口缺少设备/图形库，渲染负载上升后出现 CPU 调度拥塞；旧控制周期和步态使用墙钟，使物理时间变慢时控制时序失配。无界面对照没有出现大幅雷达性能回退。没有做宿主原生仿真的独立对照，因此本报告不量化 Docker 自身的开销。

修复包括 NVIDIA 图形设备及匹配用户态库透传、仿真控制节拍/步态统一使用 ROS `/clock`，以及新增填充 42 mm 间隙的固定支架。支架为水平安装的近似模型，0.10 kg；原有 trunk 到雷达扫描原点的变换不变。ODE 物理与 Livox 射线检测仍为 CPU 实现。

## 最终测试

- 连续前进 60.00 仿真秒，约 70.20 墙秒，前进 2.883 m。
- 最低机身高度 0.4265 m；最大绝对横滚 0.381°，最大绝对俯仰 0.187°。
- 点云 10.00 Hz、雷达 IMU 199.31 Hz，均按仿真时间计；全程点云每帧含 724～1576 个实际命中点。
- 空格停车后观察 5.01 仿真秒，平面漂移 0.0020 m。
- 暂停物理后 Ctrl+C，控制器 0.264 秒内退出，返回码 0。
- ROS 使用仿真时间但尚无首帧 `/clock` 时取消启动，0.316 秒内退出，返回码 0。
- ROS 工作区编译成功，147 项单元/组件测试通过；时钟测试覆盖暂停/等待与非仿真分支，支架测试覆盖默认及升高安装后的几何连接。

## 原始记录

- 裸机，无界面：[阶段统计](../../logs/keyboard-trial-RgLFW2/summary.json)、[姿态采样](../../logs/keyboard-trial-RgLFW2/samples.json)。
- MID360，无界面，旧时钟：[阶段统计](../../logs/keyboard-trial-HSj4sE/summary.json)、[姿态采样](../../logs/keyboard-trial-HSj4sE/samples.json)。
- MID360，GUI 未接通 GPU，旧时钟：[阶段统计](../../logs/keyboard-trial-1r3JDQ/summary.json)、[姿态采样](../../logs/keyboard-trial-1r3JDQ/samples.json)。
- MID360+支架，GPU GUI，旧时钟：[阶段统计](../../logs/keyboard-trial-STBjgS/summary.json)、[姿态采样](../../logs/keyboard-trial-STBjgS/samples.json)。
- MID360+支架，GPU GUI，仿真时钟：[阶段统计](../../logs/keyboard-trial-8d2INm/summary.json)、[姿态采样](../../logs/keyboard-trial-8d2INm/samples.json)。
- 最终 GPU：[Gazebo Ogre 日志](../../logs/keyboard-trial-8d2INm/ogre.log)。
- 首帧时钟前取消：[记录](../../logs/keyboard-trial-8d2INm/startup-cancel/summary.json)。
- [编译日志](../../logs/mid360-controller-build.log)、[单元测试日志](../../logs/mid360-unit-tests.log)。

## 复现

```bash
./scripts/verification/ros1_keyboard_trial.sh --launch mid360 --seconds 60 --gui --check-pause
```

当前 Guide 仍沿用近似的 AlienGo 动力学参数和 Go1 增益；本次没有做载荷辨识或真机验证。以上结论限定本机、平地、两次 W 的低速测试。
