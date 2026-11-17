# GBPlanner 转弯撞柱修复记录

日期：2026-11-14。用户确认 AlienGo 在转弯或调整朝向时，前端碰到货架立柱。原事故未保存同步路径、朝向及接触数据，不能声称精确复现了原事故。用户进一步要求不要在官方检查上增加过多限制。

## 当前修复范围

- 保留原 .75×.45×.34m 尺寸、跟随前瞻 .25m、横向偏差门限 .30m、速度及输入时效门限；没有全方向膨胀包络、额外安全余量或独立空间校验服务。
- [几何补丁](../../ros1/patches/gbplanner_turn_clearance.patch) 修补原 MapManager 检查：按实际行进朝向检查完整机身平移体积，覆盖盒内部、路径两端和非整格体素边界。使用连续分离轴检测，处理零长度、负坐标和垂直分量。
- 在原路径输出复查阶段，按当前 yaw、实际路径转角和最终 yaw 检查转身，只检查实际转角。转角分为至多 2° 区间，区间内角点运动上界小于 8mm，用于覆盖连续旋转；该数值不是人为安全距离。
- 检查不通过返回空路径，可选优化/插值之后也复查，通过前不写入全局图。保留官方未知空间许可标志，不增加一套准入政策。
- 修复使能时沿用旧站稳状态的竞态：已有站立 RPC 后等待一条新的站稳状态；原就绪条件保持不变，没有新增空间准入条件。真实 ROS 回归先复现旧状态错误放行。
- 修复协调器截取 .6m 短段后制造额外转头的问题：只对新增截断终点采用该段行进方向，完整规划路径的真实终点朝向保持不变。原跟随参数全部保留。
- 验证时还发现冷启动模型加载 32–36s，而关节控制器 spawner 默认 30s 超时退出，导致规划启动前就站立失败。仅把 [仿真加载等待](../../ros1/mid360/gazebo.launch) 延长到 120s，未改导航门限。
- 官方起点清图范围保持原样；原始 references 源码保持原样。修改从补丁可重建，不只存在于生成目录。

## 当前验证

真实链接 MapManager 回归 **32/32**，优化后路径拒绝回归 **2/2** 均已通过；ROS 正常规划和最终整机闭环均已通过。宿主测试已实际通过 **195 passed、23 skipped**；跳过项不计通过。截断终点回归在旧代码上得到 0 而非 pi/2，修复后通过；保留完整终点朝向的伴随回归通过。

真实链接的 MapManager 回归覆盖前端扫柱、内部柱子、零长度、短边、负坐标、未知空间，以及“原宽度直行、小角度转向可过，实际大转角扫柱须拒绝”。真实链接 Rrg 回归注入优化后进入未知区域的候选路径，检查两个输出入口拒绝。独立审查指出的未知空间许可语义已按旧方法分别保留，并通过真实链接回归：方法 1 允许许可的未知空间，方法 0 沿用盒查询的 Unknown 返回，非有限输入不会被转换成 Free。

正常 ROS 后端验证取得两条可执行路径；之前一次正常路径后拒绝一次候选的记录保留在 backend-current-initial.log。服务测试要求最多六次请求内获得两条可执行路径，不要求执行被拒绝的候选。

[运行时参数快照](../../logs/gbplanner-turn-clearance/runtime-params-current.json) 确认原尺寸、零 size/safety extension，以及已移除全方向包络和额外余量参数。[启动快照](../../logs/gbplanner-turn-clearance/startup-current.json) 确认 13/13 控制器运行。

最终整机 [GPlLEh 原始报告](../../logs/gbplanner-trial-GPlLEh/summary.json) **PASS，13/13**，墙钟 222.63s。手动采图 1.684m，自主完成 3 段、净位移 1.611m；命令中断、暂停时钟、主动停止及地图保存均通过。记录的 5 条人工截断路径段尾朝向与行进方向的最大误差为 2.56e-15rad。

使能握手的实际 Noetic 边界回归 **12/12** 通过，包含旧站稳消息不能代替本次请求的新回执，以及未知脚印仅供诊断、仍执行官方路径的既有回归。[最终验证清单](../../logs/gbplanner-turn-clearance/verification-manifest.json) 汇总结果及当前源文件 SHA256。

最终日志： [构建](../../logs/gbplanner-turn-clearance/build-current.log)、[宿主测试](../../logs/gbplanner-turn-clearance/host-tests-final-current.log)、[ROS 后端](../../logs/gbplanner-turn-clearance/backend-final-current.log)、[执行记录](../../logs/gbplanner-turn-clearance/progress.md)。修改前备份在 [before](../../logs/gbplanner-turn-clearance/before)。

## 已撤回的保守实验

先前全方向正方形包络、.10m 额外余量及 5cm 跟随偏差方案已按用户要求撤回；[JwxRBP](../../logs/gbplanner-trial-JwxRBP/summary.json) 的 PASS 不代表当前方案。[pVoBti](../../logs/gbplanner-trial-pVoBti/summary.json) 因 camera stopped or stale 停车，未修改该时效保护。相关记录保留，不作为当前验证依据。当前方案的 85QgLi/vxj0if 站立失败为加载器 30s 超时，S3Bkcm 的使能失败为旧站稳状态竞态；均在最终 GPlLEh 之前，原因与修复见上文。

## 范围

这是原碰撞几何和短段末端朝向的修补，不是完整腿部/步态、低障碍或动态障碍保护。原跟随器仍允许轨迹偏差和提前转向，没有新增运动中地图复核；本次不宣称对所有真实执行轨迹零碰撞。
