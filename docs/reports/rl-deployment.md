# 第12课离线契约验收记录

日期：2026-11-11；目录：`/home/jack/aliengo_learning`。

范围：标准库程序，明示action fixture，无神经网络运行时、ROS发布、SDK报文或机器人连接。历史实验使用网络隔离容器；以下保留当时输出。现行复现入口为 `ros1_course.sh offline`，ROS 1 镜像中已验证 demo。

TDD：先添加16项行为测试，未实现时运行得到 `FAILED (failures=16)`；实现后16项通过。随后增加metadata缓存/计数上限测试，先得到2个失败子用例，再增加限制，最终第12课17项通过。

原实验输出摘录（表中命令已按现行入口调整；不是重新运行全部历史案例的记录）：

| 命令 | 实跑结果 |
|---|---|
| `python3 -m unittest discover -s tests -p 'test_lesson12.py' -v` | `Ran 17 tests`、`OK`，exit 0 |
| `python3 -m unittest discover -s tests -v` | `Ran 125 tests in 0.737s`、`OK`，exit 0（含并行编写的第9～10课测试） |
| `./scripts/ros1_course.sh offline lesson12/offline_policy_contract.py --metadata examples/lesson12/metadata.json --case valid` | accepted true；obs前9项 `[0.5,-1,0.25,0,-0,-1,1,-0.5,0.5]`；12条目标，首腿q_des `[0.25,0.55,-1.375]`，exit 0 |
| `./scripts/ros1_course.sh offline lesson12/offline_policy_contract.py --case shuffled` | accepted true；目标仍按FR、FL、RR、RL排列，exit 0 |
| `./scripts/ros1_course.sh offline lesson12/offline_policy_contract.py --input examples/lesson12/fixture.json --now 10.0` | accepted true；45维观测；首目标FR_hip_joint为0.25，exit 0 |
| `./scripts/ros1_course.sh offline lesson12/offline_policy_contract.py --demo --output examples/lesson12/decisions.json` | demo_passed true；2个正常案例接受、10个异常案例拒绝；exit 0 |
| `./scripts/ros1_course.sh offline lesson12/offline_policy_contract.py --case stale` | accepted false；DEGRADED；targets空；`snapshot is stale or in the future`；预期exit 1 |

demo关键输出：

```text
source: explicit_action_fixture_no_model
observation_dim: 45
history_dim: 135
example_pd_hz: 500.0
example_policy_hz: 50.0
policy_ticks: [0, 10, 20]
reset_previous_action: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

完整demo JSON：[decisions.json](../../examples/lesson12/decisions.json)。异常案例为stale、future、missing、duplicate_joint、nan、zero_quaternion、duplicate_stamp、backwards_stamp、bad_dt、bad_action。测试另覆盖未知关节、其他非有限字段、布尔/字符串、数组长度、metadata顺序与周期、拒绝锁定和reset。

教材区分教学45维观测与训练源码中的48/235维配置；真实训练与部署的关节顺序、默认姿态和PD参数仍需逐项确认。

未验证：真实checkpoint的导出前后数值等价、现有policy.pt来源、神经网络推理性能/实时deadline、RL控制器编译启动、Sim2Sim物理闭环、真实固件与电机行为。500/50Hz是合成时间线配置，DEGRADED是离线记录状态。
