# SDD ledger — plan: docs/superpowers/plans/2026-11-11-rl-courses.md


2026-11-12清理说明：下文过程日志路径为历史记录，原始日志已按要求删除；课程与验收摘要保留。

## 授权与执行选择

用户要求补充强化学习运动控制课程，完成后关机。沿用已授权的中文教学形式，自主完成教材、示例、验证与正常关机。

Ruling: 根目录是学习资料工作区，非Git仓库；新增教材与实验在本目录完成，不初始化Git、不改已有上游控制代码。各实现任务独占文件，独立并行，根代理负责全局索引与集成检查。

Ruling: 不安装大型训练运行时或显卡驱动。本机PCI可见Intel与NVIDIA显卡，但没有nvidia-smi、torch等训练组件；课程提供标准库可运行实验与精确版本的条件训练路线。若需完整训练，先验证驱动及相容环境。

后续用户明确授权“需要什么驱动自己下载”，并再次要求无人值守完成后关机。已确认GPU为RTX 5060 Laptop，Ubuntu推荐595-open；root下载Ubuntu软件源的驱动依赖包并校验SHA256。`sudo -n true`需要密码，故系统安装受管理员认证阻止，不利用其他途径绕过。上述Ruling由用户新授权细化为“完成所需驱动下载，安装状态按权限与实际结果记录”。

## 任务状态

- Task1：rl_foundations 完成第9～10课、Q-learning/GAE/PPO数值实验与18项测试，RED→GREEN；8条实际Docker命令通过。
- Task2：rl_training 完成第11课与资料索引、下载3个固定训练参考；平地48/粗糙235观测、旧API与Blackwell边界、seed覆盖时机均核查；完整训练未执行。
- Task3：rl_deployment 完成第12课与离线策略契约、17项测试，RED→GREEN；metadata数量上限测试先复现失败再修复；实际CLI与异常拒绝通过。
- Task4：完成。root 已更新全局入口、学习路线与进度、根锁共10仓库。独立重跑125项通过，10条DockerCLI符合退出码，逐包复查18份驱动SHA256与包元数据。第11课文档AST/XML块实际通过。rl_final_review独立审阅无未解决的Critical/Important/Minor；35项新增测试亦独立重跑通过。
- Task5：已在完成任务1～4并同步磁盘后向logind安排正常poweroff；系统属性确认时间为 2026-11-11T20:15:06+08:00。实际断电发生在本会话结束后，不声称已观察到断电。结果见 logs/rl/shutdown.json。

## 预检查

原有90项单元测试通过，日志 logs/rl/baseline-tests.log。既有上游文件状态记录在 logs/rl/source-status-before.json，保留用户已有注释修改。初始没有运行仿真容器。系统CanPowerOff返回yes，无需再次请求用户授权。

## 集成验证

- logs/rl/unittest.log：125 tests, OK；logs/rl/final/commands.json：10条容器CLI全部符合预期且无Traceback。
- logs/rl/lesson11-documented-source-check.log：文档源码块真实执行，48/12/12、200/50Hz、0.48s/98304；未导入GPU框架。
- logs/rl/source-status-after.json：10仓HEAD符合锁定，原有7仓状态与开始前一致。
- downloads/nvidia-595-open：18包314,913,470bytes，SHA256与dpkg包名/版本均复查；未安装，原因是缺管理员认证。
- docs/assets/lesson10_ppo.png：正/负优势裁剪曲线已从实际数值函数绘制、视觉检查。
- 独立审阅关闭两项：第11课补清旧rsl_rl用当前V补偿超时与标准最终状态bootstrap的差异；播放命令用&&确保checkpoint缺失不继续。未改锁定上游；31文档239本地链接零问题。
