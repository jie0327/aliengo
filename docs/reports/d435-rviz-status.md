# D435 RViz 状态诊断（2026-11-14）

用户报告 RViz 的 D435 显示偶尔发黄，未保留状态文字。本次自行运行同一 AlienGo + D435 + Super-LIO 配置，用嵌入式 RViz 加载实际 `mapping.rviz`，每 100 ms 读取真实显示属性树，记录文字、颜色、仿真时间和状态变化。

## 复现到的两种情况

1. **D435 彩色图像启动时黄色**：实际文字为 `Image: No Image received`，颜色 `#c08000`。收到第一帧后转为正常，后续行走/转向期间没有再次记录图像黄色。这表示显示初始化时等待首帧；本次没有复现持续图像中断。
2. **D435 彩色点云反复红色**：实际为 `Status: Error`、颜色 `#c02020`，反复提示 `Lookup would require extrapolation ... into the future`，随后又恢复 `Transform OK`。

点云时间戳来自相机当前仿真时刻；`lio_map → base` 的 TF 由雷达频率的 `/lio/odom` 估计结果发布。80 个实际点云样本在收到时都需要等待更新的 TF，最大时间差约 0.1072 仿真秒。原显示队列只有两帧，等待期间就可能丢弃尚未获得对应位姿的点云。

## 修改

仅把 `ros1/super_lio/mapping.rviz` 中 **D435 彩色点云** 的 `Queue Size` 从 2 改为 10。相机消息时间戳、LIO 估计和 TF 时间均保持原语义。图像仍使用原来的两帧队列，保留其真实首帧等待提示。

## 实测日志

- 修改前：[RViz 状态](../../logs/super-lio-trial-PyXWSH/rviz-status.jsonl)、[相机/TF 时间样本](../../logs/super-lio-trial-PyXWSH/camera-tf-timing.json)。统计运行阶段记录到 164 条未来 TF 错误状态；图像只记录到一次启动等待。
- 修改后：[RViz 状态](../../logs/super-lio-trial-LgTSa6/rviz-status.jsonl)。以相同启动、站立、建图、行走、停车和转向流程复测，运行阶段错误状态记录为 0；两次完整运动验证均 PASS。汇总见 [comparison.json](../../logs/d435-warning/comparison.json)。
- 诊断工具：[状态探针](../../logs/d435-warning/rviz_status_probe.py)，读取 RViz 的实际 Status 子属性，不根据终端日志猜测颜色。

统计从首次 `Transform OK` 后一仿真秒开始，到最后一个运动阶段结束，避开估计器尚未生成地图 TF 和测试主动关闭 LIO 的阶段。修改前窗口为 17.9914–37.7932 仿真秒、374 条状态记录；修改后为 12.4402–37.657 仿真秒、257 条记录。修改前探针启动得较晚，修改后覆盖了全部运动阶段。这些是每 100 ms 采样得到的变化状态记录，不代表独立丢帧数量。

修改后完整日志仍有 13 条点云错误状态：3 条在地图 TF 尚未就绪的启动阶段，10 条在最后运动阶段结束、LIO 停止后的关闭阶段。没有发现其余运行错误。第一次地图 TF 出现前和估计器停止发布后，点云仍可能显示 TF 提示；十帧缓冲不能补出不存在的位姿。
