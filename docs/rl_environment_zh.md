# 本机强化学习环境与驱动状态

检查日期：2026-11-12（Asia/Shanghai）。课程入口见 [第9～12课目录](course_index_zh.md)，训练参考与版本见 [资料说明](rl_sources_zh.md)。驱动已安装并加载，完整记录见 [安装验收](driver_installation_2026-11-12_zh.md)。

**2026-11-16 更新：** 已为第 10 课建立独立 `.venv-ppo`（Python 3.12、PyTorch 2.7.1+cu128、CUDA 12.8），在 RTX 5060 Laptop 上实测前向/反向传播并完成 80 轮 PPO 连续速度训练。启动入口、保存与续训见 [第 10 课](lesson_10_zh.md#9-运行完整-ppo-训练器)，实际设备和指标见 [验证报告](reports/lesson10-ppo-trainer.md)。下面保留的是 11 月 12 日的驱动安装验收记录；ROS 容器的原有 CPU PyTorch 与系统 Python 未被替换。

## 当前实际状态

| 项目 | 检查结果 |
|---|---|
| 系统 | Ubuntu 24.04，当前内核6.8.0-147-generic |
| 显卡 | `nvidia-smi`实际识别NVIDIA GeForce RTX 5060 Laptop GPU，8151 MiB显存；Intel核显继续运行 |
| NVIDIA训练驱动 | 595.99.02已加载，普通用户运行nvidia-smi通过；PCI设备绑定nvidia，CUDA Driver API初始化通过、计算能力12.0 |
| 已安装包 | `nvidia-driver-595-open`，版本 `595.99.02-0ubuntu0.24.04.1`，共18个包；DKMS为当前内核安装成功 |
| 训练Python组件 | 主机未安装torch、MuJoCo、Gymnasium、onnxruntime；没有为本次课程安装全尺寸训练环境 |
| 可立即学习 | 第9、10、12课的标准库CPU离线实验，通过现有无网络Docker入口执行 |
| 默认物理仿真 | 切换到 ROS 1 Noetic、Gazebo 11 和适配 AlienGo 的 Guide，不自动启用强化学习；验证范围见 ROS 1 仿真说明 |

显卡识别依据是 [NVIDIA官方支持列表](https://github.com/NVIDIA/open-gpu-kernel-modules/blob/main/README.md) 与本机PCI ID；驱动选择来自本机Ubuntu工具，而非把网站上的最新版本直接替换进系统。[Ubuntu官方安装说明](https://ubuntu.com/server/docs/nvidia-drivers-installation/)

## 下载与安装记录

驱动安装材料放在 `downloads/nvidia-595-open/`。`manifest.json`记录每个包的来源URL、版本、大小与SHA256，`SHA256SUMS`用于本地复核；下载时逐包比较APT软件源元数据中的SHA256。实际数量和总大小见 [本次验证](rl_verification.md)。这些是Ubuntu仓库的deb包，不是NVIDIA的.run安装器。

2026-11-11只完成下载，安装当时受到管理员认证限制。2026-11-12用户明确授权安装并完成管理员认证，已使用校验后的本地包安装；APT退出0、dpkg审计通过、18个包版本逐项一致、依赖损坏计数为0。认证输入没有写入脚本、环境文件或安装日志。

## 当前验收与下次开机检查

先进入工作区并验证下载文件：

```bash
cd /home/jack/aliengo_learning
(cd downloads/nvidia-595-open && sha256sum -c SHA256SUMS)
```

驱动已在当前会话正常加载，无需再次安装。当前与下次开机均可运行：

```bash
nvidia-smi
dkms status
```

本次实际看到驱动595.99.02、RTX 5060 Laptop、8151 MiB显存，以及当前内核的DKMS状态installed。未重启电脑，也未切换桌面的主显卡；下次开机后的自动加载还应再检查一次。

`nvidia-smi`显示CUDA 13.2表示驱动支持能力，不能据此推定CUDA Toolkit或PyTorch已安装。另用系统 `libcuda.so.1` 实际调用cuInit和设备查询成功，得到1张GPU、计算能力12.0；本次没有执行GPU计算内核或PyTorch张量运算。

后续训练环境应使用独立venv/容器。RTX 50系列属于新架构；第11课锁定的旧Isaac Gym训练代码用于阅读与复现设计，不能照旧版Python/PyTorch/CUDA组合直接认定本机可跑。现代训练路线需按对应仿真器官方支持矩阵选择版本，再移植AlienGo任务，见第11课。不要在 ROS 1 仿真镜像内混装旧Python训练环境。

具备合适PyTorch环境后，应至少验证设备识别和一次实际GPU运算，而不是只检查import：

```python
import torch
assert torch.cuda.is_available()
print(torch.__version__, torch.version.cuda)
print(torch.cuda.get_device_name(0), torch.cuda.get_device_capability(0))
x = torch.randn(512, 512, device="cuda")
y = x @ x.T
torch.cuda.synchronize()
assert torch.isfinite(y).all().item()
```

这段GPU检查没有在本次执行；完整训练还要通过仿真器自己的小场景与单环境测试，最后再扩展并行环境数。驱动、张量运算、仿真器加载和AlienGo训练是分别需要验收的步骤。
