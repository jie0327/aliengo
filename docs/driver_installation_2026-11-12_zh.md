# NVIDIA驱动安装验收

日期：2026-11-12（Asia/Shanghai）。用户明确授权安装；本次范围是此前准备的NVIDIA显卡驱动与18个依赖包。没有安装PyTorch、训练仿真器或CUDA Toolkit，没有重启或关闭电脑。

## 安装与验证结果

| 检查 | 实际结果 |
|---|---|
| 安装包 | 18个本地deb逐包SHA256通过，包名与版本核对通过 |
| APT | 安装成功，退出0；0升级、18新增、0删除 |
| 驱动版本 | nvidia-driver-595-open `595.99.02-0ubuntu0.24.04.1` |
| 内核 | 6.8.0-147-generic；匹配headers与gcc13.3.0齐全；Secure Boot关闭 |
| DKMS | `nvidia/595.99.02, 6.8.0-147-generic, x86_64: installed` |
| 内核模块 | version 595.99.02，vermagic与当前内核一致，license Dual MIT/GPL；initramfs已更新 |
| 包状态 | dpkg审计为空；18个包安装版本均与清单一致；APT依赖损坏计数0 |
| 驱动加载 | 正常加载nvidia和nvidia_uvm；PCI `01:00.0` 实际绑定nvidia |
| 普通用户GPU查询 | nvidia-smi退出0；RTX 5060 Laptop GPU，8151 MiB显存，驱动595.99.02 |
| CUDA驱动接口 | libcuda.so.1的cuInit成功，设备数1，设备名正确，计算能力12.0 |

安装初次尝试使用APT的no-download选项，但包尚在工作目录而非APT缓存，APT选择软件源候选后无法取包，退出100，未解包。将已验证包放入 `/var/cache/apt/archives/` 后，使用同一no-download安装成功，没有再次下载315 MB。

安装完成时驱动尚未加载，第一次nvidia-smi退出9。随后正常执行modprobe加载nvidia和nvidia_uvm，udev完成处理，普通用户查询通过。没有卸载nouveau、停止显示管理器、执行prime-select或改写X配置；Intel i915继续运行。

## 保留的验收摘要

2026-11-12按要求清理原始安装日志与调试快照，本页保留版本、结果、处理经过和验证范围。下载包清单 `downloads/nvidia-595-open/manifest.json` 保留安装与运行验证状态。认证输入没有写入文件。

需要重新检查当前状态时，可运行：

```bash
nvidia-smi
dkms status
dpkg --audit
```

## 训练环境边界

本次通过的是驱动安装、模块加载、GPU识别与CUDA驱动初始化。`nvidia-smi`中的CUDA 13.2不是CUDA Toolkit安装证明；未执行GPU计算内核、PyTorch运算、训练仿真器场景或AlienGo训练，也没有验证下次启动后的自动加载。

下一步按 [强化学习环境说明](rl_environment_zh.md) 和 [第11课](lesson_11_zh.md) 选择兼容RTX 50系列的独立训练环境，先测试张量计算与官方小场景，再配置AlienGo任务。
