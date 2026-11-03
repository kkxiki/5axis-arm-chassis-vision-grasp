# 5轴抓取项目整理版

更新时间：2026-09-08

当前项目以 Jetson `/home/nvidia/calibration` 内的测试版 demo 为运行基准；Windows 目录用于备份、查看、文档整理和后续同步。不要用 Windows 旧脚本覆盖 Jetson 测试版，除非明确确认要同步修改。

## 目录说明

- 00_项目说明文档：项目总说明、v3 当前进度、入口命令和操作说明。
- 01_标定数据：本次相机-机械臂标定图片、样本、最终外参、求解脚本和日志。
- 02_固定标定位姿：本次采集的机械臂 TCP 位姿，下次重新标定可按这些点位复现。
- 03_手柄控制_xbox：Xbox 控制机械臂/夹爪相关代码、配置和可执行文件。
- 04_机械臂网关_arm_gateway：ArmGateway 代码、配置和可执行文件。
- 05_机械臂SDK_ROS_源码：臻识/知时机械臂 SDK 与 ROS2 封装源码，已排除 build/install/log 构建产物。
- 06_历史参考_legacy：旧项目里用于参考的历史标定资料、Jetson 历史备份、已归档散落目录。
- 07_夹爪CTAG2F120：知行 CTAG2F120 二指夹爪控制脚本和 SDK 文件。
- 08_v3单步抓水瓶测试：当前已验证的 v3 抓水瓶闭环程序、Jetson 镜像、运行记录和历史备份。
- 09_辅助工具_tools：Windows 侧辅助脚本，例如 Jetson 文件传输、ROI 辅助工具。
- 99_原始备份包：从 Jetson 拉下来的原始 tar.gz 备份包和 manifest。

## 当前主线入口

Jetson 运行目录：

```bash
cd /home/nvidia/calibration
```

启动相机实时画面：

```bash
bash /home/nvidia/calibration/start_v3_bottle_vision.sh
```

实时画面：

```text
http://192.168.8.100:8090/
```

执行三轮抓水瓶程序：

```bash
python3 /home/nvidia/calibration/run_three_cycles_v3.py
```

详细说明见：

```text
00_项目说明文档/V3_BOTTLE_PROJECT_README_20260908.md
```

## 本次标定结果

最终外参文件：01_标定数据/results/best_eye_in_hand_result.yaml

- translation_m: [0.032654410804, 0.138084416530, -0.152119718610]
- quaternion_xyzw: [0.001216607412, -0.170169262469, 0.985405094289, -0.004212140450]
- translation_rms_mm: 4.562804
- rotation_rms_deg: 1.424805

## 注意

自动回放固定标定位姿前，必须确认机械臂、相机安装、标定板位置和现场安全区域与本次一致。

当前 v3 抓水瓶流程的点位文件在 `08_v3单步抓水瓶测试/01_Jetson运行程序/poses` 中有 Windows 镜像；Jetson 实际运行点位在 `/home/nvidia/calibration/poses`。
