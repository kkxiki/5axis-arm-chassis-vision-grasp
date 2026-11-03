# 五轴协作机械臂视觉抓取系统

<p>
<img alt="ROS 2" src="https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&logoColor=white">
<img alt="Python" src="https://img.shields.io/badge/Python-3.10-3776AB?logo=python&logoColor=white">
<img alt="C++" src="https://img.shields.io/badge/C%2B%2B-17-00599C?logo=c%2B%2B&logoColor=white">
<img alt="Jetson" src="https://img.shields.io/badge/Platform-Jetson-76B900?logo=nvidia&logoColor=white">
<img alt="OpenCV" src="https://img.shields.io/badge/OpenCV-4.x-5C3EE8?logo=opencv&logoColor=white">
<img alt="YOLOv8" src="https://img.shields.io/badge/Detection-YOLOv8-00FFFF?logo=yolo&logoColor=black">
<a href="LICENSE"><img alt="License" src="https://img.shields.io/badge/License-MIT-yellow.svg"></a>
</p>

面向五轴协作机械臂的**手眼标定 + 视觉闭环抓取**完整实现：从相机-机械臂外参标定、YOLOv8 目标检测、深度反投影补偿，到机械臂运动规划与二指夹爪抓放执行，跑在 Jetson 上，通过 ROS 2 与上层系统对接。

---

## 技术指标

| 指标 | 数值 | 说明 |
| --- | --- | --- |
| 手眼标定平移误差 | **4.56 mm RMS** | `01_标定数据/results/best_eye_in_hand_result.yaml` |
| 手眼标定旋转误差 | **1.42° RMS** | 同上 |
| 标定样本量 | 13 组（v1）/ 11 组（v3）/ 12 组（v4） | 四轮采集，逐轮重标 |
| 抓取闭环阶段 | 9 阶段状态机 | 每阶段独立失败处理与回退 |
| 连续作业 | 3 轮自动循环，单轮超时 30 s | 无目标自动退出，不空转 |
| 控制链路延迟 | RPC 网关 + ZMQ 状态订阅（8 ms 推送） | `04_机械臂网关_arm_gateway` |

标定精度低于 5 mm 意味着每轮抓取前**必须**做腕部相机精定位补偿——这正是本项目采用"粗定位 → 底盘接近 → 精定位"两段式视觉的原因。

---

## 系统架构

```
                    ┌──────────────────────────────────────────┐
                    │            Windows / 上位机               │
                    │   MJPEG 实时画面  ·  运行记录  ·  点位录制  │
                    └────────────────────┬─────────────────────┘
                                         │ HTTP (MJPEG :8090)
  ┌──────────────────────────────────────▼──────────────────────────────────────┐
  │                              Jetson  (ROS 2 Humble)                          │
  │                                                                              │
  │   ┌────────────┐      ┌──────────────┐      ┌──────────────────┐            │
  │   │  RGB-D 相机  │─────▶│  YOLOv8 检测  │─────▶│  深度反投影 +     │            │
  │   │  腕部安装    │      │  bottle 类别  │      │  表面→中心补偿     │            │
  │   └────────────┘      └──────────────┘      └────────┬─────────┘            │
  │                                                       │ 目标位姿 (TCP 系)     │
  │   ┌──────────────────────────────────────────────────▼──────────────────┐  │
  │   │                    抓取任务状态机（9 阶段）                            │  │
  │   │  goto_photo → above → grasp → close → lift → place → release → home  │  │
  │   └───────┬──────────────────────────┬────────────────────────┬─────────┘  │
  └───────────┼──────────────────────────┼────────────────────────┼─────────────┘
              │ RPC (TCP :6200)          │ RS485 (/dev/ttyUSB0)   │ ROS 2 topic
              │                          │                        │
      ┌───────▼────────┐        ┌────────▼─────────┐      ┌───────▼────────┐
      │  贞实控制器     │        │ CTAG2F120 夹爪    │      │ Xbox 手柄遥操作 │
      │  192.168.11.11 │        │ 力/位参数化       │      │ velocity_mapper │
      │  ArmGateway    │        └──────────────────┘      └────────────────┘
      └────────────────┘
```

---

## 核心技术难点与解法

这四块是整个系统真正花时间的地方，也是为什么标定精度能压到 4.56 mm。

### 1. 四元数约定歧义

厂商 SDK 给的位姿是 `xyzw` 还是 `wxyz`、是否取逆、旋转方向是 TCP→相机还是相机→TCP——组合起来有 8 种可能，选错一种标定残差就会从 4 mm 跳到几十毫米，但**看起来仍然"有解"**。

解法：`quaternion_convention_audit.py` 枚举全部约定组合，`01_标定数据/results/calib_*/` 下同时保存四份结果对比：

```
robust_stored/          robust_stored_inverse/
robust_wxyz/            robust_wxyz_inverse/
```

用重投影残差最小的那组，而不是"看起来对"的那组。

### 2. 标定板角点方向翻转

棋盘格角点顺序翻转会导致解算出一条数学上自洽、物理上错误的变换。`exhaustive_flip_solve.py` 穷举翻转组合，`flip_graph_solve.py` 用图搜索找一致解，两条路径互相校验。

### 3. 深度表面→中心补偿

目标检测框给的是瓶子**表面**的深度，但抓取点要的是**中心轴**。直接用表面深度会导致夹爪偏半个瓶径。系统用 CameraInfo 内参和检测框几何做补偿，`v3_bottle_label_target_yolov8_center_compensated.jpg` 是补偿前后的对比留档。

### 4. 网关失效快速恢复

机械臂 RPC 网关偶发断开后，朴素实现会卡在超时。系统加入了 `--allow-gateway-reenable` 的显式恢复路径，并保留 `bottle_pick_place_demo_v3_before_fast_gateway_recovery_20260908.py` 作为改进前基线，便于回归对比。

---

## 抓取流程

```
① goto_photo   移动到抓取拍照点
② above        移动到目标水瓶上方
③ grasp        下降到抓取位
④ close        夹爪闭合（力 60 / 速度 40，可通过参数覆盖）
⑤ lift         抬升脱离
⑥ place_photo  移动到放置拍照位，统计放置区已有瓶数
⑦ place        按瓶数选偏移后放置
⑧ release      打开夹爪
⑨ home         回到抓取拍照点，准备下一轮
```

**放置偏移策略**（v3，验证通过）：

| 放置区已有瓶数 | 偏移量 |
| --- | --- |
| 0 | 原放置位 |
| 1 | `X − 12.5 cm` |
| 2 | `Y + 12.5 cm` |
| ≥ 3 | `X − 12.5 cm, Y + 12.5 cm` |

偏移量硬编码在 `bottle_pick_place_demo_v3.py` 中，改放置间距需要同步修改。

---

## 模块清单

| 模块 | 语言 | 职责 |
| --- | --- | --- |
| `03_手柄控制_xbox` | C++17 | Xbox 手柄 → 速度指令映射，`speedl_ipc_client` 直连机械臂，含触感死区与速度限幅 |
| `04_机械臂网关_arm_gateway` | C++17 | RPC 网关封装，ZMQ 状态订阅（8 ms 实时态 / 500 ms 非实时态），异常自动重连 |
| `05_机械臂SDK_ROS_源码` | Python / ROS 2 | 厂商 SDK 的 ROS 2 封装（`zhenshi_driver_py` + `zhenshi_interfaces`），含 dry-run、冒烟测试、环境自检 |
| `07_夹爪CTAG2F120` | Python | RS485 Modbus 驱动，夹持力与速度参数化，支持 open / pre / close 三态 |
| `08_v3单步抓水瓶测试` | Python | 抓取闭环主程序，9 阶段状态机 + 三轮循环入口 + 点位录制 |
| `10_v4移动抓取放置_demo` | Python | 移动抓取放置版本，前后相机双视觉，模块化 `v4_robot` 包 |
| `11_v5相机直连Jetson_demo` | Python | 相机直连 Jetson 版本，去掉中间转发，降低一帧延迟 |
| `01_标定数据` | Python | 标定采集、角点检测、多约定求解、迭代精化、IPPE 求解 |

---

## 目录结构

| 目录 | 内容 |
| --- | --- |
| `00_项目说明文档` | 项目总说明、v3 详细说明、目录规范 |
| `01_标定数据` | 标定图片、角点图、samples、外参结果、求解日志（v1/v3/v4 四轮） |
| `02_固定标定位姿` | 本次采集的机械臂 TCP 位姿，用于复现标定 |
| `03_手柄控制_xbox` | Xbox 遥操作源码与配置 |
| `04_机械臂网关_arm_gateway` | ArmGateway 源码、配置、可执行文件 |
| `05_机械臂SDK_ROS_源码` | 机械臂 SDK 与 ROS 2 封装 |
| `06_历史参考_legacy` | 历史标定资料、旧 Jetson 备份、已归档目录 |
| `07_夹爪CTAG2F120` | 夹爪 SDK 与控制脚本 |
| `08_v3单步抓水瓶测试` | v3 闭环程序、Jetson 镜像、运行记录、历史备份 |
| `09_辅助工具_tools` | Jetson 传输、ROI 选取等辅助脚本 |
| `10_v4移动抓取放置_demo` | v4 移动抓取放置 demo |
| `11_v5相机直连Jetson_demo` | v5 相机直连 demo |
| `99_原始备份包` | 从 Jetson 拉取的原始 `tar.gz` 与 manifest |
| `work` | 调试过程脚本与现场照片 |

---

## 硬件连接

| 设备 | 地址 / 接口 |
| --- | --- |
| Jetson 主控 | `192.168.8.100` |
| 机械臂控制器 | `192.168.11.11` |
| 机械臂 RPC 网关 | Jetson 本机 `127.0.0.1:6200` |
| RGB-D 相机 | USB 接 Jetson，ROS 2 话题 `/camera/camera/color/image_raw` |
| 二指夹爪 | 知行 CTAG2F120，RS485 接 Jetson，`/dev/ttyUSB0` |
| 实时画面 | `http://192.168.8.100:8090/` |

---

## 快速开始

```bash
cd /home/nvidia/calibration

# 启动相机 + MJPEG 推流 + YOLO + 位姿写入 + TF/localizer
bash start_v3_bottle_vision.sh

# 单轮完整抓取放置
python3 bottle_pick_place_demo_v3.py

# 三轮连续抓取（每轮最多等待 30 秒，无水瓶则正常退出）
python3 run_three_cycles_v3.py

# 只打印各阶段目标点，不运动（安全验证用）
python3 bottle_pick_place_demo_v3.py --plan-only

# 录制当前 TCP 位姿到点位文件
python3 bottle_pick_place_demo_v3.py --record grasp_photo
```

夹爪控制：

```bash
python3 ctag2f120_control.py --action open  --force 60 --speed 40
python3 ctag2f120_control.py --action pre   --force 60 --speed 40
python3 ctag2f120_control.py --action close --force 60 --speed 40
```

> `--force` 是夹爪 SDK 的百分比/设备参数，不是牛顿值。

主要命令行参数：

| 参数 | 作用 |
| --- | --- |
| `--plan-only` | 只输出规划点位，不驱动机器人 |
| `--allow-placeholders` | 允许使用占位点位运行（未标定的新环境） |
| `--robot-ip` | 覆盖机械臂控制器地址 |
| `--force` / `--gripper-speed` | 夹爪夹持力 / 动作速度 |
| `--detect-timeout-sec` | 单轮目标检测超时 |
| `--allow-gateway-reenable` | 允许网关掉线后自动重使能 |
| `--record <pose>` | 录制当前 TCP 位姿 |

---

## 版本演进

| 版本 | 关键改动 |
| --- | --- |
| **v3** | 固定工位单步抓放闭环；三轮连续入口；按放置区瓶数自动选偏移 |
| **v4** | 前后双相机 + 腕部相机；移动抓取放置；`v4_robot` 模块化封装 |
| **v5** | 相机直连 Jetson，去掉中间转发层；`v5_robot` 统一运动指令抽象 |

每版都保留上一版的完整能力，不是分叉重写。

---

## ⚠️ 安全须知

1. **回放固定标定位姿前**，必须确认机械臂、相机安装、标定板位置和现场安全区域与采集时一致。
2. **首次在新环境运行**，先用 `--plan-only` 或 `--allow-placeholders` 验证点位，再开真实运动。
3. 当前主程序 `MoveBlend` 速度为 `v25`，改速度需要编辑 `bottle_pick_place_demo_v3.py` 中的 `--speed=` 参数。
4. 调试抓取前清空工作区无关人员，保持急停可达。

---

## Roadmap

以下为**规划中、尚未实现**的功能，列出以便说明技术方向：

- [x] 三轮连续抓取与超时退出
- [x] 按放置区瓶数自动选择放置偏移
- [x] 多约定四元数标定自动择优
- [ ] 抓取失败自动重试（当前失败即中止，无重试策略）
- [ ] 断点续抓：中断后从上次状态机阶段恢复
- [ ] 多类别目标识别（当前仅 `bottle` 单类）
- [ ] 标定结果在线自检：运行时用重投影残差实时判断外参是否漂移
- [ ] 抓取成功率自动统计与日志聚合

---

## 说明

- 仓库中的内网 IP、串口号均为原项目调试用值，仅适用于隔离的实验局域网环境；请勿在公网环境直接复用。
- `09_辅助工具_tools/jetson.py` 不保存任何口令，运行前需自行设置环境变量 `JETSON_PASS`（以及可选的 `JETSON_HOST`、`JETSON_USER`）。
- 更详细的运行说明见 [`00_项目说明文档/V3_BOTTLE_PROJECT_README_20260908.md`](00_项目说明文档/V3_BOTTLE_PROJECT_README_20260908.md) 与 [`README_先看我.md`](README_先看我.md)。

## 许可

本项目基于 [MIT License](LICENSE) 开源。
