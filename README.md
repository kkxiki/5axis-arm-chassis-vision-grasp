# 5 轴机械臂抓取项目（整理版）

五轴协作机械臂 + 腕部相机的视觉抓取项目，包含手眼标定、Xbox 手柄遥操作、机械臂网关、ROS 2 SDK 封装、二指夹爪控制，以及在 Jetson 上运行的 YOLOv8 抓水瓶闭环程序。

> 本仓库是项目资料整理版：Jetson 上的 `/home/nvidia/calibration` 是实际运行基准，本仓库用于备份、查阅、文档整理和版本管理。**除非明确确认要同步修改，不要用这里的旧脚本覆盖 Jetson 上的测试版。**

## 项目概览

- **硬件**：五轴协作臂 + USB 相机（腕部安装，eye-in-hand）+ 知行 CTAG2F120 二指夹爪（RS485）+ Jetson 主控
- **视觉**：YOLOv8 检测 `bottle` 类别，相机实时画面以 MJPEG 推送到浏览器
- **标定**：相机-机械臂手眼标定，产物为 `01_标定数据/results/best_eye_in_hand_result.yaml`
- **闭环**：抓取拍照点识别 → 移动到瓶子上方 → 下降抓取 → 移动到放置拍照位 → 按放置区已存瓶数选择偏移量 → 放置 → 回到抓取拍照点

## 目录结构

| 目录 | 内容 |
| --- | --- |
| `00_项目说明文档` | 项目总说明、v3 详细说明、目录规范、入口命令 |
| `01_标定数据` | 标定图片、角点图、samples、外参结果与求解日志（多轮采集版本） |
| `02_固定标定位姿` | 本次采集的机械臂 TCP 位姿，用于下次复现标定 |
| `03_手柄控制_xbox` | Xbox 手柄控制机械臂/夹爪的代码与配置 |
| `04_机械臂网关_arm_gateway` | ArmGateway 代码、配置和可执行文件 |
| `05_机械臂SDK_ROS_源码` | 机械臂 SDK 与 ROS 2 封装源码（已排除构建产物） |
| `06_历史参考_legacy` | 历史标定资料、旧 Jetson 备份、已归档的散落目录 |
| `07_夹爪CTAG2F120` | 夹爪 SDK 与控制脚本 |
| `08_v3单步抓水瓶测试` | v3 抓水瓶闭环程序、Jetson 镜像、运行记录与历史备份 |
| `09_辅助工具_tools` | Windows 侧辅助脚本（Jetson 传输、ROI 选取等） |
| `10_v4移动抓取放置_demo` | v4 移动抓取放置 demo（ROS 2 工作区风格） |
| `11_v5相机直连Jetson_demo` | v5 相机直连 Jetson demo |
| `99_原始备份包` | 从 Jetson 拉取的原始 `tar.gz` 备份包与 manifest |
| `work` | 调试过程中的临时脚本与现场照片 |

## 硬件连接

| 设备 | 地址 / 接口 |
| --- | --- |
| Jetson | `192.168.8.100` |
| 机械臂控制器 | `192.168.11.11` |
| 机械臂网关 | Jetson 本机 `127.0.0.1:6200` |
| 相机 | USB 接 Jetson，ROS 2 话题 `/camera/camera/color/image_raw` |
| 夹爪 | 知行 CTAG2F120，RS485 接 Jetson，默认串口 `/dev/ttyUSB0` |
| 实时画面 | `http://192.168.8.100:8090/` |

## 快速开始（Jetson）

```bash
cd /home/nvidia/calibration

# 启动相机 + 实时画面 + YOLO + 位姿写入 + TF/localizer
bash /home/nvidia/calibration/start_v3_bottle_vision.sh

# 单轮完整抓取放置
python3 /home/nvidia/calibration/bottle_pick_place_demo_v3.py

# 三轮连续抓取（每轮等待 30 秒无水瓶则退出）
python3 /home/nvidia/calibration/run_three_cycles_v3.py

# 只规划点位，不运动
python3 /home/nvidia/calibration/bottle_pick_place_demo_v3.py --plan-only
```

夹爪控制：

```bash
python3 /home/nvidia/calibration/ctag2f120_control.py --action open  --force 60 --speed 40
python3 /home/nvidia/calibration/ctag2f120_control.py --action pre   --force 60 --speed 40
python3 /home/nvidia/calibration/ctag2f120_control.py --action close --force 60 --speed 40
```

> `--force` 是夹爪 SDK 的百分比/设备参数，不是牛顿值。

## 本次标定结果

最终外参文件：`01_标定数据/results/best_eye_in_hand_result.yaml`

```
translation_m:      [0.032654410804, 0.138084416530, -0.152119718610]
quaternion_xyzw:    [0.001216607412, -0.170169262469, 0.985405094289, -0.004212140450]
translation_rms_mm: 4.562804
rotation_rms_deg:   1.424805
```

## 放置偏移策略（v3）

放置区已存瓶数决定放置位与"放置上方"位置的整体偏移：

| 已有瓶数 | 偏移 |
| --- | --- |
| 0 | 放到原放置位 |
| 1 | `X - 12.5cm` |
| 2 | `Y + 12.5cm` |
| ≥ 3 | `X - 12.5cm, Y + 12.5cm` |

## 安全须知

- 自动回放固定标定位姿前，必须确认机械臂、相机安装、标定板位置和现场安全区域与本次一致。
- 当前主程序 `MoveBlend` 速度为 `v25`，修改速度需要编辑 `bottle_pick_place_demo_v3.py` 中 `--speed=` 参数。
- 调试抓取前请清空工作区内的无关人员，并保持急停可达。

## 说明

- 仓库中的内网 IP、串口号均为原项目调试用值，仅适用于隔离的实验局域网环境；请勿在公网环境直接复用。
- `09_辅助工具_tools/jetson.py` 不保存任何口令，运行前需自行设置环境变量 `JETSON_PASS`（以及可选的 `JETSON_HOST`、`JETSON_USER`）。
- 更详细的运行说明见 [`00_项目说明文档/V3_BOTTLE_PROJECT_README_20260908.md`](00_项目说明文档/V3_BOTTLE_PROJECT_README_20260908.md) 与 [`README_先看我.md`](README_先看我.md)。
