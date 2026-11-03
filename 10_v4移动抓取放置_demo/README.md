# V4 双相机移动抓取放置 Demo

这是在整理版项目旁边新建的独立 v4，未修改 v1/v3、标定、网关或 SDK 目录。

当前版本实现了可直接执行的闭环仿真，以及连接真实硬件所需的协议适配层：

1. 从地图接收粗目标 `[x, y, yaw]`。
2. 底盘导航到粗目标。
3. 车载前视相机返回物体方位角和距离。
4. 底盘转向并前进到机械臂工作距离。
5. 机械臂到腕部观察位。
6. 腕部相机通过新手眼标定返回 `arm_base` 三维坐标。
7. 安全工作空间检查后执行上方点、抓取点、抬升点。
8. 夹爪命令经控制器发送，不再访问 Jetson 的 `/dev/ttyUSB0`。
9. 把物品放到小车上的示教托盘位，机械臂回 Home。
10. 每个状态及设备结果写入 JSONL 任务日志；故障时停止底盘和机械臂并保持夹爪状态。

## S1～S12 标准流程

- **S1 任务创建与地图标点**：操作人员在地图中标记物品所在的大致坐标，生成 `map_goal=[x,y,yaw]` 和目标类别。
- **S2 全系统预检**：Jetson 检查底盘定位、前视相机、机械臂控制器、腕部相机和夹爪是否在线；任一设备异常则禁止运动。
- **S3 小车粗导航**：Jetson 将 `map_goal` 发送到底盘，底盘依靠自身地图、定位和避障能力行驶到目标附近。
- **S4 前视相机粗识别**：Jetson 从路由器上的车载前视相机拉取视频，识别目标并得到相对方位角和大致距离。
- **S5 小车视觉对准**：Jetson 向底盘发送转向和前进距离，使目标进入机械臂工作区；再次识别确认角度和距离合格。
- **S6 机械臂到腕部观察位**：底盘停止后，机械臂移动到预先示教的 `observe` 位姿，让腕部相机朝向目标区域。
- **S7 腕部相机精定位**：腕部相机获取目标像素和深度，经新手眼标定及坐标变换，输出目标在 `arm_base` 下的精确三维坐标。
- **S8 抓取规划与安全检查**：根据精确坐标生成目标上方点、抓取点和抬升点，并检查识别置信度、数据时间戳、坐标系及机械臂安全工作空间。
- **S9 接近并抓取**：夹爪打开，机械臂先到目标上方，再低速下降到抓取点；夹爪通过控制器闭合并等待夹持稳定。
- **S10 垂直抬升**：机械臂携带物品垂直抬升到安全高度，避免与桌面、目标周边或车体碰撞。
- **S11 放置物品**：机械臂移动到小车托盘上方和放置点，控制器打开夹爪释放物品，再退回托盘上方。
- **S12 回零与任务完成**：机械臂回到 `home`，记录任务结果并进入 `COMPLETE`；如果 S2～S11 任一步骤失败，则转入 `SAFE_ABORT`，停止底盘和机械臂，并保持夹爪最后状态等待人工处理。

## 立即运行

Windows PowerShell：

```powershell
py -3 .\scripts\run_v4_demo.py --mode sim
```

Linux / Jetson：

```bash
python3 scripts/run_v4_demo.py --mode sim
```

覆盖地图标记点：

```bash
python3 scripts/run_v4_demo.py --mode sim --goal 2.0 0.8 0.0 --label bottle
```

验证安全中止：

```bash
python3 scripts/run_v4_demo.py --mode sim --fail-at wrist_detect
```

运行测试：

```bash
python3 -m unittest discover -s tests -v
```

运行结果保存在 `runtime/v4-*.jsonl`。进程成功返回 `0`，任务安全中止返回 `2`，配置锁未解除返回 `3`。

当前真实接入进度与现场阻塞见 [实现状态_20260909](docs/实现状态_20260909.md)。

## V3 水瓶 Demo 迁移

固定底盘的 V4 水瓶抓取第一阶段已经建立独立配置和离线规划器：

```bash
python3 scripts/plan_v4_bottle_demo.py
python3 scripts/plan_v4_bottle_demo.py --target X Y Z
```

该入口只做校验和轨迹规划，绝不发送运动命令。它使用本次 1280x720 V4 手眼结果，
并复用 V3 已验证的“运行检测坐标减示教检测坐标”抓取补偿。V3 的 `U05ME_0`
示教坐标不会复制给 V4 的 `taiKeU058T_0`；六个 V4 点位必须重新示教并逐项确认。
完整迁移步骤见 `docs/V3水瓶Demo迁移到V4规划.md`。

## 真实硬件接入

先阅读 [数据传输与部署设计](docs/数据传输与部署设计.md)，然后修改 `config/v4_config.json`。v4 的硬件安装结构已变化，因此配置默认包含四道锁：`hardware_confirmed`、前视外参、腕部手眼外参、底盘到机械臂外参。全部确认前，`--mode real` 会在建立运动任务前退出。

真实模式入口：

```bash
python3 scripts/run_v4_demo.py --mode real --goal 2.0 0.8 0.0
```

真实模式目前采用清晰的边界接口：

- 底盘：JSON-lines/TCP，命令见设计文档。
- 机械臂：兼容现有 ArmGateway 的 `state/set_owner/send` 接口。
- 夹爪：控制器侧 JSON-lines/TCP 服务；实际控制器 SDK 不同，只需替换 `ControllerGripper`。
- 前视相机：已按 Gemini 335Le 的 Orbbec SDK v2/ROS2 驱动实现。`front_vision_server.py` 将彩色、对齐深度、CameraInfo 和 YOLO 检测同步为 HTTP 粗定位结果，并在同一端口提供 MJPEG 实时画面。
- 腕部相机：Jetson 本机 HTTP 视觉服务的 `camera_id=wrist` 接口；待新安装与手眼标定完成后接入。

### 控制器腕部相机初版

现场已确认腕部相机通过控制器 `giant_frame_data` 发布 1280x720 JPEG，Jetson 直接订阅：

```bash
bash /home/nvidia/5axis_v4/scripts/start_v4_wrist_vision.sh
```

服务默认监听 `8092`，提供 `/health`、`/frame.jpg`、`/mjpeg` 和 `/detect`。当前只确认彩色帧，
所以健康信息明确标记 `metric_depth=false`；在控制器提供可度量深度且完成 V4 手眼标定前，
`/detect` 不返回三维坐标，真实抓取仍会被安全锁阻止。

控制器夹爪使用现有 HTTP 事件 `MoveTwoFingersGripper`，默认张开距离 70、闭合距离 0、间隔 25。
实现不会调用任何 `Add*` 或 `SaveXml` 指令。

前视相机型号和历史驱动现已确认；底盘及控制器侧夹爪的真实 SDK/协议仍保留为可替换适配器。默认仿真是完整可运行的功能 demo，真实运动仍受硬件与三项标定确认锁保护。

## Gemini 335Le 前视相机

该相机不是普通 RTSP 摄像头，使用 Orbbec SDK v2 网络设备端口 `8090`。把本目录部署为 Jetson 的 `/home/nvidia/5axis_v4` 后执行：

```bash
bash /home/nvidia/5axis_v4/scripts/start_v4_front_vision.sh
```

服务接口：

- Windows 实时画面：`http://192.168.50.20:8088/`
- 健康检查：`http://127.0.0.1:8088/health?camera_id=front`
- 水瓶粗定位：`http://127.0.0.1:8088/detect?camera_id=front&label=bottle`

启动脚本会先检查相机 IP 的 ARP 响应；若相机未供电、PoE/网线未接通或静态 IP 不符，会退出而不会启动一个假健康的视觉服务。

## 目录

```text
10_v4移动抓取放置_demo/
├─ config/v4_config.json          网络、任务、点位、安全区、标定确认锁
├─ docs/数据传输与部署设计.md       数据路径、端口、协议及上线步骤
├─ scripts/run_v4_demo.py         统一入口
├─ scripts/start_v4_front_vision.sh Gemini 335Le、YOLO、HTTP/MJPEG 启动入口
├─ scripts/front_vision_server.py  前视 RGB-D/检测到 V4 HTTP API 的桥接
├─ tests/test_v4_demo.py          完整流程、故障、安全锁测试
├─ runtime/                       每次任务的 JSONL 审计日志
└─ v4_robot/
   ├─ config.py                   配置加载与真实模式锁
   ├─ front_geometry.py           前视角度、检测框和稳健深度计算
   ├─ devices.py                  仿真及真实设备适配器
   ├─ mission.py                  任务状态机与安全检查
   ├─ models.py                   坐标/识别/任务数据模型
   └─ transport.py                带 seq 的 JSON-lines TCP 传输
```
