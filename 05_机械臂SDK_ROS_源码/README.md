# 贞实控制器 ROS 2 Python 快速集成

这是一个面向 ROS 2 Humble 的贞实控制器 Python 驱动骨架。

当前目标：

- 固定使用贞实官方 Python 收发一体 SDK 的指定提交。
- 默认以 `dry_run=true` 启动，不连接控制器、不发送真实指令。
- 使用 ROS 2 服务封装关节运动、笛卡尔运动、夹爪和基础控制命令。
- 使用 ROS 2 话题发布控制器状态。
- 保留现有 Gemini、RealSense、YOLO、深度和手眼标定代码，后续通过 ROS 2 接口接入。

## 固定环境

- Ubuntu 22.04.4 LTS
- Python 3.10
- ROS 2 Humble
- CUDA 12.6.68
- CUDA Toolkit 12.6.3
- cuDNN 9.15.0.57
- TensorRT 10.3.0.30

官方 SDK 固定信息见 `UPSTREAM_SDK.lock`。

## 安全默认值

```yaml
dry_run: true
allow_raw_commands: false
```

因此默认启动时不加载官方动态库、不连接控制器，所有服务只记录将要发送的命令。官方示例中的 `initialize_system()` 会自动发送初始化指令，本工程不会调用它。

## Linux 第一轮运行

```bash
cd ~/zhenshi_ros2_py
source /opt/ros/humble/setup.bash

./scripts/check_environment.sh
./scripts/fetch_sdk.sh
./scripts/build.sh
./scripts/run_dry_run.sh
```

另一个终端查看模拟状态：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /zhenshi/state
```

调用一次 dry-run 关节运动：

```bash
ros2 service call /zhenshi/move_joint \
  zhenshi_interfaces/srv/MoveJoint \
  "{model_index: 0, joints: [0,0,0,0,0,0,0,0,0,0], timeout_ms: 5000}"
```

## 实机验证顺序

1. `smoke_sdk_readonly.py`：只连接状态端口，确认 `model_index`。
2. `smoke_sdk_connect.py`：只连接 RPC 5868，不发送命令。
3. ROS 2 节点保持 `dry_run=true`，检查命令字符串。
4. 确认急停、低速、空载、安全工作空间。
5. 设置确认变量后使用 `run_real.sh`。

`run_real.sh` 不会自动发送 `Clear/Recover/Enable`，只开放 ROS 2 服务。真实命令必须由操作者主动调用。

## 与旧视觉工程的关系

旧工程保持不动，后续按以下方式连接：

```text
YOLO检测
  -> gather_depth
  -> eye_to_hand
  -> grasp_manager
  -> /zhenshi/move_pose
  -> /zhenshi/set_gripper
```

旧的 `yolo_result_subscriber.cpp` 不再直接连接 `CPPClient`。

