# 5轴协作臂抓水瓶项目说明（v3整理版）

更新时间：2026-09-08

## 1. 当前进度

当前已经完成并验证过的主线流程：

1. Jetson 连接相机、机械臂控制器、CTAG2F120 二指夹爪。
2. 相机实时画面通过 MJPEG 服务传到 Windows 浏览器。
3. YOLOv8 使用 `bottle` 类别识别水瓶。
4. 机械臂从抓取拍照点识别水瓶，移动到水瓶上方，下降到抓取点，夹爪抓取。
5. 机械臂移动到放置拍照位，判断放置区已有水瓶数量。
6. 根据放置区水瓶数量选择放置偏移：
   - 0 个水瓶：放到原放置位。
   - 1 个水瓶：放置位和放置上方位一起 `X - 12.5cm`。
   - 2 个水瓶：放置位和放置上方位一起 `Y + 12.5cm`。
   - 3 个及以上：放置位和放置上方位一起 `X - 12.5cm, Y + 12.5cm`。
7. 放置后打开夹爪，再回到抓取拍照点。
8. 已增加三轮连续执行入口：最多执行 3 轮，每轮等待 30 秒仍未识别到水瓶则退出。

注意：当前主程序里 MoveBlend 速度为 `v25`。如要统一改成 `v10` 或其他速度，需要改 `bottle_pick_place_demo_v3.py` 中 MoveBlend 插入行的 `--speed=v25`。

## 2. 硬件连接

Jetson IP：`192.168.8.100`

机械臂控制器：`192.168.11.11`

机械臂网关：Jetson 本机 `127.0.0.1:6200`

相机：USB 接 Jetson，ROS2 话题 `/camera/camera/color/image_raw`

夹爪：知行 CTAG2F120 二指夹爪，RS485 接 Jetson，默认串口 `/dev/ttyUSB0`

## 3. Jetson 程序入口

进入工作目录：

```bash
cd /home/nvidia/calibration
```

启动相机、实时画面、YOLO、位姿写入、TF/localizer：

```bash
bash /home/nvidia/calibration/start_v3_bottle_vision.sh
```

实时画面地址：

```text
http://192.168.8.100:8090/
```

执行单轮完整抓取放置：

```bash
python3 /home/nvidia/calibration/bottle_pick_place_demo_v3.py
```

执行三轮完整抓取放置；每轮抓取拍照点最多等待 30 秒，无水瓶则正常退出：

```bash
python3 /home/nvidia/calibration/run_three_cycles_v3.py
```

只查看规划点位，不运动：

```bash
python3 /home/nvidia/calibration/bottle_pick_place_demo_v3.py --plan-only
```

记录当前机械臂 TCP 位姿到点位文件：

```bash
python3 /home/nvidia/calibration/bottle_pick_place_demo_v3.py --record grasp_photo
python3 /home/nvidia/calibration/bottle_pick_place_demo_v3.py --record above_ref
python3 /home/nvidia/calibration/bottle_pick_place_demo_v3.py --record grasp_ref
python3 /home/nvidia/calibration/bottle_pick_place_demo_v3.py --record place_photo
python3 /home/nvidia/calibration/bottle_pick_place_demo_v3.py --record place_mid
python3 /home/nvidia/calibration/bottle_pick_place_demo_v3.py --record place
```

## 4. 夹爪命令

夹爪控制脚本：

```bash
/home/nvidia/calibration/ctag2f120_control.py
```

打开夹爪：

```bash
python3 /home/nvidia/calibration/ctag2f120_control.py --action open --force 60 --speed 40
```

预夹位置：

```bash
python3 /home/nvidia/calibration/ctag2f120_control.py --action pre --force 60 --speed 40
```

闭合夹爪：

```bash
python3 /home/nvidia/calibration/ctag2f120_control.py --action close --force 60 --speed 40
```

说明：当前 `--force` 是夹爪 SDK 的百分比/设备参数，不是牛顿值。之前测试默认使用 `force=60`、`speed=40`。

## 5. 关键文件

Jetson 运行目录：`/home/nvidia/calibration`

Windows 整理目录：`D:\workspace\5轴抓取项目1\5轴抓取项目_整理版\08_v3单步抓水瓶测试`

Windows Jetson 镜像目录：`D:\workspace\5轴抓取项目1\5轴抓取项目_整理版\08_v3单步抓水瓶测试\01_Jetson运行程序`

主程序：`bottle_pick_place_demo_v3.py`

三轮入口：`run_three_cycles_v3.py`

识别程序：`bottle_target_once_v3.py`

相机实时服务脚本：`start_v3_bottle_vision.sh`

MJPEG 服务：`mjpeg_server.py`

夹爪脚本：`ctag2f120_control.py`

夹爪 SDK：`Changingtek_rtu_psdk.py`

点位目录：`poses/`

## 6. 点位文件

抓取拍照点：`poses/grasp_photo_pose_v3.json`

水瓶上方参考点：`poses/bottle_above_pose_v3.json`

抓取点：`poses/grasp_pose_v3.json`

放置拍照点：`poses/place_photo_pose_v3.json`

放置上方点：`poses/place_mid_pose_v3.json`

放置点：`poses/place_pose_v3.json`

这些点位已经是当前程序使用的固定点位。重新示教时优先使用 `--record` 命令，不要手工改 JSON。

## 7. 程序流程

`run_three_cycles_v3.py` 是外层循环：

1. 启动第 1 轮 `bottle_pick_place_demo_v3.py --detect-timeout-sec 30`。
2. 如果识别不到水瓶，主程序返回退出码 3，外层程序正常停止。
3. 如果本轮成功，继续下一轮。
4. 最多执行 3 轮。
5. 任意非识别超时错误会中止，避免继续运动。

`bottle_pick_place_demo_v3.py` 是单轮流程：

1. 回到抓取拍照点。
2. YOLOv8 识别水瓶。
3. 根据当前识别坐标和示教时参考识别坐标计算补偿。
4. 移动到水瓶上方点。
5. 下降到抓取点。
6. 等机械臂稳定，闭合夹爪，等待抓稳。
7. 抬升并移动到放置拍照点。
8. 快速检测放置区水瓶数量。
9. 根据数量调整放置位和放置上方位。
10. 移动到放置位并打开夹爪。
11. 回到抓取拍照点。

## 8. 目录规范

当前 Windows 整理目录按以下规则放置：

`00_说明文档`：项目说明、当前进度、操作入口。

`01_Jetson运行程序`：与 Jetson `/home/nvidia/calibration` 同步的核心运行文件和点位。

`02_运行记录`：运行日志。

`03_图片与识别结果`：识别截图、调试图片、检测结果 JSON。

`90_历史备份`：历史版本、旧脚本、旧点位副本和回滚文件。

根目录只保留当前最常用的三个程序快捷副本：

`bottle_pick_place_demo_v3.py`

`bottle_target_once_v3.py`

`run_three_cycles_v3.py`

## 9. Windows 到 Jetson 同步

基准原则：demo 以 Jetson `/home/nvidia/calibration` 当前测试版为准。Windows `01_Jetson运行程序` 只是镜像备份和查看目录，不能把 Windows 旧文件反向覆盖 Jetson 测试版，除非明确确认要同步修改。

当前已将 Jetson 上正在使用的文件拉回 Windows `01_Jetson运行程序` 镜像目录。

如果后续在 Windows 修改了镜像目录里的文件，需要同步到 Jetson 对应位置：

```powershell
$env:JETSON_ARCHIVE_PASSWORD='<你的密码>'
py .\work\upload_jetson_file.py "D:\workspace\5轴抓取项目1\5轴抓取项目_整理版\08_v3单步抓水瓶测试\01_Jetson运行程序\bottle_pick_place_demo_v3.py" /home/nvidia/calibration/bottle_pick_place_demo_v3.py
py .\work\upload_jetson_file.py "D:\workspace\5轴抓取项目1\5轴抓取项目_整理版\08_v3单步抓水瓶测试\01_Jetson运行程序\bottle_target_once_v3.py" /home/nvidia/calibration/bottle_target_once_v3.py
py .\work\upload_jetson_file.py "D:\workspace\5轴抓取项目1\5轴抓取项目_整理版\08_v3单步抓水瓶测试\01_Jetson运行程序\run_three_cycles_v3.py" /home/nvidia/calibration/run_three_cycles_v3.py
```

同步后建议在 Jetson 编译检查：

```bash
python3 -m py_compile /home/nvidia/calibration/bottle_pick_place_demo_v3.py /home/nvidia/calibration/bottle_target_once_v3.py /home/nvidia/calibration/run_three_cycles_v3.py /home/nvidia/calibration/ctag2f120_control.py
```

## 10. 运行前检查

检查机械臂网关状态：

```bash
echo '{"cmd":"state"}' | timeout 3 nc 127.0.0.1 6200
```

理想状态包含：

```text
"connected":true
"arm_enabled":true
"state":"READY"
"owner":"IDLE"
```

如果相机页面打不开，先重新执行：

```bash
bash /home/nvidia/calibration/start_v3_bottle_vision.sh
```

如果夹爪无反应，先检查 `/dev/ttyUSB0` 是否存在，再单独执行打开/闭合命令验证。

