# V3 水瓶抓取 Demo 迁移到 V4

## 已确定的方案

V4 首个水瓶 Demo 先做固定底盘、固定桌面高度、单瓶抓取放置，复用 V3 已验证的核心策略：

1. 机械臂到 V4 腕部观察位；
2. 在 1280x720 RGB 图中检测 COCO `bottle`（class 39）；
3. 当前由 RGB 像素射线与 `arm_base` 下固定的水瓶抓取中心 Z 平面求交；
4. 后续收到与 RGB 对齐的原始 `uint16` 深度后，切换为 15x15 有效深度中位数反投影；
5. 用同一帧的实时 TCP 位姿计算 `base_T_tcp @ tcp_T_camera @ point_camera`；
6. 连续采 15 次，至少保留 9 个 8 mm 内点；
7. 采用 V3 验证过的示教补偿：`运行检测 - 示教检测` 平移 `above_ref/grasp_ref`；
8. 执行观察、张开、上方点、抓取点、闭合、抬升、放置、回 Home。

V4 手眼结果已按现场确认启用：eye-in-hand、DANIILIDIS、1280x720，平移一致性 RMS 约 9.24 mm。

## 不从 V3 直接复制的内容

V3 点位属于 `U05ME_0`，V4 标定记录的机器人为 `taiKeU058T_0`。所以 V3 的观察、抓取和放置坐标不能直接用于 V4 真机。V4 配置中的六个点默认均未确认，任何一点未重新示教时，离线规划器都会拒绝生成真机抓取轨迹。

## 当前实施顺序

1. 确认固定桌面上标准水瓶抓取中心在 `arm_base` 下的 Z 高度。
2. 在 V4 观察位重新示教 `observe`。
3. 放一只标准水瓶，记录当时检测坐标与真正夹持中心，示教 `above_ref`、`grasp_ref` 和 `ref_detection_base_m`。
4. 示教 `place_above`、`place`、`home`，逐点低速空跑。
5. 先运行离线规划，再运行仅视觉，最后才解除真实运动执行锁。
6. 固定底盘抓放稳定后，再接入原始深度、V4 前视相机和底盘移动流程。

## 离线检查

```bash
python3 scripts/plan_v4_bottle_demo.py
python3 scripts/plan_v4_bottle_demo.py --target X Y Z
```

该脚本不会连接控制器，也不会发送机械臂或夹爪命令。当前预期返回未就绪，因为 V4 六个专用示教点还没有采集。
