Gemini 335Le 顶部固定相机—右机械臂手眼重新标定

用途：
相机已经移动，旧 righthandeye_matrix.npz 不能再用于真实抓取。
本目录只生成新文件，不覆盖旧标定。

标定模型：
相机固定在机械臂外部，棋盘格刚性固定在右手/右末端。
每个样本满足：
base_T_gripper * gripper_T_target =
base_T_camera * camera_T_target

标定板默认参数：
内角点 8 列 x 6 行；方格边长 29 mm。
必须实际测量确认。如不一致，启动采集脚本时修改 --cols、--rows、--square-mm。

运行顺序：
1. 启动 Gemini 335Le ROS2 相机节点。
2. 启动只读 robot_state_bridge；双臂必须确认 model_index。
3. 启动 capture_handeye_samples.py。
4. 每次人工把右臂移到一个新姿态，完全停止后调用：
   ros2 service call /handeye/capture std_srvs/srv/Trigger "{}"
5. 至少采 10 个，建议 15-20 个姿态。位置和朝向都要明显变化，
   棋盘要覆盖画面中央、四角、远近，并始终完整清晰可见。
6. 运行 solve_eye_to_hand.py。
7. 只有报告 quality_passed=true，且先做 dry-run 坐标验证后，
   才允许把新 npz 路径写入抓取配置。

输出：
samples.json：图像、相机内参、控制器原始位姿和每帧 PnP 结果。
right_handeye_result_new.json：误差报告和米制 4x4 变换。
righthandeye_matrix_new.npz：兼容旧转换服务；X 的平移单位为 mm。

重要：
robot_state_bridge 不包含 RPC 运动控制接口，只订阅控制器 19091 状态端口。
标定阶段保持 execute_enabled=false。
