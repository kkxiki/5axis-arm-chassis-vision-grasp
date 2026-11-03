set -eu

LOG=/home/nvidia/calibration/runtime/v3_bottle_test
mkdir -p "$LOG"

for pattern in '[r]ealsense2_camera' '[m]jpeg_server.py' '[y]olov8_node.py' '[w]rist_localizer_node.py' '[f]ive_axis_tf_bridge_node.py' '[s]dk_pose_writer.py'; do
  for pid in $(pgrep -f "$pattern" || true); do
    if [ "$pid" != "$$" ] && [ "$pid" != "$PPID" ]; then
      kill "$pid" 2>/dev/null || true
    fi
  done
done
sleep 2

SDK=/home/nvidia/codex/zhenshi_ros2_py/vendor/sdk_send_recevice_demo_py/Base_SDK_py
ROS_PY=/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages

nohup env PYTHONPATH="$SDK:$SDK/robot_state" \
  LD_LIBRARY_PATH="$SDK/robot_state/lib/linux/arm/22.04" \
  python3 /home/nvidia/calibration/sdk_pose_writer.py \
    --sdk-root "$SDK" --robot-ip 192.168.11.11 --output /tmp/five_axis_pose.json \
  >"$LOG/sdk_pose_writer.log" 2>&1 </dev/null &
echo $! >"$LOG/sdk_pose_writer.pid"

nohup bash -lc 'source /opt/ros/humble/setup.bash; exec ros2 launch realsense2_camera rs_launch.py camera_name:=camera rgb_camera.color_profile:=1280x720x15 depth_module.depth_profile:=1280x720x15 enable_depth:=true align_depth.enable:=true enable_infra1:=false enable_infra2:=false' \
  >"$LOG/realsense.log" 2>&1 </dev/null &
echo $! >"$LOG/realsense.pid"
sleep 9

nohup env PYTHONPATH="$ROS_PY" LD_LIBRARY_PATH="/opt/ros/humble/lib" \
  python3 /home/nvidia/calibration/mjpeg_server.py \
  >"$LOG/mjpeg.log" 2>&1 </dev/null &
echo $! >"$LOG/mjpeg.pid"

nohup bash -lc 'source /opt/ros/humble/setup.bash; exec python3 /home/nvidia/five_axis_bottle_ws/src/five_axis_bottle_sort/five_axis_bottle_sort/five_axis_tf_bridge_node.py --ros-args -p pose_json:=/tmp/five_axis_pose.json -p calibration_yaml:=/home/nvidia/calibration/标定数据/v3_重新采集_20260907/wrist_d435i_to_tcp_corrected.yaml -p max_pose_age_sec:=1.0' \
  >"$LOG/tf_bridge.log" 2>&1 </dev/null &
echo $! >"$LOG/tf_bridge.pid"

nohup env PYTHONPATH="/home/nvidia/camera_1/yolov8_ros2_ws/src/yolov8_detector:$ROS_PY" LD_LIBRARY_PATH="/opt/ros/humble/lib" \
  /home/nvidia/robot_grasp-main/.venv/bin/python \
  /home/nvidia/camera_1/yolov8_ros2_ws/src/yolov8_detector/yolov8_detector/yolov8_node.py \
    --ros-args -p model_path:=/home/nvidia/camera_1/yolov8_ros2_ws/yolov8s.pt \
    -p image_topic:=/camera/camera/color/image_raw -p conf_threshold:=0.30 \
    -p device:=cpu -p imgsz:=640 -p debug:=true \
  >"$LOG/yolo.log" 2>&1 </dev/null &
echo $! >"$LOG/yolo.pid"

nohup bash -lc 'source /opt/ros/humble/setup.bash; exec python3 /home/nvidia/five_axis_bottle_ws/src/five_axis_bottle_sort/five_axis_bottle_sort/wrist_localizer_node.py --ros-args -p detection_topic:=/yolov8/detections -p depth_topic:=/camera/camera/aligned_depth_to_color/image_raw -p camera_info_topic:=/camera/camera/color/camera_info -p base_frame:=base_link -p min_score:=0.30 -p stable_samples:=5' \
  >"$LOG/localizer.log" 2>&1 </dev/null &
echo $! >"$LOG/localizer.pid"

sleep 12
echo STARTED
