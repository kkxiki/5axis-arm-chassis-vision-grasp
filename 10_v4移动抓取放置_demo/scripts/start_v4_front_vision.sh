#!/usr/bin/env bash
set -euo pipefail

ROOT="${V4_ROOT:-/home/nvidia/5axis_v4}"
CAMERA_IP="${V4_FRONT_CAMERA_IP:-192.168.30.120}"
CAMERA_PORT="${V4_FRONT_CAMERA_PORT:-8090}"
LOG_DIR="$ROOT/runtime/front_vision"
ROS_PY="/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages"
ORBBEC_SETUP="${V4_ORBBEC_SETUP:-/home/nvidia/camera_1/orbbec_ros_ws/install/setup.bash}"
YOLO_SOURCE="/home/nvidia/camera_1/yolov8_ros2_ws/src/yolov8_detector"
YOLO_MODEL="${V4_YOLO_MODEL:-/home/nvidia/camera_1/yolov8_ros2_ws/yolov8s.pt}"
PYTHON_BIN="${V4_YOLO_PYTHON:-/home/nvidia/robot_grasp-main/.venv/bin/python}"

mkdir -p "$LOG_DIR"
for pid_file in camera yolo http; do
  if [[ -f "$LOG_DIR/$pid_file.pid" ]]; then
    pid="$(cat "$LOG_DIR/$pid_file.pid")"
    if kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
    fi
    rm -f "$LOG_DIR/$pid_file.pid"
  fi
done

if ! ip neigh show "$CAMERA_IP" | grep -q 'lladdr'; then
  ping -c 1 -W 1 "$CAMERA_IP" >/dev/null 2>&1 || true
fi
if ! ip neigh show "$CAMERA_IP" | grep -q 'lladdr'; then
  echo "ERROR: Gemini 335Le $CAMERA_IP has no ARP response. Check PoE/power, cable, switch/VLAN, and static IP." >&2
  exit 4
fi

nohup bash -lc "source /opt/ros/humble/setup.bash; source $ORBBEC_SETUP; exec ros2 launch orbbec_camera gemini_330_series.launch.py \
  camera_name:=gemini335le enumerate_net_device:=false net_device_ip:=$CAMERA_IP net_device_port:=$CAMERA_PORT \
  color_width:=1280 color_height:=720 color_fps:=15 color_format:=MJPG \
  depth_width:=640 depth_height:=400 depth_fps:=15 depth_format:=Y16 enable_color:=true enable_depth:=true \
  depth_registration:=true enable_point_cloud:=false enable_colored_point_cloud:=false enable_ptp_config:=false" \
  >"$LOG_DIR/camera.log" 2>&1 </dev/null &
echo $! >"$LOG_DIR/camera.pid"

ready=0
for _ in $(seq 1 25); do
  if bash -lc "source /opt/ros/humble/setup.bash; timeout 2 ros2 topic echo /gemini335le/color/camera_info --once" \
      >/dev/null 2>&1; then
    ready=1
    break
  fi
  sleep 1
done
if [[ "$ready" != 1 ]]; then
  echo "ERROR: Orbbec driver started but no camera_info arrived; see $LOG_DIR/camera.log" >&2
  exit 5
fi

nohup env PYTHONPATH="$YOLO_SOURCE:$ROS_PY" LD_LIBRARY_PATH="/opt/ros/humble/lib" \
  "$PYTHON_BIN" "$YOLO_SOURCE/yolov8_detector/yolov8_node.py" \
    --ros-args -p model_path:="$YOLO_MODEL" -p image_topic:=/gemini335le/color/image_raw \
    -p conf_threshold:=0.30 -p device:=cpu -p imgsz:=640 -p debug:=true \
  >"$LOG_DIR/yolo.log" 2>&1 </dev/null &
echo $! >"$LOG_DIR/yolo.pid"

nohup env PYTHONPATH="$ROOT:$ROS_PY" LD_LIBRARY_PATH="/opt/ros/humble/lib" \
  python3 "$ROOT/scripts/front_vision_server.py" --camera-ip "$CAMERA_IP" \
    --advertise-host "${V4_ADVERTISE_HOST:-192.168.50.20}" \
  >"$LOG_DIR/http.log" 2>&1 </dev/null &
echo $! >"$LOG_DIR/http.pid"

sleep 3
curl --fail --silent "http://127.0.0.1:8088/health?camera_id=front" || {
  echo "ERROR: front vision HTTP service failed; see $LOG_DIR/http.log" >&2
  exit 6
}
echo
echo "V4 front vision started: http://${V4_ADVERTISE_HOST:-192.168.50.20}:8088/"
