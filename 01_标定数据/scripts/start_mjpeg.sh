#!/bin/bash
source /opt/ros/humble/setup.bash
cd /home/nvidia/calibration || exit 9
setsid nohup python3 mjpeg_server.py > mjpeg_server.log 2>&1 &
echo $! > /home/nvidia/calibration/mjpeg_server.pid
echo "mjpeg started pid=$(cat /home/nvidia/calibration/mjpeg_server.pid)"