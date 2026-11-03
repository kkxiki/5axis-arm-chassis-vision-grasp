#!/usr/bin/env python3
"""Read the single-arm pose through the official SDK and atomically cache it."""
import argparse
import json
import os
import sys
import time
from pathlib import Path
import math

p = argparse.ArgumentParser()
p.add_argument("--sdk-root", required=True)
p.add_argument("--robot-ip", default="192.168.11.11")
p.add_argument("--output", default="/tmp/five_axis_pose.json")
a = p.parse_args()
sys.path.insert(0, a.sdk_root)
sys.path.insert(0, str(Path(a.sdk_root) / "robot_state"))

from robot_state.platform_loader import get_topic_module
from robot_state.system_state_reader import (
    get_current_robottarget, get_header_timestamp, get_model_count_rt,
    get_model_name, has_rt_data
)

get_topic_module().start_subscriber(a.robot_ip)
out = Path(a.output)
previous_pose = None
previous_time = None
stable_since = time.time()
previous_source_stamp = None
while True:
    if has_rt_data() and get_model_count_rt() > 0:
        now = time.time()
        source_stamp = int(get_header_timestamp())
        # The SDK keeps returning its last decoded message if the ZMQ stream
        # stalls.  Do not refresh the wall-clock timestamp in that case: the
        # capture process must see the pose as stale and reject the sample.
        if source_stamp == previous_source_stamp:
            time.sleep(0.01)
            continue
        pose = list(get_current_robottarget(0))
        linear_speed = 0.0
        angular_speed = 0.0
        if previous_pose is not None and now > previous_time:
            dt = now - previous_time
            linear_speed = math.sqrt(sum((pose[i] - previous_pose[i]) ** 2 for i in range(3))) / dt
            dot = abs(sum(pose[i] * previous_pose[i] for i in range(3, 7)))
            dot = max(-1.0, min(1.0, dot))
            angular_speed = 2.0 * math.acos(dot) / dt
        data = {"time": now, "model_index": 0,
                "model_name": get_model_name(0),
                "source_header_timestamp": source_stamp,
                "robottarget": pose,
                "linear_speed_mps": linear_speed,
                "angular_speed_radps": angular_speed}
        if linear_speed > 0.002 or angular_speed > math.radians(0.5):
            stable_since = now
        data["stable_duration_sec"] = max(0.0, now - stable_since)
        tmp = out.with_suffix(".tmp")
        tmp.write_text(json.dumps(data), encoding="utf-8")
        os.replace(tmp, out)
        previous_pose = pose
        previous_time = now
        previous_source_stamp = source_stamp
    time.sleep(0.05)
