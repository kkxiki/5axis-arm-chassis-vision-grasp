#!/usr/bin/env python3
"""Single-shot CTAG2F120 command used by Xbox teleoperation."""
import argparse
import fcntl
from Changingtek_rtu_psdk import Changingtek_rtu_psdk


p = argparse.ArgumentParser()
p.add_argument("--action", choices=("open", "close", "pre"), required=True)
p.add_argument("--port", default="/dev/ttyUSB0")
p.add_argument("--slave", type=int, default=1)
p.add_argument("--speed", type=int, default=40)
p.add_argument("--force", type=int, default=60,
               help="device force percentage; SDK recommended default")
a = p.parse_args()

# Device units are 0.01 mm for the 120 mm stroke.
positions = {"open": 0, "pre": 8000, "close": 12000}
with open("/tmp/ctag2f120.lock", "w") as lock:
    fcntl.flock(lock, fcntl.LOCK_EX)
    gripper = Changingtek_rtu_psdk(a.port, a.slave, baudrate=115200, timeout=0.5)
    if not gripper.connect():
        raise SystemExit("CTAG2F120 connection failed")
    gripper.enable(True)
    gripper.temp_move(positions[a.action], a.speed, a.force, 100, 100, True)
    print(f"CTAG2F120 action={a.action} position={positions[a.action]} force_pct={a.force}")
