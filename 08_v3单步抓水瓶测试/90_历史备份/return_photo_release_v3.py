#!/usr/bin/env python3
"""Return to the taught grasp-photo pose and release the CTAG2F120 gripper."""
from bottle_grasp_v3 import Gateway, GraspError, RobotState, gripper, load_pose, move_exact, PHOTO_POSE

gw = Gateway()
state = RobotState()
try:
    current = gw.state()
    if current.get("owner") != "IDLE" or current.get("state") != "READY" or not current.get("arm_enabled"):
        raise GraspError(f"ArmGateway not free and ready: {current}")
    owner = gw.acquire()
    if not owner.get("ok"):
        raise GraspError(f"cannot acquire gateway: {owner}")
    move_exact(gw, state, load_pose(PHOTO_POSE), "return_to_grasp_photo")
    gripper("open", 60, 40)
    print("[DONE] returned to grasp-photo pose and released", flush=True)
finally:
    gw.zero()
    gw.release()
    gw.close()
