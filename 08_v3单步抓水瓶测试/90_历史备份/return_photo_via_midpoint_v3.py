#!/usr/bin/env python3
"""Return to the grasp-photo pose through a bounded midpoint."""
import math

from bottle_grasp_v3 import (
    Gateway,
    GraspError,
    PHOTO_POSE,
    RobotState,
    ensure_gateway_ready,
    load_pose,
    move_exact,
)


def slerp(a, b, t):
    dot = sum(x * y for x, y in zip(a, b))
    if dot < 0:
        b = [-x for x in b]
        dot = -dot
    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9995:
        q = [(1 - t) * x + t * y for x, y in zip(a, b)]
    else:
        theta = math.acos(dot)
        q = [
            math.sin((1 - t) * theta) / math.sin(theta) * x
            + math.sin(t * theta) / math.sin(theta) * y
            for x, y in zip(a, b)
        ]
    n = math.sqrt(sum(x * x for x in q))
    return [x / n for x in q]


gw = Gateway()
state = RobotState()
try:
    ensure_gateway_ready(gw)
    owner = gw.acquire()
    if not owner.get("ok"):
        raise GraspError(f"cannot acquire gateway: {owner}")
    current = state.pose()
    target = load_pose(PHOTO_POSE)
    distance = math.dist(current[:3], target[:3])
    if distance > 0.45:
        midpoint = [
            (current[0] + target[0]) / 2,
            (current[1] + target[1]) / 2,
            max(current[2], target[2]) + 0.03,
        ] + slerp(current[3:7], target[3:7], 0.5)
        move_exact(gw, state, midpoint, "return_midpoint")
        ensure_gateway_ready(gw, owner_required=True)
    move_exact(gw, state, target, "return_to_grasp_photo")
    print("[DONE] grasp-photo pose restored", flush=True)
finally:
    gw.zero()
    gw.release()
    gw.close()
