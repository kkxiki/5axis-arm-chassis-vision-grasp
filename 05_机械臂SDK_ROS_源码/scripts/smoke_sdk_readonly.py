#!/usr/bin/env python3
"""Subscribe to controller state without opening the RPC command client."""

import os
from pathlib import Path
import sys
import time


def main():
    root = Path(os.environ["ZHENSHI_SDK_ROOT"]).resolve()
    sys.path.insert(0, str(root))
    sys.path.insert(0, str(root / "robot_state"))

    from robot_state.platform_loader import get_topic_module
    from robot_state.system_state_reader import (
        get_current_jointtarget,
        get_current_robottarget,
        get_model_count_rt,
        get_model_name,
        has_rt_data,
    )

    robot_ip = os.environ.get("ZHENSHI_ROBOT_IP", "192.168.11.11")
    timeout_s = float(os.environ.get("ZHENSHI_STATE_TIMEOUT_S", "15"))
    get_topic_module().start_subscriber(robot_ip)
    print(f"Waiting for read-only state from {robot_ip}:19091 ...")

    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if has_rt_data():
            count = get_model_count_rt()
            print(f"PASS: received RT state, model_count={count}")
            for index in range(count):
                print(f"model[{index}] name={get_model_name(index)}")
                print(f"  robottarget={get_current_robottarget(index)}")
                print(f"  jointtarget={get_current_jointtarget(index)}")
            return 0
        time.sleep(0.2)

    print("FAIL: no RT state received before timeout")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())

