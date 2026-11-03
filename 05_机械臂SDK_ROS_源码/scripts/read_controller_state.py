#!/usr/bin/env python3
"""Read extended controller state without creating an RPC command client."""

import argparse
import json
import time

from robot_state.platform_loader import get_topic_module
from robot_state import system_state_reader as state


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-ip", default="192.168.11.11")
    parser.add_argument("--wait-seconds", type=float, default=2.0)
    parser.add_argument("--require-enabled-model", type=int)
    args = parser.parse_args()

    get_topic_module().start_subscriber(args.robot_ip)
    time.sleep(args.wait_seconds)

    model_count = state.get_model_count_rt()
    result = {
        "system_running": state.is_system_running(),
        "system_init": state.is_system_init(),
        "model_count": model_count,
        "models": [],
    }
    for model_index in range(model_count):
        joint_count = state.get_joint_count(model_index)
        result["models"].append(
            {
                "model_index": model_index,
                "model_name": state.get_model_name(model_index),
                "model_state": state.get_model_state(model_index),
                "error_code": state.get_model_error_code(model_index),
                "error_message": state.get_model_error_msg(model_index),
                "joint_count": joint_count,
                "joint_enabled": [
                    state.get_joint_is_enabled(model_index, joint_index)
                    for joint_index in range(joint_count)
                ],
                "joint_modes": [
                    state.get_joint_mode(model_index, joint_index)
                    for joint_index in range(joint_count)
                ],
                "jointtarget": list(
                    state.get_current_jointtarget(model_index)
                ),
            }
        )
    print(json.dumps(result, indent=2, ensure_ascii=False))
    if args.require_enabled_model is not None:
        index = args.require_enabled_model
        if not 0 <= index < len(result["models"]):
            raise SystemExit(2)
        if not all(result["models"][index]["joint_enabled"]):
            raise SystemExit(3)


if __name__ == "__main__":
    main()
