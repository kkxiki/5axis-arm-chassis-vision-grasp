#!/usr/bin/env python3
"""A bounded first-motion test derived from the official double-arm demo."""

import argparse
import json
import math
import time

from robot_command.rpc_client import RpcClient, send_rpc_async
from robot_state.platform_loader import get_topic_module
from robot_state import system_state_reader as state


INIT_COMMANDS = [
    "{Clear}",
    "{Disable}",
    "{Mode}",
    "{SetMaxToq}",
    "{Recover}",
    "{SetRate}",
    "{Enable}",
]
EXPECTED_MODELS = 2
EXPECTED_JOINTS_PER_MODEL = 7
COMMAND_SLOTS = 10
MAX_DELTA_RAD = math.radians(0.5)


def format_values(values):
    return "{" + ",".join(f"{float(value):.12g}" for value in values) + "}"


def padded(joints):
    values = list(joints)
    if len(values) != EXPECTED_JOINTS_PER_MODEL:
        raise RuntimeError(
            f"expected {EXPECTED_JOINTS_PER_MODEL} joints, got {len(values)}"
        )
    return values + [0.0] * (COMMAND_SLOTS - len(values))


def call_await(client, command, timeout_ms):
    message, sequence = client._new_msg_with_seq(command)
    print(f"send[seq={sequence} timeout_ms={timeout_ms}]: {command}")
    status, responses = client.inner.CallAwait(message, int(timeout_ms))
    result = {
        "sequence": sequence,
        "command": command,
        "transport_status": int(status),
        "responses": [
            {
                "index": int(response.index),
                "code": int(response.code),
                "message": str(response.message),
            }
            for response in responses
        ],
    }
    print(json.dumps(result, ensure_ascii=False))
    return result


def wait_for_state(robot_ip, timeout_s=8.0):
    get_topic_module().start_subscriber(robot_ip)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if state.has_rt_data() and state.get_model_count_rt() == EXPECTED_MODELS:
            counts = [
                state.get_joint_count(model_index)
                for model_index in range(EXPECTED_MODELS)
            ]
            if counts == [EXPECTED_JOINTS_PER_MODEL] * EXPECTED_MODELS:
                return
        time.sleep(0.1)
    raise RuntimeError("real-time state did not report two 7-axis models")


def snapshot():
    result = []
    for model_index in range(EXPECTED_MODELS):
        result.append(
            {
                "model_index": model_index,
                "error_code": state.get_model_error_code(model_index),
                "error_message": state.get_model_error_msg(model_index),
                "enabled": [
                    state.get_joint_is_enabled(model_index, joint_index)
                    for joint_index in range(EXPECTED_JOINTS_PER_MODEL)
                ],
                "joints": list(
                    state.get_current_jointtarget(model_index)
                ),
            }
        )
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-ip", default="192.168.11.11")
    parser.add_argument("--model-index", type=int, default=0)
    parser.add_argument("--joint-index", type=int, default=6)
    parser.add_argument("--delta-deg", type=float, default=0.5)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    if args.model_index not in (0, 1):
        raise SystemExit("model-index must be 0 or 1")
    if not 0 <= args.joint_index < EXPECTED_JOINTS_PER_MODEL:
        raise SystemExit("joint-index must be within 0..6")
    delta_rad = math.radians(args.delta_deg)
    if delta_rad == 0.0 or abs(delta_rad) > MAX_DELTA_RAD + 1e-12:
        raise SystemExit("delta must be non-zero and no more than 0.5 degree")

    wait_for_state(args.robot_ip)
    before = snapshot()
    if any(model["error_code"] != 0 for model in before):
        raise RuntimeError(f"controller model error before motion: {before}")

    targets = [list(model["joints"]) for model in before]
    targets[args.model_index][args.joint_index] += delta_rad
    variable_names = ["codex_first_m0", "codex_first_m1"]
    var_commands = [
        (
            "{Var --type=jointtarget "
            f"--name={variable_names[index]} "
            f"--value={format_values(padded(targets[index]))}"
            "}"
        )
        for index in range(EXPECTED_MODELS)
    ]
    move_command = (
        "{MoveAbsJ --jointtarget_var=codex_first_m0 --speed=v1"
        "||MoveAbsJ --jointtarget_var=codex_first_m1 --speed=v1}"
    )

    preview = {
        "before": before,
        "target_model": args.model_index,
        "target_joint": args.joint_index,
        "delta_deg": args.delta_deg,
        "targets": targets,
        "var_commands": var_commands,
        "move_command": move_command,
    }
    print(json.dumps(preview, indent=2, ensure_ascii=False))
    if not args.execute:
        print("PREVIEW ONLY: pass --execute to send controller commands")
        return

    client = RpcClient(args.robot_ip)
    if not client.is_connected():
        raise RuntimeError(f"RPC connection failed: {client.error_info()}")

    # Match the official demo: short synchronous initialization calls and
    # continue through idempotent command errors/timeouts while connected.
    for command in INIT_COMMANDS:
        call_await(client, command, 500)
        if not client.is_connected():
            raise RuntimeError(f"RPC disconnected during init: {client.error_info()}")
        time.sleep(0.1)

    time.sleep(3.0)
    enabled_state = snapshot()
    if not all(enabled_state[args.model_index]["enabled"]):
        raise RuntimeError(
            f"target model was not enabled after official init: {enabled_state}"
        )

    for command in var_commands:
        result = call_await(client, command, 5000)
        if result["transport_status"] != 0:
            raise RuntimeError("joint target variable command failed")
        if any(item["code"] != 0 for item in result["responses"]):
            raise RuntimeError("controller rejected joint target variable")

    motion_started = False
    try:
        motion_started = True
        result = call_await(client, move_command, 50000)
        if result["transport_status"] != 0:
            raise RuntimeError("MoveAbsJ transport failed")
        if any(item["code"] != 0 for item in result["responses"]):
            raise RuntimeError("controller rejected MoveAbsJ")
        time.sleep(2.0)
        after = snapshot()
        print(json.dumps({"after": after}, indent=2, ensure_ascii=False))
        actual = after[args.model_index]["joints"][args.joint_index]
        expected = targets[args.model_index][args.joint_index]
        if abs(actual - expected) > math.radians(0.1):
            raise RuntimeError(
                f"joint did not reach target: actual={actual}, expected={expected}"
            )
        print("FIRST_MOTION_TEST_SUCCESS")
    except Exception:
        if motion_started and client.is_connected():
            send_rpc_async(
                client,
                ["{Stop||Stop}"],
                timeout_ms=10000,
                wait_s=0.0,
            )
        raise
    finally:
        if client.is_connected():
            call_await(client, "{Disable}", 5000)


if __name__ == "__main__":
    main()
