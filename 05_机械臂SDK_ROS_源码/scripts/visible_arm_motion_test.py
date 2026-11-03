#!/usr/bin/env python3
"""Run a clearly visible, reversible single-joint test on a dual-arm controller."""

import argparse
import json
import math
import time

from robot_command.rpc_client import RpcClient, send_rpc_async
from robot_state.platform_loader import get_topic_module
from robot_state import system_state_reader as state


MODEL_COUNT = 2
JOINT_COUNT = 7
COMMAND_SLOTS = 10
INIT_COMMANDS = [
    "{Clear}",
    "{Disable}",
    "{Mode}",
    "{SetMaxToq}",
    "{Recover}",
    "{SetRate}",
    "{Enable}",
]


def format_values(values):
    return "{" + ",".join(f"{float(value):.12g}" for value in values) + "}"


def padded(joints):
    return list(joints) + [0.0] * (COMMAND_SLOTS - len(joints))


def call_await(client, command, timeout_ms):
    message, sequence = client._new_msg_with_seq(command)
    print(f"SEND seq={sequence}: {command}", flush=True)
    status, responses = client.inner.CallAwait(message, int(timeout_ms))
    result = {
        "transport_status": int(status),
        "responses": [
            {"index": int(r.index), "code": int(r.code), "message": str(r.message)}
            for r in responses
        ],
    }
    print(json.dumps(result, ensure_ascii=False), flush=True)
    if result["transport_status"] != 0:
        raise RuntimeError(f"RPC transport failed: {result}")
    if any(item["code"] < 0 for item in result["responses"]):
        raise RuntimeError(f"controller rejected command: {result}")
    return result


def wait_for_state(robot_ip, timeout_s=8.0):
    get_topic_module().start_subscriber(robot_ip)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if state.has_rt_data() and state.get_model_count_rt() == MODEL_COUNT:
            if all(state.get_joint_count(i) == JOINT_COUNT for i in range(MODEL_COUNT)):
                return
        time.sleep(0.1)
    raise RuntimeError("real-time state did not report two 7-axis models")


def joints_snapshot():
    return [list(state.get_current_jointtarget(i)) for i in range(MODEL_COUNT)]


def wait_at_target(targets, tolerance_deg=0.15, timeout_s=15.0):
    tolerance = math.radians(tolerance_deg)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        actual = joints_snapshot()
        error = max(
            abs(actual[m][j] - targets[m][j])
            for m in range(MODEL_COUNT)
            for j in range(JOINT_COUNT)
        )
        if error <= tolerance:
            return actual
        time.sleep(0.1)
    raise RuntimeError(f"target timeout; actual={joints_snapshot()} target={targets}")


def define_and_move(client, targets, tag):
    names = [f"codex_visible_{tag}_m0", f"codex_visible_{tag}_m1"]
    for model_index, name in enumerate(names):
        call_await(
            client,
            "{Var --type=jointtarget "
            f"--name={name} --value={format_values(padded(targets[model_index]))}"
            "}",
            5000,
        )
    call_await(
        client,
        "{MoveAbsJ --jointtarget_var=" + names[0] + " --speed=v1"
        "||MoveAbsJ --jointtarget_var=" + names[1] + " --speed=v1}",
        50000,
    )
    return wait_at_target(targets)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-ip", default="192.168.11.11")
    parser.add_argument("--model-index", type=int, default=0, choices=(0, 1))
    parser.add_argument("--joint-index", type=int, default=1, choices=range(JOINT_COUNT))
    parser.add_argument("--delta-deg", type=float, default=7.0)
    parser.add_argument("--hold-seconds", type=float, default=30.0)
    parser.add_argument("--countdown-seconds", type=float, default=5.0)
    args = parser.parse_args()
    if not 1.0 <= abs(args.delta_deg) <= 12.0:
        raise SystemExit("delta magnitude must be within 1..12 degrees")
    if not 0.0 <= args.hold_seconds <= 120.0:
        raise SystemExit("hold-seconds must be within 0..120")

    wait_for_state(args.robot_ip)
    original = joints_snapshot()
    target = [row[:] for row in original]
    target[args.model_index][args.joint_index] += math.radians(args.delta_deg)
    print(
        json.dumps(
            {
                "model_index": args.model_index,
                "joint_index": args.joint_index,
                "delta_deg": args.delta_deg,
                "hold_seconds": args.hold_seconds,
                "original": original,
                "target": target,
            },
            indent=2,
        ),
        flush=True,
    )
    print(f"STARTING IN {args.countdown_seconds:.1f}s", flush=True)
    time.sleep(args.countdown_seconds)

    client = RpcClient(args.robot_ip)
    if not client.is_connected():
        raise RuntimeError(f"RPC connection failed: {client.error_info()}")

    moved = False
    try:
        for command in INIT_COMMANDS:
            call_await(client, command, 1000)
            time.sleep(0.1)
        time.sleep(3.0)
        if not all(
            state.get_joint_is_enabled(args.model_index, j) for j in range(JOINT_COUNT)
        ):
            raise RuntimeError("target model did not enable")

        reached = define_and_move(client, target, "away")
        moved = True
        actual_delta = math.degrees(
            reached[args.model_index][args.joint_index]
            - original[args.model_index][args.joint_index]
        )
        print(f"VISIBLE_POSITION_REACHED actual_delta_deg={actual_delta:.3f}", flush=True)
        print(f"HOLDING {args.hold_seconds:.1f}s", flush=True)
        time.sleep(args.hold_seconds)

        returned = define_and_move(client, original, "return")
        return_error = math.degrees(
            returned[args.model_index][args.joint_index]
            - original[args.model_index][args.joint_index]
        )
        print(f"RETURN_REACHED error_deg={return_error:.3f}", flush=True)
        moved = False
        print("VISIBLE_MOTION_TEST_SUCCESS", flush=True)
    except Exception:
        if moved and client.is_connected():
            send_rpc_async(client, ["{Stop||Stop}"], timeout_ms=10000, wait_s=0.0)
        raise
    finally:
        if client.is_connected():
            call_await(client, "{Disable}", 5000)


if __name__ == "__main__":
    main()
