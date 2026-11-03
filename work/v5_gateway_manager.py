#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import socket
import subprocess
import time
from pathlib import Path


HOST_CONFIG = Path("/home/nvidia/speedl_deploy/arm_gateway/config/arm_gateway.yaml")
CONTAINER = "isaac_ros_jetson"
CONTAINER_CONFIG = "/root/XuK/arm_gateway/config/arm_gateway.yaml"
START_SCRIPT = "/home/nvidia/speedl_deploy/arm_gateway/scripts/start_gateway.sh"
STOP_SCRIPT = "/home/nvidia/speedl_deploy/arm_gateway/scripts/stop_gateway.sh"


def run(cmd: list[str], check: bool = False) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, text=True, capture_output=True, check=check)


def status() -> dict:
    try:
        with socket.create_connection(("127.0.0.1", 6200), timeout=2.0) as sock:
            sock.sendall(b'{"cmd":"state"}\n')
            line = sock.makefile("r", encoding="utf-8").readline()
        value = json.loads(line)
        return {"ok": True, "gateway": value}
    except Exception as exc:
        return {"ok": False, "error": str(exc)}


def gateway_request(payload: dict, timeout: float = 30.0) -> dict:
    with socket.create_connection(("127.0.0.1", 6200), timeout=timeout) as sock:
        sock.settimeout(timeout)
        sock.sendall((json.dumps(payload, separators=(",", ":")) + "\n").encode("utf-8"))
        line = sock.makefile("r", encoding="utf-8").readline()
    if not line:
        raise RuntimeError("gateway returned no reply")
    return json.loads(line)


def clear_error() -> dict:
    """Clear the controller fault without changing its servo enable state."""
    return gateway_request({"cmd": "clear_error"})


def set_controller_ip(ip: str) -> None:
    for path in (HOST_CONFIG,):
        text = path.read_text(encoding="utf-8")
        lines = []
        replaced = False
        for line in text.splitlines():
            if line.startswith("controller_ip:"):
                lines.append(f"controller_ip: {ip}")
                replaced = True
            else:
                lines.append(line)
        if not replaced:
            lines.insert(0, f"controller_ip: {ip}")
        path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    script = (
        f"sed -i 's/^controller_ip:.*/controller_ip: {ip}/' "
        f"{CONTAINER_CONFIG}"
    )
    run(["docker", "exec", CONTAINER, "bash", "-lc", script])


def main() -> int:
    parser = argparse.ArgumentParser(description="V5 ArmGateway manager. Does not move the robot.")
    parser.add_argument("command", choices=("status", "start", "stop", "restart", "clear", "set-ip"))
    parser.add_argument("--controller-ip", default="192.168.30.11")
    args = parser.parse_args()

    if args.command == "set-ip":
        set_controller_ip(args.controller_ip)
        print(json.dumps({"ok": True, "controller_ip": args.controller_ip}, ensure_ascii=False, indent=2))
        return 0
    if args.command in ("start", "restart"):
        current = status()
        if args.command == "start" and current.get("ok"):
            # Idempotent start: an already running gateway must be left intact.
            print(json.dumps(current, ensure_ascii=False, indent=2))
            return 0
        if args.command == "restart":
            run(["bash", STOP_SCRIPT])
            time.sleep(0.5)
        set_controller_ip(args.controller_ip)
        run(["docker", "start", CONTAINER])
        run(["bash", START_SCRIPT])
        time.sleep(1.0)
        print(json.dumps(status(), ensure_ascii=False, indent=2))
        return 0
    if args.command == "clear":
        try:
            result = clear_error()
        except Exception as exc:
            print(json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False, indent=2))
            return 2
        print(json.dumps({"ok": bool(result.get("ok")), "clear_error": result},
                         ensure_ascii=False, indent=2))
        return 0 if result.get("ok") else 2
    if args.command == "stop":
        run(["bash", STOP_SCRIPT])
        print(json.dumps(status(), ensure_ascii=False, indent=2))
        return 0
    print(json.dumps(status(), ensure_ascii=False, indent=2))
    return 0 if status().get("ok") else 2


if __name__ == "__main__":
    raise SystemExit(main())
