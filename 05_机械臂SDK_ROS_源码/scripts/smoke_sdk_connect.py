#!/usr/bin/env python3
"""Connect to RPC port without sending any controller command."""

import os
from pathlib import Path
import sys


def main():
    root = Path(os.environ["ZHENSHI_SDK_ROOT"]).resolve()
    sys.path.insert(0, str(root))
    from robot_command.rpc_client import RpcClient

    robot_ip = os.environ.get("ZHENSHI_ROBOT_IP", "192.168.11.11")
    client = RpcClient(robot_ip)
    print(f"ip={robot_ip} connected={client.is_connected()}")
    if not client.is_connected():
        print(client.error_info())
        return 1
    print("PASS: RPC connected; no command was sent.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

