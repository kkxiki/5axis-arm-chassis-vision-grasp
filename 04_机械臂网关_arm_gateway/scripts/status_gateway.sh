#!/usr/bin/env bash
set -euo pipefail
CONTAINER="${CONTAINER:-isaac_ros_jetson}"
docker exec "${CONTAINER}" python3 -c 'import json,socket;s=socket.create_connection(("127.0.0.1",6200),2);s.sendall(b"{\"cmd\":\"state\"}\n");print(s.makefile().readline().strip())'
