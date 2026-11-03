#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec python3 "$ROOT/scripts/wrist_vision_server.py" \
  --controller 192.168.30.11 \
  --topic-port 19091 \
  --subsystem 'RsCameraSensor#0#0' \
  --listen-port 8092
