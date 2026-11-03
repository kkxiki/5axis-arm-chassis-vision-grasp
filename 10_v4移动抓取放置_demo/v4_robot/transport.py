from __future__ import annotations

import json
import socket
import threading
from typing import Any


class TransportError(RuntimeError):
    pass


class JsonLineClient:
    """One request/one reply JSON-lines TCP transport with sequence IDs."""

    def __init__(self, host: str, port: int, timeout_s: float = 3.0):
        self.host = host
        self.port = int(port)
        self.timeout_s = float(timeout_s)
        self._sock: socket.socket | None = None
        self._file = None
        self._seq = 0
        self._lock = threading.Lock()

    def connect(self) -> None:
        if self._sock is not None:
            return
        self._sock = socket.create_connection((self.host, self.port), self.timeout_s)
        self._sock.settimeout(self.timeout_s)
        self._file = self._sock.makefile("r", encoding="utf-8", newline="\n")

    def close(self) -> None:
        try:
            if self._file:
                self._file.close()
        finally:
            if self._sock:
                self._sock.close()
            self._file = None
            self._sock = None

    def request(self, payload: dict[str, Any], timeout_s: float | None = None) -> dict[str, Any]:
        with self._lock:
            self.connect()
            assert self._sock is not None and self._file is not None
            self._seq += 1
            message = dict(payload)
            message.setdefault("seq", self._seq)
            old_timeout = self._sock.gettimeout()
            self._sock.settimeout(timeout_s or self.timeout_s)
            try:
                self._sock.sendall((json.dumps(message, separators=(",", ":")) + "\n").encode("utf-8"))
                line = self._file.readline()
                if not line:
                    raise TransportError(f"{self.host}:{self.port} 连接已关闭")
                reply = json.loads(line)
                if reply.get("seq") not in (None, message["seq"]):
                    raise TransportError("响应 seq 与请求不一致")
                return reply
            except (OSError, json.JSONDecodeError) as exc:
                self.close()
                raise TransportError(f"{self.host}:{self.port} 通信失败: {exc}") from exc
            finally:
                if self._sock is not None:
                    self._sock.settimeout(old_timeout)

    def __enter__(self) -> "JsonLineClient":
        self.connect()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

