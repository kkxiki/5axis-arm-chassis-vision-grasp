"""JSON-Lines over TCP 传输层：移动底盘等设备用它做请求/应答通信。

协议约定：一条连接上「一行一个 JSON」——请求写一行（以 \\n 结尾），
对端回一行 JSON 作为应答，每条请求带上自增的 seq 字段，回包必须带同样的 seq，
这样可以在同一条长连接上把请求和应答严格配对，不必每次重新建连。
"""
from __future__ import annotations

import json
import socket
import threading
from typing import Any


class TransportError(RuntimeError):
    """传输层异常：连接关闭、超时、JSON 解析失败、seq 不匹配等统一用它抛出。"""
    pass


class JsonLineClient:
    """一条长连接的 JSON-Lines 客户端：一次请求对应一次应答，并用自增 seq 做配对校验。"""

    def __init__(self, host: str, port: int, timeout_s: float = 3.0):
        self.host = host
        self.port = int(port)
        # 默认读写超时（秒）；单次 request 可以覆盖
        self.timeout_s = float(timeout_s)
        # 以下几个都是懒初始化的连接状态：首次 request 时才真正建连
        self._sock: socket.socket | None = None
        self._file = None
        self._seq = 0
        # 同一连接上请求/应答必须串行，用锁把并发调用排队
        self._lock = threading.Lock()

    def connect(self) -> None:
        """建立 TCP 连接（已连接则直接返回）。

        makefile 以文本方式按行读取，newline="\\n" 表示只按 \n 切行（协议就是一行一个 JSON）。
        """
        if self._sock is not None:
            return
        self._sock = socket.create_connection((self.host, self.port), self.timeout_s)
        self._sock.settimeout(self.timeout_s)
        self._file = self._sock.makefile("r", encoding="utf-8", newline="\n")

    def close(self) -> None:
        """关闭连接：先关文件对象再关 socket，并把状态复位，以便下次调用自动重连。"""
        try:
            if self._file:
                self._file.close()
        finally:
            if self._sock:
                self._sock.close()
            self._file = None
            self._sock = None

    def request(self, payload: dict[str, Any], timeout_s: float | None = None) -> dict[str, Any]:
        """发送一条请求并返回应答字典。

        参数：
            payload   —— 请求内容（会被复制一份并补上 seq），不会被就地修改
            timeout_s —— 本次请求的超时时间（秒）；为 None 时用实例默认超时
        返回：
            对端返回的 JSON 字典（含 ok / seq 等字段，具体由各设备协议定义）
        行为：
            整个「发送 + 读应答」过程加锁串行；通信失败会关闭连接（迫使下次重连）并抛 TransportError。
        """
        with self._lock:
            self.connect()
            # connect() 之后这两个字段必定非空，这里的断言只用于类型收窄
            assert self._sock is not None and self._file is not None
            self._seq += 1
            message = dict(payload)
            # 复制一份再补 seq（调用方若已显式给出 seq 则保留其值）
            message.setdefault("seq", self._seq)
            old_timeout = self._sock.gettimeout()
            self._sock.settimeout(timeout_s or self.timeout_s)
            try:
                # 紧凑 JSON + 换行，作为一帧发出（ensure_ascii 默认转义非 ASCII 更安全）
                self._sock.sendall((json.dumps(message, separators=(",", ":")) + "\n").encode("utf-8"))
                line = self._file.readline()
                # 读到空行说明对端已关闭连接
                if not line:
                    raise TransportError(f"{self.host}:{self.port} 连接已关闭")
                reply = json.loads(line)
                # 对端可以不带 seq（视作兼容），但带了就必须与本次请求一致
                if reply.get("seq") not in (None, message["seq"]):
                    raise TransportError("响应 seq 与请求不一致")
                return reply
            except (OSError, json.JSONDecodeError) as exc:
                # 网络或解析出错后连接状态已不可信，直接关闭，下次 request 会重新建连
                self.close()
                raise TransportError(f"{self.host}:{self.port} 通信失败: {exc}") from exc
            finally:
                # 恢复原有超时，避免本次的超时设置影响后续调用
                if self._sock is not None:
                    self._sock.settimeout(old_timeout)

    def __enter__(self) -> "JsonLineClient":
        """支持 with 用法：进入时建连。"""
        self.connect()
        return self

    def __exit__(self, *_: object) -> None:
        """支持 with 用法：退出时无论如何都关闭连接。"""
        self.close()

