"""机械臂 controller 的 HTTP 事件接口客户端。

控制器（如示教器上的 Web 控制台）是通过 HTTP 事件来触发动作的：向前端接口
POST /backend/send-event-cmd 发送一个 JSON，里面描述「点了哪个事件、带哪些参数」。
本模块复刻同样的报文格式，让程序可以把控制器当成一台受控设备来用，
因此夹爪这类已由控制器管理的执行器不需要再额外接一套 SDK。

约定：host 形如 "192.168.1.10"（或带端口的 "ip:port"），默认超时 5 秒。
"""
from __future__ import annotations

import json
from typing import Any
from urllib.request import Request, urlopen


def event_payload(event_name: str, **parameters: Any) -> dict[str, Any]:
    """拼装一次控制器事件调用的报文，格式与控制器 Web UI 点按钮时发出的完全一致。

    event_name 为控制器里的事件名（例如 "MoveTwoFingersGripper"），
    **parameters 为该事件需要的参数（例如 distance=70, interval=25）。
    返回字典含 event_name / user_action / sub_commands 三部分。
    """
    return {
        "event_name": event_name,
        "user_action": "Click",
        "sub_commands": [{"selected": True, **parameters}],
    }


class ControllerEventClient:
    """控制器事件接口客户端：负责探测服务可达性，以及发送事件并检查控制器返回码。"""
    def __init__(self, host: str, timeout_s: float = 5.0):
        # host 直接拼成 http://<host>，因此 host 里可以带端口（如 "192.168.1.10:8080"）
        self.base_url = f"http://{host}"
        self.timeout_s = timeout_s

    def health(self) -> dict[str, Any]:
        """探测控制器 Web 服务是否可达：GET / 返回 200 即视为健康。

        该方法不抛异常，失败时返回 ok=False 并把错误信息放在 error 字段里，便于预检统一收集。
        """
        try:
            with urlopen(self.base_url + "/", timeout=self.timeout_s) as response:
                return {"ok": response.status == 200, "http_status": response.status}
        except Exception as exc:
            return {"ok": False, "error": str(exc)}

    def send(self, event_name: str, **parameters: Any) -> Any:
        """向控制器发送一个事件并解析响应，返回控制器原始返回内容。

        为什么还要遍历响应：不同固件版本返回的结构不一样（嵌套数组 / 字典 / 状态字符串），
        只有字典里才带数值型的控制器返回码（return_code 或 code），
        凡是出现负的返回码就说明该事件被控制器拒绝，这里统一抛 RuntimeError。
        """
        raw = json.dumps(event_payload(event_name, **parameters)).encode("utf-8")
        # 与控制器前端一致：POST /backend/send-event-cmd，Content-Type 为 application/json
        request = Request(self.base_url + "/backend/send-event-cmd", raw,
                          {"Content-Type": "application/json"})
        with urlopen(request, timeout=self.timeout_s) as response:
            result = json.loads(response.read().decode("utf-8"))
        failures = []

        def collect_failures(value: Any) -> None:
            # 控制器固件版本可能回嵌套数组、字典或状态字符串；
            # 只有字典才携带数值型控制器返回码，因此只对字典做检查，并递归遍历其所有子值。
            if isinstance(value, dict):
                code = value.get("return_code", value.get("code"))
                if code is not None:
                    try:
                        # 返回码小于 0 表示该事件执行失败/被拒绝
                        if int(code) < 0:
                            failures.append(value)
                    except (TypeError, ValueError):
                        # 返回码不是数字（例如字符串状态）时忽略，不做失败判定
                        pass
                for child in value.values():
                    collect_failures(child)
            elif isinstance(value, list):
                for child in value:
                    collect_failures(child)

        collect_failures(result)
        if failures:
            raise RuntimeError(f"controller rejected {event_name}: {failures}")
        return result
