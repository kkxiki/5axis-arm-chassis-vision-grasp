from __future__ import annotations

import json
from typing import Any
from urllib.request import Request, urlopen


def event_payload(event_name: str, **parameters: Any) -> dict[str, Any]:
    """Build the same payload as the controller web UI for one robot."""
    return {
        "event_name": event_name,
        "user_action": "Click",
        "sub_commands": [{"selected": True, **parameters}],
    }


class ControllerEventClient:
    def __init__(self, host: str, timeout_s: float = 5.0):
        self.base_url = f"http://{host}"
        self.timeout_s = timeout_s

    def health(self) -> dict[str, Any]:
        try:
            with urlopen(self.base_url + "/", timeout=self.timeout_s) as response:
                return {"ok": response.status == 200, "http_status": response.status}
        except Exception as exc:
            return {"ok": False, "error": str(exc)}

    def send(self, event_name: str, **parameters: Any) -> Any:
        raw = json.dumps(event_payload(event_name, **parameters)).encode("utf-8")
        request = Request(self.base_url + "/backend/send-event-cmd", raw,
                          {"Content-Type": "application/json"})
        with urlopen(request, timeout=self.timeout_s) as response:
            result = json.loads(response.read().decode("utf-8"))
        failures = []

        def collect_failures(value: Any) -> None:
            # Controller firmware versions return either nested arrays, a
            # dictionary, or status strings. Only dictionaries carry a
            # numeric controller return code.
            if isinstance(value, dict):
                code = value.get("return_code", value.get("code"))
                if code is not None:
                    try:
                        if int(code) < 0:
                            failures.append(value)
                    except (TypeError, ValueError):
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
