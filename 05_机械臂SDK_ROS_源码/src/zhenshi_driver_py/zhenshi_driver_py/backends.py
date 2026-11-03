"""Controller backends for dry-run and the pinned official Python SDK."""

from abc import ABC, abstractmethod
import importlib
from pathlib import Path
import sys
import threading
from typing import List

from .models import CommandResult, RobotStateSnapshot


class ControllerBackend(ABC):
    @abstractmethod
    def send_command(self, command: str, timeout_ms: int) -> CommandResult:
        raise NotImplementedError

    @abstractmethod
    def send_stop(self, timeout_ms: int) -> CommandResult:
        raise NotImplementedError

    @abstractmethod
    def send_commands_async(
        self, commands: List[str], timeout_ms: int
    ) -> CommandResult:
        raise NotImplementedError

    @abstractmethod
    def read_state(self, model_index: int) -> RobotStateSnapshot:
        raise NotImplementedError

    @abstractmethod
    def is_connected(self) -> bool:
        raise NotImplementedError


class DryRunBackend(ControllerBackend):
    def __init__(self, model_count: int = 2, joint_count: int = 7):
        self.model_count = int(model_count)
        self.joint_count = int(joint_count)
        self.history: List[str] = []
        self._lock = threading.Lock()

    def send_command(self, command: str, timeout_ms: int) -> CommandResult:
        del timeout_ms
        with self._lock:
            self.history.append(command)
        return CommandResult(True, 0, f"DRY RUN: {command}")

    def send_stop(self, timeout_ms: int) -> CommandResult:
        command = "{" + "||".join(["Stop"] * self.model_count) + "}"
        return self.send_command(command, timeout_ms)

    def send_commands_async(
        self, commands: List[str], timeout_ms: int
    ) -> CommandResult:
        del timeout_ms
        with self._lock:
            self.history.extend(commands)
        return CommandResult(
            True, 0, "DRY RUN ASYNC: " + "; ".join(commands)
        )

    def read_state(self, model_index: int) -> RobotStateSnapshot:
        if not 0 <= int(model_index) < self.model_count:
            raise ValueError("model_index outside dry-run model range")
        return RobotStateSnapshot(
            connected=False,
            data_valid=True,
            system_running=False,
            source="dry_run",
            status_text="DRY_RUN",
            model_index=int(model_index),
            model_count=self.model_count,
            model_name=f"mock_model_{int(model_index)}",
            model_type="mock",
            robottarget=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            jointtarget=[0.0] * self.joint_count,
        )

    def is_connected(self) -> bool:
        return False


class OfficialSdkBackend(ControllerBackend):
    """Adapter for commit-pinned ``Base_SDK_py`` modules."""

    def __init__(
        self,
        sdk_root: str,
        robot_ip: str,
        start_state_subscription: bool = True,
        connect_rpc: bool = True,
    ):
        root = Path(sdk_root).expanduser().resolve()
        if root.name != "Base_SDK_py" and (root / "Base_SDK_py").is_dir():
            root = root / "Base_SDK_py"
        if not (root / "robot_command" / "rpc_client.py").is_file():
            raise FileNotFoundError(f"invalid Base_SDK_py path: {root}")
        if not (root / "robot_state" / "system_state_reader.py").is_file():
            raise FileNotFoundError(f"state reader missing under SDK path: {root}")

        for path in (root, root / "robot_state"):
            path_text = str(path)
            if path_text not in sys.path:
                sys.path.insert(0, path_text)

        self._robot_ip = str(robot_ip)
        self._rpc_client = None
        self._rpc_module = None
        self._state_reader = importlib.import_module(
            "robot_state.system_state_reader"
        )
        self._topic_module = importlib.import_module(
            "robot_state.platform_loader"
        ).get_topic_module()

        if start_state_subscription:
            self._topic_module.start_subscriber(self._robot_ip)

        if connect_rpc:
            self._rpc_module = importlib.import_module(
                "robot_command.rpc_client"
            )
            self._rpc_client = self._rpc_module.RpcClient(self._robot_ip)
            if not self._rpc_client.is_connected():
                raise ConnectionError(
                    "RPC connection failed: "
                    + self._rpc_client.error_info()
                )

        self._send_lock = threading.Lock()

    def _require_rpc(self):
        if self._rpc_client is None:
            raise RuntimeError("RPC client was not enabled")
        return self._rpc_client

    def send_command(self, command: str, timeout_ms: int) -> CommandResult:
        client = self._require_rpc()
        if not client.is_connected():
            return CommandResult(False, -1, client.error_info())

        with self._send_lock:
            message, sequence = client._new_msg_with_seq(command)
            status, responses = client.inner.CallAwait(
                message, int(timeout_ms)
            )

        details = []
        response_ok = True
        for response in responses:
            code = int(response.code)
            response_ok = response_ok and code == 0
            details.append(
                f"seq={sequence} index={response.index} "
                f"code={code} message={response.message}"
            )
        if not details:
            details.append(f"seq={sequence} transport_status={status}")
        return CommandResult(
            success=int(status) == 0 and response_ok,
            transport_status=int(status),
            message="; ".join(details),
        )

    def send_stop(self, timeout_ms: int) -> CommandResult:
        client = self._require_rpc()
        if not client.is_connected():
            return CommandResult(False, -1, client.error_info())
        # The official double-arm examples stop each model slot. A global
        # {Stop} can stop the controller runtime instead of only the arms.
        with self._send_lock:
            self._rpc_module.send_rpc_async(
                client,
                ["{Stop||Stop}"],
                timeout_ms=int(timeout_ms),
                wait_s=0.0,
            )
        submitted = True
        return CommandResult(
            submitted,
            0 if submitted else -1,
            "Double-arm Stop submitted"
            if submitted
            else "Stop submission failed",
        )

    def send_commands_async(
        self, commands: List[str], timeout_ms: int
    ) -> CommandResult:
        client = self._require_rpc()
        if not client.is_connected():
            return CommandResult(False, -1, client.error_info())
        with self._send_lock:
            self._rpc_module.send_rpc_async(
                client,
                list(commands),
                timeout_ms=int(timeout_ms),
                wait_s=0.2,
            )
        return CommandResult(
            True,
            0,
            "asynchronous controller commands submitted: "
            + "; ".join(commands),
        )

    def read_state(self, model_index: int) -> RobotStateSnapshot:
        reader = self._state_reader
        has_data = bool(reader.has_rt_data())
        model_count = int(reader.get_model_count_rt()) if has_data else 0
        if not has_data:
            return RobotStateSnapshot(
                connected=self.is_connected(),
                data_valid=False,
                source="official_sdk",
                status_text="WAITING_FOR_RT_STATE",
                model_index=int(model_index),
                model_count=model_count,
            )
        if not 0 <= int(model_index) < model_count:
            return RobotStateSnapshot(
                connected=self.is_connected(),
                data_valid=False,
                source="official_sdk",
                status_text=(
                    f"MODEL_INDEX_OUT_OF_RANGE:{model_index}/{model_count}"
                ),
                model_index=int(model_index),
                model_count=model_count,
            )

        index = int(model_index)
        return RobotStateSnapshot(
            connected=self.is_connected(),
            data_valid=True,
            system_running=bool(reader.is_system_running()),
            source="official_sdk",
            status_text="RT_STATE_OK",
            model_index=index,
            model_count=model_count,
            model_name=str(reader.get_model_name(index)),
            model_type=str(reader.get_model_type(index)),
            model_state=int(reader.get_model_state(index)),
            error_code=int(reader.get_model_error_code(index)),
            error_message=str(reader.get_model_error_msg(index)),
            robottarget=list(reader.get_current_robottarget(index)),
            jointtarget=list(reader.get_current_jointtarget(index)),
        )

    def is_connected(self) -> bool:
        return bool(
            self._rpc_client is not None
            and self._rpc_client.is_connected()
        )
