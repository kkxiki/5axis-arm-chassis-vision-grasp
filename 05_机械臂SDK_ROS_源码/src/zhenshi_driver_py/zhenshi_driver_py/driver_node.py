"""ROS 2 node wrapping the official Zhenshi Python SDK."""

import os
import threading
from typing import Iterable

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from zhenshi_interfaces.msg import RobotState
from zhenshi_interfaces.srv import (
    ExecuteCommand,
    JogJoint,
    MoveJoint,
    MovePose,
    SetGripper,
)

from .backends import DryRunBackend, OfficialSdkBackend
from .commands import (
    build_gripper,
    build_joint_jog,
    build_move_joint,
    build_move_pose_sequence,
    raw_command,
)
from .models import CommandResult


SYSTEM_INIT_COMMANDS = (
    "{Clear}",
    "{Mode}",
    "{SetMaxToq}",
    "{Recover}",
    "{SetRate}",
    "{Enable}",
)


class ZhenshiDriverNode(Node):
    def __init__(self):
        super().__init__("zhenshi_driver")

        self.declare_parameter("robot_ip", "192.168.11.11")
        self.declare_parameter("model_index", 0)
        self.declare_parameter("model_count", 2)
        self.declare_parameter("joint_count", 7)
        self.declare_parameter("command_joint_slots", 10)
        self.declare_parameter("joint_speed_value", 0.01)
        self.declare_parameter("max_joint_step_rad", 0.008726646259971648)
        self.declare_parameter("state_publish_hz", 10.0)
        self.declare_parameter("rpc_timeout_ms", 5000)
        self.declare_parameter("dry_run", True)
        self.declare_parameter("allow_raw_commands", False)
        self.declare_parameter(
            "sdk_root",
            os.environ.get("ZHENSHI_SDK_ROOT", ""),
        )
        self.declare_parameter("workspace_min_m", [0.0, -0.7, -0.3])
        self.declare_parameter("workspace_max_m", [1.0, 0.7, 0.5])

        self.robot_ip = str(self.get_parameter("robot_ip").value)
        self.model_index = int(self.get_parameter("model_index").value)
        self.model_count = int(self.get_parameter("model_count").value)
        self.joint_count = int(self.get_parameter("joint_count").value)
        self.command_joint_slots = int(
            self.get_parameter("command_joint_slots").value
        )
        self.joint_speed_value = float(
            self.get_parameter("joint_speed_value").value
        )
        self.max_joint_step_rad = float(
            self.get_parameter("max_joint_step_rad").value
        )
        self.rpc_timeout_ms = int(self.get_parameter("rpc_timeout_ms").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.allow_raw_commands = bool(
            self.get_parameter("allow_raw_commands").value
        )
        self.workspace_min_m = list(
            self.get_parameter("workspace_min_m").value
        )
        self.workspace_max_m = list(
            self.get_parameter("workspace_max_m").value
        )

        if self.model_count < 1:
            raise ValueError("model_count must be positive")
        if not 0 <= self.model_index < self.model_count:
            raise ValueError("model_index outside configured model range")
        if self.joint_count < 1:
            raise ValueError("joint_count must be positive")
        if self.command_joint_slots < self.joint_count:
            raise ValueError("command_joint_slots must be >= joint_count")
        if not 0.0 < self.joint_speed_value <= 0.1:
            raise ValueError("joint_speed_value must be within (0, 0.1]")
        if not 0.0 < self.max_joint_step_rad <= 0.008726646259971648:
            raise ValueError("max_joint_step_rad must be within (0, 0.5 degree]")
        state_hz = float(self.get_parameter("state_publish_hz").value)
        if state_hz <= 0.0:
            raise ValueError("state_publish_hz must be positive")

        if self.dry_run:
            self.backend = DryRunBackend(self.model_count, self.joint_count)
            self.get_logger().warning(
                "DRY-RUN MODE: SDK is not loaded and no controller command "
                "will be sent."
            )
        else:
            sdk_root = str(self.get_parameter("sdk_root").value)
            if not sdk_root:
                raise ValueError(
                    "sdk_root or ZHENSHI_SDK_ROOT is required in real mode"
                )
            self.backend = OfficialSdkBackend(
                sdk_root=sdk_root,
                robot_ip=self.robot_ip,
                start_state_subscription=True,
                connect_rpc=True,
            )
            self.get_logger().warning(
                "REAL MODE: service calls can send commands to the controller."
            )

        self._command_lock = threading.Lock()
        self.state_publisher = self.create_publisher(
            RobotState, "/zhenshi/state", 10
        )
        self.create_timer(1.0 / state_hz, self._publish_state)

        self.create_service(
            ExecuteCommand,
            "/zhenshi/execute_command",
            self._handle_execute_command,
        )
        self.create_service(
            JogJoint,
            "/zhenshi/jog_joint",
            self._handle_jog_joint,
        )
        self.create_service(
            MoveJoint,
            "/zhenshi/move_joint",
            self._handle_move_joint,
        )
        self.create_service(
            MovePose,
            "/zhenshi/move_pose",
            self._handle_move_pose,
        )
        self.create_service(
            SetGripper,
            "/zhenshi/set_gripper",
            self._handle_set_gripper,
        )
        self.create_service(Trigger, "/zhenshi/enable", self._handle_enable)
        self.create_service(Trigger, "/zhenshi/disable", self._handle_disable)
        self.create_service(Trigger, "/zhenshi/clear", self._handle_clear)
        self.create_service(Trigger, "/zhenshi/recover", self._handle_recover)
        self.create_service(
            Trigger, "/zhenshi/initialize", self._handle_initialize
        )
        self.create_service(Trigger, "/zhenshi/stop", self._handle_stop)

    def _timeout(self, requested: int) -> int:
        return int(requested) if int(requested) > 0 else self.rpc_timeout_ms

    @staticmethod
    def _fill_response(response, result: CommandResult):
        response.success = bool(result.success)
        response.transport_status = int(result.transport_status)
        response.message = str(result.message)
        return response

    def _send(self, command: str, timeout_ms: int) -> CommandResult:
        self.get_logger().info(f"controller command: {command}")
        with self._command_lock:
            return self.backend.send_command(command, timeout_ms)

    def _send_sequence(
        self, commands: Iterable[str], timeout_ms: int
    ) -> CommandResult:
        messages = []
        last_status = 0
        for command in commands:
            result = self._send(command, timeout_ms)
            messages.append(result.message)
            last_status = result.transport_status
            if not result.success:
                return CommandResult(False, last_status, "; ".join(messages))
        return CommandResult(True, last_status, "; ".join(messages))

    def _publish_state(self):
        try:
            snapshot = self.backend.read_state(self.model_index)
            message = RobotState()
            message.header.stamp = self.get_clock().now().to_msg()
            for field in (
                "connected",
                "data_valid",
                "system_running",
                "source",
                "status_text",
                "model_index",
                "model_count",
                "model_name",
                "model_type",
                "model_state",
                "error_code",
                "error_message",
                "robottarget",
                "jointtarget",
            ):
                setattr(message, field, getattr(snapshot, field))
            self.state_publisher.publish(message)
        except Exception as exc:
            self.get_logger().error(f"state publish failed: {exc}")

    def _handle_execute_command(self, request, response):
        if not self.allow_raw_commands:
            return self._fill_response(
                response,
                CommandResult(
                    False,
                    -1,
                    "raw command service is disabled by configuration",
                ),
            )
        try:
            result = self._send(
                raw_command(request.command),
                self._timeout(request.timeout_ms),
            )
        except Exception as exc:
            result = CommandResult(False, -1, str(exc))
        return self._fill_response(response, result)

    def _handle_move_joint(self, request, response):
        try:
            snapshot = self.backend.read_state(request.model_index)
            if not snapshot.data_valid:
                raise RuntimeError(f"state is not valid: {snapshot.status_text}")
            if len(snapshot.jointtarget) != self.joint_count:
                raise RuntimeError(
                    "controller joint count changed: "
                    f"expected {self.joint_count}, "
                    f"received {len(snapshot.jointtarget)}"
                )
            requested = list(request.joints)
            if len(requested) != self.joint_count:
                raise ValueError(
                    f"expected {self.joint_count} joint values, "
                    f"received {len(requested)}"
                )
            largest_step = max(
                abs(float(target) - float(current))
                for target, current in zip(requested, snapshot.jointtarget)
            )
            if largest_step > self.max_joint_step_rad:
                raise ValueError(
                    f"largest joint step {largest_step:.12g} exceeds "
                    f"safety limit {self.max_joint_step_rad:.12g} rad"
                )
            command = build_move_joint(
                request.model_index,
                self.model_count,
                requested,
                self.joint_count,
                self.command_joint_slots,
                self.joint_speed_value,
            )
            result = self._send(command, self._timeout(request.timeout_ms))
        except Exception as exc:
            result = CommandResult(False, -1, str(exc))
        return self._fill_response(response, result)

    def _handle_jog_joint(self, request, response):
        try:
            snapshot = self.backend.read_state(request.model_index)
            if not snapshot.data_valid:
                raise RuntimeError(f"state is not valid: {snapshot.status_text}")
            if snapshot.error_code != 0:
                raise RuntimeError(
                    f"model error {snapshot.error_code}: "
                    f"{snapshot.error_message}"
                )
            if not self.dry_run and not snapshot.system_running:
                raise RuntimeError("controller system is not running")
            command = build_joint_jog(
                model_index=request.model_index,
                model_count=self.model_count,
                current_joints=snapshot.jointtarget,
                joint_index=request.joint_index,
                delta_rad=request.delta_rad,
                joint_count=self.joint_count,
                command_joint_slots=self.command_joint_slots,
                speed_value=self.joint_speed_value,
                max_step_rad=self.max_joint_step_rad,
            )
            result = self._send(command, self._timeout(request.timeout_ms))
        except Exception as exc:
            result = CommandResult(False, -1, str(exc))
        return self._fill_response(response, result)

    def _handle_move_pose(self, request, response):
        try:
            commands = build_move_pose_sequence(
                request.model_index,
                self.model_count,
                request.position_m,
                request.quaternion_xyzw,
                request.zone,
                self.workspace_min_m,
                self.workspace_max_m,
            )
            result = self._send_sequence(
                commands, self._timeout(request.timeout_ms)
            )
        except Exception as exc:
            result = CommandResult(False, -1, str(exc))
        return self._fill_response(response, result)

    def _handle_set_gripper(self, request, response):
        try:
            command = build_gripper(
                request.model_index, self.model_count, request.state
            )
            result = self._send(command, self._timeout(request.timeout_ms))
        except Exception as exc:
            result = CommandResult(False, -1, str(exc))
        return self._fill_response(response, result)

    def _trigger_command(self, response, body: str):
        try:
            # Clear/Enable/Recover/etc. are controller-level commands in the
            # official SDK and must not be placed in per-model motion slots.
            command = raw_command(body)
            result = self._send(command, self.rpc_timeout_ms)
            response.success = result.success
            response.message = result.message
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _handle_enable(self, request, response):
        del request
        return self._trigger_command(response, "Enable")

    def _handle_disable(self, request, response):
        del request
        return self._trigger_command(response, "Disable")

    def _handle_clear(self, request, response):
        del request
        return self._trigger_command(response, "Clear")

    def _handle_recover(self, request, response):
        del request
        return self._trigger_command(response, "Recover")

    def _handle_initialize(self, request, response):
        del request
        try:
            with self._command_lock:
                result = self.backend.send_commands_async(
                    list(SYSTEM_INIT_COMMANDS),
                    max(self.rpc_timeout_ms, 30000),
                )
            response.success = result.success
            response.message = result.message
            if result.success:
                self.get_logger().warning(
                    "controller initialization was submitted asynchronously; "
                    "verify real-time joint enabled state before motion"
                )
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _handle_stop(self, request, response):
        del request
        try:
            result = self.backend.send_stop(self.rpc_timeout_ms)
            response.success = result.success
            response.message = result.message
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ZhenshiDriverNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
