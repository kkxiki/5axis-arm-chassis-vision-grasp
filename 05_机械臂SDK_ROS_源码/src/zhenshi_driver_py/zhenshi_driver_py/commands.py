"""Validated construction of controller command strings."""

import math
from typing import Iterable, List, Sequence


NO_OP = "NotRunExecute"


def _finite_values(values: Iterable[float], name: str) -> List[float]:
    result = [float(value) for value in values]
    if not result:
        raise ValueError(f"{name} must not be empty")
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must contain finite numbers")
    return result


def format_values(values: Iterable[float]) -> str:
    checked = _finite_values(values, "values")
    return "{" + ",".join(f"{value:.12g}" for value in checked) + "}"


def command_body(command: str) -> str:
    body = str(command).strip()
    if body.startswith("{") and body.endswith("}"):
        body = body[1:-1].strip()
    if not body:
        raise ValueError("command must not be empty")
    return body


def raw_command(command: str) -> str:
    return "{" + command_body(command) + "}"


def scope_command(command: str, model_index: int, model_count: int) -> str:
    """Place one command in the selected model slot."""
    body = command_body(command)
    if "||" in body:
        raise ValueError("already multiplexed commands cannot be scoped")
    if model_count < 1:
        raise ValueError("model_count must be positive")
    if not 0 <= int(model_index) < int(model_count):
        raise ValueError(
            f"model_index {model_index} outside [0,{int(model_count) - 1}]"
        )
    slots = [NO_OP] * int(model_count)
    slots[int(model_index)] = body
    return "{" + "||".join(slots) + "}"


def build_move_joint(
    model_index: int,
    model_count: int,
    joints: Sequence[float],
    joint_count: int,
    command_joint_slots: int = 10,
    speed_value: float = 0.01,
) -> str:
    checked = _finite_values(joints, "joints")
    if len(checked) != int(joint_count):
        raise ValueError(
            f"expected {joint_count} joint values, received {len(checked)}"
        )
    if int(command_joint_slots) < int(joint_count):
        raise ValueError("command_joint_slots must be >= joint_count")
    checked_speed = float(speed_value)
    if not math.isfinite(checked_speed) or not 0.0 < checked_speed <= 0.1:
        raise ValueError("speed_value must be within (0, 0.1]")
    padded = checked + [0.0] * (int(command_joint_slots) - len(checked))
    body = (
        f"MoveAbsJ --jointtarget_value={format_values(padded)} "
        f"--speed_value={checked_speed:.12g} --zone=fine"
    )
    return scope_command(body, model_index, model_count)


def build_joint_jog(
    model_index: int,
    model_count: int,
    current_joints: Sequence[float],
    joint_index: int,
    delta_rad: float,
    joint_count: int,
    command_joint_slots: int = 10,
    speed_value: float = 0.01,
    max_step_rad: float = math.radians(0.5),
) -> str:
    current = _finite_values(current_joints, "current_joints")
    if len(current) != int(joint_count):
        raise ValueError(
            f"expected {joint_count} current joint values, received {len(current)}"
        )
    index = int(joint_index)
    if not 0 <= index < int(joint_count):
        raise ValueError(
            f"joint_index {index} outside [0,{int(joint_count) - 1}]"
        )
    checked_delta = float(delta_rad)
    checked_limit = float(max_step_rad)
    if not math.isfinite(checked_delta):
        raise ValueError("delta_rad must be finite")
    if not math.isfinite(checked_limit) or checked_limit <= 0.0:
        raise ValueError("max_step_rad must be positive and finite")
    if checked_delta == 0.0 or abs(checked_delta) > checked_limit:
        raise ValueError(
            f"delta_rad must be non-zero and within +/-{checked_limit:.12g}"
        )
    target = list(current)
    target[index] += checked_delta
    return build_move_joint(
        model_index=model_index,
        model_count=model_count,
        joints=target,
        joint_count=joint_count,
        command_joint_slots=command_joint_slots,
        speed_value=speed_value,
    )


def normalize_quaternion(values: Sequence[float]) -> List[float]:
    checked = _finite_values(values, "quaternion")
    if len(checked) != 4:
        raise ValueError("quaternion must contain 4 values in x,y,z,w order")
    norm = math.sqrt(sum(value * value for value in checked))
    if norm < 1e-9:
        raise ValueError("quaternion norm is zero")
    if abs(norm - 1.0) > 0.05:
        raise ValueError(f"quaternion norm {norm:.6f} is too far from 1")
    return [value / norm for value in checked]


def validate_position(
    position_m: Sequence[float],
    workspace_min_m: Sequence[float],
    workspace_max_m: Sequence[float],
) -> List[float]:
    position = _finite_values(position_m, "position_m")
    minimum = _finite_values(workspace_min_m, "workspace_min_m")
    maximum = _finite_values(workspace_max_m, "workspace_max_m")
    if not (len(position) == len(minimum) == len(maximum) == 3):
        raise ValueError("position and workspace bounds must each contain 3 values")
    for axis, value, low, high in zip("xyz", position, minimum, maximum):
        if low > high:
            raise ValueError(f"workspace {axis} minimum exceeds maximum")
        if not low <= value <= high:
            raise ValueError(
                f"position {axis}={value:.6f}m outside [{low:.6f},{high:.6f}]"
            )
    return position


def build_move_pose_sequence(
    model_index: int,
    model_count: int,
    position_m: Sequence[float],
    quaternion_xyzw: Sequence[float],
    zone: Sequence[float],
    workspace_min_m: Sequence[float],
    workspace_max_m: Sequence[float],
    target_name: str = "ros_grasp_target",
) -> List[str]:
    position = validate_position(position_m, workspace_min_m, workspace_max_m)
    quaternion = normalize_quaternion(quaternion_xyzw)
    checked_zone = _finite_values(zone, "zone")
    if len(checked_zone) != 2 or any(value < 0.0 for value in checked_zone):
        raise ValueError("zone must contain 2 non-negative values")
    if not target_name.replace("_", "").isalnum():
        raise ValueError("target_name must contain only letters, digits or underscores")

    pose = format_values(position + quaternion)
    zone_text = format_values(checked_zone)
    bodies = [
        f"DeleteVar --name={target_name}",
        f"Var --type=robottarget --name={target_name} --value={pose}",
        "MoveBlend --type=first_insert",
        (
            "MoveBlend --type=insert_line "
            f"--robottarget_var={target_name} --zone={zone_text}"
        ),
        "MoveBlend --type=start",
        f"DeleteVar --name={target_name}",
    ]
    return [
        scope_command(body, model_index, model_count)
        for body in bodies
    ]


def build_gripper(
    model_index: int,
    model_count: int,
    state: int,
) -> str:
    if int(state) not in (0, 1, 2):
        raise ValueError("gripper state must be 0, 1 or 2")
    return scope_command(
        f"SetHandState --state={int(state)}",
        model_index,
        model_count,
    )
