"""Pure Python data models shared by real and dry-run backends."""

from dataclasses import dataclass, field
from typing import List


@dataclass(frozen=True)
class CommandResult:
    success: bool
    transport_status: int
    message: str


@dataclass
class RobotStateSnapshot:
    connected: bool = False
    data_valid: bool = False
    system_running: bool = False
    source: str = "unknown"
    status_text: str = ""
    model_index: int = 0
    model_count: int = 0
    model_name: str = ""
    model_type: str = ""
    model_state: int = 0
    error_code: int = 0
    error_message: str = ""
    robottarget: List[float] = field(default_factory=list)
    jointtarget: List[float] = field(default_factory=list)

