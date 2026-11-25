"""Gripper interface; normalized command 0 = open, 1 = closed."""

from dataclasses import dataclass
from typing import Protocol


@dataclass(frozen=True)
class GripperCommandResult:
    command: float | None
    state: str
    executed: bool
    reasons: tuple[str, ...] = ()
    position_raw: int | None = None
    object_status: int | None = None
    fault_code: int | None = None
    feedback_monotonic_ns: int | None = None


class GripperBackend(Protocol):
    def set_command(self, command: float | None) -> GripperCommandResult: ...
    def stop(self) -> None: ...
    def close(self) -> None: ...


class DisabledGripper:
    def set_command(self, command: float | None) -> GripperCommandResult:
        return GripperCommandResult(command, "disabled", False)

    def stop(self) -> None:
        pass

    def close(self) -> None:
        pass
