"""Strict Quest v2 input boundary. Never coerce strings to booleans."""

import json
import math
from dataclasses import dataclass

from .teleop import Pose7

MODES = {"idle", "translate", "default_grasp", "free_wrist"}


def _object(value: object, name: str) -> dict:
    if not isinstance(value, dict):
        raise ValueError(f"{name} must be an object")
    return value


def _bool(data: dict, name: str) -> bool:
    value = data.get(name)
    if type(value) is not bool:
        raise ValueError(f"{name} must be a boolean")
    return value


def _int(data: dict, name: str) -> int:
    value = data.get(name)
    if type(value) is not int or value < 0:
        raise ValueError(f"{name} must be a nonnegative integer")
    return value


def _unit(data: dict, name: str) -> float:
    value = data.get(name)
    if type(value) not in (int, float) or not math.isfinite(value) or not 0 <= value <= 1:
        raise ValueError(f"{name} must be a finite number in [0, 1]")
    return float(value)


def _vector(value: object, length: int) -> list:
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f"expected a {length}-element vector")
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in value):
        raise ValueError("vector components must be finite numbers")
    return value


@dataclass(frozen=True)
class QuestPacket:
    sequence: int
    timestamp_ns: int
    controller_pose: Pose7
    clutch: bool
    trigger: float
    mode: str
    valid: bool
    session_id: str
    recenter: bool = False
    stop_episode: bool = False

    @classmethod
    def decode(cls, payload: bytes) -> "QuestPacket":
        if len(payload) > 4096:
            raise ValueError("packet exceeds 4096 bytes")
        try:
            data = _object(json.loads(payload), "packet")
            if type(data.get("protocol_version")) is not int or data["protocol_version"] != 2:
                raise ValueError("only protocol_version 2 is supported")
            if (data.get("source"), data.get("stream"), data.get("source_timestamp_clock")) != (
                "quest3_unity",
                "right_controller",
                "quest_realtime_since_startup",
            ):
                raise ValueError("unexpected input source/stream/clock")
            controller = _object(data.get("controller"), "controller")
            if controller.get("frame") != "quest_tracking":
                raise ValueError("unexpected controller frame")
            flags = _object(data.get("valid"), "valid")
            valid = all([_bool(flags, "controller_pose"), _bool(flags, "buttons"), _bool(flags, "tracking")])
            valid = _bool(data, "heartbeat") and valid
            buttons = _object(data.get("buttons"), "buttons")
            clutch = _bool(buttons, "clutch")
            mode = data.get("mode")
            if not isinstance(mode, str) or mode not in MODES or (mode == "idle") == clutch:
                raise ValueError("mode disagrees with clutch")
            session = data.get("session_id", "legacy")
            if not isinstance(session, str) or not 1 <= len(session) <= 64:
                raise ValueError("invalid session_id")
            # Invalid tracking may carry a placeholder pose, but it never commands motion.
            pose = Pose7.from_values(_vector(controller["position"], 3), _vector(controller["quaternion_xyzw"], 4))
            return cls(
                _int(data, "sequence_id"),
                _int(data, "source_timestamp_ns"),
                pose,
                clutch,
                _unit(buttons, "trigger"),
                mode,
                valid,
                session,
                _bool(buttons, "recenter"),
                _bool(buttons, "stop_episode"),
            )
        except (KeyError, TypeError, UnicodeError, OverflowError, RecursionError) as exc:
            raise ValueError(f"invalid Quest packet: {exc}") from exc
