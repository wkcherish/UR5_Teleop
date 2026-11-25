"""Validated JSON configuration shared by all desktop operating systems."""

import ipaddress
import json
import math
from dataclasses import asdict, dataclass, fields
from pathlib import Path

import numpy as np

from .safety import Ur5SafetyConfig
from .teleop import PoseFrameTransform


@dataclass(frozen=True)
class Config:
    listen_host: str = "0.0.0.0"
    listen_port: int = 8080
    feedback_port: int = 8081
    quest_ip: str = ""
    robot_ip: str = ""
    rate_hz: float = 100.0
    input_timeout_s: float = 0.15
    translation_scale: float = 1.0
    input_yaw_degrees: float = 0.0
    smooth_step: float = 0.25
    input_translation_deadband_m: float = 0.001
    input_rotation_deadband_deg: float = 0.5
    max_input_step_m: float = 0.10
    max_input_step_deg: float = 60.0
    workspace_min: tuple = (-0.7, -0.7, 0.10)
    workspace_max: tuple = (0.7, 0.7, 0.90)
    max_linear_speed_mps: float = 0.035
    max_angular_speed_deg_s: float = 14.0
    max_linear_step_m: float = 0.005
    max_angular_step_deg: float = 2.0
    max_command_lead_m: float = 0.025
    max_command_lead_deg: float = 8.0
    gripper: str = "disabled"
    gripper_port: int = 63352
    gripper_slave_id: int | None = None
    gripper_speed: int = 96
    gripper_force: int = 32

    def __post_init__(self) -> None:
        for name in ("listen_host", "quest_ip", "robot_ip"):
            if not isinstance(getattr(self, name), str):
                raise ValueError(f"{name} must be an IPv4 address string")
        ipaddress.IPv4Address(self.listen_host)
        for name in ("quest_ip", "robot_ip"):
            if getattr(self, name):
                ipaddress.IPv4Address(getattr(self, name))
        for name in ("listen_port", "feedback_port", "gripper_port"):
            if type(getattr(self, name)) is not int or not 1 <= getattr(self, name) <= 65535:
                raise ValueError(f"invalid {name}")
        if self.listen_port == self.feedback_port:
            raise ValueError("command and feedback ports must differ")
        for field in fields(self):
            default = field.default
            value = getattr(self, field.name)
            if isinstance(default, float) and (type(value) not in (int, float) or not math.isfinite(value)):
                raise ValueError(f"{field.name} must be finite")
        if not 10 <= self.rate_hz <= 125:
            raise ValueError("rate_hz must be in [10, 125]")
        if not 2 / self.rate_hz <= self.input_timeout_s <= 0.5:
            raise ValueError("input_timeout_s must be between two control periods and 0.5 s")
        if not 0 < self.smooth_step <= 1 or not 0 < self.translation_scale <= 4:
            raise ValueError("invalid smoothing/translation scale")
        for name in (
            "max_input_step_m",
            "max_input_step_deg",
            "max_linear_speed_mps",
            "max_angular_speed_deg_s",
            "max_linear_step_m",
            "max_angular_step_deg",
            "max_command_lead_m",
            "max_command_lead_deg",
        ):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive")
        if min(self.input_translation_deadband_m, self.input_rotation_deadband_deg) < 0:
            raise ValueError("deadbands must be nonnegative")
        if self.gripper not in {"disabled", "robotiq"}:
            raise ValueError("gripper must be disabled or robotiq")
        for name in ("gripper_speed", "gripper_force"):
            if type(getattr(self, name)) is not int or not 0 <= getattr(self, name) <= 255:
                raise ValueError(f"invalid {name}")
        if self.gripper_slave_id is not None and (
            type(self.gripper_slave_id) is not int or not 1 <= self.gripper_slave_id <= 247
        ):
            raise ValueError("invalid gripper_slave_id")
        if any(np.asarray(v).shape != (3,) for v in (self.workspace_min, self.workspace_max)):
            raise ValueError("workspace bounds need three values")
        self.safety_config()
        PoseFrameTransform.ur10_unity_to_robot().with_robot_yaw(self.input_yaw_degrees)

    def safety_config(self) -> Ur5SafetyConfig:
        names = {f.name for f in fields(Ur5SafetyConfig)}
        return Ur5SafetyConfig(**{k: v for k, v in asdict(self).items() if k in names})


def load_config(path: Path | None) -> Config:
    if path is None:
        return Config()
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("configuration must be a JSON object")
    known = {f.name for f in fields(Config)}
    unknown = set(data) - known
    if unknown:
        raise ValueError(f"unknown configuration keys: {sorted(unknown)}")
    return Config(**data)
