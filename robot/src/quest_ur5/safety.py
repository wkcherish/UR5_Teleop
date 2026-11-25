# Adapted from third-party teleoperation code; see third_party/teleoperation/NOTICE.
"""UR5 safety filtering for teleop and future policy commands."""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

import numpy as np

from .teleop import Pose7, RobotCommand, quat_angle, quat_slerp


@dataclass(frozen=True)
class UrSafetyState:
    """Motion-relevant UR safety state snapshot.

    Unknown robot/safety modes fail closed. RTDE fills both modes explicitly.
    """

    protective_stopped: bool = False
    emergency_stopped: bool = False
    robot_power_on: bool = True
    safety_mode: str | int | None = "normal"
    robot_mode: str | int | None = "running"

    @classmethod
    def normal(cls) -> UrSafetyState:
        return cls()

    @property
    def motion_allowed(self) -> bool:
        if self.protective_stopped or self.emergency_stopped or not self.robot_power_on:
            return False
        return _safety_mode_allows_motion(self.safety_mode) and self.robot_mode in {7, "running"}


@dataclass(frozen=True)
class Ur5SafetyConfig:
    workspace_min: tuple[float, float, float] | None = None
    workspace_max: tuple[float, float, float] | None = None
    max_linear_step_m: float | None = None
    max_angular_step_deg: float | None = None
    max_linear_speed_mps: float | None = None
    max_angular_speed_deg_s: float | None = None
    # Bounded command preview avoids reducing every target to one step ahead of
    # a lagging TCP, while keeping a measured TCP-to-command safety envelope.
    max_command_lead_m: float | None = None
    max_command_lead_deg: float | None = None
    max_linear_acceleration_mps2: float | None = None
    linear_braking: bool = False

    def __post_init__(self) -> None:
        for name in (
            "max_linear_step_m",
            "max_angular_step_deg",
            "max_linear_speed_mps",
            "max_angular_speed_deg_s",
            "max_command_lead_m",
            "max_command_lead_deg",
            "max_linear_acceleration_mps2",
        ):
            value = getattr(self, name)
            if value is not None and (not math.isfinite(value) or value < 0):
                raise ValueError(f"{name} must be finite and nonnegative")
        for bound in (self.workspace_min, self.workspace_max):
            if bound is not None and (len(bound) != 3 or not np.all(np.isfinite(bound))):
                raise ValueError("workspace bounds must contain three finite values")
        if (
            self.workspace_min is not None
            and self.workspace_max is not None
            and np.any(np.asarray(self.workspace_min) >= np.asarray(self.workspace_max))
        ):
            raise ValueError("workspace_min must be below workspace_max")
        if self.max_command_lead_m is not None and (
            self.max_linear_speed_mps is None or self.max_linear_step_m is None
        ):
            raise ValueError("linear command lead requires linear speed and step limits")
        if self.max_command_lead_deg is not None and (
            self.max_angular_speed_deg_s is None or self.max_angular_step_deg is None
        ):
            raise ValueError("angular command lead requires angular speed and step limits")
        if self.max_linear_acceleration_mps2 is not None and (
            self.max_linear_acceleration_mps2 <= 0 or self.max_command_lead_m is None
        ):
            raise ValueError("linear acceleration requires a positive limit and linear command lead")
        if not isinstance(self.linear_braking, bool):
            raise ValueError("linear braking must be a boolean")
        if self.linear_braking and self.max_linear_acceleration_mps2 is None:
            raise ValueError("linear braking requires a linear acceleration limit")


@dataclass(frozen=True)
class SafetyFilterResult:
    allowed: bool
    command: RobotCommand | None
    reasons: tuple[str, ...] = ()
    requested_command: RobotCommand | None = None


class Ur5SafetyFilter:
    """Clamp or reject UR5 commands using the measured TCP as reference."""

    def __init__(self, config: Ur5SafetyConfig | None = None) -> None:
        self.config = config or Ur5SafetyConfig()

    def apply(
        self,
        command: RobotCommand,
        *,
        actual_pose: Pose7,
        period_s: float,
        safety_state: UrSafetyState,
        previous_command_pose: Pose7 | None = None,
        previous_command_velocity: np.ndarray | None = None,
    ) -> SafetyFilterResult:
        if not safety_state.motion_allowed:
            return SafetyFilterResult(
                allowed=False,
                command=None,
                reasons=("ur_safety_state",),
                requested_command=command,
            )

        preview = self.config.max_command_lead_m is not None or self.config.max_command_lead_deg is not None
        if preview:
            if not math.isfinite(period_s) or period_s <= 0:
                return SafetyFilterResult(
                    allowed=False, command=None, reasons=("invalid_period",), requested_command=command
                )
            _, outside = self._clip_workspace(actual_pose.position)
            if outside:
                return SafetyFilterResult(
                    allowed=False, command=None, reasons=("actual_outside_workspace",), requested_command=command
                )
            if previous_command_pose is not None:
                _, outside = self._clip_workspace(previous_command_pose.position)
                if outside:
                    return SafetyFilterResult(
                        allowed=False, command=None, reasons=("reference_outside_workspace",), requested_command=command
                    )
                violations = self._lead_violations(actual_pose, previous_command_pose)
                if violations:
                    return SafetyFilterResult(
                        allowed=False, command=None, reasons=violations, requested_command=command
                    )

        target_position, workspace_limited = self._clip_workspace(command.target_pose.position)
        reasons: list[str] = []
        if workspace_limited:
            reasons.append("workspace")

        motion_reference = previous_command_pose or actual_pose
        target_position, linear_reasons = self._limit_linear_motion(
            actual_position=(motion_reference if self.config.max_command_lead_m is not None else actual_pose).position,
            requested_position=target_position,
            period_s=period_s,
        )
        reasons.extend(linear_reasons)

        if self.config.max_linear_acceleration_mps2 is not None:
            delta = target_position - motion_reference.position
            velocity = delta / period_s
            acceleration = self.config.max_linear_acceleration_mps2
            speed = float(np.linalg.norm(velocity))
            if self.config.linear_braking and speed > 0.0:
                goal, _ = self._clip_workspace((command.logical_pose or command.target_pose).position)
                remaining = max(0.0, float(np.dot(goal - motion_reference.position, velocity / speed)))
                # Reserve one command step plus v^2/(2a) before the logical
                # goal. Braking toward the per-tick smoothed target stalls tracking.
                step_speed = acceleration * period_s
                braking_speed = (2.0 * acceleration * remaining) / (
                    math.sqrt(step_speed**2 + 2.0 * acceleration * remaining) + step_speed
                )
                if speed > braking_speed:
                    velocity *= braking_speed / speed
                    reasons.append("linear_braking")
            previous_velocity = np.zeros(3) if previous_command_velocity is None else previous_command_velocity
            change = velocity - previous_velocity
            change_norm = float(np.linalg.norm(change))
            if change_norm > acceleration * period_s:
                velocity = previous_velocity + change * (acceleration * period_s / change_norm)
                reasons.append("linear_acceleration")
            target_position = motion_reference.position + velocity * period_s
            _, outside = self._clip_workspace(target_position)
            if (
                outside
                or np.linalg.norm(target_position - motion_reference.position) > self.config.max_linear_step_m + 1e-9
                or np.linalg.norm(velocity) > self.config.max_linear_speed_mps + 1e-9
            ):
                return SafetyFilterResult(
                    allowed=False, command=None, reasons=("acceleration_envelope",), requested_command=command
                )

        target_quaternion, angular_reasons = self._limit_angular_motion(
            actual_pose=motion_reference if self.config.max_command_lead_deg is not None else actual_pose,
            requested_pose=command.target_pose,
            period_s=period_s,
        )
        reasons.extend(angular_reasons)

        filtered_pose = Pose7.from_values(target_position, target_quaternion)
        violations = self._lead_violations(actual_pose, filtered_pose)
        if violations:
            # Do not project back onto the actual TCP after rate limiting:
            # that projection can itself violate the command speed limit.
            return SafetyFilterResult(
                allowed=False, command=None, reasons=tuple(reasons) + violations, requested_command=command
            )
        filtered_command = replace(
            command,
            target_pose=filtered_pose,
            safety_limited=command.safety_limited or bool(reasons),
            target_delta_m=float(np.linalg.norm(filtered_pose.position - actual_pose.position)),
            target_delta_deg=quat_angle(actual_pose.quaternion, filtered_pose.quaternion),
        )
        return SafetyFilterResult(
            allowed=True,
            command=filtered_command,
            reasons=tuple(dict.fromkeys(reasons)),
            requested_command=command,
        )

    def _lead_violations(self, actual_pose: Pose7, target: Pose7) -> tuple[str, ...]:
        reasons = []
        if self.config.max_command_lead_m is not None and (
            np.linalg.norm(target.position - actual_pose.position) > self.config.max_command_lead_m + 1e-9
        ):
            reasons.append("command_lead")
        if self.config.max_command_lead_deg is not None and (
            quat_angle(actual_pose.quaternion, target.quaternion) > self.config.max_command_lead_deg + 1e-7
        ):
            reasons.append("command_lead_angle")
        return tuple(reasons)

    def _clip_workspace(self, position: np.ndarray) -> tuple[np.ndarray, bool]:
        clipped = np.asarray(position, dtype=np.float64).copy()
        if self.config.workspace_min is None and self.config.workspace_max is None:
            return clipped, False

        lower = np.asarray(
            self.config.workspace_min if self.config.workspace_min is not None else [-np.inf, -np.inf, -np.inf],
            dtype=np.float64,
        )
        upper = np.asarray(
            self.config.workspace_max if self.config.workspace_max is not None else [np.inf, np.inf, np.inf],
            dtype=np.float64,
        )
        clipped = np.minimum(np.maximum(clipped, lower), upper)
        return clipped, not np.allclose(clipped, position, atol=1e-12, rtol=0.0)

    def _limit_linear_motion(
        self,
        *,
        actual_position: np.ndarray,
        requested_position: np.ndarray,
        period_s: float,
    ) -> tuple[np.ndarray, list[str]]:
        reasons: list[str] = []
        limits: list[tuple[float, str]] = []
        if self.config.max_linear_step_m is not None:
            limits.append((max(0.0, float(self.config.max_linear_step_m)), "linear_step"))
        if self.config.max_linear_speed_mps is not None:
            limits.append((max(0.0, float(self.config.max_linear_speed_mps)) * _safe_period(period_s), "linear_speed"))
        if not limits:
            return requested_position, reasons

        delta = requested_position - actual_position
        distance = float(np.linalg.norm(delta))
        for limit, reason in limits:
            if distance > limit:
                reasons.append(reason)
        effective_limit = min(limit for limit, _reason in limits)
        if distance <= effective_limit:
            return requested_position, reasons
        if effective_limit <= 0.0 or distance <= 1e-12:
            return actual_position.copy(), reasons
        return actual_position + delta / distance * effective_limit, reasons

    def _limit_angular_motion(
        self,
        *,
        actual_pose: Pose7,
        requested_pose: Pose7,
        period_s: float,
    ) -> tuple[np.ndarray, list[str]]:
        reasons: list[str] = []
        limits: list[tuple[float, str]] = []
        if self.config.max_angular_step_deg is not None:
            limits.append((max(0.0, float(self.config.max_angular_step_deg)), "angular_step"))
        if self.config.max_angular_speed_deg_s is not None:
            limits.append(
                (max(0.0, float(self.config.max_angular_speed_deg_s)) * _safe_period(period_s), "angular_speed")
            )
        if not limits:
            return requested_pose.quaternion, reasons

        angle = quat_angle(actual_pose.quaternion, requested_pose.quaternion)
        for limit, reason in limits:
            if angle > limit:
                reasons.append(reason)
        effective_limit = min(limit for limit, _reason in limits)
        if angle <= effective_limit:
            return requested_pose.quaternion, reasons
        if effective_limit <= 0.0 or angle <= 1e-12:
            return actual_pose.quaternion.copy(), reasons
        return quat_slerp(actual_pose.quaternion, requested_pose.quaternion, effective_limit / angle), reasons


def _safe_period(period_s: float) -> float:
    if not math.isfinite(period_s) or period_s <= 0.0:
        return 0.0
    return float(period_s)


def _safety_mode_allows_motion(mode: str | int | None) -> bool:
    if type(mode) is int:
        return mode in {1, 2}
    return isinstance(mode, str) and mode.strip().lower() in {"normal", "reduced"}
