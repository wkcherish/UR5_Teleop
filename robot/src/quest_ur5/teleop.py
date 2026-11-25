# Portions adapted from third-party teleoperation code; see third_party/teleoperation/NOTICE.
"""Quest 3 to UR5 teleoperation primitives.

This module mirrors the proven Open-Teach/UR10_Teleop control shape:
Quest sends raw controller pose and buttons; the robot-side process captures
controller/tool anchors on clutch press; every following target is derived from
that fixed pair and filtered exactly once.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
import numpy.typing as npt

ArrayLike = npt.ArrayLike

QUEST_TELEOP_PROTOCOL_VERSION = 2
QUEST_TELEOP_MODES = frozenset(("idle", "translate", "default_grasp", "free_wrist"))


def _as_vector(values: ArrayLike, length: int, name: str) -> npt.NDArray[np.float64]:
    array = np.asarray(values, dtype=np.float64)
    if array.shape != (length,):
        raise ValueError(f"{name} must have shape ({length},), got {array.shape}")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} contains non-finite values")
    return array


def quat_normalize(quaternion: ArrayLike) -> npt.NDArray[np.float64]:
    quat = _as_vector(quaternion, 4, "quaternion")
    norm = np.linalg.norm(quat)
    if norm <= 1e-12:
        raise ValueError("quaternion magnitude is too small")
    return quat / norm


def quat_conjugate(quaternion: ArrayLike) -> npt.NDArray[np.float64]:
    quat = quat_normalize(quaternion)
    return np.array([-quat[0], -quat[1], -quat[2], quat[3]], dtype=np.float64)


def quat_multiply(lhs: ArrayLike, rhs: ArrayLike) -> npt.NDArray[np.float64]:
    """Multiply xyzw quaternions."""

    x1, y1, z1, w1 = quat_normalize(lhs)
    x2, y2, z2, w2 = quat_normalize(rhs)
    return quat_normalize(
        np.array(
            [
                w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
                w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            ],
            dtype=np.float64,
        )
    )


def quat_slerp(start: ArrayLike, end: ArrayLike, step: float) -> npt.NDArray[np.float64]:
    start_quat = quat_normalize(start)
    end_quat = quat_normalize(end)
    dot = float(np.dot(start_quat, end_quat))
    if dot < 0.0:
        end_quat = -end_quat
        dot = -dot

    amount = float(np.clip(step, 0.0, 1.0))
    if dot > 0.9995:
        return quat_normalize(start_quat + amount * (end_quat - start_quat))

    theta_0 = math.acos(float(np.clip(dot, -1.0, 1.0)))
    theta = theta_0 * amount
    sin_theta = math.sin(theta)
    sin_theta_0 = math.sin(theta_0)
    scale_start = math.cos(theta) - dot * sin_theta / sin_theta_0
    scale_end = sin_theta / sin_theta_0
    return quat_normalize(scale_start * start_quat + scale_end * end_quat)


def quat_angle(lhs: ArrayLike, rhs: ArrayLike) -> float:
    lhs_quat = quat_normalize(lhs)
    rhs_quat = quat_normalize(rhs)
    dot = abs(float(np.dot(lhs_quat, rhs_quat)))
    return math.degrees(2.0 * math.acos(float(np.clip(dot, -1.0, 1.0))))


def quat_to_rotvec(quaternion: ArrayLike) -> npt.NDArray[np.float64]:
    quat = quat_normalize(quaternion)
    if quat[3] < 0.0:
        quat = -quat
    angle = 2.0 * math.acos(float(np.clip(quat[3], -1.0, 1.0)))
    sin_half = math.sqrt(max(0.0, 1.0 - quat[3] * quat[3]))
    if sin_half < 1e-9:
        return np.zeros(3, dtype=np.float64)
    return quat[:3] / sin_half * angle


def rotvec_to_quat(rotvec: ArrayLike) -> npt.NDArray[np.float64]:
    vector = _as_vector(rotvec, 3, "rotvec")
    angle = float(np.linalg.norm(vector))
    if angle < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    axis = vector / angle
    half = angle * 0.5
    return quat_normalize(np.concatenate([axis * math.sin(half), [math.cos(half)]]))


def apply_vector_deadband(vector: ArrayLike, deadband: float) -> npt.NDArray[np.float64]:
    value = _as_vector(vector, 3, "vector")
    threshold = max(0.0, float(deadband))
    distance = float(np.linalg.norm(value))
    if threshold <= 0.0 or distance <= 0.0:
        return value
    if distance <= threshold:
        return np.zeros(3, dtype=np.float64)
    return value / distance * (distance - threshold)


def apply_scalar_deadband(value: float, deadband: float) -> float:
    """Deadband one controller axis without coupling it to other axes."""
    amount = float(value)
    threshold = max(0.0, float(deadband))
    if abs(amount) <= threshold:
        return 0.0
    return math.copysign(abs(amount) - threshold, amount)


def apply_quaternion_deadband(quaternion: ArrayLike, deadband_degrees: float) -> npt.NDArray[np.float64]:
    quat = quat_normalize(quaternion)
    threshold = max(0.0, float(deadband_degrees))
    if threshold <= 0.0:
        return quat
    if quat[3] < 0.0:
        quat = -quat

    angle = 2.0 * math.acos(float(np.clip(quat[3], -1.0, 1.0)))
    angle_degrees = math.degrees(angle)
    if angle_degrees <= threshold:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)

    sin_half = math.sqrt(max(0.0, 1.0 - quat[3] * quat[3]))
    if sin_half < 1e-9:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)

    axis = quat[:3] / sin_half
    return rotvec_to_quat(axis * math.radians(angle_degrees - threshold))


def quat_to_matrix(quaternion: ArrayLike) -> npt.NDArray[np.float64]:
    x, y, z, w = quat_normalize(quaternion)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def matrix_to_quat(matrix: ArrayLike) -> npt.NDArray[np.float64]:
    mat = np.asarray(matrix, dtype=np.float64)
    if mat.shape != (3, 3) or not np.all(np.isfinite(mat)):
        raise ValueError("rotation matrix must have shape (3, 3) and finite values")

    trace = float(np.trace(mat))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (mat[2, 1] - mat[1, 2]) / s
        y = (mat[0, 2] - mat[2, 0]) / s
        z = (mat[1, 0] - mat[0, 1]) / s
    elif mat[0, 0] > mat[1, 1] and mat[0, 0] > mat[2, 2]:
        s = math.sqrt(1.0 + mat[0, 0] - mat[1, 1] - mat[2, 2]) * 2.0
        w = (mat[2, 1] - mat[1, 2]) / s
        x = 0.25 * s
        y = (mat[0, 1] + mat[1, 0]) / s
        z = (mat[0, 2] + mat[2, 0]) / s
    elif mat[1, 1] > mat[2, 2]:
        s = math.sqrt(1.0 + mat[1, 1] - mat[0, 0] - mat[2, 2]) * 2.0
        w = (mat[0, 2] - mat[2, 0]) / s
        x = (mat[0, 1] + mat[1, 0]) / s
        y = 0.25 * s
        z = (mat[1, 2] + mat[2, 1]) / s
    else:
        s = math.sqrt(1.0 + mat[2, 2] - mat[0, 0] - mat[1, 1]) * 2.0
        w = (mat[1, 0] - mat[0, 1]) / s
        x = (mat[0, 2] + mat[2, 0]) / s
        y = (mat[1, 2] + mat[2, 1]) / s
        z = 0.25 * s
    return quat_normalize([x, y, z, w])


@dataclass(frozen=True)
class Pose7:
    """Position plus xyzw quaternion."""

    position: npt.NDArray[np.float64]
    quaternion: npt.NDArray[np.float64]

    @classmethod
    def from_values(cls, position: ArrayLike, quaternion: ArrayLike) -> Pose7:
        return cls(
            position=_as_vector(position, 3, "position"),
            quaternion=quat_normalize(quaternion),
        )

    @classmethod
    def from_rtde_pose(cls, tcp_pose: ArrayLike) -> Pose7:
        pose = _as_vector(tcp_pose, 6, "tcp_pose")
        return cls.from_values(pose[:3], rotvec_to_quat(pose[3:]))

    @staticmethod
    def quaternion_from_axis_angle(axis: ArrayLike, degrees: float) -> npt.NDArray[np.float64]:
        axis_vector = _as_vector(axis, 3, "axis")
        norm = np.linalg.norm(axis_vector)
        if norm <= 1e-12:
            raise ValueError("axis magnitude is too small")
        half_angle = math.radians(float(degrees)) * 0.5
        return quat_normalize(np.concatenate([axis_vector / norm * math.sin(half_angle), [math.cos(half_angle)]]))

    @staticmethod
    def quat_conjugate(quaternion: ArrayLike) -> npt.NDArray[np.float64]:
        return quat_conjugate(quaternion)

    def as_rtde_pose(self) -> npt.NDArray[np.float64]:
        return np.concatenate([self.position, quat_to_rotvec(self.quaternion)])


@dataclass(frozen=True)
class PoseFrameTransform:
    """Constant coordinate-frame transform for Quest tracking pose to robot base.

    The matrix may include the handedness conversion used by Unity/OpenXR
    adapters. Orientation is therefore transformed by matrix conjugation
    (C * R * C^T), not by treating C itself as a quaternion.
    """

    matrix: npt.NDArray[np.float64]

    @classmethod
    def identity(cls) -> PoseFrameTransform:
        return cls.from_matrix(np.eye(3, dtype=np.float64))

    @classmethod
    def from_matrix(cls, matrix: ArrayLike) -> PoseFrameTransform:
        mat = np.asarray(matrix, dtype=np.float64)
        if mat.shape != (3, 3) or not np.all(np.isfinite(mat)):
            raise ValueError("frame transform matrix must have shape (3, 3) and finite values")
        if not np.allclose(mat @ mat.T, np.eye(3), atol=1e-6):
            raise ValueError("frame transform matrix must be orthonormal")
        determinant = abs(float(np.linalg.det(mat)))
        if not np.isclose(determinant, 1.0, atol=1e-6):
            raise ValueError("frame transform matrix determinant magnitude must be 1")
        return cls(matrix=mat)

    @classmethod
    def ur10_unity_to_robot(cls) -> PoseFrameTransform:
        return cls.from_matrix([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])

    def with_robot_yaw(self, degrees: float) -> PoseFrameTransform:
        """Left-multiply by a measured heading correction about robot_base +Z."""
        if not math.isfinite(degrees):
            raise ValueError("input yaw must be finite")
        rotation = quat_to_matrix(Pose7.quaternion_from_axis_angle([0, 0, 1], degrees))
        return self.from_matrix(rotation @ self.matrix)

    def apply(self, pose: Pose7) -> Pose7:
        rotation = self.matrix @ quat_to_matrix(pose.quaternion) @ self.matrix.T
        return Pose7.from_values(self.matrix @ pose.position, matrix_to_quat(rotation))


@dataclass(frozen=True)
class SafetyLimits:
    """Per-cycle limits applied after the single Open-Teach smoothing step."""

    max_linear_step_m: float | None = None
    max_angular_step_deg: float | None = None
    workspace_min: tuple[float, float, float] | None = None
    workspace_max: tuple[float, float, float] | None = None

    def apply(self, previous: Pose7, requested: Pose7) -> Pose7:
        position = requested.position.copy()
        if self.workspace_min is not None or self.workspace_max is not None:
            lower = np.asarray(self.workspace_min if self.workspace_min is not None else [-np.inf, -np.inf, -np.inf])
            upper = np.asarray(self.workspace_max if self.workspace_max is not None else [np.inf, np.inf, np.inf])
            position = np.minimum(np.maximum(position, lower), upper)

        if self.max_linear_step_m is not None and self.max_linear_step_m >= 0.0:
            delta = position - previous.position
            distance = float(np.linalg.norm(delta))
            if distance > self.max_linear_step_m > 0.0:
                position = previous.position + delta / distance * self.max_linear_step_m
            elif self.max_linear_step_m == 0.0:
                position = previous.position.copy()

        rotation = requested.quaternion
        if self.max_angular_step_deg is not None and self.max_angular_step_deg >= 0.0:
            angle = quat_angle(previous.quaternion, rotation)
            if angle > self.max_angular_step_deg > 0.0:
                rotation = quat_slerp(previous.quaternion, rotation, self.max_angular_step_deg / angle)
            elif self.max_angular_step_deg == 0.0:
                rotation = previous.quaternion.copy()

        return Pose7.from_values(position, rotation)


@dataclass(frozen=True)
class RobotCommand:
    target_pose: Pose7
    gripper: float | None = None
    sequence: int | None = None
    logical_pose: Pose7 | None = None
    position_locked: bool = False
    orientation_locked: bool = False
    target_delta_m: float = 0.0
    target_delta_deg: float = 0.0
    safety_limited: bool = False
    translation_backlog_limited: bool = False
    translation_reversal_reset: bool = False


class OpenTeachTeleopTracker:
    """Fixed-anchor Quest pose tracker for robot-side execution."""

    def __init__(
        self,
        *,
        smooth_step: float = 0.10,
        pos_mapping: tuple[float, float, float] = (1.0, 1.0, 1.0),
        input_translation_deadband_m: float = 0.0,
        input_rotation_deadband_deg: float = 0.0,
        safety_limits: SafetyLimits | None = None,
        translation_time_constant_s: float | None = None,
        max_pending_translation_m: float | None = None,
        stationary_pending_snap_m: float | None = None,
        translation_reversal_reset: bool = False,
        translation_reversal_threshold_m: float | None = None,
    ) -> None:
        if (translation_time_constant_s is None) != (max_pending_translation_m is None):
            raise ValueError("translation time constant and pending limit must be configured together")
        for value in (translation_time_constant_s, max_pending_translation_m):
            if value is not None and (not math.isfinite(value) or value <= 0):
                raise ValueError("translation response parameters must be finite and positive")
        self.translation_time_constant_s = translation_time_constant_s
        self.max_pending_translation_m = max_pending_translation_m
        if stationary_pending_snap_m is not None and (
            not math.isfinite(stationary_pending_snap_m) or stationary_pending_snap_m <= 0
        ):
            raise ValueError("stationary pending snap must be finite and positive")
        self.stationary_pending_snap_m = stationary_pending_snap_m
        if translation_reversal_reset and max_pending_translation_m is None:
            raise ValueError("translation reversal reset requires bounded translation")
        self.translation_reversal_reset = translation_reversal_reset
        if translation_reversal_threshold_m is not None and (
            not math.isfinite(translation_reversal_threshold_m)
            or translation_reversal_threshold_m <= 0
            or not translation_reversal_reset
        ):
            raise ValueError("translation reversal threshold requires a positive distance and reversal reset")
        self.translation_reversal_threshold_m = translation_reversal_threshold_m
        self._held_input_position: npt.NDArray[np.float64] | None = None
        self._pending_position: npt.NDArray[np.float64] | None = None
        self._last_input_position: npt.NDArray[np.float64] | None = None
        self._reverse_movement = np.zeros(3)
        self.smooth_step = float(np.clip(smooth_step, 0.0, 1.0))
        self.pos_mapping = _as_vector(pos_mapping, 3, "pos_mapping")
        self.input_translation_deadband_m = max(0.0, float(input_translation_deadband_m))
        self.input_rotation_deadband_deg = max(0.0, float(input_rotation_deadband_deg))
        self.safety_limits = safety_limits or SafetyLimits()
        self.input_anchor: Pose7 | None = None
        self.tool_anchor: Pose7 | None = None
        self.target: Pose7 | None = None
        self.locked_tool_orientation: npt.NDArray[np.float64] | None = None
        self._mode: tuple[bool, bool] = (False, False)
        self._has_tracked_command = False
        self.paused = True

    def resume(self, input_anchor: Pose7, tool_anchor: Pose7) -> Pose7:
        self.input_anchor = input_anchor
        self.tool_anchor = tool_anchor
        self.target = tool_anchor
        self._held_input_position = input_anchor.position * self.pos_mapping
        self._last_input_position = self._held_input_position.copy()
        self._reverse_movement = np.zeros(3)
        self._pending_position = tool_anchor.position.copy()
        self.locked_tool_orientation = tool_anchor.quaternion.copy()
        self._mode = (False, False)
        self._has_tracked_command = False
        self.paused = False
        return tool_anchor

    def pause(self) -> None:
        self.paused = True

    def accept_target(self, pose: Pose7) -> None:
        """Keep the smoother at the accepted target after downstream clipping."""
        if not self.paused:
            self.target = pose
            self._bound_pending_translation()

    def _bound_pending_translation(self) -> bool:
        if self.max_pending_translation_m is None or self._pending_position is None or self.target is None:
            return False
        delta = self._pending_position - self.target.position
        distance = float(np.linalg.norm(delta))
        if distance <= self.max_pending_translation_m:
            return False
        # Discard excess input permanently; stationary hands must not replay it.
        self._pending_position = self.target.position + delta * (self.max_pending_translation_m / distance)
        return True

    def track(
        self,
        user_input: Pose7,
        *,
        gripper: float | None = None,
        sequence: int | None = None,
        position_locked: bool = False,
        orientation_locked: bool = False,
        period_s: float | None = None,
    ) -> RobotCommand | None:
        if (
            self.paused
            or self.input_anchor is None
            or self.tool_anchor is None
            or self.target is None
            or self.locked_tool_orientation is None
        ):
            return None

        mode = (bool(position_locked), bool(orientation_locked))
        if self._has_tracked_command and mode != self._mode:
            was_position_locked = self._mode[0]
            if was_position_locked and not position_locked:
                self.locked_tool_orientation = self.target.quaternion.copy()
            self.input_anchor = user_input
            self.tool_anchor = self.target
            self._held_input_position = user_input.position * self.pos_mapping
            self._last_input_position = self._held_input_position.copy()
            self._reverse_movement = np.zeros(3)
            self._pending_position = self.target.position.copy()

        relative_translation = (
            apply_vector_deadband(
                user_input.position - self.input_anchor.position,
                self.input_translation_deadband_m,
            )
            * self.pos_mapping
        )
        relative_quaternion = apply_quaternion_deadband(
            quat_multiply(user_input.quaternion, quat_conjugate(self.input_anchor.quaternion)),
            self.input_rotation_deadband_deg,
        )
        if position_locked:
            requested_position = self.tool_anchor.position
        else:
            requested_position = self.tool_anchor.position + relative_translation

        translation_step = self.smooth_step
        backlog_limited = False
        reversal_reset = False
        if self.translation_time_constant_s is not None:
            if period_s is None or not math.isfinite(period_s) or period_s <= 0:
                raise ValueError("time-based translation response requires a finite positive period_s")
            translation_step = -math.expm1(-period_s / self.translation_time_constant_s)
            mapped_position = user_input.position * self.pos_mapping
            raw_movement = mapped_position - self._last_input_position
            self._last_input_position = mapped_position.copy()
            if position_locked:
                self._held_input_position = mapped_position
                self._pending_position = requested_position.copy()
                self._reverse_movement = np.zeros(3)
            else:
                # A sliding deadband suppresses hold jitter at every position;
                # slow motion still accumulates until it leaves the band.
                movement = apply_vector_deadband(
                    mapped_position - self._held_input_position, self.input_translation_deadband_m
                )
                if self.translation_reversal_reset and np.linalg.norm(raw_movement) > 1e-12:
                    pending = self._pending_position - self.target.position
                    if np.dot(raw_movement, pending) < 0:
                        self._reverse_movement += raw_movement
                    else:
                        self._reverse_movement = np.zeros(3)
                    reverse_distance = float(np.linalg.norm(self._reverse_movement))
                    confirmation_distance = reverse_distance
                    threshold = self.translation_reversal_threshold_m
                    if threshold is not None:
                        # Only displacement opposing pending travel confirms a
                        # reversal; a large tangential move must not erase it.
                        pending_distance = float(np.linalg.norm(pending))
                        confirmation_distance = (
                            max(0.0, -float(np.dot(self._reverse_movement, pending)) / pending_distance)
                            if pending_distance > 1e-12
                            else 0.0
                        )
                    if confirmation_distance > max(threshold or 0.0, self.input_translation_deadband_m, 1e-12):
                        direction = self._reverse_movement / reverse_distance
                        opposing = min(0.0, float(np.dot(pending, direction)))
                        # Remove only unsent input opposing the new direction.
                        # Accepted position/velocity stay continuous downstream.
                        self._pending_position = self._pending_position - opposing * direction
                        if threshold is None:
                            movement = direction * (reverse_distance - self.input_translation_deadband_m)
                            self._held_input_position = mapped_position - direction * self.input_translation_deadband_m
                        # With a separate threshold, earlier small movements
                        # already reached the pending target; do not replay them.
                        self._reverse_movement = np.zeros(3)
                        reversal_reset = True
                if not reversal_reset or self.translation_reversal_threshold_m is not None:
                    self._held_input_position = self._held_input_position + movement
                self._pending_position = self._pending_position + movement
                if self.stationary_pending_snap_m is not None and float(np.linalg.norm(raw_movement)) <= 1e-12:
                    pending_error = float(np.linalg.norm(self._pending_position - self.target.position))
                    if pending_error <= self.stationary_pending_snap_m:
                        self._pending_position = self.target.position.copy()
                        self._held_input_position = mapped_position.copy()
                        self._reverse_movement = np.zeros(3)
                backlog_limited = self._bound_pending_translation()
                requested_position = self._pending_position.copy()

        if orientation_locked:
            requested_rotation = self.locked_tool_orientation
        else:
            requested_rotation = quat_multiply(relative_quaternion, self.tool_anchor.quaternion)

        requested = Pose7.from_values(requested_position, requested_rotation)

        previous_target = self.target
        smoothed = Pose7.from_values(
            self.target.position + translation_step * (requested.position - self.target.position),
            quat_slerp(self.target.quaternion, requested.quaternion, self.smooth_step),
        )
        limited = self.safety_limits.apply(self.target, smoothed)
        safety_limited = (
            not np.allclose(limited.position, smoothed.position, atol=1e-12)
            or quat_angle(limited.quaternion, smoothed.quaternion) > 1e-9
        )
        self.target = limited
        self._mode = mode
        self._has_tracked_command = True
        return RobotCommand(
            target_pose=self.target,
            gripper=gripper,
            sequence=sequence,
            logical_pose=requested,
            position_locked=position_locked,
            orientation_locked=orientation_locked,
            target_delta_m=float(np.linalg.norm(self.target.position - previous_target.position)),
            target_delta_deg=quat_angle(self.target.quaternion, previous_target.quaternion),
            safety_limited=safety_limited,
            translation_backlog_limited=backlog_limited,
            translation_reversal_reset=reversal_reset,
        )
