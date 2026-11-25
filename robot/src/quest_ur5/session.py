"""Clutch state machine and motion limits, independent of sockets and hardware."""

import numpy as np

from .backend import RobotBackend, RobotState
from .config import Config
from .protocol import QuestPacket
from .safety import SafetyFilterResult, Ur5SafetyFilter
from .teleop import (
    OpenTeachTeleopTracker,
    Pose7,
    PoseFrameTransform,
    quat_angle,
    quat_conjugate,
    quat_multiply,
)


class TeleopSession:
    def __init__(self, config: Config, backend: RobotBackend) -> None:
        self.config = config
        self.backend = backend
        self.transform = PoseFrameTransform.ur10_unity_to_robot().with_robot_yaw(config.input_yaw_degrees)
        self.tracker = OpenTeachTeleopTracker(
            smooth_step=config.smooth_step,
            pos_mapping=(config.translation_scale,) * 3,
            input_translation_deadband_m=config.input_translation_deadband_m,
            input_rotation_deadband_deg=config.input_rotation_deadband_deg,
            translation_time_constant_s=0.04,
            max_pending_translation_m=config.max_command_lead_m,
        )
        self.filter = Ur5SafetyFilter(config.safety_config())
        self.armed = False
        self.moving = False
        self.reason = "release_grip_to_arm"
        self.latest: QuestPacket | None = None
        self.last_received = float("-inf")
        self._previous: QuestPacket | None = None
        self._session_id: str | None = None
        self._retired_sessions: set[str] = set()
        self._command_pose = None
        self._last_servo_time = None
        self._mode = "idle"
        self._rotation_anchor = None
        self.gripper_command = -1.0

    def halt(self, reason: str, *, disarm: bool = True) -> None:
        if self.moving:
            self.backend.stop()
        self.moving = False
        self.tracker.pause()
        self._command_pose = None
        self._last_servo_time = None
        self.latest = None
        self.reason = reason
        if disarm:
            self.armed = False

    def ingest(self, packet: QuestPacket, now: float) -> bool:
        """Return false for a stop event. Caller must not hide it with a newer packet."""
        if packet.session_id in self._retired_sessions:
            self.halt("retired_session")
            return False
        if self._session_id != packet.session_id:
            if self._session_id is not None:
                self._retired_sessions.add(self._session_id)
            self._session_id = packet.session_id
            self._previous = None
            self.halt("new_session_release_grip")
        previous = self._previous
        if previous is not None and (
            packet.sequence <= previous.sequence or packet.timestamp_ns <= previous.timestamp_ns
        ):
            self.halt("out_of_order")
            return False
        self._previous = packet
        if now - self.last_received > self.config.input_timeout_s:
            self.halt("input_timeout")
        if not packet.valid or packet.recenter or packet.stop_episode:
            self.halt("invalid_tracking_or_stop")
            return False
        if previous is not None and previous.clutch and packet.clutch:
            if (
                np.linalg.norm(packet.controller_pose.position - previous.controller_pose.position)
                > self.config.max_input_step_m
                or quat_angle(packet.controller_pose.quaternion, previous.controller_pose.quaternion)
                > self.config.max_input_step_deg
                or (packet.timestamp_ns - previous.timestamp_ns) / 1e9 > self.config.input_timeout_s
            ):
                self.halt("controller_pose_jump_or_gap")
                return False
        self.last_received = now
        if not packet.clutch:
            self.halt("ready", disarm=False)
            self.armed = True
            return False
        if not self.armed:
            self.reason = "release_grip_to_arm"
            return False
        self.latest = packet
        return True

    def tick(self, now: float, state: RobotState) -> SafetyFilterResult | None:
        if not state.safety.motion_allowed:
            self.halt("robot_safety_state")
            return None
        if now - self.last_received >= self.config.input_timeout_s:
            self.halt("input_timeout")
            return None
        packet = self.latest
        if packet is None or not self.armed:
            return None
        period = 1.0 / self.config.rate_hz
        if self._last_servo_time is not None and now - self._last_servo_time > self.config.input_timeout_s:
            self.halt("control_loop_stalled")
            return None
        input_pose = self.transform.apply(packet.controller_pose)
        if self.tracker.paused or packet.mode != self._mode:
            self.tracker.resume(input_pose, state.pose)
            self._command_pose = state.pose
            self._mode = packet.mode
            # Grip+A always enters the same downward orientation reference.
            # Grip+B remains a free relative rotation from the measured TCP.
            self._rotation_anchor = (
                Pose7.from_values(state.pose.position, [1, 0, 0, 0]).quaternion
                if packet.mode == "default_grasp"
                else state.pose.quaternion
            )
            self.tracker.tool_anchor = Pose7.from_values(state.pose.position, self._rotation_anchor)
        if packet.mode == "default_grasp":
            # A adjusts heading while keeping the downward grasp reference;
            # B is the explicit route for unrestricted pitch/roll changes.
            delta = quat_multiply(input_pose.quaternion, quat_conjugate(self.tracker.input_anchor.quaternion))
            twist = np.array([0.0, 0.0, delta[2], delta[3]])
            if np.linalg.norm(twist) < 1e-9:
                twist = np.array([0.0, 0.0, 0.0, 1.0])
            input_pose = Pose7.from_values(
                input_pose.position, quat_multiply(twist, self.tracker.input_anchor.quaternion)
            )
        command = self.tracker.track(
            input_pose,
            gripper=packet.trigger,
            sequence=packet.sequence,
            position_locked=packet.mode != "translate",
            orientation_locked=packet.mode == "translate",
            period_s=period,
        )
        budget = period if self._last_servo_time is None else min(period, max(0.0, now - self._last_servo_time))
        if budget <= 0:
            return None
        result = self.filter.apply(
            command,
            actual_pose=state.pose,
            period_s=budget,
            safety_state=state.safety,
            previous_command_pose=self._command_pose,
        )
        if not result.allowed:
            self.halt("|".join(result.reasons))
            return result
        self.backend.servo(result.command.target_pose, period)
        self.moving = True
        self._last_servo_time = now
        self._command_pose = result.command.target_pose
        self.tracker.accept_target(result.command.target_pose)
        gripper = self.backend.gripper.set_command(packet.trigger)
        if gripper.state == "blocked":
            self.halt("gripper_fault")
            return result
        self.gripper_command = packet.trigger if gripper.state != "disabled" else -1.0
        self.reason = "moving"
        return result
