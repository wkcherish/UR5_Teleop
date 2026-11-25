"""Hardware boundary. Virtual mode never imports or connects RTDE."""

import importlib
import logging
import time
from dataclasses import dataclass
from typing import Protocol

import numpy as np

from .config import Config
from .gripper import DisabledGripper, GripperBackend
from .safety import UrSafetyState
from .teleop import Pose7

LOG = logging.getLogger(__name__)


@dataclass(frozen=True)
class RobotState:
    pose: Pose7
    joints: list[float]
    safety: UrSafetyState


class RobotBackend(Protocol):
    measured_joints: bool
    gripper: GripperBackend

    def read(self) -> RobotState: ...
    def servo(self, pose: Pose7, period: float) -> None: ...
    def heartbeat(self) -> None: ...
    def stop(self) -> None: ...
    def close(self) -> None: ...


class VirtualBackend:
    """TCP-only simulation; joint feedback is deliberately unavailable (no fake IK)."""

    measured_joints = False

    def __init__(self) -> None:
        self.pose = Pose7.from_values([0.35, 0.0, 0.35], [1.0, 0.0, 0.0, 0.0])
        self.gripper = DisabledGripper()
        self.stop_count = 0

    def read(self) -> RobotState:
        return RobotState(self.pose, [], UrSafetyState.normal())

    def servo(self, pose: Pose7, period: float) -> None:
        self.pose = pose

    def heartbeat(self) -> None:
        pass

    def stop(self) -> None:
        self.stop_count += 1
        self.gripper.stop()

    def close(self) -> None:
        self.stop()


class RtdeBackend:
    measured_joints = True

    def __init__(self, config: Config, *, activate_gripper: bool = False) -> None:
        self._control = None
        self._receive = None
        self.gripper = DisabledGripper()
        self._last_robot_time = None
        self._last_state_change = time.monotonic()
        self._state_timeout = config.input_timeout_s
        self._last_joints = []
        self._speed = config.max_linear_speed_mps
        try:
            control = importlib.import_module("rtde_control")
            receive = importlib.import_module("rtde_receive")
            self._receive = receive.RTDEReceiveInterface(config.robot_ip, config.rate_hz)
            self._control = control.RTDEControlInterface(config.robot_ip, config.rate_hz)
            if config.gripper == "robotiq":
                from .robotiq import RobotiqConfig, RobotiqTcpGripperBackend

                self.gripper = RobotiqTcpGripperBackend(
                    config.robot_ip,
                    RobotiqConfig(
                        port=config.gripper_port,
                        slave_id=config.gripper_slave_id,
                        speed=config.gripper_speed,
                        force=config.gripper_force,
                    ),
                    activate=activate_gripper,
                )
            if not self._control.setWatchdog(5.0):
                raise RuntimeError("UR5 watchdog setup failed")
            self.read()
        except BaseException:
            self.close()
            raise

    def read(self) -> RobotState:
        if not self._receive.isConnected() or not self._control.isConnected():
            raise RuntimeError("RTDE disconnected")
        timestamp = self._receive.getTimestamp()
        now = time.monotonic()
        if not np.isfinite(timestamp):
            raise RuntimeError("invalid RTDE timestamp")
        if self._last_robot_time is not None and timestamp < self._last_robot_time:
            raise RuntimeError("UR5 state clock restarted")
        if timestamp != self._last_robot_time:
            self._last_robot_time = timestamp
            self._last_state_change = now
        if now - self._last_state_change >= self._state_timeout:
            raise RuntimeError("stale RTDE state")
        pose = Pose7.from_rtde_pose(self._receive.getActualTCPPose())
        joints = np.asarray(self._receive.getActualQ(), dtype=float)
        if joints.shape != (6,) or not np.all(np.isfinite(joints)):
            raise RuntimeError("invalid RTDE joint state")
        self._last_joints = joints.tolist()
        scaling = self._receive.getSpeedScaling()
        state = UrSafetyState(
            protective_stopped=self._receive.isProtectiveStopped(),
            emergency_stopped=self._receive.isEmergencyStopped(),
            robot_power_on=bool(np.isfinite(scaling) and scaling > 0),
            safety_mode=self._receive.getSafetyMode(),
            robot_mode=self._receive.getRobotMode(),
        )
        return RobotState(pose, self._last_joints, state)

    def servo(self, pose: Pose7, period: float) -> None:
        target = pose.as_rtde_pose().tolist()
        if not self._control.getInverseKinematicsHasSolution(target, self._last_joints):
            raise RuntimeError("UR5 target has no inverse kinematics solution")
        if not self._control.isPoseWithinSafetyLimits(target):
            raise RuntimeError("UR5 target violates controller safety limits")
        if not self._control.servoL(target, self._speed, 0.1, period, 0.1, 200):
            raise RuntimeError("UR5 rejected servoL")

    def heartbeat(self) -> None:
        if not self._control.kickWatchdog():
            raise RuntimeError("UR5 watchdog heartbeat failed")

    def stop(self) -> None:
        # Stop both devices even if one transport has already failed.
        try:
            if self._control is not None and self._control.isConnected():
                self._control.servoStop(1.0)
        finally:
            self.gripper.stop()

    def close(self) -> None:
        # Cleanup is best effort, but each resource gets its own attempt.
        actions = [self.stop, self.gripper.close]
        if self._control is not None:
            actions.extend([self._control.stopScript, self._control.disconnect])
        if self._receive is not None:
            actions.append(self._receive.disconnect)
        for action in actions:
            try:
                action()
            except Exception:
                LOG.exception("RTDE shutdown step failed")
