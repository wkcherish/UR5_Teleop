"""Bounded UDP receive loop, paced servo execution and read-only state feedback."""

import json
import logging
import select
import socket
import time
import uuid
from pathlib import Path

from .backend import RobotBackend, RobotState
from .config import Config
from .protocol import QuestPacket
from .session import TeleopSession

LOG = logging.getLogger(__name__)
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def feedback_payload(state: RobotState, session: TeleopSession, sequence: int, session_id: str) -> bytes:
    """Only the hardware backend may publish measured joint feedback."""
    return json.dumps(
        {
            "protocol_version": 1,
            "stream": "ur5_joint_state",
            "source": "quest_ur5_host",
            "session_id": session_id,
            "source_timestamp_ns": time.monotonic_ns(),
            "sequence": sequence,
            "joint_names": JOINT_NAMES,
            "joint_positions_rad": state.joints,
            "gripper_command": session.gripper_command,
            "control_state": session.reason,
        },
        allow_nan=False,
        separators=(",", ":"),
    ).encode("utf-8")


def run(config: Config, backend: RobotBackend, *, log_path: Path, duration: float | None = None) -> None:
    session = TeleopSession(config, backend)
    host_session = uuid.uuid4().hex
    peer_ip = config.quest_ip or None
    period = 1.0 / config.rate_hz
    sequence = 0
    next_tick = time.monotonic()
    started = next_tick
    last_reason = None
    log_path.parent.mkdir(parents=True, exist_ok=True)
    # Bind before starting control; the caller closes hardware on bind/log failures.
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock, log_path.open("x", encoding="utf-8") as log:
        sock.bind((config.listen_host, config.listen_port))
        sock.setblocking(False)
        LOG.info(
            "Listening on %s:%d; Quest feedback port %d", config.listen_host, config.listen_port, config.feedback_port
        )
        LOG.info("Release Grip once, then hold Grip to move. Log: %s", log_path)
        while duration is None or time.monotonic() - started < duration:
            now = time.monotonic()
            readable, _, _ = select.select([sock], [], [], max(0.0, next_tick - now))
            if readable:
                stop_seen = False
                for _ in range(64):
                    try:
                        payload, address = sock.recvfrom(8192)
                    except BlockingIOError:
                        break
                    if peer_ip is not None and address[0] != peer_ip:
                        continue
                    try:
                        packet = QuestPacket.decode(payload)
                    except ValueError as exc:
                        session.halt("malformed_packet")
                        stop_seen = True
                        LOG.debug("Discarded packet: %s", exc)
                        continue
                    if peer_ip is None:
                        peer_ip = address[0]
                        LOG.info("Pinned Quest sender to %s for this process", peer_ip)
                    if not session.ingest(packet, time.monotonic()):
                        stop_seen = True
                else:
                    session.halt("udp_batch_saturated")
                    stop_seen = True
                if stop_seen:
                    # Keep an accepted release's arm state, but never execute a
                    # subsequent clutched packet from this same receive batch.
                    session.halt(session.reason, disarm=False)
            now = time.monotonic()
            if now < next_tick:
                continue
            # No burst catch-up after scheduling/RTDE delays.
            next_tick = max(next_tick + period, now)
            state = backend.read()
            backend.heartbeat()
            result = session.tick(now, state)
            if backend.measured_joints and peer_ip:
                try:
                    sock.sendto(
                        feedback_payload(state, session, sequence, host_session), (peer_ip, config.feedback_port)
                    )
                except OSError as exc:
                    # A failed feedback path must also stop physical motion.
                    session.halt("feedback_send_failed")
                    LOG.warning("Quest feedback failed: %s", exc)
            record = {
                "host_time_ns": time.monotonic_ns(),
                "sequence": sequence,
                "input_sequence": session.latest.sequence if session.latest else None,
                "state": session.reason,
                "armed": session.armed,
                "actual_tcp": state.pose.as_rtde_pose().tolist(),
                "target_tcp": result.command.target_pose.as_rtde_pose().tolist() if result and result.command else None,
                "limits": list(result.reasons) if result else [],
                "gripper_command": session.gripper_command,
            }
            log.write(json.dumps(record, allow_nan=False) + "\n")
            if session.reason != last_reason:
                LOG.info("Control: %s", session.reason)
                last_reason = session.reason
                log.flush()
            elif sequence % 100 == 0:
                log.flush()
            sequence += 1
