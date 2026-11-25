# Adapted from third-party teleoperation code; see third_party/teleoperation/NOTICE.
"""Robotiq URCap ASCII TCP gripper backend (default port 63352).

GET/SET register commands address the controller-side Robotiq service, not UR
digital outputs. Activation is explicit and finishes before the worker starts.
Motion calls only update a latest-command mailbox; socket IO stays off the
UR5 servo thread. POS/OBJ are physical feedback; an ack alone is not completion.
"""

import math
import socket
import threading
import time
from dataclasses import dataclass

from .gripper import GripperCommandResult


@dataclass(frozen=True)
class RobotiqConfig:
    port: int = 63352
    slave_id: int | None = None
    speed: int = 96
    force: int = 32
    open_position: int = 0
    closed_position: int = 255
    threshold: float = 0.5
    socket_timeout_s: float = 0.5
    activation_timeout_s: float = 10.0
    poll_interval_s: float = 0.1

    def __post_init__(self):
        for key in ("speed", "force", "open_position", "closed_position"):
            value = getattr(self, key)
            if type(value) is not int or not 0 <= value <= 255:
                raise ValueError(f"Robotiq {key} must be an integer in [0, 255]")
        if type(self.port) is not int or not 1 <= self.port <= 65535:
            raise ValueError("invalid Robotiq port")
        if self.slave_id is not None and (type(self.slave_id) is not int or not 1 <= self.slave_id <= 247):
            raise ValueError("Robotiq slave_id must be an integer in [1, 247]")
        if self.open_position >= self.closed_position:
            raise ValueError("Robotiq open_position must be below closed_position")
        if not math.isfinite(self.threshold) or not 0 < self.threshold < 1:
            raise ValueError("Robotiq threshold must be in (0, 1)")
        for key in ("socket_timeout_s", "activation_timeout_s", "poll_interval_s"):
            value = getattr(self, key)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"Robotiq {key} must be finite and positive")


class RobotiqConnection:
    """Serialized request/reply transport; GET replies must be newline framed."""

    def __init__(self, host: str, config: RobotiqConfig):
        self.socket = socket.create_connection((host, config.port), timeout=config.socket_timeout_s)
        self.socket.settimeout(config.socket_timeout_s)
        self.timeout_s = config.socket_timeout_s
        self.buffer = bytearray()

    def _request(self, command: str, *, ack: bool = False) -> str:
        self.socket.settimeout(self.timeout_s)
        self.socket.sendall((command + "\n").encode("ascii"))
        deadline = time.monotonic() + self.timeout_s
        while True:
            self.buffer = self.buffer.lstrip(b"\r\n")
            if ack and self.buffer.startswith(b"ack"):
                del self.buffer[:3]
                return "ack"
            if b"\n" in self.buffer:
                response, _, remainder = self.buffer.partition(b"\n")
                self.buffer = bytearray(remainder)
                return response.decode("ascii").strip()
            if len(self.buffer) > 1024:
                raise RuntimeError("Robotiq response exceeded 1024 bytes")
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("Robotiq response timed out")
            self.socket.settimeout(remaining)
            chunk = self.socket.recv(1024)
            if not chunk:
                raise ConnectionError("Robotiq connection closed")
            self.buffer.extend(chunk)

    def get(self, name: str) -> int:
        raw = self._request(f"GET {name}")
        fields = raw.split()
        if len(fields) != 2 or fields[0] != name:
            raise RuntimeError(f"Unexpected Robotiq response for {name}")
        try:
            return int(fields[1])
        except ValueError as exc:
            if fields[1] == "?":
                raise RuntimeError(
                    f"Robotiq register {name} returned '?'; the URCap socket is reachable, but the selected "
                    "gripper/SID is not readable. Pass --gripper-slave-id with the Robotiq socket SID "
                    "configured on the teach pendant, commonly 9 for PolyScope gripper ID 1."
                ) from exc
            raise RuntimeError(f"Robotiq register {name} returned non-integer value {fields[1]!r}") from exc

    def set(self, **registers: int) -> None:
        command = "SET " + " ".join(f"{key} {value}" for key, value in registers.items())
        if self._request(command, ack=True) != "ack":
            raise RuntimeError("Robotiq SET rejected")

    def close(self):
        self.socket.close()


class RobotiqTcpGripperBackend:
    backend_name = "robotiq_tcp_gripper"

    def __init__(self, host: str, config: RobotiqConfig | None = None, *, activate: bool = False):
        self.config = config or RobotiqConfig()
        self._io = RobotiqConnection(host, self.config)
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._closing = threading.Event()
        self._desired: str | None = None
        self._pending: tuple[int, str] | None = None
        self._request_id = 0
        self._completed_id = 0
        self._reported_id = 0
        self._cancel = False
        self._fault: str | None = None
        self._feedback: dict[str, int] = {}
        try:
            if self.config.slave_id is not None:
                self._io.set(SID=self.config.slave_id)
            active = self._io.get("ACT")
            status = self._io.get("STA")
            if active != 1 or status != 3:
                if not activate:
                    raise RuntimeError("Robotiq is inactive; use --gripper-activate or activate on the teach pendant")
                if active != 1:
                    self._io.set(ACT=1)
                deadline = time.monotonic() + self.config.activation_timeout_s
                while self._io.get("STA") != 3:
                    if time.monotonic() >= deadline:
                        raise TimeoutError("Robotiq activation timed out")
                    time.sleep(0.05)
            self._poll_feedback()
        except BaseException:
            self._io.close()
            raise
        self._worker = threading.Thread(target=self._run, name="robotiq-io", daemon=True)
        self._worker.start()

    def _poll_feedback(self):
        status = self._io.get("STA")
        fault = self._io.get("FLT")
        feedback = {
            "position_raw": self._io.get("POS"),
            "object_status": self._io.get("OBJ"),
            "fault_code": fault,
            "feedback_monotonic_ns": time.monotonic_ns(),
        }
        with self._lock:
            self._feedback = feedback
        if status != 3 or fault != 0:
            raise RuntimeError(f"robotiq_status_{status}_fault_{fault}")

    def set_command(self, command: float | None) -> GripperCommandResult:
        if command is None or not math.isfinite(command) or not 0 <= command <= 1:
            return GripperCommandResult(
                command=None,
                state="no_command",
                executed=False,
                reasons=("no_command",),
            )
        state = "closed" if command >= self.config.threshold else "open"
        with self._lock:
            if self._fault is not None or self._closing.is_set():
                return GripperCommandResult(
                    command=command,
                    state="blocked",
                    executed=False,
                    reasons=(self._fault or "closed",),
                    **self._feedback,
                )
            if state != self._desired:
                self._desired = state
                self._request_id += 1
                self._pending = (self._request_id, state)
                self._wake.set()
            executed = self._completed_id == self._request_id and self._reported_id != self._completed_id
            if executed:
                self._reported_id = self._completed_id
            reason = "acknowledged" if executed else "queued" if self._completed_id != self._request_id else "unchanged"
            return GripperCommandResult(command, state, executed, (reason,), **self._feedback)

    def _run(self):
        try:
            while True:
                self._wake.wait(self.config.poll_interval_s)
                self._wake.clear()
                if self._closing.is_set():
                    self._io.set(GTO=0)
                    break
                with self._lock:
                    cancel = self._cancel
                    self._cancel = False
                    pending = self._pending
                    self._pending = None
                if cancel:
                    self._io.set(GTO=0)
                if pending is not None:
                    request_id, state = pending
                    with self._lock:
                        if request_id != self._request_id or self._cancel:
                            continue
                    position = self.config.closed_position if state == "closed" else self.config.open_position
                    self._io.set(POS=position, SPE=self.config.speed, FOR=self.config.force, GTO=1)
                    with self._lock:
                        self._completed_id = request_id
                self._poll_feedback()
        except (OSError, RuntimeError, ValueError) as exc:
            with self._lock:
                self._fault = f"backend_error:{type(exc).__name__}:{exc}"
                self._pending = None
        finally:
            self._io.close()

    def stop(self):
        with self._lock:
            self._request_id += 1
            self._pending = None
            self._desired = None
            self._cancel = True
        self._wake.set()

    def close(self):
        self._closing.set()
        self._wake.set()
        self._worker.join(timeout=self.config.socket_timeout_s * 5 + 1)
        if self._worker.is_alive():
            self._io.close()
            self._worker.join(timeout=self.config.socket_timeout_s + 1)
