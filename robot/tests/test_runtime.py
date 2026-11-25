import json
import socket
import subprocess
import sys
import time
from pathlib import Path

from quest_ur5.backend import RobotState, VirtualBackend
from quest_ur5.config import Config
from quest_ur5.runtime import feedback_payload
from quest_ur5.session import TeleopSession


def free_udp_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def test_real_udp_input_moves_virtual_tcp_and_times_out(tmp_path, wire_packet):
    port = free_udp_port()
    feedback_port = free_udp_port()
    while feedback_port == port:
        feedback_port = free_udp_port()
    config = tmp_path / "config.json"
    config.write_text(json.dumps({"listen_host": "127.0.0.1", "listen_port": port, "feedback_port": feedback_port}))
    log = tmp_path / "teleop.jsonl"
    process = subprocess.Popen(
        [
            sys.executable,
            "-m",
            "quest_ur5",
            "--config",
            str(config),
            "--backend",
            "virtual",
            "--duration",
            "1.0",
            "--log",
            str(log),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    try:
        assert "Listening" in process.stdout.readline()
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            for sequence in range(1, 14):
                payload = wire_packet(sequence, clutch=sequence >= 3, position=[0, 0, max(0, sequence - 4) * 0.002])
                sock.sendto(json.dumps(payload).encode(), ("127.0.0.1", port))
                time.sleep(0.02)
        output, _ = process.communicate(timeout=8)
        assert process.returncode == 0, output
        records = [json.loads(line) for line in log.read_text().splitlines()]
        moving = [r for r in records if r["state"] == "moving"]
        assert moving and max(r["target_tcp"][0] for r in moving) > 0.35
        assert records[-1]["state"] == "input_timeout"
        assert not records[-1]["armed"]
    finally:
        if process.poll() is None:
            process.kill()
            process.communicate()


def test_feedback_schema_and_restart_identity():
    backend = VirtualBackend()
    state = RobotState(backend.pose, [0, 1, 2, 3, 4, 5], backend.read().safety)
    data = json.loads(feedback_payload(state, TeleopSession(Config(), backend), 0, "host-b"))
    assert data["protocol_version"] == 1
    assert data["stream"] == "ur5_joint_state"
    assert data["session_id"] == "host-b"
    assert data["joint_names"][0] == "shoulder_pan_joint"
    assert data["joint_positions_rad"] == [0, 1, 2, 3, 4, 5]


def test_real_backend_cannot_be_started_without_explicit_authorization():
    process = subprocess.run(
        [sys.executable, "-m", "quest_ur5", "--backend", "rtde"], text=True, capture_output=True, timeout=10
    )
    assert process.returncode == 2
    assert "--confirm-real-robot" in process.stderr


def test_portable_export_never_overwrites_source_or_existing_directory(tmp_path):
    tool = Path(__file__).resolve().parents[2] / "tools/prepare_unity_project.py"
    for output in (tool.parent.parent, tmp_path):
        result = subprocess.run(
            [sys.executable, str(tool), "--output", str(output)], text=True, capture_output=True, timeout=10
        )
        assert result.returncode == 2
