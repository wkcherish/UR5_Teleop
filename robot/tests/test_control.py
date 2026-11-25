import json
from dataclasses import replace

import numpy as np
import pytest

from quest_ur5.backend import VirtualBackend
from quest_ur5.config import Config, load_config
from quest_ur5.protocol import QuestPacket
from quest_ur5.safety import UrSafetyState
from quest_ur5.session import TeleopSession
from quest_ur5.teleop import Pose7, quat_angle


@pytest.mark.parametrize(
    "path,value",
    [
        (("valid", "tracking"), "false"),
        (("buttons", "clutch"), "false"),
        (("buttons", "trigger"), float("nan")),
        (("sequence_id",), True),
        (("mode",), "unsupported"),
        (("protocol_version",), 1),
        (("controller", "position"), [0, float("inf"), 0]),
        (("controller", "position"), [True, 0, 0]),
        (("controller", "position"), ["1", 0, 0]),
        (("controller", "quaternion_xyzw"), [0, 0, 0, 0]),
        (("stream",), "left_controller"),
        (("heartbeat",), None),
    ],
)
def test_strict_boundary_rejects_invalid_fields(wire_packet, path, value):
    data = wire_packet()
    container = data
    for name in path[:-1]:
        container = container[name]
    container[path[-1]] = value
    with pytest.raises(ValueError):
        QuestPacket.decode(json.dumps(data).encode())


@pytest.mark.parametrize("payload", [b"[]", b"null", b"{", b"\xff", b" " * 5000])
def test_malformed_datagrams(payload):
    with pytest.raises(ValueError):
        QuestPacket.decode(payload)


def test_startup_requires_observed_release_and_has_no_position_jump(packet):
    backend = VirtualBackend()
    session = TeleopSession(Config(), backend)
    start = backend.pose.position.copy()
    assert not session.ingest(packet(1, clutch=True), 1.0)
    assert session.tick(1.0, backend.read()) is None
    session.ingest(packet(2), 1.01)
    session.ingest(packet(3, clutch=True, position=[0.2, 0.5, 1]), 1.02)
    session.tick(1.02, backend.read())
    np.testing.assert_allclose(backend.pose.position, start)


def moving_session(packet, config=None):
    backend = VirtualBackend()
    session = TeleopSession(config or Config(), backend)
    session.ingest(packet(1), 1.0)
    session.ingest(packet(2, clutch=True), 1.01)
    session.tick(1.01, backend.read())
    return session, backend


def test_unity_forward_maps_robot_x_and_respects_speed_limit(packet):
    session, backend = moving_session(packet)
    start = backend.pose.position.copy()
    session.ingest(packet(3, clutch=True, position=[0, 0, 0.02]), 1.02)
    session.tick(1.02, backend.read())
    assert 0 < backend.pose.position[0] - start[0] <= 0.035 * 0.01 + 1e-10
    np.testing.assert_allclose(backend.pose.position[1:], start[1:])


def test_release_stops_and_discards_old_target(packet):
    session, backend = moving_session(packet)
    session.ingest(packet(3), 1.02)
    held = backend.pose.position.copy()
    assert backend.stop_count == 1
    assert session.tick(1.03, backend.read()) is None
    np.testing.assert_allclose(backend.pose.position, held)


@pytest.mark.parametrize("fault", ["timeout", "tracking", "jump", "sequence", "restart", "robot"])
def test_fault_requires_release_before_resume(packet, fault):
    session, backend = moving_session(packet)
    session_name = "quest-a"
    if fault == "timeout":
        session.tick(1.3, backend.read())
    elif fault == "tracking":
        session.ingest(packet(3, clutch=True, valid=False), 1.02)
    elif fault == "jump":
        session.ingest(packet(3, clutch=True, position=[1, 0, 0]), 1.02)
    elif fault == "sequence":
        session.ingest(packet(1, clutch=True), 1.02)
    elif fault == "restart":
        session_name = "quest-b"
        session.ingest(packet(1, clutch=True, session=session_name), 1.02)
    else:
        state = replace(backend.read(), safety=UrSafetyState(safety_mode=3))
        session.tick(1.02, state)
    assert not session.armed and not session.moving
    session.ingest(packet(4, clutch=True, session=session_name), 1.31)
    assert session.tick(1.31, backend.read()) is None
    session.ingest(packet(5, session=session_name), 1.32)
    session.ingest(packet(6, clutch=True, session=session_name), 1.33)
    session.tick(1.33, backend.read())
    assert session.moving


def test_retired_session_cannot_take_back_control(packet):
    session, _ = moving_session(packet)
    session.ingest(packet(1, session="quest-b"), 1.02)
    assert not session.ingest(packet(4, clutch=True), 1.03)
    assert not session.armed


def test_downward_and_free_wrist_modes_preserve_position(packet):
    session, backend = moving_session(packet)
    backend.pose = Pose7.from_values(backend.pose.position, [0, 0, 0, 1])
    start = backend.pose.position.copy()
    session.ingest(packet(3, clutch=True, mode="default_grasp", position=[0.01, 0, 0]), 1.02)
    session.tick(1.02, backend.read())
    np.testing.assert_allclose(backend.pose.position, start)
    assert 0 < quat_angle([0, 0, 0, 1], backend.pose.quaternion) <= 0.14 + 1e-7
    session.ingest(packet(4, clutch=True, mode="free_wrist", position=[0.02, 0, 0]), 1.03)
    session.tick(1.03, backend.read())
    np.testing.assert_allclose(backend.pose.position, start)


def test_default_grasp_ignores_tilt_but_free_wrist_allows_it(packet):
    session, backend = moving_session(packet)
    initial = backend.pose.quaternion.copy()
    session.ingest(packet(3, clutch=True, mode="default_grasp"), 1.02)
    session.tick(1.02, backend.read())
    tilted = Pose7.from_values([0, 0, 0], [0.258819, 0, 0, 0.965926])
    session.ingest(replace(packet(4, clutch=True, mode="default_grasp"), controller_pose=tilted), 1.03)
    session.tick(1.03, backend.read())
    assert quat_angle(initial, backend.pose.quaternion) < 1e-6
    session.ingest(packet(5, clutch=True, mode="free_wrist"), 1.04)
    session.tick(1.04, backend.read())
    session.ingest(replace(packet(6, clutch=True, mode="free_wrist"), controller_pose=tilted), 1.05)
    session.tick(1.05, backend.read())
    assert quat_angle(initial, backend.pose.quaternion) > 0.01


def test_actual_pose_outside_workspace_blocks_motion(packet):
    session, backend = moving_session(packet)
    backend.pose = Pose7.from_values([0, 0, -0.1], [1, 0, 0, 0])
    result = session.tick(1.02, backend.read())
    assert not result.allowed and not session.armed


@pytest.mark.parametrize("mode", [None, "unknown", 0, 3, 9, True])
def test_unknown_or_stopped_safety_mode_fails_closed(mode):
    assert not UrSafetyState(safety_mode=mode).motion_allowed


@pytest.mark.parametrize(
    "changes",
    [
        {"rate_hz": float("nan")},
        {"listen_port": True},
        {"input_timeout_s": 0.001},
        {"workspace_min": [1, 2, 3], "workspace_max": [0, 0, 0]},
        {"gripper": "typo"},
        {"max_linear_speed_mps": -1},
        {"max_command_lead_m": None},
        {"quest_ip": "bad"},
    ],
)
def test_bad_config_rejected(changes):
    with pytest.raises((ValueError, TypeError)):
        Config(**changes)


def test_config_rejects_unknown_keys(tmp_path):
    path = tmp_path / "config.json"
    path.write_text('{"robot_ipp":"192.0.2.1"}')
    with pytest.raises(ValueError, match="unknown"):
        load_config(path)
