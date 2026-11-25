from dataclasses import replace
from types import SimpleNamespace

import pytest

from quest_ur5.backend import RtdeBackend
from quest_ur5.config import Config


@pytest.fixture
def rtde_modules(monkeypatch):
    class Device:
        def __init__(self, *_args):
            self.calls = []
            self.connected = True
            self.timestamp = 1.0
            self.ik = True
            self.safe = True

        def isConnected(self):
            return self.connected

        def getTimestamp(self):
            self.timestamp += 0.01
            return self.timestamp

        def getActualTCPPose(self):
            return [0.35, 0, 0.35, 3.141592653589793, 0, 0]

        def getActualQ(self):
            return [0, -1.5, 1.5, -1.5, -1.5, 0]

        def getSpeedScaling(self):
            return 1.0

        def isProtectiveStopped(self):
            return False

        def isEmergencyStopped(self):
            return False

        def getSafetyMode(self):
            return 1

        def getRobotMode(self):
            return 7

        def setWatchdog(self, hz):
            self.calls.append(("watchdog", hz))
            return True

        def kickWatchdog(self):
            self.calls.append(("kick",))
            return True

        def getInverseKinematicsHasSolution(self, *_args):
            return self.ik

        def isPoseWithinSafetyLimits(self, *_args):
            return self.safe

        def servoL(self, *args):
            self.calls.append(("servo", *args))
            return True

        def servoStop(self, *_args):
            self.calls.append(("stop",))

        def stopScript(self):
            self.calls.append(("stop_script",))

        def disconnect(self):
            self.calls.append(("disconnect",))
            self.connected = False

    control, receive = Device(), Device()
    monkeypatch.setattr(
        "quest_ur5.backend.importlib.import_module",
        lambda name: (
            SimpleNamespace(RTDEControlInterface=lambda *_args: control)
            if name == "rtde_control"
            else SimpleNamespace(RTDEReceiveInterface=lambda *_args: receive)
        ),
    )
    return control, receive


def test_rtde_checks_ik_safety_and_installs_watchdog(rtde_modules):
    control, _ = rtde_modules
    backend = RtdeBackend(replace(Config(), robot_ip="192.0.2.1"))
    state = backend.read()
    assert state.safety.motion_allowed
    backend.heartbeat()
    backend.servo(state.pose, 0.01)
    control.ik = False
    with pytest.raises(RuntimeError, match="kinematics"):
        backend.servo(state.pose, 0.01)
    control.ik = True
    control.safe = False
    with pytest.raises(RuntimeError, match="safety"):
        backend.servo(state.pose, 0.01)
    assert sum(c[0] == "servo" for c in control.calls) == 1
    assert ("watchdog", 5.0) in control.calls
    backend.close()
    assert ("stop_script",) in control.calls
    assert not control.connected


def test_stale_and_disconnected_rtde_fail_closed(rtde_modules, monkeypatch):
    _, receive = rtde_modules
    backend = RtdeBackend(Config())
    receive.getTimestamp = lambda: 1.01
    monkeypatch.setattr("quest_ur5.backend.time.monotonic", lambda: backend._last_state_change + 1)
    with pytest.raises(RuntimeError, match="stale"):
        backend.read()
    receive.connected = False
    with pytest.raises(RuntimeError, match="disconnected"):
        backend.read()
    backend.close()


def test_receive_resource_closed_if_control_initialization_fails(rtde_modules, monkeypatch):
    _, receive = rtde_modules

    def fail(*_args):
        raise RuntimeError("connection failed")

    monkeypatch.setattr(
        "quest_ur5.backend.importlib.import_module",
        lambda name: (
            SimpleNamespace(RTDEControlInterface=fail)
            if name == "rtde_control"
            else SimpleNamespace(RTDEReceiveInterface=lambda *_args: receive)
        ),
    )
    with pytest.raises(RuntimeError, match="connection failed"):
        RtdeBackend(Config())
    assert not receive.connected
