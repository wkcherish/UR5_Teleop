import json

import pytest

from quest_ur5.protocol import QuestPacket


@pytest.fixture
def wire_packet():
    def make(sequence=1, *, clutch=False, mode=None, position=None, session="quest-a", valid=True):
        return {
            "protocol_version": 2,
            "session_id": session,
            "sequence_id": sequence,
            "source_timestamp_ns": 1_000_000_000 + sequence * 10_000_000,
            "source_timestamp_clock": "quest_realtime_since_startup",
            "source": "quest3_unity",
            "stream": "right_controller",
            "mode": mode or ("translate" if clutch else "idle"),
            "valid": {"controller_pose": valid, "buttons": True, "tracking": valid},
            "controller": {
                "position": position or [0, 0, 0],
                "quaternion_xyzw": [0, 0, 0, 1],
                "frame": "quest_tracking",
            },
            "buttons": {
                "clutch": clutch,
                "grip": float(clutch),
                "trigger": 0.4,
                "recenter": False,
                "stop_episode": False,
            },
            "heartbeat": True,
        }

    return make


@pytest.fixture
def packet(wire_packet):
    def make(*args, **kwargs):
        return QuestPacket.decode(json.dumps(wire_packet(*args, **kwargs)).encode())

    return make
