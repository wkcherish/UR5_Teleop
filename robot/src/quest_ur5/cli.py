"""Cross-platform command line; real hardware always requires explicit opt-in."""

import argparse
import json
import logging
import math
import signal
import time
from dataclasses import asdict
from pathlib import Path

from .backend import RtdeBackend, VirtualBackend
from .config import load_config
from .runtime import run


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Standalone Quest 3 / UR5 control host")
    parser.add_argument("--config", type=Path, help="JSON config; required for real hardware")
    parser.add_argument("--backend", choices=("virtual", "rtde"), default="virtual")
    parser.add_argument("--confirm-real-robot", action="store_true")
    parser.add_argument("--activate-gripper", action="store_true", help="Allow Robotiq activation (may move jaws)")
    parser.add_argument("--check-config", action="store_true", help="Validate and print config without hardware access")
    parser.add_argument("--log", type=Path, help="New JSONL log path (existing files are never overwritten)")
    parser.add_argument("--duration", type=float, help="Exit after this many seconds")
    args = parser.parse_args(argv)
    try:
        config = load_config(args.config)
        if args.duration is not None and (not math.isfinite(args.duration) or args.duration <= 0):
            parser.error("--duration must be finite and positive")
        if args.check_config:
            print(json.dumps(asdict(config), indent=2))
            return 0
        if args.backend == "rtde" and (
            not args.confirm_real_robot or args.config is None or not config.robot_ip or not config.quest_ip
        ):
            parser.error("RTDE requires --config, robot_ip, quest_ip and --confirm-real-robot")
        if args.activate_gripper and (args.backend != "rtde" or config.gripper != "robotiq"):
            parser.error("--activate-gripper requires RTDE and gripper=robotiq")
    except (ValueError, TypeError, OSError) as exc:
        parser.error(str(exc))
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    backend = None
    previous_sigterm = signal.getsignal(signal.SIGTERM)

    def shutdown(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, shutdown)
    try:
        backend = (
            RtdeBackend(config, activate_gripper=args.activate_gripper) if args.backend == "rtde" else VirtualBackend()
        )
        run(
            config,
            backend,
            log_path=args.log or Path("logs") / f"teleop-{time.time_ns()}.jsonl",
            duration=args.duration,
        )
        return 0
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        logging.error("Control host stopped: %s", exc)
        return 1
    finally:
        if backend is not None:
            backend.close()
        signal.signal(signal.SIGTERM, previous_sigterm)
