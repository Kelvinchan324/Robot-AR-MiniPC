"""Interactive keyboard test for the Robot AR yaw/pitch gimbal."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys

from .gimbal import GimbalConfig, GimbalController, load_config


DEFAULT_CONFIG = Path(__file__).resolve().parent.parent / "config" / "gimbal.json"


def read_key() -> str:
    if os.name == "nt":
        import msvcrt

        return msvcrt.getwch().lower()

    import termios
    import tty

    fd = sys.stdin.fileno()
    previous = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1).lower()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, previous)


def with_port(config: GimbalConfig, port: str | None) -> GimbalConfig:
    if not port:
        return config
    return GimbalConfig(port, config.baud, config.yaw, config.pitch, config.motion)


def main() -> int:
    parser = argparse.ArgumentParser(description="Keyboard test for the STS3215 VR gimbal")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--port", help="override the serial port in gimbal.json")
    parser.add_argument("--dry-run", action="store_true", help="test without opening hardware")
    args = parser.parse_args()

    config = with_port(load_config(args.config), args.port)
    gimbal = GimbalController(config, dry_run=args.dry_run)
    try:
        positions = gimbal.connect()
        print(f"Connected with torque OFF. Current position: {positions}")
        print("E enable/hold | W/S pitch | A/D yaw | C center | SPACE torque off | Q quit")
        print("Check that the mechanism is clear, then press E.")
        while True:
            key = read_key()
            try:
                if key == "q" or key == "\x03":
                    break
                if key == "e":
                    print("hold", gimbal.enable_hold())
                elif key == " ":
                    gimbal.torque_off()
                    print("TORQUE OFF — press E to hold the current pose again")
                elif key == "c":
                    print("center", gimbal.center())
                elif key == "a":
                    print(
                        "move",
                        gimbal.nudge(
                            yaw_ticks=-config.yaw.direction * config.motion.keyboard_step
                        ),
                    )
                elif key == "d":
                    print(
                        "move",
                        gimbal.nudge(
                            yaw_ticks=config.yaw.direction * config.motion.keyboard_step
                        ),
                    )
                elif key == "w":
                    print(
                        "move",
                        gimbal.nudge(
                            pitch_ticks=config.pitch.direction * config.motion.keyboard_step
                        ),
                    )
                elif key == "s":
                    print(
                        "move",
                        gimbal.nudge(
                            pitch_ticks=-config.pitch.direction * config.motion.keyboard_step
                        ),
                    )
            except RuntimeError as exc:
                print(f"blocked: {exc}")
    except KeyboardInterrupt:
        pass
    finally:
        gimbal.close()
        print("Torque off; serial port closed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
