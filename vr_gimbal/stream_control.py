"""Read VR orientation as JSON lines and drive the yaw/pitch gimbal.

Accepted records:
  {"yaw_deg": 12.4, "pitch_deg": -3.0}
  {"quaternion": {"w": 1, "x": 0, "y": 0, "z": 0}}
  {"command": "recenter"}
  {"command": "stop"}
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

from .gimbal import GimbalConfig, GimbalController, load_config
from .keyboard_control import with_port
from .pose import PoseMapper, quaternion_to_yaw_pitch


DEFAULT_CONFIG = Path(__file__).resolve().parent.parent / "config" / "gimbal.json"


def orientation(record: dict) -> tuple[float, float]:
    if "quaternion" in record:
        q = record["quaternion"]
        return quaternion_to_yaw_pitch(float(q["w"]), float(q["x"]), float(q["y"]), float(q["z"]))
    return float(record["yaw_deg"]), float(record["pitch_deg"])


def main() -> int:
    parser = argparse.ArgumentParser(description="Stream VR yaw/pitch to the STS3215 gimbal")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--port", help="override the serial port in gimbal.json")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--max-hz", type=float, default=30.0)
    args = parser.parse_args()
    if args.max_hz <= 0:
        parser.error("--max-hz must be positive")

    config: GimbalConfig = with_port(load_config(args.config), args.port)
    gimbal = GimbalController(config, dry_run=args.dry_run)
    mapper: PoseMapper | None = None
    last_write = 0.0
    try:
        positions = gimbal.connect()
        gimbal.enable_hold()
        mapper = PoseMapper(config, positions["yaw"], positions["pitch"])
        print(json.dumps({"ready": True, "position": positions}), flush=True)

        for line in sys.stdin:
            if not line.strip():
                continue
            try:
                record = json.loads(line)
                command = record.get("command")
                if command == "stop":
                    gimbal.torque_off()
                    print(json.dumps({"stopped": True}), flush=True)
                    continue
                if command == "enable":
                    positions = gimbal.enable_hold()
                    mapper = PoseMapper(config, positions["yaw"], positions["pitch"])
                    print(json.dumps({"enabled": True, "position": positions}), flush=True)
                    continue
                if command == "recenter":
                    mapper = PoseMapper(config, gimbal.targets["yaw"], gimbal.targets["pitch"])
                    print(json.dumps({"recenter": "waiting for next pose"}), flush=True)
                    continue

                now = time.monotonic()
                minimum_period = 1.0 / args.max_hz
                if now - last_write < minimum_period:
                    continue
                yaw, pitch = orientation(record)
                assert mapper is not None
                target_yaw, target_pitch = mapper.map(yaw, pitch)
                targets = gimbal.set_targets(target_yaw, target_pitch)
                last_write = now
                print(json.dumps({"target": targets}), flush=True)
            except (KeyError, TypeError, ValueError, RuntimeError, json.JSONDecodeError) as exc:
                print(json.dumps({"error": str(exc)}), flush=True)
    finally:
        gimbal.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
