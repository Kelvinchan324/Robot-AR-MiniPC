"""Safe two-axis STS3215 gimbal control over a Waveshare serial adapter."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any, Protocol

from robot_bridge.feetech_bus import FeetechBus


TICKS_PER_REVOLUTION = 4096
DEGREES_PER_REVOLUTION = 360.0


class ServoBus(Protocol):
    def open(self) -> None: ...
    def close(self) -> None: ...
    def ping(self, sid: int) -> bool: ...
    def read_position(self, sid: int) -> int | None: ...
    def set_accel(self, sid: int, accel: int) -> bool: ...
    def set_speed(self, sid: int, speed: int) -> bool: ...
    def set_goal(self, sid: int, pos: int) -> bool: ...
    def set_torque(self, sid: int, on: bool) -> bool: ...


@dataclass(frozen=True)
class AxisConfig:
    servo_id: int
    center: int
    minimum: int
    maximum: int
    direction: int = 1

    def __post_init__(self) -> None:
        if not 1 <= self.servo_id <= 253:
            raise ValueError(f"invalid servo ID {self.servo_id}")
        if not 0 <= self.minimum < self.maximum <= 4095:
            raise ValueError("axis limits must satisfy 0 <= min < max <= 4095")
        if not self.minimum <= self.center <= self.maximum:
            raise ValueError("axis center must be inside its limits")
        if self.direction not in (-1, 1):
            raise ValueError("axis direction must be -1 or 1")

    def clamp(self, ticks: int | float) -> int:
        return max(self.minimum, min(self.maximum, int(round(ticks))))

    def validate_measured_position(self, ticks: int, name: str) -> int:
        ticks = int(ticks)
        if not self.minimum <= ticks <= self.maximum:
            raise RuntimeError(
                f"{name} position {ticks} is outside configured safe limits "
                f"[{self.minimum}, {self.maximum}]; torque remains off"
            )
        return ticks


@dataclass(frozen=True)
class MotionConfig:
    speed: int = 250
    acceleration: int = 20
    keyboard_step: int = 32
    vr_gain: float = 1.0
    smoothing_alpha: float = 0.25
    deadband_degrees: float = 0.5

    def __post_init__(self) -> None:
        if not 0 <= self.speed <= 32767:
            raise ValueError("speed must be between 0 and 32767")
        if not 0 <= self.acceleration <= 254:
            raise ValueError("acceleration must be between 0 and 254")
        if self.keyboard_step <= 0:
            raise ValueError("keyboard_step must be positive")
        if not 0 < self.smoothing_alpha <= 1:
            raise ValueError("smoothing_alpha must be in (0, 1]")
        if self.deadband_degrees < 0:
            raise ValueError("deadband_degrees must not be negative")


@dataclass(frozen=True)
class GimbalConfig:
    port: str
    baud: int
    yaw: AxisConfig
    pitch: AxisConfig
    motion: MotionConfig

    def __post_init__(self) -> None:
        if self.yaw.servo_id == self.pitch.servo_id:
            raise ValueError("yaw and pitch servos must have different IDs")
        if self.baud <= 0:
            raise ValueError("baud must be positive")


def _axis(raw: dict[str, Any]) -> AxisConfig:
    return AxisConfig(
        servo_id=int(raw["id"]),
        center=int(raw["center"]),
        minimum=int(raw["min"]),
        maximum=int(raw["max"]),
        direction=int(raw.get("direction", 1)),
    )


def load_config(path: str | Path) -> GimbalConfig:
    with Path(path).open(encoding="utf-8") as handle:
        raw = json.load(handle)
    return GimbalConfig(
        port=str(raw["port"]),
        baud=int(raw.get("baud", 1_000_000)),
        yaw=_axis(raw["yaw"]),
        pitch=_axis(raw["pitch"]),
        motion=MotionConfig(**raw.get("motion", {})),
    )


class DryRunBus:
    """In-memory bus used for setup and automated tests."""

    def __init__(self, positions: dict[int, int]) -> None:
        self.positions = dict(positions)
        self.torque: dict[int, bool] = {sid: False for sid in positions}

    def open(self) -> None:
        return None

    def close(self) -> None:
        return None

    def ping(self, sid: int) -> bool:
        return sid in self.positions

    def read_position(self, sid: int) -> int | None:
        return self.positions.get(sid)

    def set_accel(self, sid: int, accel: int) -> bool:
        return sid in self.positions

    def set_speed(self, sid: int, speed: int) -> bool:
        return sid in self.positions

    def set_goal(self, sid: int, pos: int) -> bool:
        if sid not in self.positions:
            return False
        self.positions[sid] = int(pos)
        return True

    def set_torque(self, sid: int, on: bool) -> bool:
        if sid not in self.positions:
            return False
        self.torque[sid] = bool(on)
        return True


class GimbalController:
    """Owns one serial bus and enforces hard yaw/pitch position limits."""

    def __init__(
        self,
        config: GimbalConfig,
        bus: ServoBus | None = None,
        *,
        dry_run: bool = False,
    ) -> None:
        self.config = config
        if bus is not None:
            self.bus = bus
        elif dry_run:
            self.bus = DryRunBus(
                {
                    config.yaw.servo_id: config.yaw.center,
                    config.pitch.servo_id: config.pitch.center,
                }
            )
        else:
            self.bus = FeetechBus(config.port, config.baud)
        self.connected = False
        self.enabled = False
        self.targets = {
            "yaw": config.yaw.center,
            "pitch": config.pitch.center,
        }

    @property
    def axes(self) -> dict[str, AxisConfig]:
        return {"yaw": self.config.yaw, "pitch": self.config.pitch}

    def connect(self) -> dict[str, int]:
        self.bus.open()
        missing = [name for name, axis in self.axes.items() if not self.bus.ping(axis.servo_id)]
        if missing:
            self.bus.close()
            ids = {name: axis.servo_id for name, axis in self.axes.items()}
            raise RuntimeError(f"gimbal servo(s) not found: {missing}; configured IDs: {ids}")

        positions: dict[str, int] = {}
        for name, axis in self.axes.items():
            raw = self.bus.read_position(axis.servo_id)
            if raw is None:
                self.bus.close()
                raise RuntimeError(f"could not read {name} servo ID {axis.servo_id}")
            try:
                positions[name] = axis.validate_measured_position(raw, name)
            except RuntimeError:
                self.bus.close()
                raise

        # Start from the measured pose so connecting never commands a jump to center.
        self.targets.update(positions)
        self.connected = True
        self.enabled = False
        return dict(positions)

    def enable_hold(self) -> dict[str, int]:
        self._require_connected()
        # Re-read immediately before enabling torque, then hold that exact pose.
        for name, axis in self.axes.items():
            raw = self.bus.read_position(axis.servo_id)
            if raw is None:
                raise RuntimeError(f"lost {name} servo before torque enable")
            self.targets[name] = axis.validate_measured_position(raw, name)
        self._write_targets(self.targets)
        self.enabled = True
        return dict(self.targets)

    def nudge(self, yaw_ticks: int = 0, pitch_ticks: int = 0) -> dict[str, int]:
        self._require_enabled()
        targets = {
            "yaw": self.config.yaw.clamp(self.targets["yaw"] + yaw_ticks),
            "pitch": self.config.pitch.clamp(self.targets["pitch"] + pitch_ticks),
        }
        self._write_targets(targets)
        self.targets.update(targets)
        return dict(self.targets)

    def set_targets(self, yaw_ticks: int | float, pitch_ticks: int | float) -> dict[str, int]:
        self._require_enabled()
        targets = {
            "yaw": self.config.yaw.clamp(yaw_ticks),
            "pitch": self.config.pitch.clamp(pitch_ticks),
        }
        self._write_targets(targets)
        self.targets.update(targets)
        return dict(self.targets)

    def center(self) -> dict[str, int]:
        return self.set_targets(self.config.yaw.center, self.config.pitch.center)

    def torque_off(self) -> None:
        if not self.connected:
            return
        for axis in self.axes.values():
            try:
                self.bus.set_torque(axis.servo_id, False)
            except Exception:
                pass
        self.enabled = False

    def close(self) -> None:
        self.torque_off()
        self.bus.close()
        self.connected = False

    def _write_targets(self, targets: dict[str, int]) -> None:
        motion = self.config.motion
        for name, axis in self.axes.items():
            sid = axis.servo_id
            goal = axis.clamp(targets[name])
            ok = self.bus.set_accel(sid, motion.acceleration)
            ok = self.bus.set_speed(sid, motion.speed) and ok
            ok = self.bus.set_goal(sid, goal) and ok
            ok = self.bus.set_torque(sid, True) and ok
            if not ok:
                self.torque_off()
                raise RuntimeError(f"write failed for {name} servo ID {sid}; torque disabled")

    def _require_connected(self) -> None:
        if not self.connected:
            raise RuntimeError("gimbal is not connected")

    def _require_enabled(self) -> None:
        self._require_connected()
        if not self.enabled:
            raise RuntimeError("gimbal torque is off; enable hold first")


def degrees_to_ticks(degrees: float) -> float:
    return float(degrees) * TICKS_PER_REVOLUTION / DEGREES_PER_REVOLUTION
