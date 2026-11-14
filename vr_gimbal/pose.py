"""Map relative VR head orientation to bounded STS3215 targets."""

from __future__ import annotations

from dataclasses import dataclass
import math

from .gimbal import GimbalConfig, degrees_to_ticks


def wrapped_delta_degrees(value: float, origin: float) -> float:
    """Return the shortest signed angular difference in [-180, 180)."""
    return (float(value) - float(origin) + 180.0) % 360.0 - 180.0


def quaternion_to_yaw_pitch(w: float, x: float, y: float, z: float) -> tuple[float, float]:
    """Convert a normalized quaternion to yaw and pitch in degrees.

    This uses the common Z-yaw, Y-pitch convention. A VR-specific adapter may
    need to swap or invert axes before feeding the result to PoseMapper.
    """
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 1e-9:
        raise ValueError("zero-length quaternion")
    w, x, y, z = (v / norm for v in (w, x, y, z))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    pitch_term = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(pitch_term)
    return math.degrees(yaw), math.degrees(pitch)


@dataclass
class PoseMapper:
    config: GimbalConfig
    servo_zero_yaw: int
    servo_zero_pitch: int
    input_zero_yaw: float | None = None
    input_zero_pitch: float | None = None
    output_yaw: float | None = None
    output_pitch: float | None = None

    @property
    def calibrated(self) -> bool:
        return self.input_zero_yaw is not None and self.input_zero_pitch is not None

    def calibrate(self, yaw_degrees: float, pitch_degrees: float) -> None:
        self.input_zero_yaw = float(yaw_degrees)
        self.input_zero_pitch = float(pitch_degrees)
        self.output_yaw = float(self.servo_zero_yaw)
        self.output_pitch = float(self.servo_zero_pitch)

    def map(self, yaw_degrees: float, pitch_degrees: float) -> tuple[int, int]:
        if not self.calibrated:
            self.calibrate(yaw_degrees, pitch_degrees)

        assert self.input_zero_yaw is not None
        assert self.input_zero_pitch is not None
        yaw_delta = wrapped_delta_degrees(yaw_degrees, self.input_zero_yaw)
        pitch_delta = wrapped_delta_degrees(pitch_degrees, self.input_zero_pitch)
        deadband = self.config.motion.deadband_degrees
        if abs(yaw_delta) < deadband:
            yaw_delta = 0.0
        if abs(pitch_delta) < deadband:
            pitch_delta = 0.0

        gain = self.config.motion.vr_gain
        desired_yaw = self.servo_zero_yaw + (
            self.config.yaw.direction * degrees_to_ticks(yaw_delta) * gain
        )
        desired_pitch = self.servo_zero_pitch + (
            self.config.pitch.direction * degrees_to_ticks(pitch_delta) * gain
        )

        alpha = self.config.motion.smoothing_alpha
        if self.output_yaw is None or self.output_pitch is None:
            self.output_yaw = desired_yaw
            self.output_pitch = desired_pitch
        else:
            self.output_yaw += alpha * (desired_yaw - self.output_yaw)
            self.output_pitch += alpha * (desired_pitch - self.output_pitch)

        return (
            self.config.yaw.clamp(self.output_yaw),
            self.config.pitch.clamp(self.output_pitch),
        )
