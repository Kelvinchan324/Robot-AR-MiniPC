from __future__ import annotations

from .gimbal import AxisConfig, DryRunBus, GimbalConfig, GimbalController, MotionConfig
from .pose import PoseMapper, quaternion_to_yaw_pitch, wrapped_delta_degrees


def make_config() -> GimbalConfig:
    return GimbalConfig(
        port="mock",
        baud=1_000_000,
        yaw=AxisConfig(1, 2048, 1024, 3072, 1),
        pitch=AxisConfig(2, 2048, 1536, 2560, -1),
        motion=MotionConfig(keyboard_step=40, smoothing_alpha=1.0, deadband_degrees=0.5),
    )


def test_controller_limits_and_torque() -> None:
    config = make_config()
    controller = GimbalController(config, dry_run=True)
    assert controller.connect() == {"yaw": 2048, "pitch": 2048}
    assert not controller.enabled
    controller.enable_hold()
    assert controller.nudge(yaw_ticks=10_000, pitch_ticks=-10_000) == {
        "yaw": 3072,
        "pitch": 1536,
    }
    controller.close()
    assert not controller.connected


def test_pose_mapping() -> None:
    config = make_config()
    mapper = PoseMapper(config, 2048, 2048)
    assert mapper.map(179.0, 10.0) == (2048, 2048)  # first sample calibrates
    yaw, pitch = mapper.map(-179.0, 20.0)
    assert 2069 <= yaw <= 2071  # wrapped delta is +2 degrees
    assert 1933 <= pitch <= 1935  # pitch direction is inverted
    assert wrapped_delta_degrees(-179, 179) == 2


def test_out_of_limit_start_is_rejected() -> None:
    config = make_config()
    bus = DryRunBus({1: 100, 2: 2048})
    controller = GimbalController(config, bus=bus)
    try:
        controller.connect()
    except RuntimeError as exc:
        assert "outside configured safe limits" in str(exc)
    else:
        raise AssertionError("unsafe starting position should be rejected")
    assert not controller.connected


def test_quaternion_identity() -> None:
    yaw, pitch = quaternion_to_yaw_pitch(1, 0, 0, 0)
    assert abs(yaw) < 1e-9
    assert abs(pitch) < 1e-9


if __name__ == "__main__":
    test_controller_limits_and_torque()
    test_pose_mapping()
    test_out_of_limit_start_is_rejected()
    test_quaternion_identity()
    print("gimbal tests ok")
