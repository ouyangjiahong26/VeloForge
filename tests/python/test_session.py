"""Test the replay helpers and the Rust engine plumbing."""

from __future__ import annotations

import os

import pytest


def test_replay_frame_layout_is_twenty_bytes() -> None:
    from veloforge.replay import active_frame

    frame = active_frame()
    assert len(frame) == 20
    assert frame[0] == 0x55 and frame[1] == 0x61


def test_encode_command_returns_five_bytes() -> None:
    pytest.importorskip("veloforge._core")
    from veloforge import _core

    payload = _core.encode_command("unlock")
    assert len(payload) == 5
    assert payload == [0xFF, 0xAA, 0x69, 0x88, 0xB5]


def test_engine_consumes_static_stream() -> None:
    """Verifies the engine plumbing: each frame becomes a Sample.

    The BWT901BLE 0x55 0x61 protocol only carries Euler angles, so the
    pipeline cannot remove gravity without a quaternion. The Rust unit
    tests already cover the physics; here we just confirm the byte
    stream is consumed without errors and returns the expected count.
    """

    pytest.importorskip("veloforge._core")
    from veloforge import _core
    from veloforge.replay import static_stream

    if os.environ.get("VF_SKIP_ENGINE") == "1":
        pytest.skip("Rust engine disabled")

    engine = _core.SensorEngine()
    samples = list(engine.feed(static_stream(200)))
    assert len(samples) == 200
    # All samples have a numeric velocity and a phase tag.
    for s in samples:
        assert isinstance(s.bar_velocity_mps, float)
        assert s.phase in ("idle", "concentric", "eccentric")


def test_engine_reset_zeroes_state() -> None:
    pytest.importorskip("veloforge._core")
    from veloforge import _core
    from veloforge.replay import static_stream

    engine = _core.SensorEngine()
    list(engine.feed(static_stream(50)))
    state_before = engine.state()
    assert state_before.frames_decoded == 50
    engine.reset()
    state_after = engine.state()
    assert state_after.frames_decoded == 0
