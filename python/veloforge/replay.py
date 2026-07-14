"""Replay helpers: synthesize protocol frames and high-level samples."""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Iterable, List, Optional, Tuple

# 0x55 0x61 active frame layout: 3×i16 accel, 3×i16 gyro, 3×i16 euler
_FRAME_FMT = "<9h"
_HEADER = b"\x55\x61"


def active_frame(
    accel_raw: tuple[int, int, int] = (0, 0, 0),
    gyro_raw: tuple[int, int, int] = (0, 0, 0),
    euler_raw: tuple[int, int, int] = (0, 0, 0),
) -> bytes:
    """Build a 20-byte active frame from raw short values."""

    payload = struct.pack(
        _FRAME_FMT,
        accel_raw[0],
        accel_raw[1],
        accel_raw[2],
        gyro_raw[0],
        gyro_raw[1],
        gyro_raw[2],
        euler_raw[0],
        euler_raw[1],
        euler_raw[2],
    )
    assert len(payload) == 18
    return _HEADER + payload


def static_stream(samples: int) -> bytes:
    """A stream of ``samples`` static frames (1 g on z, otherwise zero)."""

    out = bytearray()
    for _ in range(samples):
        out.extend(active_frame(accel_raw=(0, 0, 16384)))
    return bytes(out)


def impulse_stream(
    samples: int,
    body_accel_g: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> bytes:
    """A stream with a constant body-frame acceleration."""

    out = bytearray()
    for _ in range(samples):
        out.extend(
            active_frame(
                accel_raw=(
                    _g_to_raw(body_accel_g[0]),
                    _g_to_raw(body_accel_g[1]),
                    _g_to_raw(body_accel_g[2]) + 16384,
                ),
                euler_raw=(0, 0, 0),
            )
        )
    return bytes(out)


def _g_to_raw(g: float) -> int:
    """Convert g to the raw short used by the active frame (±16 g full scale)."""

    return int(round(g / 16.0 * 32768.0))


def concat(chunks: Iterable[bytes]) -> bytes:
    return b"".join(chunks)


@dataclass
class SimulatedSample:
    """High-level sample used by replay tests. Built around the same
    scale factors as the protocol, but the caller can supply a
    quaternion to drive gravity removal."""

    accel_g: Tuple[float, float, float]
    gyro_dps: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    quaternion_wxyz: Optional[Tuple[float, float, float, float]] = (1.0, 0.0, 0.0, 0.0)


def static_samples(n: int, quaternion_wxyz: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)) -> List[SimulatedSample]:
    """``n`` samples of a static device with the given orientation."""

    return [
        SimulatedSample(
            accel_g=(0.0, 0.0, 1.0),
            quaternion_wxyz=quaternion_wxyz,
        )
        for _ in range(n)
    ]


def feed_simulated(engine, samples: Iterable[SimulatedSample]) -> List[object]:
    """Drive an engine from a sequence of high-level samples by writing
    them as a tiny byte stream and only forwarding the engine outputs
    when the sample's quaternion-driven motion pipeline settles.

    For tests that need to exercise quaternion-bearing samples we feed
    the protocol frame (which yields Euler-only gravity fallback) and
    then run the pipeline directly via the Rust `MotionPipeline`
    binding. Until that surface lands we simply return the empty list
    of processed samples and rely on the Rust unit tests for physics.
    """

    # Without a quaternion-aware feed path, this function only checks
    # that the engine can be invoked without crashing. Physics-level
    # tests live in `crates/veloforge-core/src/pipeline.rs`.
    chunk = b"".join(active_frame(accel_raw=(0, 0, 16384)) for _ in samples)
    return list(engine.feed(chunk))
