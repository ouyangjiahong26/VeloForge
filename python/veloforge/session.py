"""Lightweight session used by tests and the replay UI."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable, List, Optional

try:  # The Rust extension is built by maturin; tests fall back to mock.
    from veloforge import _core as _core_mod
except Exception:  # pragma: no cover - extension may be missing during lint.
    _core_mod = None  # type: ignore[assignment]


@dataclass
class SessionConfig:
    """Configuration shared by replay and live sessions."""

    nominal_rate_hz: float = 200.0
    gravity_mps2: float = 9.7915
    body_to_bar_axis: tuple[float, float, float] = (0.0, 0.0, 1.0)


@dataclass
class Session:
    """Owns a Rust sensor engine and the most recent samples."""

    engine: object
    config: SessionConfig = field(default_factory=SessionConfig)
    samples: List[object] = field(default_factory=list)
    last_state: Optional[object] = None

    def feed(self, chunk: bytes) -> List[object]:
        samples = list(self.engine.feed(chunk))  # type: ignore[attr-defined]
        self.samples.extend(samples)
        self.last_state = self.engine.state()  # type: ignore[attr-defined]
        return samples

    def reset(self) -> None:
        self.engine.reset()  # type: ignore[attr-defined]
        self.samples.clear()
        self.last_state = None


def make_engine() -> object:
    """Create a Rust engine. The factory is split out for easy mocking."""

    if _core_mod is None:  # pragma: no cover
        raise RuntimeError("veloforge._core extension is not built")
    return _core_mod.SensorEngine()
