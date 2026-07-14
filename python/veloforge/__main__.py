"""Command-line entry point that delegates to :mod:`veloforge.app`."""

from __future__ import annotations

from veloforge.app import main

if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
