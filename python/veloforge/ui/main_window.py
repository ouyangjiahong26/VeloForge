"""PySide6 main window for the VeloForge desktop client."""

from __future__ import annotations

from typing import Optional

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QComboBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QSpinBox,
    QStatusBar,
    QVBoxLayout,
    QWidget,
)

try:
    from veloforge import _core as _core_mod
except Exception:  # pragma: no cover
    _core_mod = None  # type: ignore[assignment]


class MainWindow(QMainWindow):
    """Minimal PySide6 shell that exercises the Rust engine via replay."""

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("VeloForge")
        self.resize(960, 600)
        self._engine: Optional[object] = None
        self._frame_count = 0

        central = QWidget(self)
        layout = QHBoxLayout(central)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.addWidget(self._build_controls(), 0)
        layout.addWidget(self._build_log(), 1)
        self.setCentralWidget(central)
        self.setStatusBar(QStatusBar(self))

        self._timer = QTimer(self)
        self._timer.setInterval(50)  # 20 Hz UI refresh
        self._timer.timeout.connect(self._tick)

    # ------------------------------------------------------------------
    def _build_controls(self) -> QGroupBox:
        box = QGroupBox("Engine")
        form = QFormLayout(box)

        self._device = QComboBox(box)
        self._device.addItem("Offline replay (synthetic)", "replay")
        form.addRow("Source", self._device)

        self._rate = QSpinBox(box)
        self._rate.setRange(1, 1000)
        self._rate.setValue(200)
        form.addRow("Nominal rate (Hz)", self._rate)

        self._gravity = QSpinBox(box)
        self._gravity.setRange(9000, 11000)
        self._gravity.setValue(9792)  # 1000 * m/s², default 9.792
        form.addRow("Gravity (×1000 m/s²)", self._gravity)

        self._connect_btn = QPushButton("Open engine", box)
        self._connect_btn.clicked.connect(self._open_engine)
        form.addRow(self._connect_btn)

        self._start_btn = QPushButton("Start", box)
        self._start_btn.setEnabled(False)
        self._start_btn.clicked.connect(self._start)
        form.addRow(self._start_btn)

        self._stop_btn = QPushButton("Stop", box)
        self._stop_btn.setEnabled(False)
        self._stop_btn.clicked.connect(self._stop)
        form.addRow(self._stop_btn)

        self._reset_btn = QPushButton("Reset", box)
        self._reset_btn.setEnabled(False)
        self._reset_btn.clicked.connect(self._reset)
        form.addRow(self._reset_btn)

        self._send_unlocked = QPushButton("Send unlock command (replay)", box)
        self._send_unlocked.setEnabled(False)
        self._send_unlocked.clicked.connect(self._send_unlock)
        form.addRow(self._send_unlocked)

        return box

    def _build_log(self) -> QGroupBox:
        box = QGroupBox("Log")
        v = QVBoxLayout(box)
        self._log = QPlainTextEdit(box)
        self._log.setReadOnly(True)
        self._log.setMaximumBlockCount(2000)
        v.addWidget(self._log, 1)
        return box

    # ------------------------------------------------------------------
    def _open_engine(self) -> None:
        if _core_mod is None:
            QMessageBox.critical(
                self,
                "VeloForge",
                "Rust extension veloforge._core is not built.\n"
                "Run `maturin develop -m crates/veloforge-py/Cargo.toml`.",
            )
            return
        self._engine = _core_mod.SensorEngine()
        self._frame_count = 0
        self._log.appendPlainText("Engine opened.")
        self._connect_btn.setEnabled(False)
        self._start_btn.setEnabled(True)
        self._reset_btn.setEnabled(True)
        self._send_unlocked.setEnabled(True)

    def _start(self) -> None:
        if self._engine is None:
            return
        self._timer.start()
        self._start_btn.setEnabled(False)
        self._stop_btn.setEnabled(True)
        self._log.appendPlainText("Acquisition started (replay loop).")

    def _stop(self) -> None:
        self._timer.stop()
        self._start_btn.setEnabled(True)
        self._stop_btn.setEnabled(False)
        self._log.appendPlainText("Acquisition stopped.")

    def _reset(self) -> None:
        if self._engine is None:
            return
        self._engine.reset()
        self._frame_count = 0
        self._log.appendPlainText("Engine reset.")

    def _send_unlock(self) -> None:
        if _core_mod is None:
            return
        payload = _core_mod.encode_command("unlock")
        self._log.appendPlainText(
            f"Unlock bytes: {payload.hex(' ')}"
        )

    def _tick(self) -> None:
        if self._engine is None:
            return
        from veloforge.replay import active_frame

        # Five 5 ms frames per tick to simulate 200 Hz.
        chunk = b"".join(active_frame(accel_raw=(0, 0, 16384)) for _ in range(5))
        samples = list(self._engine.feed(chunk))
        self._frame_count += len(samples)
        if samples:
            last = samples[-1]
            self._log.appendPlainText(
                f"v={last.bar_velocity_mps:+.3f} m/s, "
                f"phase={last.phase}, stationary={last.stationary}, "
                f"quality=0x{last.quality:04x}"
            )
        if self._frame_count % 200 == 0:
            state = self._engine.state()
            self._log.appendPlainText(
                f"frames={state.frames_decoded}, bad_prefixes={state.bad_prefixes}"
            )
