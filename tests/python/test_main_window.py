"""Headless UI smoke test: launch the main window under offscreen Qt."""

from __future__ import annotations

import os
import sys

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")


@pytest.mark.skipif(
    os.environ.get("VF_SKIP_UI") == "1",
    reason="Qt UI disabled",
)
def test_main_window_can_open_and_close() -> None:
    pytest.importorskip("PySide6")
    from PySide6.QtWidgets import QApplication

    from veloforge.ui.main_window import MainWindow

    app = QApplication.instance() or QApplication(sys.argv)
    window = MainWindow()
    window.show()
    app.processEvents()
    window.close()
    app.processEvents()
