"""Application entry point."""

from __future__ import annotations

import os
import sys
from pathlib import Path


def main() -> int:
    """Launch the VeloForge PySide6 desktop client.

    Honors ``QT_QPA_PLATFORM=offscreen`` for headless smoke tests.
    """
    here = Path(__file__).resolve().parent
    if str(here) not in sys.path:
        sys.path.insert(0, str(here.parent))

    os.environ.setdefault("QT_LOGGING_RULES", "qt.qpa.*=false")
    from PySide6.QtWidgets import QApplication  # imported lazily for headless smoke

    from veloforge.ui.main_window import MainWindow

    app = QApplication.instance() or QApplication(sys.argv)
    app.setApplicationName("VeloForge")
    app.setOrganizationName("VeloForge")
    window = MainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
