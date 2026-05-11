"""
PyQt6 desktop GUI for vehicle counting.

Usage:
    python app.py
    python app.py --config my_config.yaml
"""

import argparse
import sys

from PyQt6.QtCore import Qt
from PyQt6.QtGui import QColor, QPalette
from PyQt6.QtWidgets import QApplication

from vehicle_counter.gui.main_window import MainWindow


def _dark_palette() -> QPalette:
    p = QPalette()
    p.setColor(QPalette.ColorRole.Window,          QColor(28, 28, 28))
    p.setColor(QPalette.ColorRole.WindowText,      QColor(220, 220, 220))
    p.setColor(QPalette.ColorRole.Base,            QColor(20, 20, 20))
    p.setColor(QPalette.ColorRole.AlternateBase,   QColor(35, 35, 35))
    p.setColor(QPalette.ColorRole.Text,            QColor(220, 220, 220))
    p.setColor(QPalette.ColorRole.Button,          QColor(45, 45, 45))
    p.setColor(QPalette.ColorRole.ButtonText,      QColor(220, 220, 220))
    p.setColor(QPalette.ColorRole.Highlight,       QColor(42, 130, 218))
    p.setColor(QPalette.ColorRole.HighlightedText, QColor(0, 0, 0))
    p.setColor(QPalette.ColorRole.ToolTipBase,     QColor(50, 50, 50))
    p.setColor(QPalette.ColorRole.ToolTipText,     QColor(220, 220, 220))
    p.setColor(QPalette.ColorRole.Link,            QColor(80, 160, 240))
    return p


def main():
    parser = argparse.ArgumentParser(description="Vehicle Counter GUI")
    parser.add_argument("--config", default="config.yaml",
                        help="Path to config YAML (default: config.yaml)")
    args = parser.parse_args()

    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setPalette(_dark_palette())
    app.setStyleSheet("""
        QToolBar { background: #242424; border-bottom: 1px solid #333; }
        QStatusBar { background: #1a1a1a; color: #888888; font-size: 11px; }
        QToolBar QToolButton {
            color: #dddddd;
            padding: 4px 10px;
            border-radius: 4px;
        }
        QToolBar QToolButton:hover  { background: #383838; }
        QToolBar QToolButton:pressed { background: #2a6496; }
        QToolBar QToolButton:disabled { color: #555555; }
    """)

    window = MainWindow(config_path=args.config)
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
