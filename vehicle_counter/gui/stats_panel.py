from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import QFrame, QLabel, QSizePolicy, QVBoxLayout, QWidget


def _separator() -> QFrame:
    line = QFrame()
    line.setFrameShape(QFrame.Shape.HLine)
    line.setStyleSheet("color: #3a3a3a; margin: 4px 0;")
    return line


def _header(text: str) -> QLabel:
    lbl = QLabel(text)
    lbl.setFont(QFont("Arial", 11, QFont.Weight.Bold))
    lbl.setStyleSheet("color: #cccccc;")
    return lbl


class StatsPanel(QWidget):
    """Right-side panel showing live vehicle counts."""

    _SUMMARY_STYLE = "font-size: 15px; padding: 2px 0;"
    _CLASS_STYLE = "font-size: 12px; color: #999999; padding: 1px 0;"

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedWidth(200)
        self.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Expanding)
        self.setStyleSheet("background: #1c1c1c;")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 16, 14, 16)
        layout.setSpacing(6)
        layout.setAlignment(Qt.AlignmentFlag.AlignTop)

        layout.addWidget(_header("Counts"))
        layout.addWidget(_separator())

        self._lbl: dict[str, QLabel] = {}

        for key, text, color in [
            ("total", "Total",    "#ffffff"),
            ("down",  "Down  ↓",  "#5dbb5d"),
            ("up",    "Up    ↑",  "#6a84d6"),
        ]:
            lbl = QLabel(f"{text}:  0")
            lbl.setStyleSheet(f"color: {color}; {self._SUMMARY_STYLE}")
            self._lbl[key] = lbl
            layout.addWidget(lbl)

        layout.addSpacing(8)
        layout.addWidget(_header("By class"))
        layout.addWidget(_separator())

        for cls in ("car", "motorcycle", "bus", "truck"):
            lbl = QLabel(f"{cls:<12}0")
            lbl.setStyleSheet(self._CLASS_STYLE)
            lbl.setFont(QFont("Courier", 11))
            self._lbl[f"cls_{cls}"] = lbl
            layout.addWidget(lbl)

        layout.addStretch()

    # ------------------------------------------------------------------

    def update_counts(self, counts: dict):
        self._lbl["total"].setText(f"Total:  {counts.get('total', 0)}")
        self._lbl["down"].setText(f"Down  ↓:  {counts.get('down', 0)}")
        self._lbl["up"].setText(f"Up    ↑:  {counts.get('up', 0)}")
        classes = counts.get("classes", {})
        for cls in ("car", "motorcycle", "bus", "truck"):
            self._lbl[f"cls_{cls}"].setText(f"{cls:<12}{classes.get(cls, 0)}")

    def reset(self):
        self.update_counts({"total": 0, "up": 0, "down": 0, "classes": {}})
