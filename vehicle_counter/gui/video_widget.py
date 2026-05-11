import cv2
import numpy as np

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QColor, QImage, QPainter, QPen, QPixmap
from PyQt6.QtWidgets import QLabel, QSizePolicy


class VideoWidget(QLabel):
    """
    Displays video frames and lets the user draw a counting line by clicking
    and dragging. The line is rendered as a QPainter overlay so it always
    appears crisp on top of the frame regardless of video resolution.
    """

    line_drawn = pyqtSignal(tuple, tuple)  # (start, end) in VIDEO pixel coords

    _LINE_COLOR = QColor(0, 220, 220)
    _LINE_WIDTH = 2

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(640, 360)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setStyleSheet("background: #111111;")
        self.setCursor(Qt.CursorShape.CrossCursor)

        self._pixmap: QPixmap | None = None
        self._video_w: int = 1
        self._video_h: int = 1

        # Line in VIDEO space
        self._line_start: tuple | None = None
        self._line_end: tuple | None = None
        self._drawing = False

        # Cached display geometry (updated in set_frame / resizeEvent)
        self._disp_x = 0
        self._disp_y = 0
        self._disp_w = 1
        self._disp_h = 1

    # ------------------------------------------------------------------
    # Frame update
    # ------------------------------------------------------------------

    def set_frame(self, frame: np.ndarray):
        """Accept a BGR numpy array and update the display."""
        h, w = frame.shape[:2]
        self._video_w = w
        self._video_h = h
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = QImage(rgb.data, w, h, w * 3, QImage.Format.Format_RGB888).copy()
        self._pixmap = QPixmap.fromImage(img)
        self._update_display_geometry()
        self.update()

    # ------------------------------------------------------------------
    # Line control
    # ------------------------------------------------------------------

    def clear_line(self):
        self._line_start = None
        self._line_end = None
        self.update()

    @property
    def has_line(self) -> bool:
        return (
            self._line_start is not None
            and self._line_end is not None
            and self._line_start != self._line_end
        )

    # ------------------------------------------------------------------
    # Painting
    # ------------------------------------------------------------------

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor("#111111"))

        if self._pixmap:
            painter.drawPixmap(self._disp_x, self._disp_y, self._pixmap.scaled(
                self._disp_w, self._disp_h,
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation,
            ))

        if self._line_start and self._line_end:
            pen = QPen(self._LINE_COLOR, self._LINE_WIDTH, Qt.PenStyle.SolidLine,
                       Qt.PenCapStyle.RoundCap)
            painter.setPen(pen)
            sx, sy = self._to_widget(*self._line_start)
            ex, ey = self._to_widget(*self._line_end)
            painter.drawLine(sx, sy, ex, ey)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._update_display_geometry()

    # ------------------------------------------------------------------
    # Mouse events — drag to draw line
    # ------------------------------------------------------------------

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            coords = self._to_video(event.pos().x(), event.pos().y())
            if coords:
                self._drawing = True
                self._line_start = coords
                self._line_end = coords

    def mouseMoveEvent(self, event):
        if self._drawing:
            coords = self._to_video(event.pos().x(), event.pos().y())
            if coords:
                self._line_end = coords
                self.update()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton and self._drawing:
            self._drawing = False
            coords = self._to_video(event.pos().x(), event.pos().y())
            if coords:
                self._line_end = coords
                self.update()
                if self.has_line:
                    self.line_drawn.emit(self._line_start, self._line_end)

    # ------------------------------------------------------------------
    # Coordinate helpers
    # ------------------------------------------------------------------

    def _update_display_geometry(self):
        """Cache the top-left offset and size of the displayed (scaled) pixmap."""
        if self._pixmap is None:
            return
        scaled = self._pixmap.scaled(
            self.width(), self.height(),
            Qt.AspectRatioMode.KeepAspectRatio,
        )
        self._disp_w = scaled.width()
        self._disp_h = scaled.height()
        self._disp_x = (self.width() - self._disp_w) // 2
        self._disp_y = (self.height() - self._disp_h) // 2

    def _to_video(self, wx: int, wy: int) -> tuple | None:
        """Widget pixel → video frame pixel. Returns None if outside frame."""
        if self._disp_w == 0 or self._disp_h == 0:
            return None
        rx = (wx - self._disp_x) / self._disp_w
        ry = (wy - self._disp_y) / self._disp_h
        if not (0.0 <= rx <= 1.0 and 0.0 <= ry <= 1.0):
            return None
        return (
            max(0, min(int(rx * self._video_w), self._video_w - 1)),
            max(0, min(int(ry * self._video_h), self._video_h - 1)),
        )

    def _to_widget(self, vx: int, vy: int) -> tuple[int, int]:
        """Video frame pixel → widget pixel."""
        wx = self._disp_x + int(vx / self._video_w * self._disp_w)
        wy = self._disp_y + int(vy / self._video_h * self._disp_h)
        return wx, wy
