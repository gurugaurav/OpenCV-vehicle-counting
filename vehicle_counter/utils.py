import csv
import os
import cv2
import yaml
from pathlib import Path


def load_config(path: str) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


# ---------------------------------------------------------------------------
# Interactive line selector
# ---------------------------------------------------------------------------

class InteractiveLineSelector:
    """
    Opens the given frame in a window and lets the user drag a counting line.
    Left-click drag → sets the line. Press any key to confirm.
    """

    def __init__(self):
        self._start = None
        self._end = None
        self._drawing = False
        self._base: cv2.typing.MatLike | None = None

    def _on_mouse(self, event, x, y, _flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self._drawing = True
            self._start = (x, y)
            self._end = (x, y)

        elif event == cv2.EVENT_MOUSEMOVE and self._drawing:
            self._end = (x, y)
            preview = self._base.copy()
            cv2.line(preview, self._start, self._end, (0, 220, 220), 2, cv2.LINE_AA)
            cv2.imshow(self._WIN, preview)

        elif event == cv2.EVENT_LBUTTONUP:
            self._drawing = False
            self._end = (x, y)
            preview = self._base.copy()
            cv2.line(preview, self._start, self._end, (0, 220, 220), 2, cv2.LINE_AA)
            cv2.imshow(self._WIN, preview)

    _WIN = "Draw counting line — drag, then press any key"

    def select(self, frame) -> tuple[tuple, tuple]:
        self._base = frame.copy()
        cv2.namedWindow(self._WIN, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self._WIN, self._on_mouse)
        cv2.imshow(self._WIN, self._base)

        while True:
            key = cv2.waitKey(20) & 0xFF
            if key != 255 and self._start and self._end and self._start != self._end:
                break

        cv2.destroyWindow(self._WIN)
        return self._start, self._end


# ---------------------------------------------------------------------------
# CSV event logger
# ---------------------------------------------------------------------------

class CSVLogger:
    """Appends one row per vehicle crossing event."""

    _HEADER = ["timestamp_ms", "track_id", "class", "direction"]

    def __init__(self, path: str):
        self.path = path
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w", newline="") as f:
            csv.writer(f).writerow(self._HEADER)

    def log(self, timestamp_ms: int, track_id: int, cls_name: str, direction: str):
        with open(self.path, "a", newline="") as f:
            csv.writer(f).writerow([timestamp_ms, track_id, cls_name, direction])
