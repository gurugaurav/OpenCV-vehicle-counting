import cv2
import numpy as np

_TRACK_COLORS: dict[int, tuple] = {}


def _track_color(track_id: int) -> tuple:
    if track_id not in _TRACK_COLORS:
        rng = np.random.default_rng(int(track_id) % 10_000)
        _TRACK_COLORS[track_id] = tuple(int(c) for c in rng.integers(80, 230, 3))
    return _TRACK_COLORS[track_id]


def draw_line(frame, start, end, color=(0, 220, 220), thickness=2):
    cv2.line(frame, (int(start[0]), int(start[1])),
             (int(end[0]), int(end[1])), color, thickness, cv2.LINE_AA)


def draw_box(frame, xyxy, track_id: int, label: str, just_crossed: bool = False):
    x1, y1, x2, y2 = (int(v) for v in xyxy)
    color = (0, 50, 255) if just_crossed else _track_color(track_id)
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
    tag = f"#{track_id} {label}"
    (tw, th), _ = cv2.getTextSize(tag, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
    cv2.rectangle(frame, (x1, y1 - th - 6), (x1 + tw + 4, y1), color, -1)
    cv2.putText(frame, tag, (x1 + 2, y1 - 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)


def draw_counts(frame, counts: dict, total: int):
    overlay = frame.copy()
    cv2.rectangle(overlay, (8, 8), (195, 98), (20, 20, 20), -1)
    cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)
    cv2.putText(frame, f"Total : {total}", (18, 32),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(frame, f"Down  : {counts['down']}", (18, 58),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (100, 200, 100), 1, cv2.LINE_AA)
    cv2.putText(frame, f"Up    : {counts['up']}", (18, 84),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (100, 100, 220), 1, cv2.LINE_AA)
