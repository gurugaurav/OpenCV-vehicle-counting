import cv2
import numpy as np
from pathlib import Path

from PyQt6.QtCore import QMutex, QThread, pyqtSignal
from ultralytics import YOLO

from vehicle_counter.counter import LineCrossCounter
from vehicle_counter.utils import CSVLogger
from vehicle_counter.visualizer import draw_box, draw_counts

_VEHICLE_NAMES = {
    "car", "truck", "bus", "motorcycle", "van", "vehicle",
    "bicycle", "auto", "lcv", "motor", "tricycle", "tractor", "multiaxle",
}


def _vehicle_classes(model):
    """Return (class_id_list, id→name dict) for all vehicle-related classes in the model."""
    id_to_name = {
        idx: name
        for idx, name in model.names.items()
        if any(v in name.lower() for v in _VEHICLE_NAMES)
    }
    return list(id_to_name.keys()), id_to_name


class WorkerThread(QThread):
    """
    Runs YOLO + ByteTrack in a background thread.
    Emits annotated frames and count updates via Qt signals.
    The counting line is NOT drawn here — VideoWidget draws it as an overlay.
    """

    frame_ready = pyqtSignal(object)   # np.ndarray  (BGR, annotated)
    count_updated = pyqtSignal(object) # dict: total / up / down / classes
    finished = pyqtSignal(object, int) # final counts dict, total int

    def __init__(self, video_path: str, line_start: tuple, line_end: tuple,
                 config: dict, parent=None):
        super().__init__(parent)
        self.video_path = video_path
        self.line_start = line_start
        self.line_end = line_end
        self.config = config
        self._mutex = QMutex()
        self._stop_flag = False

    # ------------------------------------------------------------------
    def stop(self):
        self._mutex.lock()
        self._stop_flag = True
        self._mutex.unlock()

    # ------------------------------------------------------------------
    def run(self):
        model_cfg = self.config.get("model", {})
        log_cfg = self.config.get("logging", {})

        model = YOLO(model_cfg.get("weights", "yolov8m.pt"))
        cls_ids, cls_id_to_name = _vehicle_classes(model)
        cfg_classes = model_cfg.get("classes")
        if cfg_classes is not None:
            cls_ids = cfg_classes
            cls_id_to_name = {i: model.names.get(i, str(i)) for i in cfg_classes}

        counter = LineCrossCounter(self.line_start, self.line_end)
        class_counts: dict[str, int] = {}

        logger = CSVLogger(log_cfg["csv_path"]) if log_cfg.get("csv_path") else None

        if log_cfg.get("save_crops"):
            Path(log_cfg.get("crops_dir", "images")).mkdir(parents=True, exist_ok=True)

        cap = cv2.VideoCapture(self.video_path)
        fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        cap.release()

        frame_num = 0

        for result in model.track(
            source=self.video_path,
            tracker="bytetrack.yaml",
            persist=True,
            conf=model_cfg.get("confidence", 0.25),
            iou=model_cfg.get("iou", 0.45),
            imgsz=model_cfg.get("imgsz", 1280),
            classes=cls_ids,
            stream=True,
            verbose=False,
        ):
            self._mutex.lock()
            should_stop = self._stop_flag
            self._mutex.unlock()
            if should_stop:
                break

            frame = result.orig_img.copy()
            frame_num += 1
            timestamp_ms = int(frame_num / fps * 1000)
            crossed_ids: set[int] = set()

            if result.boxes is not None and result.boxes.id is not None:
                ids = result.boxes.id.int().cpu().tolist()
                xyxys = result.boxes.xyxy.cpu().tolist()
                cls_ids = result.boxes.cls.int().cpu().tolist()
                active_ids: list[int] = []

                for track_id, xyxy, cls_id in zip(ids, xyxys, cls_ids):
                    active_ids.append(track_id)
                    cls_name = cls_id_to_name.get(cls_id, model.names.get(cls_id, str(cls_id)))
                    cx = (xyxy[0] + xyxy[2]) / 2
                    cy = (xyxy[1] + xyxy[3]) / 2

                    direction = counter.update(track_id, (cx, cy))

                    if direction:
                        crossed_ids.add(track_id)
                        class_counts[cls_name] = class_counts.get(cls_name, 0) + 1
                        if logger:
                            logger.log(timestamp_ms, track_id, cls_name, direction)
                        if log_cfg.get("save_crops"):
                            x1, y1, x2, y2 = (int(v) for v in xyxy)
                            crop = result.orig_img[max(0, y1):y2, max(0, x1):x2]
                            crops_dir = log_cfg.get("crops_dir", "images")
                            cv2.imwrite(
                                f"{crops_dir}/vehicle_{counter.total}_{cls_name}.jpg",
                                crop,
                            )

                    draw_box(frame, xyxy, track_id, cls_name,
                             just_crossed=(track_id in crossed_ids))

                counter.remove_stale(active_ids)

            draw_counts(frame, counter.counts, counter.total)

            self.frame_ready.emit(frame)
            self.count_updated.emit({
                "total": counter.total,
                "up": counter.counts["up"],
                "down": counter.counts["down"],
                "classes": dict(class_counts),
            })

        self.finished.emit(
            {"up": counter.counts["up"], "down": counter.counts["down"]},
            counter.total,
        )
