"""
YOLO + ByteTrack vehicle counter
Usage:
    python main.py <video>  [options]
    python main.py <video> --line x1 y1 x2 y2   # skip interactive selection
    python main.py <video> --output out.mp4      # save annotated video
    python main.py <video> --config my.yaml      # custom config
"""

import argparse
import sys
from pathlib import Path

import cv2
from ultralytics import YOLO

from vehicle_counter.counter import LineCrossCounter
from vehicle_counter.visualizer import draw_box, draw_counts, draw_line
from vehicle_counter.utils import CSVLogger, InteractiveLineSelector, load_config

# Names (substrings) considered "vehicle" — works across COCO, KITTI, VisDrone, custom models
_VEHICLE_NAMES = {
    "car", "truck", "bus", "motorcycle", "van", "vehicle",
    "bicycle", "auto", "lcv", "motor", "tricycle", "tractor", "multiaxle",
}


def vehicle_classes(model) -> tuple[list[int], dict[int, str]]:
    """Return (class_id_filter, id→name map) for all vehicle-related classes in the model."""
    id_to_name = {
        idx: name
        for idx, name in model.names.items()
        if any(v in name.lower() for v in _VEHICLE_NAMES)
    }
    return list(id_to_name.keys()), id_to_name


def parse_args():
    p = argparse.ArgumentParser(description="YOLO + ByteTrack vehicle counter")
    p.add_argument("video", help="Path to input video file")
    p.add_argument("--config", default="config.yaml", help="Config YAML (default: config.yaml)")
    p.add_argument("--output", default=None, help="Save annotated video to this path")
    p.add_argument(
        "--line", nargs=4, type=int, metavar=("x1", "y1", "x2", "y2"),
        help="Counting line coords — skips interactive selection",
    )
    p.add_argument("--no-display", action="store_true", help="Run headless (no window)")
    return p.parse_args()


def resolve_line(args, cfg, video_path: str):
    """Return (line_start, line_end) from CLI args, config, or interactive selection."""
    if args.line:
        return (args.line[0], args.line[1]), (args.line[2], args.line[3])

    line_cfg = cfg.get("counting", {}).get("line", {})
    if line_cfg.get("start") and line_cfg.get("end"):
        return tuple(line_cfg["start"]), tuple(line_cfg["end"])

    # Fall back to interactive
    cap = cv2.VideoCapture(video_path)
    ret, first_frame = cap.read()
    cap.release()
    if not ret:
        sys.exit(f"[error] Cannot read video: {video_path}")

    print("[info] Draw the counting line on the video frame, then press any key.")
    selector = InteractiveLineSelector()
    return selector.select(first_frame)


def main():
    args = parse_args()
    cfg = load_config(args.config)

    model_cfg = cfg.get("model", {})
    log_cfg = cfg.get("logging", {})
    display = cfg.get("video", {}).get("display", True) and not args.no_display

    # ------------------------------------------------------------------
    # Counting line
    # ------------------------------------------------------------------
    line_start, line_end = resolve_line(args, cfg, args.video)
    print(f"[info] Counting line: {line_start} → {line_end}")

    # ------------------------------------------------------------------
    # Model
    # ------------------------------------------------------------------
    model = YOLO(model_cfg.get("weights", "yolov8m.pt"))
    cls_ids, cls_id_to_name = vehicle_classes(model)
    cfg_classes = model_cfg.get("classes")
    if cfg_classes is not None:
        cls_ids = cfg_classes
        cls_id_to_name = {i: model.names.get(i, str(i)) for i in cfg_classes}
    print(f"[info] Tracking classes: { {v: k for k, v in cls_id_to_name.items()} }")

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    counter = LineCrossCounter(line_start, line_end)
    logger = CSVLogger(log_cfg["csv_path"]) if log_cfg.get("csv_path") else None

    if log_cfg.get("save_crops"):
        Path(log_cfg.get("crops_dir", "images")).mkdir(parents=True, exist_ok=True)

    # Video writer — initialised on first frame so we know the dimensions
    writer = None

    # FPS for timestamp calculation
    cap_meta = cv2.VideoCapture(args.video)
    fps = cap_meta.get(cv2.CAP_PROP_FPS) or 30.0
    cap_meta.release()

    frame_num = 0
    crossed_ids_this_frame: set[int] = set()

    # ------------------------------------------------------------------
    # Main loop — stream results from YOLO tracker
    # ------------------------------------------------------------------
    track_kwargs = dict(
        source=args.video,
        tracker="bytetrack.yaml",
        persist=True,
        conf=model_cfg.get("confidence", 0.25),
        iou=model_cfg.get("iou", 0.45),
        imgsz=model_cfg.get("imgsz", 1280),
        classes=cls_ids,
        stream=True,
        verbose=False,
    )

    for result in model.track(**track_kwargs):
        frame = result.orig_img.copy()
        frame_num += 1
        timestamp_ms = int(frame_num / fps * 1000)
        crossed_ids_this_frame.clear()

        # -- Init video writer on first frame --
        if writer is None and args.output:
            h, w = frame.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            writer = cv2.VideoWriter(args.output, fourcc, fps, (w, h))

        active_ids: list[int] = []

        if result.boxes is not None and result.boxes.id is not None:
            ids = result.boxes.id.int().cpu().tolist()
            xyxys = result.boxes.xyxy.cpu().tolist()
            cls_ids = result.boxes.cls.int().cpu().tolist()
            confs = result.boxes.conf.cpu().tolist()

            for track_id, xyxy, cls_id, conf in zip(ids, xyxys, cls_ids, confs):
                active_ids.append(track_id)
                cls_name = cls_id_to_name.get(cls_id, model.names.get(cls_id, str(cls_id)))
                cx = (xyxy[0] + xyxy[2]) / 2
                cy = (xyxy[1] + xyxy[3]) / 2

                direction = counter.update(track_id, (cx, cy))

                if direction:
                    crossed_ids_this_frame.add(track_id)

                    if logger:
                        logger.log(timestamp_ms, track_id, cls_name, direction)

                    if log_cfg.get("save_crops"):
                        x1, y1, x2, y2 = (int(v) for v in xyxy)
                        crop = result.orig_img[max(0, y1):y2, max(0, x1):x2]
                        crop_dir = log_cfg.get("crops_dir", "images")
                        cv2.imwrite(
                            f"{crop_dir}/vehicle_{counter.total}_{cls_name}.jpg",
                            crop,
                        )

                    print(
                        f"[count] frame={frame_num} id={track_id} "
                        f"cls={cls_name} dir={direction} "
                        f"total={counter.total}"
                    )

                draw_box(frame, xyxy, track_id, cls_name,
                         just_crossed=(track_id in crossed_ids_this_frame))

            counter.remove_stale(active_ids)

        # -- Overlay --
        line_color = (0, 80, 255) if crossed_ids_this_frame else (0, 220, 220)
        draw_line(frame, line_start, line_end, line_color, thickness=2)
        draw_counts(frame, counter.counts, counter.total)

        if writer:
            writer.write(frame)

        if display:
            cv2.imshow("Vehicle Counter  [q to quit]", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------
    if writer:
        writer.release()
    cv2.destroyAllWindows()

    print(
        f"\n[done] Total={counter.total}  "
        f"Down={counter.counts['down']}  Up={counter.counts['up']}"
    )


if __name__ == "__main__":
    main()
