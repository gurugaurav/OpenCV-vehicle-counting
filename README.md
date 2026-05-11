# OpenCV Vehicle Counting

A Python vehicle counting system using **YOLOv8 + ByteTrack**. Detects and counts cars, trucks, buses, and motorcycles crossing a user-defined line in any video — with a PyQt6 desktop GUI and a headless CLI mode.

---

## How it works

```
Video → YOLOv8 detection → ByteTrack (persistent IDs) → Line-crossing counter → Display / CSV log
```

- **YOLOv8** detects vehicles frame-by-frame (car, truck, bus, motorcycle)
- **ByteTrack** assigns a persistent ID to each vehicle across frames
- **Line-crossing logic** uses segment-intersection geometry — each ID is counted exactly once per direction, no debounce needed
- Counts split by direction (**up / down**) and by vehicle class

---

## Demo

[![Highway 401 demo](https://img.youtube.com/vi/XszBqvQ2XCg/0.jpg)](https://www.youtube.com/watch?v=XszBqvQ2XCg)

---

## Requirements

- Python 3.11+
- macOS / Linux (Windows untested)

---

## Installation

```bash
git clone https://github.com/your-username/OpenCV-vehicle-counting.git
cd OpenCV-vehicle-counting

python3 -m venv .venv
source .venv/bin/activate        # Windows: .venv\Scripts\activate

pip install -r requirements.txt
```

Model weights (`yolov8m.pt`) are downloaded automatically on first run.

---

## Usage

### Desktop GUI

```bash
python app.py
```

1. Click **Open Video** and select your video file
2. **Click and drag** on the video frame to draw the counting line
3. Click **▶ Start** — live counts appear in the right panel
4. Click **■ Stop** at any time, or let it run to the end

---

### CLI

```bash
# Interactive — a window opens, drag to draw the line, press any key to start
python main.py video.mp4

# Skip interactive — pass line coordinates directly
python main.py video.mp4 --line x1 y1 x2 y2

# Save annotated output video
python main.py video.mp4 --line 0 400 1280 400 --output result.mp4

# Headless (no display window) — for servers / batch processing
python main.py video.mp4 --line 0 400 1280 400 --no-display --output result.mp4

# Custom config file
python main.py video.mp4 --config my_config.yaml
```

Terminal output per crossing event:

```
[count] frame=365 id=39 cls=car dir=down total=6
[count] frame=476 id=67 cls=bus dir=up   total=7
...
[done] Total=183  Down=113  Up=70
```

---

## Configuration

All parameters live in `config.yaml`:

```yaml
model:
  weights: yolov8m.pt     # yolov8n (fast) → yolov8s → yolov8m → yolov8l → yolov8x (accurate)
  confidence: 0.25        # lower = more detections, higher = fewer false positives
  iou: 0.45
  imgsz: 1280             # match your video resolution for best accuracy
  classes: null           # null = auto-detect vehicle classes from model names

counting:
  line:
    start: null           # set [x, y] to skip interactive drawing
    end:   null

logging:
  csv_path: counts.csv    # one row per crossing event; set null to disable
  save_crops: false       # save a cropped image of each counted vehicle
  crops_dir: images
```

### Model options

Any YOLOv8 `.pt` file works — point `weights:` at it. The pipeline auto-detects vehicle classes from the model's label names, so custom or fine-tuned models work without any code changes.

| Model | Speed | Accuracy | Best for |
|---|---|---|---|
| `yolov8n.pt` | Fastest | Low | Real-time on CPU |
| `yolov8s.pt` | Fast | Medium | Balanced |
| `yolov8m.pt` | Moderate | High | **Default — recommended** |
| `yolov8l.pt` | Slow | Higher | GPU available |
| `yolov8x.pt` | Slowest | Highest | Maximum accuracy |

---

## Output

**CSV log** (`counts.csv`) — one row per vehicle crossing:

| timestamp_ms | track_id | class | direction |
|---|---|---|---|
| 4594 | 15 | car | down |
| 6013 | 22 | car | down |
| 16149 | 67 | bus | up |

**Cropped images** — set `save_crops: true` in config to save a JPEG of each counted vehicle to the `images/` folder.

---

## Project structure

```
.
├── app.py                    # PyQt6 GUI entry point
├── main.py                   # CLI entry point
├── config.yaml               # All tuning parameters
├── requirements.txt
└── vehicle_counter/
    ├── counter.py            # LineCrossCounter — segment-intersection geometry
    ├── visualizer.py         # Drawing helpers (boxes, line, count overlay)
    ├── utils.py              # Config loader, interactive line selector, CSV logger
    └── gui/
        ├── worker.py         # QThread — runs YOLO+ByteTrack off the main thread
        ├── video_widget.py   # QLabel subclass — frame display + mouse line drawing
        ├── stats_panel.py    # Right-side live count panel
        └── main_window.py    # QMainWindow — toolbar, layout, wiring
```

