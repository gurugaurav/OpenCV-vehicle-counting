# Real-Time Vehicle Counter — YOLOv8 + ByteTrack

[![Python](https://img.shields.io/badge/Python-3.11%2B-blue?logo=python)](https://www.python.org/)
[![YOLOv8](https://img.shields.io/badge/YOLOv8-Ultralytics-purple)](https://github.com/ultralytics/ultralytics)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](colab_vehicle_counter.ipynb)

Accurate, real-time vehicle counting from **video files**, **RTSP / IP cameras**, and **YouTube live streams**. Uses YOLOv8 for detection and ByteTrack for multi-object tracking. Each vehicle is counted exactly once as it crosses a user-defined line — split by direction (in / out) and by class (car, truck, bus, motorcycle).

---

## Features

- **YOLOv8 detection** — supports all model sizes (n / s / m / l / x) and custom-trained weights
- **ByteTrack multi-object tracking** — stable IDs across occlusions and crowded scenes
- **Directional line counting** — angled lines supported; counts split into Up / Down (or In / Out)
- **Live stream support** — RTSP/RTMP IP cameras and YouTube live streams with auto-reconnect
- **CSV event log** — one row per crossing: timestamp, track ID, vehicle class, direction
- **Annotated video output** — save the full tracked-and-counted video as MP4
- **Vehicle crop saving** — optional JPEG crop of each counted vehicle
- **Desktop GUI** — PyQt6 app with interactive line drawing, live stats panel
- **Google Colab notebook** — GPU-accelerated cloud processing, no local GPU needed
- **Headless / server mode** — `--no-display` for running on remote machines

---

## Demo

[![Highway 401 demo](https://img.youtube.com/vi/XszBqvQ2XCg/0.jpg)](https://www.youtube.com/watch?v=XszBqvQ2XCg)

```
[count] frame=365  id=39  cls=car    dir=down  total=6
[count] frame=476  id=67  cls=bus    dir=up    total=7
...
[done]  Total=183  Down=113  Up=70
```

---

## Hardware Requirements

### Minimum (CPU-only)

| Component | Requirement |
|-----------|-------------|
| CPU | 4-core x86-64 (Intel i5 / AMD Ryzen 5 or equivalent) |
| RAM | 4 GB |
| Storage | 2 GB free (model weights + deps) |
| OS | macOS 12+, Ubuntu 20.04+, Windows 10+ |
| Python | 3.11+ |

> With `yolov8n.pt` at `imgsz: 640`, CPU-only reaches **8–12 FPS** on 1080p video — usable for offline processing.

### Recommended (GPU)

| Component | Requirement |
|-----------|-------------|
| GPU | NVIDIA with **4 GB+ VRAM** (GTX 1660 / RTX 3060 or better) |
| RAM | 8 GB+ |
| CUDA | 11.8 or 12.x |
| Python | 3.11+ |

> `yolov8m.pt` at `imgsz: 1280` on an RTX 3060 runs at **25–35 FPS** on 1080p — true real-time.

### Google Colab (free)

| Tier | GPU | Notes |
|------|-----|-------|
| Free | T4 (16 GB VRAM) | More than enough for all model sizes |
| Pro | A100 (40 GB VRAM) | Useful for batch processing long videos |

See [Running on Google Colab](#google-colab) below.

---

## Installation

```bash
git clone https://github.com/gurugaurav/OpenCV-vehicle-counting.git
cd OpenCV-vehicle-counting

python3 -m venv .venv
source .venv/bin/activate        # Windows: .venv\Scripts\activate

pip install -r requirements.txt
```

Model weights are downloaded automatically on first run. To use a specific size:

```bash
pip install ultralytics
yolo export model=yolov8m.pt     # or n / s / l / x
```

For **YouTube live stream** support, also install:

```bash
pip install yt-dlp
```

---

## Quick Start

```bash
# Video file — interactive line drawing
python main.py traffic.mp4

# Video file — line pre-defined, save output
python main.py traffic.mp4 --line 0 400 1280 400 --output result.mp4

# RTSP IP camera — headless with auto-reconnect
python main.py rtsp://admin:pass@192.168.1.100:554/stream \
    --line 0 360 1280 360 --no-display --reconnect

# YouTube live stream
python main.py "https://www.youtube.com/watch?v=LIVE_ID" \
    --line 0 360 1280 360 --no-display
```

---

## CLI Reference

```
python main.py <source> [options]
```

| Argument | Description |
|----------|-------------|
| `source` | Video file path, `rtsp://` URL, or YouTube URL |
| `--line x1 y1 x2 y2` | Counting line pixel coords — skips interactive drawing |
| `--output path.mp4` | Save annotated video to file |
| `--config path.yaml` | Custom config file (default: `config.yaml`) |
| `--no-display` | Headless mode — no window, for servers / Colab |
| `--reconnect` | Auto-reconnect if a live stream drops |

### Live stream examples

```bash
# RTSP — Hikvision / Dahua / generic IP camera
python main.py rtsp://admin:password@192.168.1.64:554/Streaming/Channels/101 \
    --line 100 540 1820 540 --no-display --reconnect --output highway.mp4

# YouTube live traffic camera
python main.py "https://www.youtube.com/watch?v=rnCnEWoGsAo" \
    --line 0 400 1280 400 --no-display

# HTTP MJPEG stream
python main.py http://192.168.1.100:8080/video \
    --line 0 360 1280 360
```

---

## Desktop GUI

```bash
python app.py
```

1. Click **Open Video** — select a video file or paste an RTSP URL
2. **Click and drag** on the preview frame to draw the counting line
3. Click **▶ Start** — live counts update in the right panel in real time
4. Click **■ Stop** at any time; counts are preserved

---

## Google Colab

Open [`colab_vehicle_counter.ipynb`](colab_vehicle_counter.ipynb) and select **Runtime → Change runtime type → T4 GPU**.

The notebook covers:
1. Install all dependencies (including `opencv-python-headless`)
2. Upload your project zip and video
3. Preview the first frame with a pixel grid to choose line coordinates
4. Run the counter headlessly and display annotated frames inline
5. View the CSV summary and download outputs

For **live streams in Colab**, cells L1–L4 let you connect to an RTSP camera or YouTube live stream and view the annotated feed inside the notebook using an `ipywidgets` display updated in real time.

---

## Configuration

All parameters are in `config.yaml`:

```yaml
model:
  weights: yolov8m.pt     # yolov8n (fastest/CPU) → yolov8x (most accurate/GPU)
  confidence: 0.25        # lower = more detections; higher = fewer false positives
  iou: 0.45               # overlap threshold for NMS
  imgsz: 1280             # 640 for CPU / real-time; 1280 for accuracy on GPU
  classes: null           # null = auto-detect from model; or e.g. [2,3,5,7] for COCO IDs

counting:
  line:
    start: null           # e.g. [0, 400] — null triggers interactive drawing
    end:   null           # e.g. [1280, 400]

logging:
  csv_path: counts.csv    # one row per crossing event; set null to disable
  save_crops: false       # save a cropped JPEG of each counted vehicle
  crops_dir: images
```

### Model size guide

| Model | Speed (CPU) | Speed (GPU) | Accuracy | Use case |
|-------|-------------|-------------|----------|----------|
| `yolov8n.pt` | ~10 FPS | ~80 FPS | Low | Real-time on CPU, embedded |
| `yolov8s.pt` | ~6 FPS | ~60 FPS | Medium | Balanced |
| `yolov8m.pt` | ~3 FPS | ~35 FPS | High | **Default — recommended** |
| `yolov8l.pt` | ~2 FPS | ~20 FPS | Higher | High-accuracy with GPU |
| `yolov8x.pt` | ~1 FPS | ~12 FPS | Highest | Maximum accuracy, GPU required |

> Speeds measured on RTX 3060 (GPU) and Intel i7-12700K (CPU) at `imgsz=640`, 1080p input.

---

## Output

**`counts.csv`** — one row per crossing event:

| timestamp_ms | track_id | class | direction |
|---|---|---|---|
| 4594 | 15 | car | down |
| 6013 | 22 | car | down |
| 16149 | 67 | bus | up |

**Annotated video** — bounding boxes with track IDs, colored by class, flashing red on crossing. Count overlay in top-left.

**Vehicle crops** — set `save_crops: true` to save `images/vehicle_<n>_<class>.jpg` for each crossing event.

---

## How it Works

```
Source (file / RTSP / YouTube)
    ↓
YOLOv8 — per-frame detection (car, truck, bus, motorcycle, ...)
    ↓
ByteTrack — assigns persistent IDs across frames, handles occlusions
    ↓
LineCrossCounter — segment-intersection geometry
    each ID counted once per direction; angled lines supported
    ↓
Display / CSV log / Annotated video
```

The counting line can be **placed at any angle**. Direction is determined by which side of the directed line (p1 → p2) the vehicle centroid moves from — making "Up" and "Down" meaningful even for diagonal or vertical lines.

---

## Project Structure

```
.
├── main.py                       # CLI entry point (file, RTSP, YouTube)
├── app.py                        # PyQt6 desktop GUI entry point
├── config.yaml                   # All tuning parameters
├── requirements.txt
├── colab_vehicle_counter.ipynb   # Google Colab notebook
├── make_test_video.py            # Generate a synthetic test video
├── test_counter.py               # Unit tests for LineCrossCounter
└── vehicle_counter/
    ├── counter.py                # LineCrossCounter — segment-intersection geometry
    ├── visualizer.py             # Drawing helpers (boxes, line, count overlay)
    ├── stream.py                 # Stream resolution (RTSP passthrough, YouTube → CDN URL)
    ├── utils.py                  # Config loader, interactive line selector, CSV logger
    └── gui/
        ├── worker.py             # QThread — YOLO+ByteTrack off the main thread
        ├── video_widget.py       # QLabel — frame display + mouse line drawing
        ├── stats_panel.py        # Live count panel
        └── main_window.py        # QMainWindow — layout and wiring
```

---

## Supported Stream Formats

| Format | Example | Notes |
|--------|---------|-------|
| Video file | `traffic.mp4`, `.avi`, `.mkv` | Any format OpenCV supports |
| RTSP | `rtsp://user:pass@ip:554/stream` | Hikvision, Dahua, Axis, generic |
| RTMP | `rtmp://live.example.com/stream` | Streaming servers |
| HTTP MJPEG | `http://ip:8080/video` | Many budget IP cameras |
| YouTube live | `https://youtube.com/watch?v=...` | Requires `pip install yt-dlp` |

---

## Contributing

Pull requests are welcome. For major changes, please open an issue first.

```bash
# Run unit tests
python -m pytest test_counter.py -v

# Generate a synthetic test video (no camera needed)
python make_test_video.py
python main.py test_video.mp4
```

---

## License

MIT — see [LICENSE](LICENSE).
