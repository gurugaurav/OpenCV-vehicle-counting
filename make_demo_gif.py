"""
Generate assets/demo.gif — animated top-down road demo of the vehicle counter.

Vehicles move vertically through a horizontal counting line, which is how
real traffic cameras work.  Uses project visualizer so output matches
real tool output exactly.

Usage:
    python make_demo_gif.py
"""

import random
import subprocess
from pathlib import Path

import cv2
import numpy as np

from vehicle_counter.counter import LineCrossCounter
from vehicle_counter.visualizer import draw_box, draw_counts, draw_line

# ── Config ────────────────────────────────────────────────────────────────────
W, H        = 640, 640
FPS         = 20
DURATION_S  = 10
N_LANES     = 4
LANE_W      = W // N_LANES
LINE_Y      = int(H * 0.55)      # counting line slightly below centre
LINE_START  = (0, LINE_Y)
LINE_END    = (W - 1, LINE_Y)
OUT_MP4     = "assets/demo_raw.mp4"
OUT_GIF     = "assets/demo.gif"
CLASSES     = ["car", "car", "car", "car", "truck", "bus", "motorcycle"]

random.seed(12)


# ── Road background (top-down view) ──────────────────────────────────────────
def make_bg() -> np.ndarray:
    img = np.full((H, W, 3), (50, 50, 50), dtype=np.uint8)
    # Kerb strips on edges
    img[:, :12]    = (70, 70, 70)
    img[:, W - 12:] = (70, 70, 70)
    # Dashed lane dividers
    for lane in range(1, N_LANES):
        x = lane * LANE_W
        for y in range(0, H, 50):
            cv2.line(img, (x, y), (x, y + 28), (150, 150, 40), 2)
    return img


BG = make_bg()


# ── Fake vehicle ──────────────────────────────────────────────────────────────
class FakeVehicle:
    _next_id = 1

    def __init__(self, frame_h: int):
        self.track_id = FakeVehicle._next_id
        FakeVehicle._next_id += 1
        self.cls_name = random.choice(CLASSES)

        sizes = {
            "truck":      (random.randint(48, 62), random.randint(90, 115)),
            "bus":        (random.randint(52, 68), random.randint(105, 130)),
            "motorcycle": (random.randint(18, 28), random.randint(30,  45)),
        }
        self.bw, self.bh = sizes.get(
            self.cls_name,
            (random.randint(42, 58), random.randint(62, 82)),
        )

        # direction: +1 = moving down, -1 = moving up
        self.dy = random.choice([+4, +5, +6, +7, -4, -5, -6, -7])

        # pick a lane and centre vehicle in it
        lane = random.randint(0, N_LANES - 1)
        margin = 6
        self.cx = lane * LANE_W + LANE_W // 2 + random.randint(-margin, margin)
        self.cy = float(-self.bh // 2 - 5 if self.dy > 0 else frame_h + self.bh // 2 + 5)

    def step(self):
        self.cy += self.dy

    @property
    def xyxy(self):
        x1 = self.cx - self.bw / 2
        y1 = self.cy - self.bh / 2
        return [x1, y1, x1 + self.bw, y1 + self.bh]

    @property
    def center(self):
        return (self.cx, self.cy)

    def visible(self) -> bool:
        return -self.bh <= self.cy <= H + self.bh


# ── Render ────────────────────────────────────────────────────────────────────
def main():
    Path("assets").mkdir(exist_ok=True)

    counter   = LineCrossCounter(LINE_START, LINE_END)
    fourcc    = cv2.VideoWriter_fourcc(*"mp4v")
    writer    = cv2.VideoWriter(OUT_MP4, fourcc, FPS, (W, H))

    vehicles: list[FakeVehicle] = []
    crossed_ids: set[int]       = set()
    spawn_every = 14             # frames between spawns

    for frame_i in range(FPS * DURATION_S):
        frame = BG.copy()

        if frame_i % spawn_every == 0:
            vehicles.append(FakeVehicle(H))

        crossed_ids.clear()
        active_ids: list[int]    = []
        alive: list[FakeVehicle] = []

        for v in vehicles:
            v.step()
            if not v.visible():
                continue
            alive.append(v)
            active_ids.append(v.track_id)
            if counter.update(v.track_id, v.center):
                crossed_ids.add(v.track_id)

        counter.remove_stale(active_ids)
        vehicles = alive

        for v in vehicles:
            draw_box(frame, v.xyxy, v.track_id, v.cls_name,
                     just_crossed=(v.track_id in crossed_ids))

        line_color = (0, 60, 255) if crossed_ids else (0, 220, 220)
        draw_line(frame, LINE_START, LINE_END, line_color, thickness=3)
        draw_counts(frame, counter.counts, counter.total)

        writer.write(frame)

    writer.release()
    print(f"Wrote {OUT_MP4}")

    subprocess.run([
        "ffmpeg", "-y", "-i", OUT_MP4,
        "-vf", (
            "fps=12,"
            "scale=640:-1:flags=lanczos,"
            "split[s0][s1];"
            "[s0]palettegen=max_colors=192:stats_mode=diff[p];"
            "[s1][p]paletteuse=dither=bayer:bayer_scale=3"
        ),
        OUT_GIF,
    ], check=True)

    import os
    size_kb = os.path.getsize(OUT_GIF) / 1024
    print(f"Wrote {OUT_GIF}  ({size_kb:.0f} KB)")


if __name__ == "__main__":
    main()
