"""
Generates a synthetic traffic video: colored rectangles moving across the frame
in both directions, crossing a horizontal band at y=300.

Output: test_traffic.mp4  (1280×720, 30 fps, ~10 seconds)
"""

import cv2
import numpy as np
import random

W, H = 1280, 720
FPS = 30
DURATION_S = 12
N_FRAMES = FPS * DURATION_S
OUT = "test_traffic.mp4"

random.seed(42)

# Road background
def make_bg():
    img = np.full((H, W, 3), (60, 60, 60), dtype=np.uint8)
    # Lane markings
    for x in range(0, W, 80):
        cv2.rectangle(img, (x, H // 2 - 3), (x + 40, H // 2 + 3), (220, 220, 50), -1)
    return img

BG = make_bg()


class Vehicle:
    COLORS = [(200, 80, 80), (80, 200, 80), (80, 80, 220),
              (200, 200, 60), (200, 80, 200), (60, 200, 200)]

    def __init__(self, vid):
        self.color = self.COLORS[vid % len(self.COLORS)]
        self.w = random.randint(60, 110)
        self.h = random.randint(35, 55)
        # direction: +1 → left-to-right (downward in terms of line side)
        #            -1 → right-to-left
        self.dx = random.choice([+3, +4, +5, -3, -4, -5])
        lane_y = random.choice([H // 4, H // 2, 3 * H // 4])
        self.y = lane_y - self.h // 2
        # start off-screen
        self.x = -self.w - 10 if self.dx > 0 else W + 10

    def step(self):
        self.x += self.dx

    def visible(self):
        return -self.w <= self.x <= W

    def draw(self, frame):
        x1, y1 = int(self.x), int(self.y)
        x2, y2 = x1 + self.w, y1 + self.h
        cv2.rectangle(frame, (x1, y1), (x2, y2), self.color, -1)
        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 255), 1)


def main():
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    out = cv2.VideoWriter(OUT, fourcc, FPS, (W, H))

    vehicles = []
    spawn_every = 25  # frames

    for frame_i in range(N_FRAMES):
        frame = BG.copy()

        # Spawn new vehicles periodically
        if frame_i % spawn_every == 0:
            vehicles.append(Vehicle(len(vehicles)))

        # Update and draw
        alive = []
        for v in vehicles:
            v.step()
            if v.visible():
                v.draw(frame)
                alive.append(v)
        vehicles = alive

        # Counting line at y = H//2
        cv2.line(frame, (0, H // 2), (W, H // 2), (0, 220, 220), 2)

        cv2.putText(frame, f"Frame {frame_i:04d}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)

        out.write(frame)

    out.release()
    print(f"Wrote {OUT}  ({N_FRAMES} frames @ {FPS} fps)")


if __name__ == "__main__":
    main()
