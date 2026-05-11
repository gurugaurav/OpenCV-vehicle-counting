import numpy as np


class LineCrossCounter:
    """
    Counts objects whose centroids cross a line segment.

    Uses segment-intersection geometry so angled lines work correctly.
    Each track ID is counted at most once per direction.
    """

    def __init__(self, line_start, line_end):
        self.p1 = np.array(line_start, dtype=float)
        self.p2 = np.array(line_end, dtype=float)
        self._prev = {}          # track_id -> last center (np.array)
        self._counted = {}       # track_id -> direction already counted
        self.counts = {"up": 0, "down": 0}

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def update(self, track_id, center):
        """
        Register a new centroid position for track_id.
        Returns "up" / "down" if the line was crossed this frame, else None.

        "down" means the object moved to the positive side of the directed
        line p1→p2 (i.e. crossing from left-of-line to right-of-line when
        standing at p1 facing p2).
        """
        center = np.array(center, dtype=float)
        prev = self._prev.get(track_id)
        self._prev[track_id] = center

        if prev is None:
            return None

        if not self._segments_cross(prev, center):
            return None

        if track_id in self._counted:
            return None

        prev_side = self._side(prev)
        curr_side = self._side(center)
        if prev_side == curr_side or prev_side == 0 or curr_side == 0:
            return None

        direction = "down" if curr_side > 0 else "up"
        self._counted[track_id] = direction
        self.counts[direction] += 1
        return direction

    def remove_stale(self, active_ids):
        """Drop state for track IDs that are no longer active."""
        active = set(active_ids)
        for tid in list(self._prev):
            if tid not in active:
                del self._prev[tid]
                self._counted.pop(tid, None)

    @property
    def total(self):
        return self.counts["up"] + self.counts["down"]

    # ------------------------------------------------------------------
    # Geometry helpers
    # ------------------------------------------------------------------

    def _side(self, pt):
        """Sign of the cross-product (p2-p1) × (pt-p1)."""
        d = (self.p2[0] - self.p1[0]) * (pt[1] - self.p1[1]) \
          - (self.p2[1] - self.p1[1]) * (pt[0] - self.p1[0])
        return np.sign(d)

    def _segments_cross(self, a, b):
        """
        True if segment a→b crosses the counting line segment p1→p2.
        Uses the standard CCW-test for segment intersection.
        """
        def ccw(A, B, C):
            return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

        A, B = self.p1, self.p2
        C, D = a, b
        return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)
