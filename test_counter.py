"""
Unit tests for LineCrossCounter — no YOLO or video needed.
Simulates centroids moving across the counting line and verifies counts.
"""

from vehicle_counter.counter import LineCrossCounter


def test_basic_down_crossing():
    c = LineCrossCounter((0, 300), (640, 300))
    # Move from above the line (y<300) to below (y>300)
    assert c.update(1, (320, 100)) is None   # first point, no prev
    assert c.update(1, (320, 200)) is None   # still above
    assert c.update(1, (320, 400)) == "down" # crossed!
    assert c.counts["down"] == 1
    assert c.counts["up"] == 0


def test_basic_up_crossing():
    c = LineCrossCounter((0, 300), (640, 300))
    assert c.update(1, (320, 500)) is None
    assert c.update(1, (320, 100)) == "up"
    assert c.counts["up"] == 1
    assert c.counts["down"] == 0


def test_no_double_count():
    c = LineCrossCounter((0, 300), (640, 300))
    c.update(1, (320, 100))
    c.update(1, (320, 400))   # first crossing → counted
    result = c.update(1, (320, 600))  # already counted, no new crossing
    assert result is None
    assert c.total == 1


def test_multiple_tracks_independent():
    c = LineCrossCounter((0, 300), (640, 300))
    c.update(1, (100, 100)); c.update(1, (100, 400))
    c.update(2, (200, 100)); c.update(2, (200, 400))
    c.update(3, (300, 500)); c.update(3, (300, 100))
    assert c.counts["down"] == 2
    assert c.counts["up"] == 1
    assert c.total == 3


def test_parallel_movement_no_count():
    """Object moving parallel to (and not crossing) the line."""
    c = LineCrossCounter((0, 300), (640, 300))
    c.update(1, (100, 100))
    assert c.update(1, (500, 100)) is None  # moved sideways, never crossed
    assert c.total == 0


def test_stale_removal():
    c = LineCrossCounter((0, 300), (640, 300))
    c.update(1, (320, 100))
    c.update(1, (320, 200))
    c.remove_stale([])   # ID 1 no longer active
    assert 1 not in c._prev
    assert 1 not in c._counted


def test_angled_line():
    """Diagonal counting line — segment-intersection must handle this."""
    c = LineCrossCounter((0, 0), (640, 480))
    # Point goes from one side of the diagonal to the other
    assert c.update(1, (0, 480)) is None     # below-left of line
    assert c.update(1, (640, 0)) == "up"     # above-right of line
    assert c.total == 1


if __name__ == "__main__":
    tests = [
        test_basic_down_crossing,
        test_basic_up_crossing,
        test_no_double_count,
        test_multiple_tracks_independent,
        test_parallel_movement_no_count,
        test_stale_removal,
        test_angled_line,
    ]
    passed = 0
    for t in tests:
        try:
            t()
            print(f"  PASS  {t.__name__}")
            passed += 1
        except AssertionError as e:
            print(f"  FAIL  {t.__name__}: {e}")
        except Exception as e:
            print(f"  ERROR {t.__name__}: {e}")
    print(f"\n{passed}/{len(tests)} tests passed")
