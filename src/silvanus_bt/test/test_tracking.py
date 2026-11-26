"""Tests of the pure tree logic (no ROS needed):  pytest test/test_tracking.py"""

import math

from silvanus_bt.tracking import (BlacklistEntry, BottleTracker, choose_target,
                                  standoff_pose)


def feed(tracker, frames, t0=0.0, dt=0.1):
    for i, dets in enumerate(frames):
        tracker.update(dets, t0 + i * dt)


def test_same_bottle_is_one_track():
    tr = BottleTracker(assoc_radius=0.3)
    feed(tr, [[(1.0, 0.0, 0.5)], [(1.05, 0.02, 0.6)], [(0.98, -0.01, 0.55)]])
    assert len(tr.tracks) == 1
    assert tr.tracks[0].hits == 3


def test_two_bottles_two_tracks():
    tr = BottleTracker(assoc_radius=0.3)
    feed(tr, [[(1.0, 0.0, 0.5), (1.0, 1.0, 0.9)]] * 3)
    assert len(tr.tracks) == 2


def test_forget_unseen_track():
    tr = BottleTracker(forget_after=1.0)
    tr.update([(1.0, 0.0, 0.5)], 0.0)
    tr.update([], 2.0)
    assert tr.tracks == []


def test_choose_highest_confidence_not_nearest():
    tr = BottleTracker()
    # near with low confidence, far with high confidence
    feed(tr, [[(0.8, 0.0, 0.50), (2.0, 0.5, 0.90)]] * 4)
    best = choose_target(tr.tracks, (0.0, 0.0), [], 1.0, min_hits=3)
    assert math.isclose(best.x, 2.0, abs_tol=0.05)
    nearest = choose_target(tr.tracks, (0.0, 0.0), [], 1.0, min_hits=3, selection='nearest')
    assert math.isclose(nearest.x, 0.8, abs_tol=0.05)


def test_needs_min_hits_and_range():
    tr = BottleTracker()
    feed(tr, [[(1.0, 0.0, 0.9)]] * 2)
    assert choose_target(tr.tracks, (0.0, 0.0), [], 1.0, min_hits=3) is None
    feed(tr, [[(4.0, 0.0, 0.9)]] * 5, t0=0.3)
    assert choose_target(tr.tracks, (0.0, 0.0), [], 1.0, min_hits=3, max_range=2.5) is None


def test_blacklist_permanent_and_expiring():
    tr = BottleTracker()
    feed(tr, [[(1.0, 0.0, 0.9)]] * 4)
    perm = [BlacklistEntry(1.0, 0.0, None)]
    assert choose_target(tr.tracks, (0.0, 0.0), perm, 1.0) is None
    temp = [BlacklistEntry(1.0, 0.0, 5.0)]
    assert choose_target(tr.tracks, (0.0, 0.0), temp, 1.0) is None
    assert choose_target(tr.tracks, (0.0, 0.0), temp, 6.0) is not None


def test_standoff_pose_faces_bottle():
    x, y, yaw = standoff_pose((0.0, 0.0), (2.0, 0.0), 0.55)
    assert math.isclose(x, 1.45) and math.isclose(y, 0.0) and math.isclose(yaw, 0.0)
    x, y, yaw = standoff_pose((0.0, 0.0), (0.0, 2.0), 0.5)
    assert math.isclose(y, 1.5) and math.isclose(yaw, math.pi / 2)
