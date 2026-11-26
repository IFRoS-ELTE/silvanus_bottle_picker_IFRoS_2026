"""Pure logic of the tree (no ROS): track bottles, choose a target, compute the approach pose.

Kept apart from the behaviours so it can be tested with pytest without a robot or ROS.

- BottleTracker: each detection (x, y, confidence) in the `map` frame is associated with a
  track if it falls within `assoc_radius` of it. The same bottle seen in many frames is ONE
  track with a smoothed position, a hit count and a mean confidence.
- Blacklist: places (x, y) not to try again (already collected, or a recent failure).
- choose_target: among the stable tracks, the one with the HIGHEST MEAN CONFIDENCE
  (ties: the nearest one).
- standoff_pose: point `distance` metres short of the bottle, on the robot->bottle line,
  facing the bottle.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple


@dataclass
class Track:
    id: int
    x: float
    y: float
    conf: float          # mean confidence (exponential moving average)
    hits: int            # how many times it has been seen
    last_seen: float     # time (s) it was last seen

    @property
    def xy(self) -> Tuple[float, float]:
        return (self.x, self.y)


@dataclass
class BlacklistEntry:
    x: float
    y: float
    until: Optional[float]   # None = forever
    reason: str = ''


class BottleTracker:
    def __init__(self, assoc_radius: float = 0.30, alpha: float = 0.3,
                 forget_after: float = 3.0) -> None:
        self.assoc_radius = assoc_radius
        self.alpha = alpha                 # weight of a new measurement in the smoothing
        self.forget_after = forget_after   # s without seeing a track before dropping it
        self.tracks: List[Track] = []
        self._next_id = 0

    def update(self, detections: List[Tuple[float, float, float]], now: float) -> None:
        """detections: list of (x, y, confidence) in `map`, all from the same camera frame."""
        used = set()
        for (x, y, conf) in detections:
            best, best_d = None, self.assoc_radius
            for t in self.tracks:
                if t.id in used:
                    continue
                d = math.hypot(t.x - x, t.y - y)
                if d <= best_d:
                    best, best_d = t, d
            if best is None:
                t = Track(self._next_id, x, y, conf, 1, now)
                self._next_id += 1
                self.tracks.append(t)
                used.add(t.id)
            else:
                a = self.alpha
                best.x = (1 - a) * best.x + a * x
                best.y = (1 - a) * best.y + a * y
                best.conf = (1 - a) * best.conf + a * conf
                best.hits += 1
                best.last_seen = now
                used.add(best.id)
        self.tracks = [t for t in self.tracks if now - t.last_seen <= self.forget_after]

    def get(self, track_id: int) -> Optional[Track]:
        for t in self.tracks:
            if t.id == track_id:
                return t
        return None


def is_blacklisted(x: float, y: float, blacklist: List[BlacklistEntry], now: float,
                   radius: float) -> bool:
    for b in blacklist:
        if b.until is not None and now > b.until:
            continue
        if math.hypot(b.x - x, b.y - y) <= radius:
            return True
    return False


def prune_blacklist(blacklist: List[BlacklistEntry], now: float) -> List[BlacklistEntry]:
    return [b for b in blacklist if b.until is None or now <= b.until]


def choose_target(tracks: List[Track], robot_xy: Tuple[float, float],
                  blacklist: List[BlacklistEntry], now: float, *,
                  min_hits: int = 3, max_range: float = 2.5, min_range: float = 0.15,
                  blacklist_radius: float = 0.40, selection: str = 'confidence'
                  ) -> Optional[Track]:
    """The track to go for, or None.

    selection='confidence': highest mean confidence; on a tie (+-0.02) the nearest.
    selection='nearest':    the nearest (the old nav_to_bottle behaviour).
    """
    rx, ry = robot_xy
    candidates = []
    for t in tracks:
        if t.hits < min_hits:
            continue
        dist = math.hypot(t.x - rx, t.y - ry)
        if dist > max_range or dist < min_range:
            continue
        if is_blacklisted(t.x, t.y, blacklist, now, blacklist_radius):
            continue
        candidates.append((t, dist))
    if not candidates:
        return None
    if selection == 'nearest':
        return min(candidates, key=lambda c: c[1])[0]
    # Round the confidence to 0.02 so that noise does not beat distance
    return min(candidates, key=lambda c: (-round(c[0].conf / 0.02), c[1]))[0]


def standoff_pose(robot_xy: Tuple[float, float], bottle_xy: Tuple[float, float],
                  distance: float) -> Tuple[float, float, float]:
    """(x, y, yaw) `distance` m from the bottle, on the robot->bottle line, facing it."""
    rx, ry = robot_xy
    bx, by = bottle_xy
    dx, dy = bx - rx, by - ry
    d = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    if d < 1e-6:
        return rx, ry, yaw
    ux, uy = dx / d, dy / d
    return bx - distance * ux, by - distance * uy, yaw


def inside_map_bounds(x: float, y: float, info, margin: float) -> bool:
    """info: nav_msgs/MapMetaData (or None: no map yet, accepted)."""
    if info is None:
        return True
    res = float(info.resolution)
    ox = float(info.origin.position.x)
    oy = float(info.origin.position.y)
    return (ox + margin <= x <= ox + info.width * res - margin
            and oy + margin <= y <= oy + info.height * res - margin)


@dataclass
class Target:
    """The bottle the tree decided to go for (stored on the blackboard)."""
    track_id: int
    x: float
    y: float
    conf: float
    extra: dict = field(default_factory=dict)
