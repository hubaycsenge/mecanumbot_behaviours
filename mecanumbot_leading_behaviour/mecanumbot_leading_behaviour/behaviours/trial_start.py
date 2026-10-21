"""
When a leading trial starts: the first sign that somebody has come in.

A leading tree used to act the moment it was loaded, so every run began with
the robot already searching -- spinning, patrolling -- before the participant
had entered, and the opening of the trial was whatever state that search had
reached. The trees now hold still after loading until one of two things says a
person has arrived:

* **a person is tracked**: `people_fusion` carries somebody -- a camera
  detection that passed the detector's gate and was placed on the map (a
  frame-filling `close_range` one only with the LiDAR's backing), or a body
  the LiDAR has followed for a while right next to the robot, where the camera
  cannot see;
* **something new stands in the room**: the room the robot saw when it was
  switched on is the baseline, and an object *in front of* it -- nearer than
  anything the baseline had in that direction -- that holds its bearing for a
  few scans is somebody walking in.

Only something appearing in front of the room counts, never the room receding
or a return going missing, and the comparison is deliberately coarse in angle.
The first version binned the scan into 2-degree bins and counted any change in
either direction, and it started every trial at once in a still room
(2026-10-21): the LD08 gives ~203 beams a turn, 1.8 degrees apart, and changes
its beam geometry every revolution, so a bin held a return in one scan and none
in the next, and at every depth edge the nearest return in a bin flipped
between the near surface and the far one. Here a beam has changed only when it
is nearer than *every* return the baseline ever had within `angle_tolerance`
of it, which no edge and no beam jitter can produce.

No ROS here; `start_gate.WaitForTrialStart` feeds it.
"""

import math
from dataclasses import dataclass

import numpy as np

FULL_TURN = 2.0 * math.pi


@dataclass(frozen=True)
class ScanChangeSettings:
    """The tunables of the scan comparison; the constants file spells them `trial_start_*`."""

    # Scans whose returns make the baseline. The robot must stand still while
    # they are taken, which it does: the tree has not started yet.
    baseline_scans: int = 20
    # A return counts as new only if it is this many metres nearer than every
    # baseline return within `angle_tolerance` of it.
    min_change: float = 0.3
    angle_tolerance: float = math.radians(2.0)
    # Only things within this range count [m]: past it a leg is one beam.
    max_range: float = 4.5
    # Neighbouring new returns that make an object: a single beam is noise. At
    # 1.8 degrees a beam, three is a pair of legs at ~2.5 m.
    min_beams: int = 3
    # Scans running the object has to be seen in, each within `persist_gate`
    # of the bearing it had in the scan before.
    persist_scans: int = 5
    persist_gate: float = math.radians(10.0)


# Resolution the baseline is kept at: finer than the LD08's beam spacing, so
# the angle tolerance, not the grid, decides what is "the same direction".
_GRID = math.radians(0.5)


def _angles(count, angle_min, angle_increment):
    return np.mod(angle_min + angle_increment * np.arange(count), FULL_TURN)


def _valid(ranges, range_min, range_max):
    return np.isfinite(ranges) & (ranges > range_min) & (ranges < range_max)


def wrap(angle):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class RoomBaseline:
    """
    The nearest return ever seen in each direction while the baseline was taken.

    Kept on a fine angular grid; a direction no baseline scan had a return in
    is +inf -- open space, or glass -- and anything appearing there is new.
    """

    def __init__(self):
        self.cells = int(round(FULL_TURN / _GRID))
        self.nearest = np.full(self.cells, np.inf)

    def add(self, ranges, angle_min, angle_increment, range_min=0.0, range_max=math.inf):
        """Fold one scan's valid returns into the baseline."""
        ranges = np.asarray(ranges, dtype=float)
        valid = _valid(ranges, range_min, range_max)
        cells = (
            _angles(ranges.size, angle_min, angle_increment)[valid] / _GRID
        ).astype(int) % self.cells
        np.minimum.at(self.nearest, cells, ranges[valid])

    def nearest_around(self, angles, tolerance):
        """Return, for each angle, the nearest baseline return within `tolerance`."""
        reach = int(math.ceil(tolerance / _GRID))
        cells = (np.asarray(angles) / _GRID).astype(int) % self.cells
        out = np.full(cells.shape, np.inf)
        for offset in range(-reach, reach + 1):
            out = np.minimum(out, self.nearest[(cells + offset) % self.cells])
        return out


def new_returns(baseline, ranges, angle_min, angle_increment, settings,
                range_min=0.0, range_max=math.inf):
    """
    Return a boolean mask over the beams: which are something new in the room.

    New means valid, within `max_range`, and `min_change` nearer than every
    baseline return within `angle_tolerance`. A missing return is never new.
    """
    ranges = np.asarray(ranges, dtype=float)
    valid = _valid(ranges, range_min, range_max) & (ranges <= settings.max_range)
    behind = baseline.nearest_around(
        _angles(ranges.size, angle_min, angle_increment), settings.angle_tolerance
    )
    new = np.zeros(ranges.shape, dtype=bool)
    new[valid] = ranges[valid] < behind[valid] - settings.min_change
    return new


def objects(mask, angle_min, angle_increment, min_beams):
    """
    Return the bearings of the runs of at least `min_beams` neighbouring new beams.

    The scan is circular, so a run across its seam is one object.
    """
    mask = np.asarray(mask, dtype=bool)
    count = mask.size
    if count == 0 or not mask.any():
        return []
    if mask.all():
        return [0.0]
    start = int(np.argmin(mask))  # a beam that is not new: runs cannot wrap past it
    order = [(start + k) % count for k in range(count)]
    found, run = [], []
    for index in order + [start]:
        if mask[index]:
            run.append(index)
            continue
        if len(run) >= min_beams:
            middle = run[len(run) // 2]
            found.append(wrap(angle_min + angle_increment * middle))
        run = []
    return found


class ScanChangeDetector:
    """
    Build a baseline of the room, then say when something new stands in it.

    `update()` returns True from the scan on which one object has been seen,
    near the same bearing, for `persist_scans` scans running, and keeps
    returning True after that: the trial has started, and the person standing
    still again does not un-start it.
    """

    def __init__(self, settings=None):
        self.settings = settings or ScanChangeSettings()
        self.baseline = RoomBaseline()
        self._baseline_count = 0
        self.changed = False
        self.reason = ""
        self._bearing = None
        self._streak = 0

    @property
    def ready(self):
        """Say whether the baseline has been built."""
        return self._baseline_count >= self.settings.baseline_scans

    def update(self, ranges, angle_min, angle_increment, range_min=0.0, range_max=math.inf):
        """Fold in one scan; return whether something new has come in."""
        if self.changed:
            return True
        cfg = self.settings
        if not self.ready:
            self.baseline.add(ranges, angle_min, angle_increment, range_min, range_max)
            self._baseline_count += 1
            return False

        mask = new_returns(
            self.baseline, ranges, angle_min, angle_increment, cfg, range_min, range_max
        )
        bearings = objects(mask, angle_min, angle_increment, cfg.min_beams)
        if self._bearing is not None:
            same = [b for b in bearings if abs(wrap(b - self._bearing)) <= cfg.persist_gate]
        else:
            same = bearings
        if same:
            reference = self._bearing
            self._bearing = (
                same[0] if reference is None
                else min(same, key=lambda b: abs(wrap(b - reference)))
            )
            self._streak += 1
        else:
            self._bearing = bearings[0] if bearings else None
            self._streak = 1 if bearings else 0

        if self._streak >= cfg.persist_scans:
            self.changed = True
            self.reason = (
                "something new stands in the room at "
                f"{math.degrees(self._bearing):+.0f} deg, for {self._streak} scans"
            )
        return self.changed
