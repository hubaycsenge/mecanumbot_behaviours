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
* **the LiDAR pattern changes**: the room the robot saw when it was switched on
  is the baseline, and a stretch of neighbouring directions in which something
  is now nearer or further than it was, for a few scans running, is somebody
  walking in.

The scan comparison is made on fixed angular bins rather than beam by beam,
because the LD08 driver changes its scan geometry every revolution (see
`mecanumbot_core`'s scan grid). Each bin keeps the nearest valid return in it.
A bin that has lost its return is not a change: the LD08 drops beams all the
time, and an empty bin says nothing about what is there.

No ROS here; `start_gate.WaitForTrialStart` feeds it.
"""

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ScanChangeSettings:
    """The tunables of the scan comparison; the constants file spells them `trial_start_*`."""

    # Width of one angular bin [rad]. A person at 3 m is ~6 degrees across.
    bin_width: float = math.radians(2.0)
    # Scans averaged (per-bin median) into the baseline before anything is
    # compared against it.
    baseline_scans: int = 10
    # Metres a bin has to move by to count as changed. The LD08's range noise
    # is a few centimetres; a person is not.
    min_change: float = 0.3
    # Only changes within this range count [m]: past it a leg is one beam.
    max_range: float = 4.5
    # Neighbouring changed bins that make one change: a single bin is noise.
    min_bins: int = 2
    # Consecutive scans the change has to be seen in.
    persist_scans: int = 3


def bin_scan(ranges, angle_min, angle_increment, bin_width, range_min=0.0, range_max=math.inf):
    """
    Return the nearest valid return in each fixed angular bin over a full turn.

    Bins with no valid return are NaN. Bin 0 starts at bearing 0; bearings are
    taken modulo a full turn, so a 0..2pi scan and a -pi..pi one bin alike.
    """
    count = max(1, int(round(2.0 * math.pi / bin_width)))
    ranges = np.asarray(ranges, dtype=float)
    angles = angle_min + angle_increment * np.arange(ranges.size)
    valid = np.isfinite(ranges) & (ranges > range_min) & (ranges < range_max)
    binned = np.full(count, np.nan)
    if not valid.any():
        return binned
    index = (np.floor(np.mod(angles[valid], 2.0 * math.pi) / (2.0 * math.pi) * count)
             .astype(int) % count)
    values = ranges[valid]
    order = np.argsort(-values)  # far first, so the nearest is written last
    binned[index[order]] = values[order]
    return binned


def changed_bins(baseline, current, min_change, max_range):
    """
    Return a boolean mask of the bins in which the room has changed.

    A bin has changed when it has a return now that is `min_change` nearer or
    further than the baseline's, within `max_range` -- or a return within
    `max_range` where the baseline had none, which is something arriving in
    open space. A bin that has lost its return is never a change.
    """
    baseline = np.asarray(baseline, dtype=float)
    current = np.asarray(current, dtype=float)
    has_now = np.isfinite(current)
    had = np.isfinite(baseline)
    both = has_now & had
    moved = np.zeros(current.shape, dtype=bool)
    moved[both] = (np.abs(current[both] - baseline[both]) > min_change) & (
        np.minimum(current[both], baseline[both]) <= max_range
    )
    appeared = has_now & ~had & (np.nan_to_num(current, nan=np.inf) <= max_range)
    return moved | appeared


def longest_run(mask):
    """Return the longest run of True in a circular mask."""
    mask = np.asarray(mask, dtype=bool)
    if mask.all():
        return int(mask.size)
    best = run = 0
    # Twice round, so a run across the seam is counted whole.
    for value in np.concatenate([mask, mask]):
        run = run + 1 if value else 0
        best = max(best, run)
    return int(min(best, mask.size))


class ScanChangeDetector:
    """
    Build a baseline of the room, then say when the scan stops matching it.

    `update()` returns True from the scan on which a change has persisted for
    `persist_scans` scans, and keeps returning True after that: the trial has
    started, and a person standing still again does not un-start it.
    """

    def __init__(self, settings=None):
        self.settings = settings or ScanChangeSettings()
        self._collected = []
        self.baseline = None
        self.changed = False
        self.reason = ""
        self._streak = 0

    @property
    def ready(self):
        """Say whether the baseline has been built."""
        return self.baseline is not None

    def update(self, ranges, angle_min, angle_increment, range_min=0.0, range_max=math.inf):
        """Fold in one scan; return whether the room has changed."""
        if self.changed:
            return True
        cfg = self.settings
        binned = bin_scan(
            ranges, angle_min, angle_increment, cfg.bin_width, range_min, range_max
        )
        if self.baseline is None:
            self._collected.append(binned)
            if len(self._collected) >= cfg.baseline_scans:
                with np.errstate(all="ignore"):
                    stack = np.vstack(self._collected)
                    # A bin that was empty in most of the baseline scans is
                    # empty in the baseline: one stray return is not the room.
                    present = np.isfinite(stack).sum(axis=0) * 2 > len(stack)
                    medians = np.full(stack.shape[1], np.nan)
                    if present.any():
                        medians[present] = np.nanmedian(stack[:, present], axis=0)
                self.baseline = medians
                self._collected = []
            return False

        run = longest_run(changed_bins(self.baseline, binned, cfg.min_change, cfg.max_range))
        if run >= cfg.min_bins:
            self._streak += 1
        else:
            self._streak = 0
        if self._streak >= cfg.persist_scans:
            self.changed = True
            self.reason = (
                f"the LiDAR pattern changed over {math.degrees(run * cfg.bin_width):.0f} "
                f"deg for {self._streak} scans"
            )
        return self.changed
