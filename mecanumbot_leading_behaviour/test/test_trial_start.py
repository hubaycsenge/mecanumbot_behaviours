#!/usr/bin/env python3
"""
Unit tests for when a leading trial starts.

The claim: a tree holds still in an unchanged room, however noisy the LD08 is,
however it shifts its beams and wherever the room has depth edges, and starts
once somebody walks in front of it. The first version did not hold still -- it
started every trial at once (2026-10-21) -- and the room scans here are built
to have what defeated it: ~203 beams a turn whose angles shift every scan, and
an object in front of a far wall whose edges a beam can land on either side of.
No ROS.
"""

import math

import numpy as np

from mecanumbot_leading_behaviour.behaviours.trial_start import (
    RoomBaseline,
    ScanChangeDetector,
    ScanChangeSettings,
    new_returns,
    objects,
)

WALL = 4.0
N = 203


def scan(person=None, noise=0.02, dropouts=10, seed=0, offset=None):
    """
    Return `(ranges, angle_min, angle_increment)` of a room with a cupboard in it.

    A wall at 4 m all round, and a cupboard at 1.5 m from 30 to 50 degrees -- two
    depth edges. `person` is `(bearing_deg, width_m, range_m)`. `offset` shifts
    the first beam, as the LD08 does every revolution; by default it is random.
    """
    rng = np.random.default_rng(seed)
    increment = 2.0 * math.pi / N
    if offset is None:
        offset = rng.uniform(0.0, increment)
    angles = np.degrees(offset + increment * np.arange(N))
    ranges = np.full(N, WALL) + rng.normal(0.0, noise, N)
    ranges[(angles >= 30.0) & (angles <= 50.0)] = 1.5 + rng.normal(0.0, noise)
    if person is not None:
        bearing, width, distance = person
        half = math.degrees(math.atan2(width / 2.0, distance))
        off = np.abs((angles - bearing + 180.0) % 360.0 - 180.0)
        ranges[off <= half] = distance
    if dropouts:
        ranges[rng.choice(N, dropouts, replace=False)] = 0.0
    return ranges, offset, increment


def feed(detector, scans):
    return [detector.update(*s, range_min=0.1, range_max=12.0) for s in scans]


def learned(settings=None):
    detector = ScanChangeDetector(settings)
    feed(detector, [scan(seed=i) for i in range(detector.settings.baseline_scans)])
    assert detector.ready
    return detector


def test_the_baseline_keeps_the_nearest_return_in_each_direction():
    base = RoomBaseline()
    base.add([3.0, 3.0], 0.0, math.radians(1.0))
    base.add([2.0, 0.0], 0.0, math.radians(1.0))
    near = base.nearest_around(np.array([0.0, math.radians(1.0)]), math.radians(0.1))
    assert near.tolist() == [2.0, 3.0]


def test_a_direction_never_seen_is_open_space():
    base = RoomBaseline()
    assert math.isinf(base.nearest_around(np.array([1.0]), math.radians(2.0))[0])


def test_only_a_return_nearer_than_the_room_is_new():
    base = RoomBaseline()
    base.add([3.0] * 360, 0.0, math.radians(1.0))
    cfg = ScanChangeSettings()
    nearer = new_returns(base, [2.0] * 360, 0.0, math.radians(1.0), cfg, 0.1, 12.0)
    further = new_returns(base, [3.9] * 360, 0.0, math.radians(1.0), cfg, 0.1, 12.0)
    missing = new_returns(base, [0.0] * 360, 0.0, math.radians(1.0), cfg, 0.1, 12.0)
    assert nearer.all() and not further.any() and not missing.any()


def test_a_depth_edge_is_not_new():
    # The room had the cupboard at 1.5 and the wall at 4 next to each other; a
    # wall return a degree into the cupboard's side is still behind something.
    base = RoomBaseline()
    base.add([1.5, 4.0], 0.0, math.radians(1.0))
    cfg = ScanChangeSettings()
    assert not new_returns(base, [4.0, 1.5], 0.0, math.radians(1.0), cfg, 0.1, 12.0).any()


def test_an_object_is_a_run_of_new_beams_and_the_seam_is_no_break():
    inc = math.radians(1.0)
    mask = np.zeros(360, dtype=bool)
    mask[[0, 1, 358, 359]] = True
    mask[100] = True
    found = objects(mask, 0.0, inc, 3)
    assert len(found) == 1
    assert abs(math.degrees(found[0])) < 2.0


def test_nothing_happens_before_the_baseline():
    detector = ScanChangeDetector()
    results = feed(detector, [scan(person=(0.0, 0.4, 1.0), seed=i) for i in range(19)])
    assert not any(results) and not detector.ready


def test_an_unchanged_noisy_room_never_starts():
    detector = learned()
    results = feed(detector, [scan(noise=0.03, dropouts=25, seed=100 + i) for i in range(500)])
    assert not any(results)


def test_somebody_walking_in_starts_it():
    detector = learned()
    results = feed(detector, [scan(person=(120.0, 0.4, 2.0), seed=200 + i) for i in range(5)])
    assert results == [False, False, False, False, True]
    bearing = float(detector.reason.split(" at ")[1].split(" deg")[0])
    assert abs(bearing - 120.0) <= 4.0  # within a couple of beams


def test_somebody_walking_across_is_still_one_object():
    detector = learned()
    walk = [scan(person=(100.0 + 3.0 * i, 0.4, 2.0), seed=300 + i) for i in range(5)]
    assert feed(detector, walk)[-1]


def test_a_flicker_somewhere_new_every_scan_does_not_start_it():
    detector = learned()
    jumps = [scan(person=(60.0 + 60.0 * i, 0.4, 2.0), seed=400 + i) for i in range(5)]
    assert not any(feed(detector, jumps))


def test_somebody_behind_the_cupboard_is_not_seen():
    detector = learned()
    assert not any(feed(detector, [scan(person=(40.0, 0.4, 2.5), seed=i) for i in range(10)]))


def test_one_scan_short_is_not_enough():
    detector = learned()
    feed(detector, [scan(person=(120.0, 0.4, 2.0), seed=i) for i in range(4)])
    assert not feed(detector, [scan(seed=9)])[0]
    assert not detector.changed


def test_a_thin_thing_is_noise():
    detector = learned(ScanChangeSettings(min_beams=3))
    assert not any(feed(detector, [scan(person=(120.0, 0.02, 2.0), seed=i) for i in range(8)]))


def test_once_started_it_stays_started():
    detector = learned()
    feed(detector, [scan(person=(120.0, 0.4, 2.0), seed=i) for i in range(5)])
    assert feed(detector, [scan()])[0]
