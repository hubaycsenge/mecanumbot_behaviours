#!/usr/bin/env python3
"""
Unit tests for when a leading trial starts.

The claim: a tree holds still in an unchanged room, however noisy the LD08 is
and however it changes its scan geometry, and starts once somebody walks in.
No ROS.
"""

import math

import numpy as np

from mecanumbot_leading_behaviour.behaviours.trial_start import (
    ScanChangeDetector,
    ScanChangeSettings,
    bin_scan,
    changed_bins,
    longest_run,
)

ROOM = 3.5


def scan(n=210, person=None, noise=0.0, dropouts=0, seed=0, offset=0.0):
    """
    Return `(ranges, angle_min, angle_increment)` of a round room.

    `person` is `(bearing_deg, width_deg, range)`. `offset` shifts the first
    beam, the way the LD08 driver shifts its geometry every revolution.
    """
    rng = np.random.default_rng(seed)
    increment = 2.0 * math.pi / n
    angles = offset + increment * np.arange(n)
    ranges = np.full(n, ROOM) + rng.normal(0.0, noise, n)
    if person is not None:
        bearing, width, distance = person
        off = np.abs((np.degrees(angles) - bearing + 180.0) % 360.0 - 180.0)
        ranges[off <= width / 2.0] = distance
    if dropouts:
        ranges[rng.choice(n, dropouts, replace=False)] = 0.0
    return ranges, offset, increment


def feed(detector, scans):
    return [detector.update(*s, range_min=0.1, range_max=12.0) for s in scans]


def learned(settings=None):
    detector = ScanChangeDetector(settings)
    feed(detector, [scan(noise=0.02, seed=i) for i in range(detector.settings.baseline_scans)])
    assert detector.ready
    return detector


def test_bins_keep_the_nearest_return():
    ranges = [3.0, 1.0, 2.0, 4.0]
    binned = bin_scan(ranges, 0.0, math.radians(0.5), math.radians(2.0))
    assert binned[0] == 1.0


def test_an_empty_bin_is_nan():
    binned = bin_scan([0.0, np.inf, np.nan], 0.0, 0.01, math.radians(2.0), range_min=0.1)
    assert np.isnan(binned).all()


def test_signed_and_unsigned_scans_bin_alike():
    ranges, _, inc = scan(person=(-30.0, 10.0, 1.0))
    a = bin_scan(ranges, 0.0, inc, math.radians(2.0))
    rolled = np.roll(ranges, len(ranges) // 2)
    b = bin_scan(rolled, -math.pi, inc, math.radians(2.0))
    assert np.allclose(a, b, equal_nan=True)


def test_a_lost_return_is_not_a_change():
    mask = changed_bins([1.0, 2.0], [np.nan, 2.0], 0.3, 4.5)
    assert not mask.any()


def test_something_arriving_in_open_space_is_a_change():
    mask = changed_bins([np.nan, np.nan], [2.0, np.nan], 0.3, 4.5)
    assert mask.tolist() == [True, False]


def test_a_change_beyond_range_is_not_one():
    mask = changed_bins([8.0], [6.0], 0.3, 4.5)
    assert not mask.any()


def test_a_run_across_the_seam_is_whole():
    assert longest_run([True, False, False, True, True]) == 3


def test_nothing_happens_before_the_baseline():
    detector = ScanChangeDetector()
    results = feed(detector, [scan(person=(0.0, 12.0, 1.0))] * 9)
    assert not any(results) and not detector.ready


def test_an_unchanged_noisy_room_never_starts():
    detector = learned()
    results = feed(
        detector,
        [scan(noise=0.03, dropouts=20, seed=100 + i, offset=0.004 * i) for i in range(200)],
    )
    assert not any(results)


def test_somebody_walking_in_starts_it():
    detector = learned()
    results = feed(detector, [scan(person=(40.0, 8.0, 2.0), seed=7)] * 3)
    assert results == [False, False, True]
    assert "changed" in detector.reason


def test_one_scan_of_it_is_not_enough():
    detector = learned()
    feed(detector, [scan(person=(40.0, 8.0, 2.0))] * 2)
    assert not feed(detector, [scan()])[0]
    assert not detector.changed


def test_a_single_bin_is_noise():
    detector = learned(ScanChangeSettings(min_bins=3))
    assert not any(feed(detector, [scan(person=(40.0, 1.0, 2.0))] * 5))


def test_once_started_it_stays_started():
    detector = learned()
    feed(detector, [scan(person=(40.0, 8.0, 2.0))] * 3)
    assert feed(detector, [scan()])[0]
