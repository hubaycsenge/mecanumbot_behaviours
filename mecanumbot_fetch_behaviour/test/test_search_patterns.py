#!/usr/bin/env python3
"""
Tests for how the robot looks for a ball: the circles, and the head sweep.

Both are pure geometry, and both encode a decision rather than a calculation --
which way the robot faces while it searches, and how its attention is spread
over the height bands the camera can see. So these are claims about the search,
not arithmetic checks.

No ROS: `search_patterns` is plain Python so this runs anywhere.
"""

import math

import pytest

from mecanumbot_fetch_behaviour.search_patterns import (
    FACING_INWARD,
    FACING_OUTWARD,
    FACING_TANGENT,
    circle,
    circle_count,
    expanding_circles,
    head_sweep,
    nearest_unvisited,
    radii_up_to,
    sweep_complete,
)


def distance(point, centre):
    """Planar distance from a `(x, y, yaw)` stop to a centre."""
    return math.hypot(point[0] - centre[0], point[1] - centre[1])


def turn_between(first, second):
    """Signed difference between two headings, folded into (-pi, pi]."""
    return math.atan2(math.sin(first - second), math.cos(first - second))


# --- the circles -------------------------------------------------------------


def test_every_stop_on_a_circle_is_at_the_radius():
    stops = circle((2.0, -1.0), 1.5, 8)
    assert all(distance(s, (2.0, -1.0)) == pytest.approx(1.5) for s in stops)


def test_stops_are_evenly_spaced_around_the_circle():
    stops = circle((0.0, 0.0), 1.0, 4)
    angles = sorted(math.atan2(y, x) for x, y, _ in stops)
    gaps = [b - a for a, b in zip(angles, angles[1:])]
    assert all(gap == pytest.approx(math.pi / 2.0) for gap in gaps)


def test_inward_stops_face_the_centre():
    """`mecanumbot_seek`'s pattern: right when there is a place to circle."""
    stops = circle((0.0, 0.0), 2.0, 4, facing=FACING_INWARD)
    for x, y, yaw in stops:
        assert turn_between(yaw, math.atan2(-y, -x)) == pytest.approx(0.0, abs=1e-9)


def test_outward_stops_face_away_from_the_centre():
    stops = circle((0.0, 0.0), 2.0, 4, facing=FACING_OUTWARD)
    for x, y, yaw in stops:
        assert turn_between(yaw, math.atan2(y, x)) == pytest.approx(0.0, abs=1e-9)


def test_tangent_stops_face_along_the_direction_of_travel():
    """
    The default, and the reason the fetch search circles at all.

    The robot does not know where the ball is, so the interesting floor is the
    floor it has not looked at, which is the floor it is about to drive over.
    """
    stops = circle((0.0, 0.0), 2.0, 4, facing=FACING_TANGENT)
    for x, y, yaw in stops:
        assert turn_between(yaw, math.atan2(y, x)) == pytest.approx(math.pi / 2.0)


def test_an_unknown_facing_is_an_error_and_not_a_default():
    with pytest.raises(ValueError):
        circle((0.0, 0.0), 1.0, 4, facing="sideways")


def test_a_circle_never_gets_fewer_stops_than_the_floor():
    """A circle with three stops is a triangle the camera looks along."""
    assert circle_count(0.3, 1.2, minimum=6) == 6


def test_a_wide_circle_is_capped_rather_than_unbounded():
    assert circle_count(50.0, 1.2, minimum=6, maximum=16) == 16


def test_stops_get_further_apart_only_up_to_the_cap():
    assert circle_count(2.0, 1.0, minimum=6, maximum=16) == 13


# --- the whole pattern -------------------------------------------------------


def test_radii_start_at_the_first_and_step_outwards():
    assert radii_up_to(4.8, 1.2, 1.2) == pytest.approx([1.2, 2.4, 3.6, 4.8])


def test_a_limit_inside_the_first_circle_still_means_look_around():
    """Refusing to look is not one of the answers this function may give."""
    assert radii_up_to(0.5, 1.2, 1.2) == [1.2]


def test_the_pattern_searches_the_near_floor_before_the_far():
    """
    Every stop at radius r comes before any stop at r + step.

    This is the whole reason the search is circles rather than a list of places:
    the ball is more likely to be near, so near is looked at first.
    """
    stops = expanding_circles((0.0, 0.0), [1.0, 2.0], spacing=1.0)
    radii = [distance(s, (0.0, 0.0)) for s in stops]
    first_far = next(i for i, r in enumerate(radii) if r > 1.5)
    assert all(r == pytest.approx(1.0) for r in radii[:first_far])
    assert all(r == pytest.approx(2.0) for r in radii[first_far:])


def test_successive_circles_do_not_all_start_at_the_same_bearing():
    """
    Otherwise every lap begins by re-driving the same radial line.

    The phase shift is half a stop's spacing, so the robot enters each circle
    at a bearing it did not enter the last one at.
    """
    stops = expanding_circles((0.0, 0.0), [1.0, 2.0], spacing=1.0, minimum=6)
    inner = math.atan2(stops[0][1], stops[0][0])
    outer_index = next(
        i for i, s in enumerate(stops) if distance(s, (0.0, 0.0)) > 1.5
    )
    outer = math.atan2(stops[outer_index][1], stops[outer_index][0])
    assert inner != pytest.approx(outer)


# --- walking it --------------------------------------------------------------


def test_the_order_is_the_search_not_the_nearest_stop():
    """
    The first unvisited stop wins, even when a later one is closer.

    Taking the nearest would undo the near-before-far property that is the only
    reason to lay the stops out in circles.
    """
    waypoints = [(5.0, 0.0, 0.0), (0.1, 0.0, 0.0)]
    assert nearest_unvisited(waypoints, (0.0, 0.0), set(), 0.5) == 0


def test_a_stop_already_driven_past_is_written_off_for_free():
    """On a circle the robot passes stops on its way to others constantly."""
    waypoints = [(0.2, 0.0, 0.0), (5.0, 0.0, 0.0)]
    visited = set()
    assert nearest_unvisited(waypoints, (0.0, 0.0), visited, 0.5) == 1
    assert 0 in visited


def test_a_lap_with_nothing_left_returns_no_stop():
    waypoints = [(1.0, 0.0, 0.0)]
    assert nearest_unvisited(waypoints, (5.0, 5.0), {0}, 0.5) is None
    assert sweep_complete(waypoints, {0})


# --- the head sweep ----------------------------------------------------------


def test_the_sweep_starts_at_the_low_pose():
    assert head_sweep(0.0, 3.0, 6.5, 6.0) == pytest.approx(3.0)


def test_the_sweep_reaches_the_high_pose_halfway_through():
    assert head_sweep(3.0, 3.0, 6.5, 6.0) == pytest.approx(6.5)


def test_the_sweep_comes_back_down_and_repeats():
    assert head_sweep(6.0, 3.0, 6.5, 6.0) == pytest.approx(3.0)
    assert head_sweep(7.5, 3.0, 6.5, 6.0) == pytest.approx(
        head_sweep(1.5, 3.0, 6.5, 6.0)
    )


def test_the_sweep_is_a_triangle_so_every_band_gets_equal_time():
    """
    A sine would linger at both ends and hurry through the middle.

    The middle of this range is the band a ball two metres away sits in, which
    is the most likely place for one to be -- so it is the last band that should
    be rushed.
    """
    quarter = head_sweep(1.5, 0.0, 4.0, 6.0)
    half = head_sweep(3.0, 0.0, 4.0, 6.0)
    assert quarter == pytest.approx(half / 2.0)


def test_a_zero_period_holds_the_head_still_rather_than_dividing_by_zero():
    assert head_sweep(4.0, 3.0, 6.5, 0.0) == pytest.approx(3.0)
