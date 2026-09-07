"""
Tests for the expanding ring search geometry.

The property the whole pattern exists for is the ordering: every point at one
radius is visited before any point further out, so the places the object is most
likely to be are searched first. That, and the waypoints facing inwards -- a
robot walking a ring around a place while looking away from it is not searching
anything.
"""

import math

import pytest

from mecanumbot_seek.search_patterns import (
    expanding_search,
    nearest_unvisited,
    radii_up_to,
    ring,
    ring_count,
    sweep_complete,
)


def distance(point, centre=(0.0, 0.0)):
    """Planar distance of a waypoint from a centre."""
    return math.hypot(point[0] - centre[0], point[1] - centre[1])


class TestRing:
    """One lap around a place."""

    def test_every_waypoint_is_at_the_radius(self):
        for x, y, _ in ring((2.0, -1.0), 3.0, 8):
            assert distance((x, y), (2.0, -1.0)) == pytest.approx(3.0)

    def test_the_waypoints_are_evenly_spaced(self):
        points = ring((0.0, 0.0), 1.0, 4)
        angles = sorted(math.atan2(y, x) for x, y, _ in points)
        gaps = [b - a for a, b in zip(angles, angles[1:])]
        assert all(gap == pytest.approx(math.pi / 2) for gap in gaps)

    def test_every_waypoint_faces_the_centre(self):
        # The whole point of walking a ring around a place: the robot looks at
        # the region it is circling, not out into the room beyond it.
        centre = (5.0, 5.0)
        for x, y, yaw in ring(centre, 2.0, 6):
            inward = math.atan2(centre[1] - y, centre[0] - x)
            assert math.atan2(
                math.sin(yaw - inward), math.cos(yaw - inward)
            ) == pytest.approx(0.0, abs=1e-9)

    def test_the_phase_rotates_the_ring(self):
        plain = ring((0.0, 0.0), 1.0, 4)
        shifted = ring((0.0, 0.0), 1.0, 4, phase=math.pi / 4)
        assert plain[0][:2] != pytest.approx(shifted[0][:2])
        assert distance(shifted[0]) == pytest.approx(1.0)

    def test_a_count_of_zero_still_gives_one_waypoint(self):
        assert len(ring((0.0, 0.0), 1.0, 0)) == 1


class TestRingCount:
    """How many stops a lap needs."""

    def test_a_bigger_ring_gets_more_stops(self):
        assert ring_count(6.0, 1.5) > ring_count(1.5, 1.5)

    def test_the_spacing_is_respected_between_the_bounds(self):
        # circumference / spacing, so a 3 m ring at 1.5 m spacing is about 12.
        assert ring_count(3.0, 1.5, minimum=1, maximum=100) == 13

    def test_a_tight_ring_still_gets_enough_stops_to_see_round(self):
        assert ring_count(0.2, 1.5, minimum=4) == 4

    def test_a_wide_ring_does_not_become_an_endless_route(self):
        assert ring_count(100.0, 0.1, maximum=16) == 16

    def test_degenerate_inputs_fall_back_to_the_minimum(self):
        assert ring_count(0.0, 1.5, minimum=4) == 4
        assert ring_count(3.0, 0.0, minimum=4) == 4


class TestRadii:
    """Which rings a search has."""

    def test_the_radii_step_out_to_the_limit(self):
        assert radii_up_to(4.5, 1.5, 1.5) == pytest.approx([1.5, 3.0, 4.5])

    def test_a_limit_below_the_first_ring_still_gives_one(self):
        # "Look around where it was", not "do not look".
        assert radii_up_to(0.5, 1.5, 1.5) == pytest.approx([1.5])

    def test_a_wider_limit_gives_more_rings(self):
        assert len(radii_up_to(6.0, 1.5, 1.5)) > len(radii_up_to(3.0, 1.5, 1.5))


class TestExpandingSearch:
    """The whole pattern."""

    def test_inner_rings_come_before_outer_ones(self):
        # The property the search exists for: the likely places first.
        waypoints = expanding_search((0.0, 0.0), [1.0, 2.0, 3.0])
        # Rounded, because two waypoints on the same ring differ in the last
        # bit or two of the trig and an exact sort would read that as disorder.
        radii = [round(distance(w), 6) for w in waypoints]
        assert radii == sorted(radii)

    def test_every_waypoint_is_on_one_of_the_rings(self):
        waypoints = expanding_search((1.0, 2.0), [1.0, 2.5])
        for waypoint in waypoints:
            assert distance(waypoint, (1.0, 2.0)) == pytest.approx(
                1.0
            ) or distance(waypoint, (1.0, 2.0)) == pytest.approx(2.5)

    def test_successive_laps_do_not_start_at_the_same_bearing(self):
        # Otherwise the robot runs back and forth along one radial line between
        # laps instead of moving on around.
        waypoints = expanding_search((0.0, 0.0), [1.0, 2.0], minimum=4, maximum=4)
        first_lap_start = math.atan2(waypoints[0][1], waypoints[0][0])
        second_lap_start = math.atan2(waypoints[4][1], waypoints[4][0])
        assert first_lap_start != pytest.approx(second_lap_start)

    def test_no_rings_means_no_waypoints(self):
        assert expanding_search((0.0, 0.0), []) == []


class TestProgress:
    """Walking the pattern."""

    def test_the_first_unvisited_waypoint_is_returned(self):
        waypoints = expanding_search((0.0, 0.0), [1.0], minimum=4, maximum=4)
        assert nearest_unvisited(waypoints, (50.0, 50.0), set(), 0.5) == 0

    def test_a_visited_waypoint_is_skipped(self):
        waypoints = expanding_search((0.0, 0.0), [1.0], minimum=4, maximum=4)
        assert nearest_unvisited(waypoints, (50.0, 50.0), {0, 1}, 0.5) == 2

    def test_standing_on_a_waypoint_marks_it_visited(self):
        # Free progress: on an expanding ring the robot drives past waypoints on
        # its way to others constantly, and re-driving them is wasted time.
        waypoints = expanding_search((0.0, 0.0), [1.0], minimum=4, maximum=4)
        visited = set()
        here = (waypoints[0][0], waypoints[0][1])
        index = nearest_unvisited(waypoints, here, visited, 0.5)
        assert 0 in visited
        assert index == 1

    def test_none_when_everything_has_been_visited(self):
        waypoints = expanding_search((0.0, 0.0), [1.0], minimum=4, maximum=4)
        assert nearest_unvisited(waypoints, (50.0, 50.0), set(range(4)), 0.5) is None

    def test_the_order_is_the_search_not_the_nearest_first(self):
        # Deliberate: the ordering is what puts likely places before unlikely
        # ones, so being closer to a later waypoint does not promote it.
        waypoints = expanding_search((0.0, 0.0), [1.0, 3.0], minimum=4, maximum=4)
        far_but_near_the_robot = waypoints[5]
        robot = (far_but_near_the_robot[0], far_but_near_the_robot[1] + 0.6)
        assert nearest_unvisited(waypoints, robot, set(), 0.5) == 0

    def test_sweep_complete_only_when_all_are_visited(self):
        waypoints = expanding_search((0.0, 0.0), [1.0], minimum=4, maximum=4)
        assert not sweep_complete(waypoints, {0, 1})
        assert sweep_complete(waypoints, set(range(4)))
