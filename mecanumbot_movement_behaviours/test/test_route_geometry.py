#!/usr/bin/env python3
"""
Unit tests for placing people on the route.

The claim under test is the one `DogResumeLeading` acts on: when the robot has
just found the human again, does leading resume at the checkpoint it is nearest
or at the one after it. Getting that backwards means the robot walks on without
the person it has just gone to the trouble of finding.

The geometry here is the real route from
`mecanumbot_leading_behaviour/config/Eto_behaviour_setting_constants.yaml`, so
the numbers in the failure cases are the ones that actually occurred.

Pure Python: no ROS, no py_trees.
"""

import math

import pytest

from mecanumbot_movement_behaviours.geometry import (
    human_is_ahead_on_route,
    route_progress,
)


class Point:
    """The `.x` / `.y` a `geometry_msgs` position supplies, without the import."""

    def __init__(self, x, y):
        self.x = float(x)
        self.y = float(y)


# The Eto route: start, then three waypoints.
ROUTE = [
    Point(1.35244, 4.01954),
    Point(-3.06462, 5.40894),
    Point(-4.28269, 1.40561),
    Point(-0.033288, -0.398331),
]

MARGIN = 0.15  # resume_passed_margin, from the same file.


def on_route(progress):
    """Return a point exactly `progress` of the way along the polyline."""
    index = min(int(progress), len(ROUTE) - 2)
    fraction = progress - index
    start, end = ROUTE[index], ROUTE[index + 1]
    return Point(
        start.x + (end.x - start.x) * fraction,
        start.y + (end.y - start.y) * fraction,
    )


class TestRouteProgress:
    def test_a_checkpoint_is_its_own_index(self):
        for index, checkpoint in enumerate(ROUTE[:-1]):
            assert route_progress(ROUTE, checkpoint) == pytest.approx(index, abs=1e-6)

    def test_halfway_along_the_first_leg(self):
        assert route_progress(ROUTE, on_route(0.5)) == pytest.approx(0.5, abs=1e-6)

    def test_a_point_beside_the_route_still_projects_onto_it(self):
        # Not a defect, but the property that makes the corridor question worth
        # asking separately: progress says nothing about how far off the route
        # somebody is standing.
        beside = on_route(0.5)
        beside.y += 0.9
        assert 0.0 < route_progress(ROUTE, beside) < 1.0


class TestHumanIsAheadOnRoute:
    def test_a_human_further_along_is_ahead(self):
        assert human_is_ahead_on_route(ROUTE, on_route(0.10), on_route(0.60), MARGIN)

    def test_a_human_behind_the_robot_is_not_ahead(self):
        assert not human_is_ahead_on_route(
            ROUTE, on_route(0.60), on_route(0.26), MARGIN
        )

    def test_the_regression_that_made_the_robot_walk_on(self):
        # The old rule compared the human's progress with the INDEX of the
        # checkpoint nearest the robot, so this case -- robot well along the
        # first leg, human behind it but past checkpoint 0 -- read as "the
        # human is past checkpoint 0, lead on to checkpoint 1" and the robot
        # left them there. Both are on the leg from checkpoint 0 to 1, so the
        # nearest checkpoint to the robot is 0 and the old test was
        # `0.40 > 0 + 0.15`, which is true.
        robot, human = on_route(0.75), on_route(0.40)
        assert route_progress(ROUTE, human) > 0 + MARGIN  # what the old rule asked
        assert not human_is_ahead_on_route(ROUTE, robot, human, MARGIN)

    def test_the_case_from_the_2026_09_23_run(self):
        # The one in the log: robot at 0.10, human at 0.26. The old rule and
        # the new one agree here -- the robot happened to be less than a margin
        # along the leg -- which is why the bug did not show up in that line.
        robot, human = on_route(0.10), on_route(0.26)
        assert human_is_ahead_on_route(ROUTE, robot, human, MARGIN)

    def test_level_is_not_ahead(self):
        here = on_route(0.4)
        assert not human_is_ahead_on_route(ROUTE, here, here, MARGIN)

    def test_the_margin_has_to_be_cleared(self):
        robot = on_route(0.40)
        assert not human_is_ahead_on_route(ROUTE, robot, on_route(0.50), MARGIN)
        assert human_is_ahead_on_route(ROUTE, robot, on_route(0.60), MARGIN)

    def test_it_works_across_a_later_leg(self):
        assert human_is_ahead_on_route(ROUTE, on_route(1.2), on_route(2.1), MARGIN)
        assert not human_is_ahead_on_route(ROUTE, on_route(2.1), on_route(1.2), MARGIN)

    def test_an_empty_route_places_nobody(self):
        assert not human_is_ahead_on_route([], Point(0, 0), Point(1, 1), MARGIN)

    def test_a_person_standing_beside_the_robot_is_not_ahead_of_it(self):
        # Standing shoulder to shoulder, a metre to the side. Their projections
        # onto the route are the same point, so neither is ahead.
        robot = on_route(0.5)
        beside = Point(robot.x, robot.y + 1.0)
        # Offset perpendicular to the leg, so the projection does not move.
        leg_x, leg_y = ROUTE[1].x - ROUTE[0].x, ROUTE[1].y - ROUTE[0].y
        length = math.hypot(leg_x, leg_y)
        beside = Point(robot.x - leg_y / length, robot.y + leg_x / length)
        assert not human_is_ahead_on_route(ROUTE, robot, beside, MARGIN)
