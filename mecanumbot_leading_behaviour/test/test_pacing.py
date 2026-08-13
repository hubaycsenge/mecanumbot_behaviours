"""
Tests for the leading pacing rules.

`behaviours/pacing.py` imports nothing, so this runs without a ROS graph and
without the workspace sourced:

    cd src/mecanumbot_behaviours/mecanumbot_leading_behaviour
    PYTHONPATH=. python3 -m pytest test/test_pacing.py
"""

import pytest

from mecanumbot_leading_behaviour.behaviours.pacing import (
    check_in_due,
    checkpoint_budget,
    route_leg,
    sweep_pattern,
)

# The shipped configuration, so the tests say what the robot actually does.
EVERY_CHECKPOINTS = 2
INTERVAL = 20.0
FOLLOW_THRESHOLD = 1.5
STALE_AFTER = 5.0
LOOKAHEAD = 3


def due(
    checkpoints_since=0,
    seconds_since=0.0,
    subject_age=0.5,
    subject_distance=1.0,
    every_checkpoints=EVERY_CHECKPOINTS,
    interval=INTERVAL,
):
    """Ask the pacing rule with a following human as the starting point."""
    return check_in_due(
        checkpoints_since=checkpoints_since,
        seconds_since=seconds_since,
        subject_age=subject_age,
        subject_distance=subject_distance,
        every_checkpoints=every_checkpoints,
        interval=interval,
        follow_threshold=FOLLOW_THRESHOLD,
        stale_after=STALE_AFTER,
    )


class TestCheckInDue:
    def test_a_human_walking_along_behind_is_left_alone(self):
        assert due()[0] is False

    def test_a_human_never_seen_is_checked_on_at_once(self):
        assert due(subject_age=None)[0] is True

    def test_a_human_out_of_sight_too_long_is_checked_on(self):
        assert due(subject_age=STALE_AFTER + 0.1)[0] is True
        assert due(subject_age=STALE_AFTER - 0.1)[0] is False

    def test_a_human_dropping_behind_is_checked_on(self):
        assert due(subject_distance=FOLLOW_THRESHOLD + 0.1)[0] is True
        assert due(subject_distance=FOLLOW_THRESHOLD - 0.1)[0] is False

    def test_an_unplaceable_human_is_not_a_reason_on_its_own(self):
        # Their age already covers a human who has not been seen; not knowing
        # where somebody is must not, by itself, stop the robot leading.
        assert due(subject_distance=None)[0] is False

    def test_the_checkpoint_habit_falls_due(self):
        assert due(checkpoints_since=EVERY_CHECKPOINTS - 1)[0] is False
        assert due(checkpoints_since=EVERY_CHECKPOINTS)[0] is True

    def test_the_interval_habit_falls_due(self):
        assert due(seconds_since=INTERVAL - 0.1)[0] is False
        assert due(seconds_since=INTERVAL)[0] is True

    def test_either_habit_can_be_switched_off(self):
        assert due(checkpoints_since=99, every_checkpoints=0)[0] is False
        assert due(seconds_since=9999.0, interval=0.0)[0] is False
        # With both off the robot still checks when something is wrong.
        assert due(
            checkpoints_since=99,
            seconds_since=9999.0,
            every_checkpoints=0,
            interval=0.0,
            subject_age=STALE_AFTER + 1.0,
        )[0] is True

    def test_the_reason_is_always_worth_logging(self):
        for kwargs in (
            {},
            {"subject_age": None},
            {"subject_distance": FOLLOW_THRESHOLD + 1.0},
            {"checkpoints_since": EVERY_CHECKPOINTS},
            {"seconds_since": INTERVAL},
        ):
            assert due(**kwargs)[1]


class TestCheckpointBudget:
    def test_the_budget_shrinks_towards_the_next_look_back(self):
        assert checkpoint_budget(0, 3, LOOKAHEAD) == 3
        assert checkpoint_budget(1, 3, LOOKAHEAD) == 2
        assert checkpoint_budget(2, 3, LOOKAHEAD) == 1

    def test_the_lookahead_caps_it(self):
        assert checkpoint_budget(0, 10, 3) == 3

    def test_a_leg_is_never_empty(self):
        # A robot that already owes a look back still has to reach the
        # checkpoint it will make it from.
        assert checkpoint_budget(5, 2, LOOKAHEAD) == 1
        assert checkpoint_budget(0, 0, 0) == 1


class TestRouteLeg:
    def test_a_leg_starts_where_the_robot_is_headed(self):
        assert route_leg(2, 7, 0, EVERY_CHECKPOINTS, LOOKAHEAD)[0] == 2

    def test_a_leg_stops_at_the_end_of_the_route(self):
        assert route_leg(6, 7, 0, EVERY_CHECKPOINTS, LOOKAHEAD) == [6, 7]
        assert route_leg(7, 7, 0, EVERY_CHECKPOINTS, LOOKAHEAD) == [7]

    def test_a_leg_stops_where_the_next_look_back_falls_due(self):
        assert route_leg(0, 7, 0, 2, LOOKAHEAD) == [0, 1]
        assert route_leg(0, 7, 1, 2, LOOKAHEAD) == [0]

    def test_the_lookahead_caps_a_long_leg(self):
        assert route_leg(0, 7, 0, 0, 3) == [0, 1, 2]

    def test_an_index_past_the_end_still_gives_something_to_drive(self):
        assert route_leg(9, 7, 0, EVERY_CHECKPOINTS, LOOKAHEAD) == [7]
        assert route_leg(-1, 7, 0, EVERY_CHECKPOINTS, LOOKAHEAD) == [0, 1]


class TestRoutePoses:
    """
    The waypoint poses a leg is sent as.

    These need `geometry_msgs`, so they are skipped when the tests are run
    against a bare interpreter and run normally under `colcon test`.
    """

    @staticmethod
    def _route():
        pytest.importorskip("geometry_msgs")
        from geometry_msgs.msg import Point

        def point(x, y):
            p = Point()
            p.x, p.y = float(x), float(y)
            return p

        # An L: east along y=0, then north at x=2.
        return [point(0, 0), point(1, 0), point(2, 0), point(2, 1)]

    @staticmethod
    def _yaw(pose):
        import math

        return math.atan2(
            2.0 * pose.orientation.w * pose.orientation.z,
            1.0 - 2.0 * pose.orientation.z**2,
        )

    def test_every_waypoint_faces_the_next_point_of_the_route(self):
        import math

        route = self._route()
        from mecanumbot_leading_behaviour.behaviours.geometry import route_poses

        poses = route_poses(route, [0, 1, 2])
        assert [round(p.position.x, 3) for p in poses] == [0.0, 1.0, 2.0]
        # Along the straight, then turned north for the corner.
        assert self._yaw(poses[0]) == pytest.approx(0.0)
        assert self._yaw(poses[1]) == pytest.approx(0.0)
        assert self._yaw(poses[2]) == pytest.approx(math.pi / 2.0)

    def test_the_last_checkpoint_faces_the_target_beyond_it(self):
        import math

        route = self._route()
        from geometry_msgs.msg import Point
        from mecanumbot_leading_behaviour.behaviours.geometry import route_poses

        target = Point()
        target.x, target.y = 2.0, 5.0
        poses = route_poses(route, [3], look_beyond=target)
        assert self._yaw(poses[0]) == pytest.approx(math.pi / 2.0)

    def test_without_a_target_the_last_checkpoint_keeps_its_approach_heading(self):
        import math

        route = self._route()
        from mecanumbot_leading_behaviour.behaviours.geometry import route_poses

        poses = route_poses(route, [3])
        assert self._yaw(poses[0]) == pytest.approx(math.pi / 2.0)


class TestSweepPattern:
    def test_the_sweep_ends_where_it_started(self):
        assert sum(sweep_pattern(0.4)) == pytest.approx(0.0)

    def test_the_sweep_reaches_one_step_either_side(self):
        offsets = sweep_pattern(0.4)
        travelled = 0.0
        reached = []
        for offset in offsets:
            travelled += offset
            reached.append(round(travelled, 6))
        assert max(reached) == pytest.approx(0.4)
        assert min(reached) == pytest.approx(-0.4)
