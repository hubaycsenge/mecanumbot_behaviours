"""
Tests for deciding why the robot cannot have the thing it found.

This is what routes an episode into the alert instead of the grasp, so the tests
pin the two judgements that matter: that height is decided before distance, and
that an unknown height does not by itself stop the robot from trying. Getting
the first wrong tells a person to look for an obstacle that is not there;
getting the second wrong makes the robot give up on everything on the floor
whenever the point cloud is thin.
"""

import pytest

from mecanumbot_seek.reachability import (
    GRIP_FAILED,
    LOST,
    NO_ROUTE,
    PHRASES,
    REACHABLE,
    REASONS,
    TOO_HIGH,
    TOO_LOW,
    assess,
    describe,
    grasp_band,
    reachable,
)

# The robot's actual geometry: shafts at z ~ 0.034 with a 0.116 m clear gap.
BAND = {"grasp_height_min": 0.03, "grasp_height_max": 0.15}


class TestHeight:
    """The band the grabbers can close on."""

    @pytest.mark.parametrize("height", [0.03, 0.07, 0.15])
    def test_inside_the_band_is_reachable(self, height):
        assert assess(height=height, distance=0.1, **BAND) == REACHABLE

    def test_a_mug_on_a_table_is_too_high(self):
        assert assess(height=0.75, distance=0.1, **BAND) == TOO_HIGH

    def test_something_in_a_recess_is_too_low(self):
        assert assess(height=0.005, distance=0.1, **BAND) == TOO_LOW

    def test_the_band_edges_are_inclusive(self):
        assert assess(height=0.03, distance=0.1, **BAND) == REACHABLE
        assert assess(height=0.15, distance=0.1, **BAND) == REACHABLE
        assert assess(height=0.1501, distance=0.1, **BAND) == TOO_HIGH


class TestPrecedence:
    """Which reason wins when more than one applies."""

    def test_height_is_judged_before_distance(self):
        # A robot parked against a table is close enough and still cannot have
        # the mug on it. Calling that a distance problem would send a person
        # looking for an obstacle that is not there.
        assert assess(height=0.75, distance=99.0, **BAND) == TOO_HIGH

    def test_distance_decides_when_the_height_is_fine(self):
        assert assess(height=0.07, distance=5.0, grasp_distance=0.3, **BAND) == NO_ROUTE

    def test_close_enough_and_the_right_height_is_reachable(self):
        assert (
            assess(height=0.07, distance=0.2, grasp_distance=0.3, **BAND) == REACHABLE
        )


class TestUnknowns:
    """What happens when the server could not place the object fully."""

    def test_an_unknown_height_does_not_refuse_the_grasp(self):
        # The cloud may be thin. Refusing on that basis would make the robot
        # give up on everything on the floor; the grasp itself is the check that
        # always runs.
        assert assess(height=None, distance=0.1, **BAND) == REACHABLE

    def test_an_unknown_height_still_lets_distance_decide(self):
        assert assess(height=None, distance=9.0, grasp_distance=0.3, **BAND) == NO_ROUTE

    def test_an_unknown_distance_does_not_refuse_either(self):
        assert assess(height=0.07, distance=None, **BAND) == REACHABLE

    def test_knowing_nothing_is_reachable(self):
        # Nothing has been established, so nothing is a reason not to try.
        assert assess(**BAND) == REACHABLE


class TestReachable:
    """The predicate the tree branches on."""

    def test_only_reachable_is_reachable(self):
        assert reachable(REACHABLE)
        for reason in REASONS:
            assert not reachable(reason)

    def test_every_reason_has_a_phrase(self):
        for reason in REASONS:
            assert reason in PHRASES
            assert PHRASES[reason]


class TestDescribe:
    """The line the robot alerts somebody with."""

    def test_it_names_the_object(self):
        line = describe(TOO_HIGH, "mug", 0.75)
        assert "mug" in line

    def test_it_falls_back_gracefully_without_a_name(self):
        line = describe(NO_ROUTE, "")
        assert "it" in line
        assert "None" not in line

    def test_too_high_reports_the_height(self):
        # The number that tells somebody whether it is a table or a shelf.
        assert "0.75" in describe(TOO_HIGH, "mug", 0.75)

    def test_other_reasons_do_not_report_a_height(self):
        assert "0.07" not in describe(GRIP_FAILED, "mug", 0.07)

    def test_a_missing_height_does_not_break_too_high(self):
        line = describe(TOO_HIGH, "mug", None)
        assert "mug" in line
        assert "None" not in line

    def test_an_unknown_reason_still_says_something(self):
        line = describe("something_new", "mug")
        assert "mug" in line
        assert line


class TestBand:
    """The log line."""

    def test_the_band_is_reported_in_metres(self):
        assert grasp_band(0.03, 0.15) == "0.03..0.15 m"


class TestReasonsAreDistinct:
    """The alert has to be able to tell the outcomes apart."""

    def test_every_reason_is_its_own_string(self):
        assert len(set(REASONS)) == len(REASONS)
        assert REACHABLE not in REASONS

    def test_the_five_outcomes_are_the_ones_documented(self):
        assert set(REASONS) == {TOO_HIGH, TOO_LOW, NO_ROUTE, GRIP_FAILED, LOST}
