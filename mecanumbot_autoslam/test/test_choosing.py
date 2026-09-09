"""
How the next goal is chosen: the frontier, or the server's uncertain region.

The interleaving is the one research decision in this package that is a pure
function, so it is tested as claims about a pass rather than watched in a run.
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from mecanumbot_autoslam.behaviours import choosing  # noqa: E402


class Frontier:
    """The one attribute `select` reads off a scored frontier."""

    def __init__(self, point):
        self.point = point


FRONTIER = Frontier((5.0, 1.0))
REVISIT = [(2.0, 2.0), (3.0, 3.0)]


def test_nothing_to_go_to_is_reported_as_nothing():
    assert choosing.select(None, [], goals_sent=1) == (None, "")


def test_a_frontier_is_the_ordinary_goal():
    point, source = choosing.select(FRONTIER, [], goals_sent=1)
    assert (point, source) == ((5.0, 1.0), choosing.FRONTIER)


def test_every_nth_goal_goes_to_the_server_instead():
    point, source = choosing.select(
        FRONTIER, REVISIT, goals_sent=3, uncertain_every=3)
    assert (point, source) == ((2.0, 2.0), choosing.UNCERTAIN)


def test_the_server_does_not_get_the_goals_in_between():
    for sent in (4, 5):
        _, source = choosing.select(
            FRONTIER, REVISIT, goals_sent=sent, uncertain_every=3)
        assert source == choosing.FRONTIER, sent


def test_uncertain_regions_are_still_visited_when_the_frontiers_run_out():
    # A pass with no frontiers left but regions outstanding should go and look
    # at them, not stop early.
    point, source = choosing.select(None, REVISIT, goals_sent=1)
    assert (point, source) == ((2.0, 2.0), choosing.UNCERTAIN)


def test_the_server_is_ignored_when_revisiting_is_switched_off():
    point, source = choosing.select(
        FRONTIER, REVISIT, goals_sent=3, revisit_uncertain=False)
    assert (point, source) == ((5.0, 1.0), choosing.FRONTIER)


def test_switching_revisiting_off_with_no_frontier_leaves_nothing_to_do():
    assert choosing.select(
        None, REVISIT, goals_sent=3, revisit_uncertain=False) == (None, "")


def test_the_best_region_is_taken_first():
    # The handler publishes the list already ordered, and drops a region once
    # the robot has been near it, so the head of the list is always the one.
    point, _ = choosing.select(None, REVISIT, goals_sent=0)
    assert point == REVISIT[0]


def test_an_interval_of_zero_does_not_divide_by_zero():
    _, source = choosing.select(
        FRONTIER, REVISIT, goals_sent=1, uncertain_every=0)
    assert source == choosing.UNCERTAIN


def test_a_rejected_goal_must_not_advance_the_interleaving():
    """
    Rolling `goals_sent` back on a rejection keeps the ratio honest.

    A pass against a nav2 that is up but not activated rejects every goal. If
    those counted, the "every Nth goal is the server's" ratio would run at the
    tick rate over runs that never happened -- so `DriveToGoal` decrements the
    counter, and this is the property that makes that matter.
    """
    sent = 3
    assert choosing.select(FRONTIER, REVISIT, sent, uncertain_every=3)[1] == \
        choosing.UNCERTAIN
    # The goal was rejected, so it did not happen and the count goes back.
    sent -= 1
    assert choosing.select(FRONTIER, REVISIT, sent, uncertain_every=3)[1] == \
        choosing.FRONTIER
