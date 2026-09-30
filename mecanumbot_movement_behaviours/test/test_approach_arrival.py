#!/usr/bin/env python3
"""
Unit tests for when a stepped approach has walked up to a person.

The claim under test is the one the dog tree's seek-attention walk acts on:
the robot keeps stepping until it stands within the closeness threshold, with a
margin for the goal tolerance nav2 parks within. The numbers are those of the
run of 2026-09-24, where a single 1 m step left the robot 3.1 m from the human
and the approach still called them reached.

Pure Python: no ROS, no py_trees.
"""

from mecanumbot_movement_behaviours.geometry import approach_arrived

CLOSENESS = 0.75  # robot_closeness_threshold in the Eto constants
MARGIN = 0.35  # approach_arrive_margin default


def test_one_step_from_three_metres_is_not_arrived():
    assert not approach_arrived(3.11, CLOSENESS, MARGIN)


def test_parked_within_nav2_tolerance_is_arrived():
    # nav2 stops within 0.30 m of the goal, which sits at the closeness threshold.
    assert approach_arrived(CLOSENESS + 0.30, CLOSENESS, MARGIN)


def test_already_closer_than_the_threshold_is_arrived():
    assert approach_arrived(0.5, CLOSENESS, MARGIN)


def test_without_margin_the_threshold_itself_is_the_limit():
    assert approach_arrived(CLOSENESS, CLOSENESS)
    assert not approach_arrived(CLOSENESS + 0.01, CLOSENESS)
