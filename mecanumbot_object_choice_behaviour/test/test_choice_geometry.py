"""
Which brick a pointing cue picks out.

The layout is the classic object-choice one: the person stands between two
bricks, facing the robot, and points at one. No ROS:
`PYTHONPATH=. python3 -m pytest test/test_choice_geometry.py`.
"""

import math

import pytest

from mecanumbot_object_choice_behaviour import choice_geometry as cg

# The person at the origin facing the robot, which stands 2.5 m away along +x.
# Bricks 1 m to either side of the person.
PERSON = (0.0, 0.0)
BRICKS = {"red": (0.0, 1.0, 0.03), "blue": (0.0, -1.0, 0.03)}
MAX = math.radians(35.0)
MARGIN = math.radians(10.0)


def deg(value):
    return math.radians(value)


def test_pointing_straight_at_a_brick_chooses_it():
    choice = cg.choose(PERSON, deg(90), BRICKS, MAX, MARGIN)
    assert choice.chosen and choice.colour == "red"
    assert choice.reason == cg.CHOSEN
    assert choice.margin == pytest.approx(math.pi)


def test_a_rough_point_still_chooses_the_right_side():
    # Arms rarely point exactly: 30 deg off towards the robot is still blue.
    choice = cg.choose(PERSON, deg(-60), BRICKS, MAX, MARGIN)
    assert choice.colour == "blue"


def test_pointing_where_there_is_no_brick_chooses_none():
    choice = cg.choose(PERSON, deg(0), BRICKS, MAX, MARGIN)
    assert not choice.chosen and choice.reason == cg.NONE_IN_RANGE


def test_two_bricks_in_line_are_ambiguous():
    bricks = {"red": (0.5, 1.0), "green": (0.6, 1.1)}
    choice = cg.choose(PERSON, deg(63), bricks, MAX, MARGIN)
    assert not choice.chosen and choice.reason == cg.AMBIGUOUS


def test_one_brick_needs_no_margin():
    choice = cg.choose(PERSON, deg(90), {"red": (0.0, 1.0)}, MAX, MARGIN)
    assert choice.colour == "red" and choice.margin is None


def test_a_brick_at_the_persons_feet_is_left_out():
    bricks = {"red": (0.05, 0.05), "blue": (0.0, -1.0)}
    choice = cg.choose(PERSON, deg(-90), bricks, MAX, MARGIN, min_distance=0.3)
    assert choice.colour == "blue"
    assert choice.candidates[-1].excluded == cg.TOO_CLOSE


def test_no_usable_bricks():
    choice = cg.choose(PERSON, deg(90), {}, MAX, MARGIN)
    assert choice.reason == cg.NO_BRICKS


def test_bearing_wraps_through_pi():
    bricks = {"red": (-1.0, 0.05), "blue": (-1.0, -0.6)}
    choice = cg.choose(PERSON, deg(-179), bricks, MAX, deg(5))
    assert choice.colour == "red"


def test_record_is_plain_data_in_degrees():
    record = cg.choose(PERSON, deg(80), BRICKS, MAX, MARGIN).as_record()
    assert record["choice"] == "red"
    first = record["candidates"][0]
    assert first["colour"] == "red" and first["deviation_deg"] == pytest.approx(10.0)
    assert first["distance_from_person"] == pytest.approx(1.0)
