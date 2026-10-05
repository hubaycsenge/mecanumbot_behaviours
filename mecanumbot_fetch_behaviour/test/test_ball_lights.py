#!/usr/bin/env python3
"""
Tests for which lights point at the ball.

Written as claims about what somebody watching sees: the panels on the ball's
side are the ones in the direction colour, a ball off a corner lights that
corner alone, and a ball on the edge between two directions does not make the
lights flicker. No ROS: `ball_lights` is plain Python.
"""

import math

import pytest

from mecanumbot_fetch_behaviour.ball_lights import (
    CORNERS,
    HYSTERESIS,
    direction_corners,
    panel_colors,
)

SPREAD = math.radians(67.5)


@pytest.mark.parametrize(
    "bearing_deg, expected",
    [
        (0.0, ("fl", "fr")),
        (45.0, ("fl",)),
        (90.0, ("fl", "bl")),
        (135.0, ("bl",)),
        (180.0, ("bl", "br")),
        (-180.0, ("bl", "br")),
        (-135.0, ("br",)),
        (-90.0, ("fr", "br")),
        (-45.0, ("fr",)),
    ],
)
def test_the_panels_on_the_balls_side_point_at_it(bearing_deg, expected):
    assert direction_corners(math.radians(bearing_deg), SPREAD) == expected


def test_a_positive_bearing_is_to_the_left():
    assert "fl" in direction_corners(math.radians(60.0), SPREAD)
    assert "fr" not in direction_corners(math.radians(60.0), SPREAD)


def test_some_panel_always_points_at_the_ball():
    for degrees in range(-180, 181):
        assert direction_corners(math.radians(degrees), SPREAD)


def test_a_bearing_past_a_full_turn_is_the_same_direction():
    assert direction_corners(math.radians(370.0), SPREAD) == direction_corners(
        math.radians(10.0), SPREAD
    )


def test_a_ball_on_the_edge_does_not_flip_the_lights():
    # 22.5 degrees is where the front-right panel stops pointing at the ball.
    edge = math.radians(22.5)
    just_past = edge + HYSTERESIS / 2.0
    assert direction_corners(just_past, SPREAD) == ("fl",)
    assert direction_corners(just_past, SPREAD, previous=("fl", "fr")) == ("fl", "fr")


def test_a_held_panel_is_let_go_once_the_ball_is_well_past():
    well_past = math.radians(22.5) + HYSTERESIS * 2.0
    assert direction_corners(well_past, SPREAD, previous=("fl", "fr")) == ("fl",)


def test_every_panel_is_lit_and_only_the_direction_ones_change_colour():
    colors = panel_colors(("fl", "bl"), color=1, direction_color=2)
    assert set(colors) == set(CORNERS)
    assert colors == {"fl": 2, "fr": 1, "bl": 2, "br": 1}
