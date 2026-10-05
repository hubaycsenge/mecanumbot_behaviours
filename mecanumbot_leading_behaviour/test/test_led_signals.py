"""What the LED condition's lights show while the robot leads."""

import math

import pytest

from mecanumbot_leading_behaviour.behaviours.led_signals import (
    BACK_CORNERS,
    FRONT_CORNERS,
    LEFT,
    PANEL_LEDS,
    direction_corners,
    RIGHT,
    STRAIGHT,
    facing_corners,
    lead_side,
    progress_fill,
)

BAND = math.radians(30.0)


# --- progress ----------------------------------------------------------------


def test_nothing_is_lit_before_the_robot_has_moved():
    assert progress_fill(8.0, 8.0) == 0


def test_the_bar_is_full_where_the_robot_parks_not_at_the_target_itself():
    assert progress_fill(8.5, 0.5, stop_distance=0.5) == PANEL_LEDS


@pytest.mark.parametrize('eighths', range(PANEL_LEDS + 1))
def test_one_led_per_eighth_of_the_way(eighths):
    distance = 8.0 - eighths
    assert progress_fill(8.0, distance) == eighths


def test_an_led_is_only_lit_once_its_eighth_is_driven():
    assert progress_fill(8.0, 7.01) == 0
    assert progress_fill(8.0, 6.99) == 1


def test_driving_away_from_the_target_shows_nothing_rather_than_less_than_nothing():
    assert progress_fill(8.0, 9.5) == 0


def test_overshooting_the_stop_distance_is_still_a_full_bar():
    assert progress_fill(8.0, 0.1, stop_distance=0.5) == PANEL_LEDS


def test_a_drive_that_starts_where_it_ends_counts_as_done():
    assert progress_fill(0.3, 0.3, stop_distance=0.5) == PANEL_LEDS


# --- direction ---------------------------------------------------------------


def test_a_destination_ahead_is_straight():
    assert lead_side(0.0, BAND) == STRAIGHT
    assert lead_side(math.radians(29.0), BAND) == STRAIGHT
    assert lead_side(math.radians(-29.0), BAND) == STRAIGHT


def test_a_positive_bearing_is_to_the_left():
    assert lead_side(math.radians(60.0), BAND) == LEFT
    assert lead_side(math.radians(-60.0), BAND) == RIGHT


def test_a_destination_behind_still_has_a_side():
    assert lead_side(math.radians(170.0), BAND) == LEFT
    assert lead_side(math.radians(-170.0), BAND) == RIGHT


def test_a_side_being_shown_is_held_inside_the_band():
    bearing = math.radians(25.0)
    assert lead_side(bearing, BAND, previous=STRAIGHT) == STRAIGHT
    assert lead_side(bearing, BAND, previous=LEFT) == LEFT


def test_a_held_side_is_let_go_near_straight_ahead():
    assert lead_side(math.radians(10.0), BAND, previous=LEFT) == STRAIGHT


def test_the_other_side_is_not_held():
    assert lead_side(math.radians(-25.0), BAND, previous=LEFT) == STRAIGHT


# --- addressing --------------------------------------------------------------


def test_a_person_in_front_is_shown_the_front_panels():
    assert facing_corners(0.0) == FRONT_CORNERS
    assert facing_corners(math.radians(80.0)) == FRONT_CORNERS
    assert facing_corners(math.radians(-80.0)) == FRONT_CORNERS


def test_a_person_behind_is_shown_the_back_panels():
    assert facing_corners(math.pi) == BACK_CORNERS
    assert facing_corners(math.radians(-100.0)) == BACK_CORNERS


# --- pointing at the target ----------------------------------------------------

SPREAD = math.radians(67.5)


@pytest.mark.parametrize(
    "bearing_deg, expected",
    [
        (0.0, ("fl", "fr")),
        (45.0, ("fl",)),
        (90.0, ("fl", "bl")),
        (135.0, ("bl",)),
        (180.0, ("bl", "br")),
        (-135.0, ("br",)),
        (-90.0, ("fr", "br")),
        (-45.0, ("fr",)),
    ],
)
def test_the_panels_on_the_targets_side_point_at_it(bearing_deg, expected):
    assert direction_corners(math.radians(bearing_deg), SPREAD) == expected


def test_a_target_behind_a_robot_facing_the_person_lights_the_back():
    assert direction_corners(math.radians(170.0), SPREAD) == BACK_CORNERS


def test_some_panel_always_points_at_the_target():
    for degrees in range(-180, 181):
        assert direction_corners(math.radians(degrees), SPREAD)


def test_a_target_on_the_edge_between_two_directions_does_not_flicker():
    just_past = math.radians(26.0)
    assert direction_corners(just_past, SPREAD) == ("fl",)
    assert direction_corners(just_past, SPREAD, previous=FRONT_CORNERS) == FRONT_CORNERS
