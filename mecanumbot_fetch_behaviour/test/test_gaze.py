#!/usr/bin/env python3
"""
Tests for where the robot points its camera at a ball.

Written as claims about the behaviour: that one held tilt sees the floor from
near the grabbers to the horizon, that the head moves towards the ball and not
away from it, that it does not chase noise, and that it stays in the range the
neck is allowed. No ROS: `gaze` is plain Python.
"""

import math

import pytest

from mecanumbot_fetch_behaviour.gaze import (
    creep_command,
    creep_distance,
    floor_band,
    image_offset,
    neck_step,
    pick_ball,
    turn_rate,
    vertical_fov,
)

WIDTH, HEIGHT, HFOV = 1280, 720, math.radians(51.0)
VFOV = vertical_fov(WIDTH, HEIGHT, HFOV)
# About where the lens is, and where a tennis ball's centre is [m].
LENS_HEIGHT = 0.2
BALL_CENTRE = 0.0335
RAD_PER_UNIT = 0.5061


def test_the_frame_is_about_30_degrees_tall():
    """The vertical view every other claim here rests on: 51 deg across, 16:9."""
    assert math.degrees(VFOV) == pytest.approx(30.0, abs=0.5)


def test_one_tilt_sees_from_near_the_grabbers_to_the_horizon():
    """
    The reason the search holds the head still.

    With the top edge a few degrees above the horizon, the frame reaches the
    floor well inside half a metre and never stops reaching further: there is
    no band of floor left over for a sweep to look at.
    """
    pitch = -(VFOV / 2.0) + math.radians(4.0)
    near, far = floor_band(LENS_HEIGHT, pitch, VFOV, BALL_CENTRE)
    assert far == math.inf
    assert near < 0.4


def test_a_level_camera_misses_the_floor_in_front_of_the_robot():
    """Why the search tilt is not simply level: the near floor falls out of view."""
    near, far = floor_band(LENS_HEIGHT, 0.0, VFOV, BALL_CENTRE)
    assert far == math.inf
    assert near > 0.45


def test_a_steep_tilt_loses_the_far_floor():
    """Why the search tilt is not the grabber view: far balls are out of frame."""
    near, far = floor_band(LENS_HEIGHT, math.radians(-40.0), VFOV, BALL_CENTRE)
    assert far < 0.5


def test_image_centre_is_straight_ahead():
    """No offset at the principal point."""
    bearing, elevation = image_offset(WIDTH / 2, HEIGHT / 2, WIDTH, HEIGHT, HFOV)
    assert bearing == pytest.approx(0.0)
    assert elevation == pytest.approx(0.0)


def test_image_offsets_follow_the_ros_conventions():
    """Left of centre is positive bearing, above centre is positive elevation."""
    bearing, elevation = image_offset(100, 100, WIDTH, HEIGHT, HFOV)
    assert bearing > 0.0
    assert elevation > 0.0
    bearing, elevation = image_offset(WIDTH - 100, HEIGHT - 100, WIDTH, HEIGHT, HFOV)
    assert bearing < 0.0
    assert elevation < 0.0


def test_the_frame_edge_is_half_the_field_of_view():
    """A pinhole: the edge column is exactly hfov / 2 off the axis."""
    bearing, _ = image_offset(0, HEIGHT / 2, WIDTH, HEIGHT, HFOV)
    assert bearing == pytest.approx(HFOV / 2.0)


def test_the_nearest_ball_is_the_one_centred_on():
    """The biggest box, which is the same ball the tree drives to."""
    far = (200.0, 300.0, 12.0, 12.0, 0.9)
    near = (900.0, 600.0, 80.0, 78.0, 0.5)
    assert pick_ball([far, near]) == near


def test_boxes_under_the_threshold_are_not_centred_on():
    """A ball the tree would not act on is not one the head should follow."""
    weak = (640.0, 360.0, 90.0, 90.0, 0.2)
    assert pick_ball([weak], threshold=0.4) is None


def test_the_head_moves_towards_a_ball_below_the_centre():
    """Ball low in the frame: the neck goes down (smaller units), not up."""
    _, elevation = image_offset(640, 650, WIDTH, HEIGHT, HFOV)
    target = neck_step(4.3, elevation, RAD_PER_UNIT, 0.6, math.radians(4.0), 3.0, 6.5)
    assert target is not None and target < 4.3


def test_the_head_moves_towards_a_ball_above_the_centre():
    """Ball high in the frame: the neck goes up."""
    _, elevation = image_offset(640, 60, WIDTH, HEIGHT, HFOV)
    target = neck_step(4.3, elevation, RAD_PER_UNIT, 0.6, math.radians(4.0), 3.0, 6.5)
    assert target is not None and target > 4.3


def test_the_step_does_not_overshoot():
    """
    With a gain below one the step is short of the full correction.

    The frame behind the elevation is already old when it is acted on, so a
    full correction would carry the ball past the centre.
    """
    elevation = math.radians(-12.0)
    target = neck_step(4.3, elevation, RAD_PER_UNIT, 0.6, math.radians(4.0), 3.0, 6.5)
    full = 4.3 + elevation / RAD_PER_UNIT
    assert full < target < 4.3


def test_the_head_does_not_chase_noise():
    """Inside the deadband there is nothing worth moving a servo for."""
    step = neck_step(4.3, math.radians(2.0), RAD_PER_UNIT, 0.6, math.radians(4.0), 3.0, 6.5)
    assert step is None


def test_the_head_stays_in_its_allowed_range():
    """However far off the ball is, the neck is clamped to low..high."""
    down = neck_step(3.1, math.radians(-30.0), RAD_PER_UNIT, 1.0, 0.0, 3.0, 6.5)
    up = neck_step(6.4, math.radians(30.0), RAD_PER_UNIT, 1.0, 0.0, 3.0, 6.5)
    assert down == pytest.approx(3.0)
    assert up == pytest.approx(6.5)


def test_at_the_limit_the_head_reports_nothing_to_do():
    """A head already clamped does not keep re-sending the same position."""
    assert neck_step(3.0, math.radians(-30.0), RAD_PER_UNIT, 1.0, 0.0, 3.0, 6.5) is None


def test_the_body_turns_towards_the_ball():
    """Ball to the left (positive bearing): positive, counter-clockwise rate."""
    assert turn_rate(math.radians(20.0), 1.5, 0.4, 0.15, math.radians(4.0)) > 0.0
    assert turn_rate(math.radians(-20.0), 1.5, 0.4, 0.15, math.radians(4.0)) < 0.0


def test_a_centred_ball_stops_the_turn():
    """Zero inside the tolerance is how the caller knows it is done."""
    assert turn_rate(math.radians(3.0), 1.5, 0.4, 0.15, math.radians(4.0)) == 0.0


def test_the_last_degrees_are_not_left_to_a_stalled_wheel():
    """Just outside the tolerance the rate is still at least the floor."""
    rate = turn_rate(math.radians(5.0), 1.5, 0.4, 0.15, math.radians(4.0))
    assert rate == pytest.approx(0.15)


def test_the_turn_is_slow_enough_to_keep_seeing_the_ball():
    """A ball at the frame edge does not whip the robot round."""
    assert turn_rate(HFOV / 2.0, 5.0, 0.4, 0.15, math.radians(4.0)) == pytest.approx(0.4)


# --- the last move into the grabbers ----------------------------------------


def test_creep_covers_the_gap_nav2_left():
    # The case seen on the robot: parked 0.70 m out, grab needs 0.28 m.
    assert creep_distance(0.70, 0.28, 0.6) == pytest.approx(0.42)


def test_creep_never_drives_backwards():
    assert creep_distance(0.20, 0.28, 0.6) == 0.0


def test_creep_is_capped_whatever_the_range_estimate_says():
    assert creep_distance(3.0, 0.28, 0.6) == 0.6


def test_creep_steers_towards_the_ball_and_within_the_limit():
    _, left = creep_command(0.3, math.radians(10.0), 0.08, 1.0, 0.3)
    _, right = creep_command(0.3, math.radians(-10.0), 0.08, 1.0, 0.3)
    assert left > 0.0 > right
    _, hard = creep_command(0.3, math.radians(40.0), 0.08, 1.0, 0.3)
    assert hard == pytest.approx(0.3)


def test_creep_holds_its_heading_once_the_ball_is_under_the_lens():
    assert creep_command(0.1, None, 0.08, 1.0, 0.3) == (0.08, 0.0)


def test_creep_stops_when_the_distance_is_used_up():
    assert creep_command(0.0, 0.2, 0.08, 1.0, 0.3) == (0.0, 0.0)
    assert creep_command(-0.02, None, 0.08, 1.0, 0.3) == (0.0, 0.0)
