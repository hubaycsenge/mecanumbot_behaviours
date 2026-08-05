"""
Unit tests for the image-to-map mapping.

Every sign convention in this package is fixed here, because these are the ones
that produce a robot which confidently drives the wrong way rather than an error
anybody notices. Run under bare pytest:

    python3 -m pytest \
        src/mecanumbot_behaviours/mecanumbot_ostensive_behaviour/test/test_cue_geometry.py
"""

import math

from mecanumbot_ostensive_behaviour.behaviours.cue_geometry import (
    angular_difference,
    bearing_between,
    bearing_from_image_x,
    bearing_offset_from_image_x,
    cue_bearing,
    cue_goal_position,
    match_by_bearing,
    nearest_index,
    wrap_angle,
)

HFOV = math.radians(60.0)


# ---------------------------------------------------------------------------
# Image column to bearing
# ---------------------------------------------------------------------------
def test_the_middle_column_is_straight_ahead():
    assert math.isclose(bearing_offset_from_image_x(0.5, HFOV), 0.0, abs_tol=1e-12)


def test_the_left_of_the_image_is_a_positive_bearing():
    # Yaw is counter-clockwise positive, so the robot's left is positive, and
    # the robot's left is the left-hand side of the image.
    assert bearing_offset_from_image_x(0.0, HFOV) > 0.0
    assert math.isclose(bearing_offset_from_image_x(0.0, HFOV), HFOV / 2, abs_tol=1e-12)


def test_the_right_of_the_image_is_a_negative_bearing():
    assert math.isclose(
        bearing_offset_from_image_x(1.0, HFOV), -HFOV / 2, abs_tol=1e-12
    )


def test_the_bearing_is_taken_from_the_robots_heading():
    yaw = math.radians(90.0)
    bearing = bearing_from_image_x(0.0, HFOV, yaw)
    assert math.isclose(math.degrees(bearing), 120.0, abs_tol=1e-9)


def test_the_bearing_wraps_past_pi():
    bearing = bearing_from_image_x(0.0, HFOV, math.radians(170.0))
    assert math.isclose(math.degrees(bearing), -160.0, abs_tol=1e-9)


# ---------------------------------------------------------------------------
# Where a pointing arm points
# ---------------------------------------------------------------------------
def test_facing_each_other_the_persons_left_is_the_robots_right():
    # Robot at the origin looking along +x, person two metres ahead facing back
    # at it. They point to their own left, a quarter turn (azimuth +90 deg).
    # Their left is the robot's right, which is -y in the map.
    bearing = cue_bearing(person_xy=(2.0, 0.0), robot_xy=(0.0, 0.0), azimuth=math.pi / 2)
    assert math.isclose(math.degrees(bearing), -90.0, abs_tol=1e-9)


def test_the_persons_right_arm_sends_the_robot_to_its_own_left():
    bearing = cue_bearing((2.0, 0.0), (0.0, 0.0), -math.pi / 2)
    assert math.isclose(math.degrees(bearing), 90.0, abs_tol=1e-9)


def test_pointing_straight_ahead_of_themselves_sends_the_robot_back_past_itself():
    # Zero azimuth is the direction the person faces, which is at the robot.
    bearing = cue_bearing((2.0, 0.0), (0.0, 0.0), 0.0)
    assert math.isclose(abs(math.degrees(bearing)), 180.0, abs_tol=1e-9)


def test_the_cue_follows_where_the_person_stands_not_where_the_robot_looks():
    # The same gesture indicates a different world direction depending on where
    # the person is standing -- which is the whole reason the ray is anchored at
    # them. Here they stand north of the robot, so they face south to look at
    # it, and their left is east: 0 degrees, a quarter turn from the case above.
    ahead = cue_bearing((2.0, 0.0), (0.0, 0.0), math.pi / 2)
    beside = cue_bearing((0.0, 2.0), (0.0, 0.0), math.pi / 2)
    assert math.isclose(math.degrees(ahead), -90.0, abs_tol=1e-9)
    assert math.isclose(math.degrees(beside), 0.0, abs_tol=1e-9)


def test_the_goal_is_placed_along_the_bearing_from_the_person():
    x, y = cue_goal_position((2.0, 0.0), math.radians(-90.0), 1.5)
    assert math.isclose(x, 2.0, abs_tol=1e-9)
    assert math.isclose(y, -1.5, abs_tol=1e-9)


def test_the_goal_distance_is_measured_from_the_person():
    origin = (1.0, -3.0)
    goal = cue_goal_position(origin, math.radians(37.0), 2.5)
    assert math.isclose(math.dist(origin, goal), 2.5, abs_tol=1e-9)


# ---------------------------------------------------------------------------
# Matching an image bearing to a fused pose
# ---------------------------------------------------------------------------
def test_the_nearest_candidate_in_bearing_is_matched():
    robot = (0.0, 0.0)
    candidates = [(0.0, 3.0), (3.0, 0.1), (-3.0, 0.0)]
    index = match_by_bearing(math.radians(2.0), robot, candidates, math.radians(20.0))
    assert index == 1


def test_a_candidate_outside_the_tolerance_is_not_matched():
    robot = (0.0, 0.0)
    candidates = [(0.0, 3.0)]  # 90 degrees away
    assert (
        match_by_bearing(math.radians(0.0), robot, candidates, math.radians(20.0))
        is None
    )


def test_an_empty_candidate_list_matches_nothing():
    assert match_by_bearing(0.0, (0.0, 0.0), [], math.radians(20.0)) is None


def test_matching_wraps_around_the_back_of_the_robot():
    # A person almost straight behind: 179 and -179 degrees are two degrees
    # apart, not 358.
    robot = (0.0, 0.0)
    candidates = [(-3.0, -0.05)]
    index = match_by_bearing(math.radians(179.0), robot, candidates, math.radians(20.0))
    assert index == 0


# ---------------------------------------------------------------------------
# Keeping hold of the same person between camera frames
# ---------------------------------------------------------------------------
def test_the_nearest_torso_is_taken_as_the_same_person():
    points = [(0.20, 0.40), (0.52, 0.41), (0.80, 0.39)]
    assert nearest_index(points, (0.50, 0.40), 0.25) == 1


def test_a_person_who_moved_too_far_is_not_the_same_person():
    assert nearest_index([(0.90, 0.40)], (0.50, 0.40), 0.25) is None


def test_people_without_a_torso_are_skipped():
    points = [None, (0.52, 0.41), None]
    assert nearest_index(points, (0.50, 0.40), 0.25) == 1


def test_a_frame_with_nobody_in_it_matches_nothing():
    assert nearest_index([], (0.50, 0.40), 0.25) is None
    assert nearest_index([None, None], (0.50, 0.40), 0.25) is None


# ---------------------------------------------------------------------------
# Angle helpers
# ---------------------------------------------------------------------------
def test_wrap_angle_brings_angles_into_range():
    assert math.isclose(wrap_angle(3.0 * math.pi), math.pi, abs_tol=1e-9)
    # Half a turn comes back as +pi or -pi depending on which side of it the
    # floating-point sine lands; both name the same direction.
    assert math.isclose(abs(wrap_angle(-3.0 * math.pi)), math.pi, abs_tol=1e-9)
    assert math.isclose(
        wrap_angle(math.radians(370.0)), math.radians(10.0), abs_tol=1e-9
    )


def test_angular_difference_takes_the_short_way_round():
    difference = angular_difference(math.radians(179.0), math.radians(-179.0))
    assert math.isclose(math.degrees(difference), 2.0, abs_tol=1e-9)


def test_bearing_between_points_the_right_way():
    assert math.isclose(bearing_between((0.0, 0.0), (1.0, 0.0)), 0.0, abs_tol=1e-12)
    assert math.isclose(
        bearing_between((0.0, 0.0), (0.0, 1.0)), math.pi / 2, abs_tol=1e-12
    )
