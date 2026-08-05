"""
Turning image-space observations into map-frame angles and goals.

This is the bridge between what the camera sees and where the robot drives, and
it is the part most easily got backwards, so the two sign conventions are
written down once here and used everywhere else.

**Image x to bearing.** Normalized image x runs 0 at the left edge to 1 at the
right. The robot's yaw is counter-clockwise-positive, so the left of the image
is the *positive* side: a person at x = 0.2 sits at a positive bearing offset
from the camera axis. This matches `cam_to_angle` in the two camera detectors of
`mecanumbot_sensorprocess_smart`, which invert x for exactly the same reason.

**Where a pointing arm points.** `gestures.pointing_cue` returns an azimuth
measured from the direction the person is facing, positive towards their left.
The person facing the robot is the assumption that makes that usable: their
facing direction is then the bearing from them to the robot, and the cue is that
bearing rotated by the azimuth. The two signs agree without a flip -- facing
each other, the person's left is the robot's right, and both are the same
absolute direction in the map.

Anchoring the resulting ray at the *person* rather than at the robot is a
deliberate choice. "Over there" is said from where the speaker stands, and with
the robot typically a metre or two off to one side, a ray drawn from the robot
instead lands somewhere the person never indicated.

Pure Python -- no ROS messages, no py_trees -- so the whole mapping is
unit-tested with plain tuples.
"""

import math


def wrap_angle(angle):
    """Wrap an angle to [-pi, pi]."""
    # Deliberately not imported from the leading package's `geometry`: keeping
    # this module free of `geometry_msgs` is what lets its tests run under bare
    # pytest, with no ROS environment sourced.
    return math.atan2(math.sin(angle), math.cos(angle))


def bearing_offset_from_image_x(x_normalized, hfov):
    """
    Return the bearing [rad] of an image column, relative to the camera axis.

    Positive is to the robot's left, which is the left-hand side of the image.
    """
    return (0.5 - float(x_normalized)) * float(hfov)


def bearing_from_image_x(x_normalized, hfov, robot_yaw):
    """Return the absolute map-frame bearing [rad] of an image column."""
    return wrap_angle(robot_yaw + bearing_offset_from_image_x(x_normalized, hfov))


def bearing_between(from_xy, to_xy):
    """Return the map-frame bearing [rad] from one (x, y) to another."""
    return math.atan2(to_xy[1] - from_xy[1], to_xy[0] - from_xy[0])


def cue_bearing(person_xy, robot_xy, azimuth):
    """
    Return the map-frame direction [rad] a pointing cue indicates.

    The person is taken to be facing the robot, so the cue is the person-to-robot
    bearing turned by `azimuth` (positive towards the person's left).
    """
    return wrap_angle(bearing_between(person_xy, robot_xy) + azimuth)


def cue_goal_position(person_xy, bearing, distance):
    """Return the (x, y) `distance` metres along `bearing` from the person."""
    return (
        person_xy[0] + distance * math.cos(bearing),
        person_xy[1] + distance * math.sin(bearing),
    )


def angular_difference(first, second):
    """Return the signed smallest angle [rad] that turns `first` into `second`."""
    return wrap_angle(second - first)


def nearest_index(points, reference, max_distance):
    """
    Return the index of the point nearest a reference, or None if none is close.

    This is the image-space half of the association: `points` are the torso
    positions of the people on the newest camera frame and `reference` is where
    the person being followed was last seen. The camera detector publishes no
    track IDs, so how far a torso moved between frames is what identity is made
    of, and `max_distance` is how big a step still counts as the same person.
    Entries may be None, for a person whose torso could not be located.
    """
    best_index, best_distance = None, float("inf")
    for index, point in enumerate(points):
        if point is None:
            continue
        distance = math.hypot(point[0] - reference[0], point[1] - reference[1])
        if distance < best_distance:
            best_index, best_distance = index, distance
    if best_index is None or best_distance > max_distance:
        return None
    return best_index


def match_by_bearing(target_bearing, robot_xy, candidates, tolerance):
    """
    Return the index of the candidate lying closest to a bearing, or None.

    This is how a person seen in the image is matched to their metric position:
    the camera gives a bearing and `people_fusion` gives map-frame poses, so the
    fused pose whose bearing from the robot agrees best is the same person.
    Candidates further off than `tolerance` are not matched at all, which is
    what keeps a second person standing elsewhere in the room from being adopted
    when the addressee's own fused pose has momentarily dropped out.
    """
    best_index, best_error = None, float("inf")
    for index, candidate in enumerate(candidates):
        error = abs(
            angular_difference(target_bearing, bearing_between(robot_xy, candidate))
        )
        if error < best_error:
            best_index, best_error = index, error
    if best_index is None or best_error > tolerance:
        return None
    return best_index
