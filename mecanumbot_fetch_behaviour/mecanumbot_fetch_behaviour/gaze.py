#!/usr/bin/env python3
"""
Where the robot points its camera at a ball: before it has seen one, and after.

## Before: one tilt, held

The camera sits on the neck about 0.2 m above the floor with a vertical field
of view of about 30 degrees (51 horizontally, measured). That is low enough
that a single tilt sees almost all the floor there is: pitched so the top edge
of the frame sits just above the horizon, the bottom edge meets the floor about
0.35 m in front of the lens, and
everything from there out to the far wall is in view at once
(`floor_band` computes it). So the search holds the head at that tilt,
`fetch_head_search`, and does not sweep.

The sweep this replaced (still available as `fetch_head_search_mode: sweep`)
paid three times for nothing. Half of each lap looked at the ceiling or at the
grabbers, where no ball on the floor can be. The head was moving in every frame,
so the detector saw a smeared ball. And the fusion node places a ball with the
neck's *goal* -- the firmware echoes the command and never reads the servo --
so with the head always moving the tilt it placed with was always wrong, and a
ball on the floor came out tens of centimetres under it: `CheckBallReachable`
threw away the rounds it found.

Holding the horizon at the top of the frame is also what finds a ball *as far
away as possible*: a far ball is near the horizon, and the horizon is in view.

## After: keep the ball in the middle of the picture

Once a ball is sighted, the robot keeps it centred -- the neck takes out the
vertical offset and the body the horizontal one. Both work from where the ball
is *in the image*, not from where the fusion node placed it in the map, and
that is on purpose: the image offset needs only the lens's field of view,
while the map position also needs the neck's calibration, which is exactly the
number that has been wrong. A centring loop in image space converges on the
ball whatever the neck's zero is.

As the robot closes on the ball, the ball sinks in the frame and the neck
follows it down, so the near floor comes into view exactly when it is needed;
the lower limit `fetch_head_low` is the pose that looks at the floor just
beyond the lens, where a ball about to be gripped is.

No ROS: the behaviours hand these functions pixel boxes and neck positions.
"""

import math

# What the head does while there is no ball yet.
GAZE_HOLD = "hold"
GAZE_SWEEP = "sweep"
SEARCH_GAZES = (GAZE_HOLD, GAZE_SWEEP)


def focal_length(pixels, fov):
    """Return the focal length in pixels for a frame dimension and its field of view."""
    return (float(pixels) / 2.0) / math.tan(float(fov) / 2.0)


def image_offset(u, v, width, height, hfov):
    """
    Return `(bearing, elevation)` of an image point from the optical axis [rad].

    Bearing is positive to the robot's **left**, elevation positive **up**, the
    same conventions `mecanumbot_sensorprocess_smart`'s `ball_locating` uses.
    A pinhole with square pixels, so the vertical focal length is the
    horizontal one -- which is what every camera on this robot has.
    """
    focal = focal_length(width, hfov)
    bearing = math.atan2(float(width) / 2.0 - float(u), focal)
    elevation = math.atan2(float(height) / 2.0 - float(v), focal)
    return bearing, elevation


def vertical_fov(width, height, hfov):
    """Return the vertical field of view implied by the frame shape [rad]."""
    return 2.0 * math.atan((float(height) / 2.0) / focal_length(width, hfov))


def pick_ball(boxes, threshold=0.0):
    """
    Return the box to centre on, or None.

    `boxes` are `(u, v, width, height, score)`. The biggest box wins, because
    the biggest ball is the nearest one -- the same choice the tree makes when
    it picks which ball to drive to, so the head and the wheels follow the same
    ball.
    """
    best = None
    for box in boxes:
        if float(box[4]) < float(threshold):
            continue
        if best is None or max(box[2], box[3]) > max(best[2], best[3]):
            best = box
    return best


def neck_step(neck, elevation, rad_per_unit, gain, deadband, low, high):
    """
    Return the next neck position that centres the ball, or None to stay put.

    `neck` is where the head was when the frame was taken, in the accessory
    board's units; `elevation` is how far above the centre of that frame the
    ball sat [rad]. A proportional step with `gain` below one, because the
    detection behind `elevation` is a frame or two old and a full correction
    would overshoot. Inside `deadband` there is nothing worth moving a servo
    for. Clamped to `[low, high]`, the range the neck is allowed.
    """
    if abs(elevation) <= deadband:
        return None
    target = float(neck) + float(gain) * float(elevation) / float(rad_per_unit)
    target = min(max(target, float(low)), float(high))
    if abs(target - float(neck)) < 1e-3:
        return None
    return target


def turn_rate(bearing, gain, max_rate, min_rate, tolerance):
    """
    Return the in-place angular velocity that centres the ball [rad/s].

    Zero inside `tolerance`, which is the caller's signal that the ball is
    centred. Otherwise proportional, with a floor of `min_rate` so the last few
    degrees are not left to a velocity the wheels cannot turn at, and a ceiling
    of `max_rate` so the detector still sees the ball go past.
    """
    if abs(bearing) <= tolerance:
        return 0.0
    rate = min(max(abs(float(gain) * bearing), float(min_rate)), float(max_rate))
    return math.copysign(rate, bearing)


def floor_band(camera_height, pitch, vfov, target_height=0.0):
    """
    Return the `(near, far)` floor distances the frame sees at a tilt [m].

    `pitch` is positive up. `far` is `math.inf` when the top edge is at or above
    the horizon, which is the point of the search tilt; `near` is None when even
    the bottom edge does not reach down to `target_height` (a ball's centre, say).
    """
    drop = float(camera_height) - float(target_height)
    bottom = float(pitch) - float(vfov) / 2.0
    top = float(pitch) + float(vfov) / 2.0
    near = drop / math.tan(-bottom) if bottom < 0.0 else None
    far = drop / math.tan(-top) if top < 0.0 else math.inf
    return near, far
