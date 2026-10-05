"""
Which of the robot's lights point at the ball, as plain arithmetic.

No ROS and no py_trees: `behaviours/lights.py` turns the answer into a
`SetLedStatus` call. While the robot closes in on a ball all four panels blink,
and the ones on the ball's side blink in another colour, so somebody watching
can read off where the robot thinks the ball is.

The four strips cut the corners of the top plate, so each panel faces a
diagonal: front-left, front-right, back-left, back-right. A panel is a
direction panel when the ball's bearing is within `spread` of the way it
faces. At the default 67.5 degrees that makes eight directions out of four
panels: a ball dead ahead lights both front panels, one off the front-left
corner lights that panel alone, one square to the left lights both left ones.
"""

import math

CORNERS = ("fl", "fr", "bl", "br")

# The way each panel faces in the robot's frame [rad], counterclockwise
# positive, so a positive bearing is to the robot's left.
CORNER_BEARINGS = {
    "fl": math.pi / 4.0,
    "fr": -math.pi / 4.0,
    "bl": 3.0 * math.pi / 4.0,
    "br": -3.0 * math.pi / 4.0,
}

# A panel that is showing the direction keeps it until the ball is this far
# beyond the spread, so a ball sitting on the edge between two directions does
# not make the lights flip on every tick.
HYSTERESIS = math.radians(8.0)


def _offset(bearing, facing):
    """Return the unsigned angle [rad] between a bearing and a panel's facing."""
    return abs(math.atan2(math.sin(bearing - facing), math.cos(bearing - facing)))


def direction_corners(bearing, spread, previous=(), hysteresis=HYSTERESIS):
    """
    Return the panels that point at a ball at this bearing, in `CORNERS` order.

    `bearing` is the ball's bearing in the robot's frame [rad], left positive.
    `previous` is what was returned last time; those panels are held for
    `hysteresis` beyond `spread`.
    """
    lit = []
    for corner in CORNERS:
        limit = spread + (hysteresis if corner in previous else 0.0)
        if _offset(bearing, CORNER_BEARINGS[corner]) <= limit:
            lit.append(corner)
    return tuple(lit)


def panel_colors(direction, color, direction_color):
    """Return `{corner: colour}`: the direction panels in their own colour."""
    return {
        corner: direction_color if corner in direction else color
        for corner in CORNERS
    }
