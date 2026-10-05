"""
The decisions behind the LED condition's signals, as plain arithmetic.

No ROS and no py_trees: what the lights show while the robot leads is decided
here from numbers, so it can be tested without a robot. `LED_behaviours.py`
turns the answers into `SetLedStatus` calls.

Four decisions:

* how much of the way is driven, as a number of lit LEDs (`progress_fill`);
* which way the light should flow -- ahead, or towards the side the
  destination lies on (`lead_side`);
* which half of the robot faces a person, for the signals that are addressed to
  somebody rather than shown to the room (`facing_corners`);
* which panels point at a place the robot is not facing, for showing somebody
  where the target is while the robot looks at them (`direction_corners`).
"""

import math

# LEDs on one panel, and so the number of steps a progress bar has.
PANEL_LEDS = 8

STRAIGHT = "straight"
LEFT = "left"
RIGHT = "right"

# Whether the light follows the side of the destination, or only ever flows ahead.
DIRECTION_TARGET = "target"
DIRECTION_NONE = "none"
DIRECTION_MODES = (DIRECTION_TARGET, DIRECTION_NONE)

# A side that is being shown is kept until the bearing is back inside this
# fraction of the band, so a destination sitting on the edge of the band does
# not make the lights flip on every tick.
SIDE_HYSTERESIS = 0.6

FRONT_CORNERS = ("fl", "fr")
BACK_CORNERS = ("bl", "br")
CORNERS = FRONT_CORNERS + BACK_CORNERS

# The four strips cut the corners of the top plate, so each panel faces a
# diagonal [rad in the robot's frame, left positive].
CORNER_BEARINGS = {
    "fl": math.pi / 4.0,
    "fr": -math.pi / 4.0,
    "bl": 3.0 * math.pi / 4.0,
    "br": -3.0 * math.pi / 4.0,
}

# A panel that is pointing keeps doing so until the place is this far beyond
# the spread, so one on the edge between two directions does not flicker.
DIRECTION_HYSTERESIS = math.radians(8.0)


def progress_fill(start_distance, distance, stop_distance=0.0, steps=PANEL_LEDS):
    """
    Return how many LEDs of a panel the drive so far has earned.

    `start_distance` is how far the destination was when the drive began,
    `distance` how far it is now, and `stop_distance` how far short of it the
    robot parks -- so the bar is full where the robot stops rather than at a
    point it never reaches. A drive that starts within the stop distance has
    nothing to show and counts as done.
    """
    span = start_distance - stop_distance
    if span <= 0.0:
        return steps
    fraction = (start_distance - distance) / span
    fraction = max(0.0, min(1.0, fraction))
    # The epsilon keeps a fraction that is a whole step, bar float error, in
    # that step: 0.5 of 8 is 4 LEDs, not 3.
    return min(steps, int(math.floor(fraction * steps + 1e-9)))


def lead_side(relative_bearing, band, previous=STRAIGHT):
    """
    Return which way the light should flow for a destination at this bearing.

    `relative_bearing` is the destination's bearing in the robot's frame [rad],
    counterclockwise positive, so a positive one is to the robot's left. Within
    `band` of straight ahead the answer is `STRAIGHT`.
    """
    magnitude = abs(relative_bearing)
    side = LEFT if relative_bearing > 0.0 else RIGHT
    if previous == side:
        return side if magnitude > band * SIDE_HYSTERESIS else STRAIGHT
    return side if magnitude > band else STRAIGHT


def facing_corners(relative_bearing):
    """Return the two panels on the half of the robot a bearing [rad] lies on."""
    if abs(relative_bearing) <= math.pi / 2.0:
        return FRONT_CORNERS
    return BACK_CORNERS


def direction_corners(relative_bearing, spread, previous=()):
    """
    Return the panels that point at a place at this bearing, in `CORNERS` order.

    A panel points at the place when its bearing [rad, left positive] is within
    `spread` of the diagonal the panel faces. At 67.5 degrees that is eight
    directions from four panels: both front ones for a place ahead, one for a
    place off a corner, both left ones for a place square to the left.
    `previous` is what was returned last time; those panels are held a little
    beyond the spread.
    """
    pointing = []
    for corner in CORNERS:
        offset = relative_bearing - CORNER_BEARINGS[corner]
        offset = abs(math.atan2(math.sin(offset), math.cos(offset)))
        limit = spread + (DIRECTION_HYSTERESIS if corner in previous else 0.0)
        if offset <= limit:
            pointing.append(corner)
    return tuple(pointing)
