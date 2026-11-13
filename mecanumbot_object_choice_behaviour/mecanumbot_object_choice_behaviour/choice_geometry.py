"""
Which brick a pointing cue picks out: the decision the object-choice task scores.

The ostensive package reads a pointing arm into a ray in the map frame -- an
origin at the person, a bearing -- and stops there. Here that ray meets the
bricks the robot surveyed before the trial, and one of them is chosen, or none.

**The measure is the angle at the person.** For each brick, the angle between
the cue's bearing and the bearing from the person to the brick. Anchored at the
person for the reason `cue_geometry` anchors the ray there: "that one" is said
from where the speaker stands. The brick with the smallest angle is the
candidate.

**The robot is allowed not to choose**, and both ways it declines are outcomes
the trial records rather than failures to be hidden:

* `none in range` -- no brick lies within `max_deviation` of the cue. The person
  pointed somewhere there is no brick, or the arm was read badly.
* `ambiguous` -- the best brick does not beat the second by `min_margin`. Two
  bricks close together in angle, as seen from the person, are a coin toss, and
  a choice taken on a coin toss is noise in the data that looks like a result.

A brick nearer the person than `min_distance` is left out, with that reason: its
bearing from the person is dominated by where on their body the ray is anchored,
which is a torso estimate, not a hand.

Pure Python -- no ROS, no py_trees -- so the decision is unit-tested on tuples.
"""

import math

CHOSEN = "chosen"
NONE_IN_RANGE = "none in range"
AMBIGUOUS = "ambiguous"
NO_BRICKS = "no bricks"
TOO_CLOSE = "too close to the person"


def wrap_angle(angle):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class Candidate:
    """One brick, as seen from the person along the cue."""

    __slots__ = ("colour", "position", "deviation", "distance", "excluded")

    def __init__(self, colour, position, deviation, distance, excluded=None):
        self.colour = str(colour)
        self.position = tuple(position)
        self.deviation = float(deviation)
        self.distance = float(distance)
        self.excluded = excluded

    def as_record(self):
        """Return the candidate as plain data, for the trial record."""
        return {
            "colour": self.colour,
            "x": round(self.position[0], 3),
            "y": round(self.position[1], 3),
            "deviation_deg": round(math.degrees(self.deviation), 1),
            "distance_from_person": round(self.distance, 3),
            "excluded": self.excluded,
        }


class Choice:
    """What the cue decided: the brick, or why there is none, and the working."""

    __slots__ = ("colour", "reason", "margin", "candidates")

    def __init__(self, colour, reason, margin, candidates):
        self.colour = colour
        self.reason = reason
        self.margin = margin
        self.candidates = candidates

    @property
    def chosen(self):
        """Say whether a brick was chosen."""
        return self.colour is not None

    def as_record(self):
        """Return the choice as plain data, for the trial record."""
        return {
            "choice": self.colour,
            "reason": self.reason,
            "margin_deg": (
                None if self.margin is None else round(math.degrees(self.margin), 1)
            ),
            "candidates": [candidate.as_record() for candidate in self.candidates],
        }


def rank(origin, bearing, bricks, min_distance=0.0):
    """
    Return every brick as a `Candidate`, best first.

    `bricks` maps a colour to an `(x, y)` or `(x, y, z)` in the map frame.
    Excluded candidates sort after all usable ones.
    """
    candidates = []
    for colour, position in bricks.items():
        dx = position[0] - origin[0]
        dy = position[1] - origin[1]
        distance = math.hypot(dx, dy)
        deviation = abs(wrap_angle(math.atan2(dy, dx) - bearing))
        excluded = TOO_CLOSE if distance < min_distance else None
        candidates.append(Candidate(colour, position[:2], deviation, distance, excluded))
    candidates.sort(key=lambda c: (c.excluded is not None, c.deviation))
    return candidates


def choose(origin, bearing, bricks, max_deviation, min_margin, min_distance=0.0):
    """
    Decide which brick the cue indicates, or that it indicates none.

    `origin` is the person's `(x, y)`, `bearing` the cue's map-frame direction
    [rad]; `max_deviation` and `min_margin` are angles [rad].
    """
    candidates = rank(origin, bearing, bricks, min_distance)
    usable = [c for c in candidates if c.excluded is None]
    if not usable:
        return Choice(None, NO_BRICKS, None, candidates)
    best = usable[0]
    margin = usable[1].deviation - best.deviation if len(usable) > 1 else None
    if best.deviation > max_deviation:
        return Choice(None, NONE_IN_RANGE, margin, candidates)
    if margin is not None and margin < min_margin:
        return Choice(None, AMBIGUOUS, margin, candidates)
    return Choice(best.colour, CHOSEN, margin, candidates)
