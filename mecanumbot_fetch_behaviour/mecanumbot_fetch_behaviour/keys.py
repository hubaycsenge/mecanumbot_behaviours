"""
How this experiment spells the blackboard keys the movement behaviours read.

`mecanumbot_movement_behaviours` reads everything through a `KeyMap` precisely
so an experiment can name things in its own vocabulary. This package respells
one field, for the same reason `mecanumbot_seek` does: the movement library's
`Approach(target_type=TARGET)` is exactly "drive to the place named on the
blackboard and stop short of it", which is what approaching the ball is. Binding
`fetch_ball_position` onto a subclass means the fetch tree gets that whole
behaviour -- nav2 goals, resends, the stop threshold, the cancel on terminate --
without a line of navigation code here.

The distances keep the library's own spellings, because they mean the same
things: how close the robot may come to the thing it is approaching, and how
close counts as having arrived.
"""

from mecanumbot_movement_behaviours.approach import Approach
from mecanumbot_movement_behaviours.keys import DEFAULT_KEYS
from mecanumbot_movement_behaviours.turning import FindPeople, TurnToward

FETCH_KEYS = DEFAULT_KEYS.derive(
    target_position="fetch_ball_position",
)


class FetchApproach(Approach):
    """`Approach`, bound to this package's key spelling."""

    KEYS = FETCH_KEYS


class FetchTurnToward(TurnToward):
    """`TurnToward`, bound to this package's key spelling."""

    KEYS = FETCH_KEYS


class FetchFindPeople(FindPeople):
    """`FindPeople`, bound to this package's key spelling."""

    KEYS = FETCH_KEYS
