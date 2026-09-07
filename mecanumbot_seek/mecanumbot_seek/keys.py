"""
How this experiment spells the blackboard keys the movement behaviours read.

`mecanumbot_movement_behaviours` reads everything through a `KeyMap` precisely
so that an experiment can name things in its own vocabulary -- the leading
conditions call their route `Dog_checkpoints`, because that is what their YAML
says. This package does the same for one field.

`target_position` is the only respelling, and it earns its place: the movement
library's `Approach(target_type=TARGET)` is exactly "drive to the place named on
the blackboard and stop short of it", which is what approaching the object's
last known location *is*. Binding `seek_target_position` onto a subclass means
the seek tree gets that whole behaviour -- nav2 goals, resends, the stop
threshold, the cancel on terminate -- without a line of navigation code here,
and without the leading experiment's key names leaking into the seek YAML.

The distances keep the library's own spellings, because they mean the same
things: how close the robot may come to the thing it is approaching, and how
close counts as having arrived.
"""

from mecanumbot_movement_behaviours.approach import Approach
from mecanumbot_movement_behaviours.keys import DEFAULT_KEYS

SEEK_KEYS = DEFAULT_KEYS.derive(
    target_position="seek_target_position",
)


class SeekApproach(Approach):
    """`Approach`, bound to this package's key spelling."""

    KEYS = SEEK_KEYS
