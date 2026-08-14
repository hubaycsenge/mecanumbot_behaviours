"""
How the leading conditions spell the blackboard keys the movement library reads.

The constants files of these experiments prefix the route and the following
distance with `Dog_`, and the loader writes what the file says -- so this is the
one place the movement behaviours are told about that spelling. Every other
difference between this experiment and another is a different YAML file, not a
different behaviour.
"""

from mecanumbot_movement_behaviours.keys import DEFAULT_KEYS

LEADING_KEYS = DEFAULT_KEYS.derive(
    checkpoints="Dog_checkpoints",
    current_checkpoint="Dog_current_checkpoint",
    max_checkpoint="Dog_max_checkpoint",
    following_threshold="Dog_following_max_threshold",
)
