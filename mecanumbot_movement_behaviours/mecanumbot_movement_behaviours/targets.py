"""
What "the target" means for a behaviour, resolved against the blackboard.

`TurnToward` and `Approach` both take a `target_type`; this module is the single
definition of what each type points at, and of the head pose that goes with it.
Which blackboard key a type reads is a `KeyMap` field rather than a name written
down here -- see `keys.py` for why.
"""

import py_trees

from mecanumbot_movement_behaviours.keys import DEFAULT_KEYS
from mecanumbot_movement_behaviours.ros_interfaces import HEAD_LEVEL, HEAD_SEEK

SUBJECT = "subject"
START = "start"
TARGET = "target"
CHECKPOINT = "checkpoint"
PATROL = "patrol"
LAST_CHECKPOINT = "last_checkpoint"

TARGET_TYPES = (SUBJECT, START, TARGET, CHECKPOINT, PATROL, LAST_CHECKPOINT)

# KeyMap fields a behaviour needs to resolve any target type.
TARGET_FIELDS = (
    "target_position",
    "start_position",
    "checkpoints",
    "patrol_checkpoints",
    "current_checkpoint",
    "patrol_current_checkpoint",
)


def register_target_keys(blackboard, keys=DEFAULT_KEYS):
    """Give a blackboard client read access to every target-related key."""
    keys.register(blackboard, py_trees.common.Access.READ, *TARGET_FIELDS)


def resolve_target_position(
    blackboard, target_type, subject_position=None, keys=DEFAULT_KEYS
):
    """
    Position [geometry_msgs/Point-like] a target type points at, or None.

    Human targets come from the caller (`subject_position`) because their source
    is a live detection rather than the blackboard.
    """
    if target_type == SUBJECT:
        return subject_position
    if target_type == START:
        return blackboard.get(keys.start_position)
    if target_type == TARGET:
        return blackboard.get(keys.target_position)
    if target_type == LAST_CHECKPOINT:
        checkpoints = blackboard.get(keys.checkpoints)
        return checkpoints[-1] if checkpoints else None
    if target_type == CHECKPOINT:
        return _checkpoint(
            blackboard.get(keys.checkpoints),
            blackboard.get(keys.current_checkpoint),
        )
    if target_type == PATROL:
        return _checkpoint(
            blackboard.get(keys.patrol_checkpoints),
            blackboard.get(keys.patrol_current_checkpoint),
        )
    raise ValueError(
        f"unknown target_type '{target_type}', expected one of {TARGET_TYPES}"
    )


def is_human(target_type):
    """Say whether a target type is a person rather than a place on the route."""
    return target_type == SUBJECT


def default_head_pose(target_type):
    """Head lifted when dealing with a human, level when driving the route."""
    return HEAD_SEEK if is_human(target_type) else HEAD_LEVEL


def _checkpoint(checkpoints, index):
    if not checkpoints:
        return None
    return checkpoints[max(0, min(int(index), len(checkpoints) - 1))]
