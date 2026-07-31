"""What "the target" means for a behaviour, resolved against the blackboard.

`TurnToward` and `Approach` both take a `target_type`; this module is the single
definition of what each type points at, and of the head pose / turn direction
that goes with it.
"""

import py_trees

from mecanumbot_leading_behaviour.behaviours.ros_interfaces import HEAD_LEVEL, HEAD_SEEK

SUBJECT = "subject"
START = "start"
TARGET = "target"
CHECKPOINT = "checkpoint"
PATROL = "patrol"
LAST_CHECKPOINT = "last_checkpoint"

TARGET_TYPES = (SUBJECT, START, TARGET, CHECKPOINT, PATROL, LAST_CHECKPOINT)

# Blackboard keys a behaviour needs to resolve any target type.
_TARGET_KEYS = (
    "target_position",
    "start_position",
    "Dog_checkpoints",
    "patrol_checkpoints",
    "Dog_current_checkpoint",
    "patrol_current_checkpoint",
)


def register_target_keys(blackboard):
    """Give a blackboard client read access to every target-related key."""
    for key in _TARGET_KEYS:
        blackboard.register_key(key=key, access=py_trees.common.Access.READ)


def resolve_target_position(blackboard, target_type, subject_position=None):
    """Position [geometry_msgs/Point-like] a target type points at, or None.

    Human targets come from the caller (`subject_position`) because their source
    is a live detection rather than the blackboard.
    """
    if target_type == SUBJECT:
        return subject_position
    if target_type == START:
        return blackboard.start_position
    if target_type == TARGET:
        return blackboard.target_position
    if target_type == LAST_CHECKPOINT:
        return blackboard.Dog_checkpoints[-1]
    if target_type == CHECKPOINT:
        return _checkpoint(
            blackboard.Dog_checkpoints, blackboard.Dog_current_checkpoint
        )
    if target_type == PATROL:
        return _checkpoint(
            blackboard.patrol_checkpoints, blackboard.patrol_current_checkpoint
        )
    raise ValueError(
        f"unknown target_type '{target_type}', expected one of {TARGET_TYPES}"
    )


def is_human(target_type):
    """True for target types that are a person rather than a place on the route."""
    return target_type == SUBJECT


def default_head_pose(target_type):
    """Head lifted when dealing with a human, level when driving the route."""
    return HEAD_SEEK if is_human(target_type) else HEAD_LEVEL


def _checkpoint(checkpoints, index):
    if not checkpoints:
        return None
    return checkpoints[max(0, min(int(index), len(checkpoints) - 1))]
