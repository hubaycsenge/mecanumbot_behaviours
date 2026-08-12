"""
The tunables: what every behaviour used to hard-code, declared in one place.

Two kinds of constant live in the YAML files. The **experiment parameters** --
the distances, the route, the LED and gesture scripts -- have always been there
and are required: a run with a missing `robot_closeness_threshold` is a run with
no defined approach distance, and inventing one silently would be worse than
failing. The **tunables** here are the numbers the behaviours used to carry as
constructor defaults and module constants: turn speeds, timeouts, head poses.
They are optional, because a YAML file written before a key existed should keep
the value the code used to hard-code, and that is exactly what its entry in
`TUNABLE_DEFAULTS` is.

A behaviour reads a tunable through `constant()`, or through `resolve()` where a
call site is still allowed to override it, so the YAML is the single place a
value is decided and no default is written down twice.

Angles live on the blackboard in radians and are declared in the YAML in degrees
under the same name with a `_deg` suffix -- the convention
`mecanumbot_ostensive_behaviour` established, because a deadband or a step angle
is thought about in degrees.

This module deliberately imports nothing from the package: the behaviours it
configures import *it*, and several of them are imported by
`blackboard_managers` in turn.
"""

import math

import py_trees
import yaml

# The YAML files keep everything under this node name, whichever tree loads them.
YAML_ROOT_KEYS = ("bottom_up_tree_node", "ros__parameters")

# Blackboard key -> the value used when the YAML does not declare it, which is
# in every case the number the behaviours hard-coded before it moved here.
# Angles are in radians; see the module docstring for the `_deg` convention.
TUNABLE_DEFAULTS = {
    # --- tree runtime, read from the file before the blackboard exists -------
    "tick_period_ms": 100.0,
    "setup_timeout": 15.0,
    # --- how fresh a people_fusion detection has to be to count as "visible" -
    "sight_timeout": 1.0,
    # --- the SmoothTurner velocity profile every in-place rotation runs on ---
    "turn_max_speed": 0.6,
    "turn_accel": 0.8,
    "turn_decel_gain": 1.6,
    "turn_min_speed": 0.12,
    "turn_tolerance": math.radians(3.0),
    # Below this the robot counts as already facing its target and does not turn
    # at all, even when a turn direction was asked for.
    "facing_epsilon": math.radians(1.5),
    # --- the turning behaviours ----------------------------------------------
    "turn_timeout": 20.0,
    # How long a turn waits for nav2 to stop driving before taking /cmd_vel.
    "turn_nav2_wait": 2.0,
    "turn_target_timeout": 3.0,
    # Re-checks of a human target after the profiled turn, for somebody who
    # stepped aside while the robot was turning.
    "turn_corrections": 1,
    # One step of the attention wiggle; the pattern is +1, -2, +2, -1 of it.
    "attention_turn_step": math.radians(15.0),
    # --- scanning for people --------------------------------------------------
    "scan_spin_speed": 0.5,
    "scan_timeout": 10.0,
    # The recovery patrol's full revolution is slower and gets longer, because
    # it is a search of the room rather than a glance over the shoulder.
    "full_scan_spin_speed": 0.3,
    "full_scan_timeout": 45.0,
    "full_scan_revolutions": 1.0,
    # --- driving to something with nav2 ---------------------------------------
    "approach_target_timeout": 3.0,
    "approach_goal_timeout": 10.0,
    # How far the robot drives towards a route target in one goal. The human
    # equivalent is `robot_approach_distance`, which is an experiment parameter.
    "route_step_distance": 1.0,
    # --- getting the human back -----------------------------------------------
    # How far past a checkpoint (as a fraction of the stretch to the next one)
    # the human has to be before that stretch counts as walked.
    "resume_passed_margin": 0.15,
    "recover_retries": 3,
    # --- pacing of the signalling ---------------------------------------------
    "thank_delay": 1.0,
    "show_turn_delay": 2.0,
    # --- accessory poses -------------------------------------------------------
    # n_pos is the neck-mounted camera tilt (2.0 .. 8.6, larger looks further
    # up). The seeking pose does double duty: it reads as the robot seeking
    # contact, and it gives the pose detector whole bodies rather than knees.
    "neck_seek_pos": 7.0,
    "neck_level_pos": 6.0,
    "gripper_left_neutral": 6.83,
    "gripper_right_neutral": 3.36,
}

# YAML name in degrees -> blackboard name in radians.
ANGLE_PARAMS = {
    "turn_tolerance_deg": "turn_tolerance",
    "facing_epsilon_deg": "facing_epsilon",
    "attention_turn_step_deg": "attention_turn_step",
}

# Tunables that are counts rather than measurements.
INTEGER_TUNABLES = ("turn_corrections", "recover_retries")

PARAM_KEYS = tuple(TUNABLE_DEFAULTS)


def register_param_keys(blackboard, *keys):
    """Give a blackboard client read access to the tunables, or to named ones."""
    for key in keys or PARAM_KEYS:
        blackboard.register_key(key=key, access=py_trees.common.Access.READ)


def constant(blackboard, key, defaults=TUNABLE_DEFAULTS):
    """
    Return a tunable from the blackboard, or its packaged default.

    The default covers a YAML file older than the key, and the ostensive trees,
    which reuse this library's turning behaviours but load their own constants
    file and so declare only the tunables they actually mean to set.

    `defaults` lets a package that builds on this library bring its own tunables
    -- `mecanumbot_demo_behaviours` does -- and get the same "declare it in the
    YAML or keep the packaged value" behaviour for them.
    """
    try:
        return getattr(blackboard, key)
    except (AttributeError, KeyError):
        if key not in defaults:
            raise
        return defaults[key]


def resolve(value, blackboard, key, defaults=TUNABLE_DEFAULTS):
    """Return an explicit call-site value, or the configured constant for `key`."""
    return constant(blackboard, key, defaults) if value is None else value


def file_constant(params, key, defaults=TUNABLE_DEFAULTS):
    """
    Return a tunable straight out of a loaded YAML block, or its default.

    For the few values needed while the tree is still being built -- the tick
    period, a decorator's retry count -- which is before any blackboard exists.
    """
    return params.get(key, defaults[key])


def load_params(yaml_path, root_keys=YAML_ROOT_KEYS):
    """Read the constants block out of a behaviour YAML file."""
    with open(yaml_path, "r") as handle:
        params = yaml.safe_load(handle)
    for key in root_keys:
        params = params[key]
    return params
