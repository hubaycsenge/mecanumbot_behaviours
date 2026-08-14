"""
The tunables of the movement behaviours: what they used to hard-code.

Every key here was once a constructor default or a module constant in the
behaviour library, and the value is the number it used to be. They are optional
in a constants file: a YAML written before a key existed keeps the value the
code used to hard-code, and a tree that borrows these behaviours while loading a
constants file of its own only has to declare the ones it actually means to set.

That is the one thing a constants file cannot say about itself, which is why it
lives in Python and here rather than in `mecanumbot_bt_config` -- the defaults
belong to the behaviours that read them. What the *experiment* is (the route,
the distances, the signalling scripts) has no default at all: a run with no
defined approach distance is not a run with a plausible one, so those keys are
declared `required` by the tree instead.

Angles are stored in radians and declared in the YAML in degrees under the same
name with a `_deg` suffix; the loader does the conversion for any key spelled
that way, so no list of which ones they are exists anywhere.
"""

import math

from mecanumbot_bt_config.blackboard import Tunables

MOVEMENT_DEFAULTS = {
    # --- how fresh a people_fusion detection has to be to count as "visible" --
    "sight_timeout": 1.0,
    # --- the SmoothTurner velocity profile every in-place rotation runs on ----
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
    # --- looking back at the human while leading -------------------------------
    # The check-in turn is slower than either scan: it reads as looking rather
    # than casting about, and a person the robot sweeps past slowly is a person
    # the camera detector gets a chance to find.
    "glance_spin_speed": 0.4,
    # Long enough for a whole revolution at `glance_spin_speed` with room to
    # spare: timing out mid-spin is a FAILURE, and a check-in that runs out of
    # clock rather than out of places to look sends the tree off to the patrol
    # as if the human were lost.
    "glance_timeout": 30.0,
    # How far the check-in turns before it gives up on finding the human. A
    # whole turn: they may have stepped off the route rather than merely
    # dropped behind, and the spin ends the moment they are seen anyway.
    "glance_revolutions": 1.0,
    # How the look backs are paced: one every so many checkpoints and one every
    # so many seconds, whichever falls due first. Either set to 0 switches that
    # half off; both at 0 means the robot only looks back when something about
    # the human says it should.
    "check_in_every_checkpoints": 2,
    "check_in_interval": 20.0,
    # How long the human has to be out of sight or trailing before a leg of the
    # route is cut short for it. Somebody stepping behind a pillar is still
    # following.
    "check_in_grace": 2.0,
    # How long the robot waits for a trailing human before it walks back to
    # them and asks for their attention again.
    "check_in_catch_up_timeout": 6.0,
    # --- driving to something with nav2 ---------------------------------------
    "approach_target_timeout": 3.0,
    "approach_goal_timeout": 10.0,
    # How far the robot drives towards a route target in one goal. The human
    # equivalent is the `approach_distance` key, which is an experiment
    # parameter and so has no default.
    "route_step_distance": 1.0,
    # How far short of a place on the route a goal stops. The human equivalent
    # is `closeness_threshold`, and the two are separate because they answer
    # different questions: how close the robot may come to a person is about the
    # person, and is the number that has to grow when the footprint says the
    # bumper would reach them; how close it parks to a checkpoint is only about
    # not fussing over a waypoint it is passing anyway.
    "route_stop_distance": 0.45,
    # How many route checkpoints one waypoint goal may cover. Longer legs mean
    # fewer interruptions of the drive; the look-back pacing shortens a leg that
    # would run past a check-in anyway.
    "route_lookahead": 3,
    # How close counts as having driven past a route checkpoint.
    "checkpoint_reached_distance": 0.5,
    # Under this, the robot sets off down a leg without turning to face it
    # first: a bend of a few degrees is the navigation stack's to drive out, and
    # stopping to rotate for it only breaks the movement up.
    "route_turn_min": math.radians(30.0),
    # Times a dropped nav2 goal is sent again before the behaviour gives up.
    "nav_goal_retries": 3,
    # --- accessory poses -------------------------------------------------------
    # n_pos is the neck-mounted camera tilt (2.0 .. 8.6, larger looks further
    # up). The seeking pose does double duty: it reads as the robot seeking
    # contact, and it gives the pose detector whole bodies rather than knees.
    "neck_seek_pos": 7.0,
    "neck_level_pos": 6.0,
    "gripper_left_neutral": 6.83,
    "gripper_right_neutral": 3.36,
}

TUNABLES = Tunables(MOVEMENT_DEFAULTS)

constant = TUNABLES.constant
resolve = TUNABLES.resolve
file_constant = TUNABLES.file_constant
register_param_keys = TUNABLES.register_param_keys

PARAM_KEYS = TUNABLES.keys


def configure_accessories(node, blackboard, values):
    """
    Hand the neck and gripper poses to the accessory commander.

    A `ParamsToBlackboard` load hook: the poses belong to the commander rather
    than to any one behaviour, so they are handed over once rather than threaded
    through every constructor that creates one.
    """
    from mecanumbot_movement_behaviours.ros_interfaces import AccessoryCommander

    AccessoryCommander.configure(
        seek_pos=values["neck_seek_pos"],
        level_pos=values["neck_level_pos"],
        gripper_left=values["gripper_left_neutral"],
        gripper_right=values["gripper_right_neutral"],
    )
