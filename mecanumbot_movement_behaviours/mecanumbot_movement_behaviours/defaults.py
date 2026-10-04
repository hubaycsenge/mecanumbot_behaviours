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
    # A route goal that has not brought the robot `route_stall_progress` metres
    # closer to its checkpoint in `route_stall_timeout` seconds is treated as
    # dropped. nav2 does not give up on a checkpoint it cannot reach -- its
    # recoveries back up 0.2 m and retry, so the robot rocks back and forth in
    # front of it for as long as the goal lives.
    "route_stall_timeout": 10.0,
    "route_stall_progress": 0.2,
    # Times a dropped nav2 goal is sent again before the behaviour gives up.
    "nav_goal_retries": 3,
    # A `mode="stepped"` approach to a person takes steps of `approach_distance`
    # until it stands within `closeness_threshold` of them, and counts itself
    # there `approach_arrive_margin` metres early -- nav2 parks within its own
    # xy goal tolerance (0.30 m), so a step that ends 0.2 m short is arrived,
    # not a reason for another step. `approach_max_steps` caps the walk, so a
    # person who keeps backing off does not lead the robot across the room.
    "approach_arrive_margin": 0.35,
    # The same allowance for the end of the route: the robot is at the last
    # checkpoint when it stands within the larger of
    # `checkpoint_reached_distance` and `route_stop_distance` of it, plus this.
    # It has to stay above nav2's xy goal tolerance, or a drive nav2 has
    # finished would not count as arrived and the robot would never show.
    "route_end_margin": 0.35,
    "approach_max_steps": 6,
    # --- accessory poses -------------------------------------------------------
    # n_pos is the neck-mounted camera tilt (2.0 .. 8.6, larger looks further
    # up). The seeking pose does double duty: it reads as the robot seeking
    # contact, and it gives the pose detector whole bodies rather than knees.
    "neck_seek_pos": 7.0,
    "neck_level_pos": 6.0,
    # Looking down onto a face at the bottom of the frame while searching
    # (look_down.py): how far the neck drops (n_pos, 0.5 is ~15 deg), how long
    # it stays down, how long it then ignores the cue, and how old a cue may be.
    "look_down_step": 0.5,
    "look_down_hold": 3.0,
    "look_down_cooldown": 4.0,
    "look_down_cue_age": 0.5,
    "gripper_left_neutral": 6.83,
    "gripper_right_neutral": 3.36,
    # --- the head following the body the LiDAR expects (body_gaze.py) ---------
    # Only a tree that installs `start_body_gaze` as a load hook has a gaze at
    # all; in such a tree this switches it off without touching the code.
    "body_gaze_enabled": True,
    # Seconds between two looks at what is in view.
    "body_gaze_period": 0.2,
    # Metres within which a body is worth tilting the head for.
    "body_gaze_max_range": 4.5,
    # How far beyond the edge of the picture a target is still attended to, and
    # how much of the frame is kept clear above and below a body.
    "body_gaze_view_margin": math.radians(12.0),
    "body_gaze_frame_margin": math.radians(3.0),
    # The neck positions the gaze may command, the smallest change worth a
    # command (0.1 is ~3 deg), the shortest time between two, and how long the
    # head waits with nothing in view before going back to `neck_seek_pos`.
    "body_gaze_min_pos": 5.5,
    "body_gaze_max_pos": 8.2,
    "body_gaze_deadband": 0.1,
    "body_gaze_min_interval": 0.4,
    "body_gaze_release_delay": 1.0,
    # How old the LiDAR's candidate list may be.
    "body_gaze_candidate_timeout": 0.5,
    # The band of heights [m] the detector needs of each kind of body, hips to
    # the top of the head, as `{'body': kind, 'low': .., 'high': ..}`. Kinds a
    # file leaves out keep `body_gaze.BODY_PROFILES`.
    "body_profiles": [],
    # Known places in the map where a body is lower than a standing one, as
    # `{'seat': kind, 'x': .., 'y': .., 'radius': ..}`. They belong to a room,
    # so the default is none: a constants file for a map lists that map's.
    "seats": [],
}

TUNABLES = Tunables(MOVEMENT_DEFAULTS)

# The `HeadLookDown` settings, in its constructor's order.
LOOK_DOWN_KEYS = (
    "look_down_step",
    "look_down_hold",
    "look_down_cooldown",
    "look_down_cue_age",
)

constant = TUNABLES.constant
resolve = TUNABLES.resolve
file_constant = TUNABLES.file_constant
register_param_keys = TUNABLES.register_param_keys

PARAM_KEYS = TUNABLES.keys


BODY_GAZE_SETTINGS = {
    "max_range": "body_gaze_max_range",
    "view_margin": "body_gaze_view_margin",
    "frame_margin": "body_gaze_frame_margin",
    "min_pos": "body_gaze_min_pos",
    "max_pos": "body_gaze_max_pos",
    "deadband": "body_gaze_deadband",
    "min_interval": "body_gaze_min_interval",
    "release_delay": "body_gaze_release_delay",
}


def build_body_gaze(values):
    """
    Build the gaze controller a set of loaded constants describes.

    Split from `start_body_gaze` so that what a constants file means can be
    checked without a node: a seat or a profile that cannot be read raises
    here, with the entry in the message.
    """
    from mecanumbot_movement_behaviours import body_gaze

    settings = body_gaze.GazeSettings(
        **{field: float(values[key]) for field, key in BODY_GAZE_SETTINGS.items()}
    )
    return body_gaze.BodyGazeController(
        seats=[body_gaze.seat_from_literal(entry) for entry in values["seats"]],
        profiles=body_gaze.profiles_from_literals(values["body_profiles"]),
        settings=settings,
    )


def start_body_gaze(node, blackboard, values):
    """
    Start the head following the body the LiDAR expects, if the file wants it.

    A `ParamsToBlackboard` load hook, and an opt-in one: a tree gets a gaze by
    listing this beside `configure_accessories`. The leading conditions differ
    in exactly this -- the dog-inspired tree moves its head and the LED and
    control trees deliberately do not -- so it is the tree's choice, not the
    constants file's, and `body_gaze_enabled` in the file can only turn it off.
    """
    from mecanumbot_movement_behaviours.ros_interfaces import BodyGaze

    if not values["body_gaze_enabled"]:
        node.get_logger().info("Body gaze: switched off in the constants file")
        return
    controller = build_body_gaze(values)
    BodyGaze.start(
        node,
        controller,
        period=values["body_gaze_period"],
        sight_timeout=values["sight_timeout"],
        candidate_timeout=values["body_gaze_candidate_timeout"],
    )
    seats = ", ".join(
        f"{seat.kind} at ({seat.x:.2f}, {seat.y:.2f})" for seat in controller.seats
    )
    node.get_logger().info(
        "Body gaze: the seeking head follows the LiDAR's candidates within "
        f"{controller.settings.max_range:.1f} m; seats: {seats or 'none'}"
    )


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
