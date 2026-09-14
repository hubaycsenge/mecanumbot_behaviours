"""
The tunables of the fetch tree, and the ones that have no default at all.

The split is the one `mecanumbot_movement_behaviours/defaults.py` explains. A
key belongs here when it describes *how the machinery works* and a constants
file written before the key existed should keep behaving as it did. A key is
`required` instead when it describes *what the run is* -- how close the robot
may come to a person, what the grabbers are set to -- because a run with an
invented grasp distance is not a run.

Angles are declared in the YAML in degrees with a `_deg` suffix and reach the
blackboard in radians under the name without it; the loader does that for any
key spelled that way, so no list of which ones they are exists anywhere. Neck
tilts are **not** angles: they are the accessory board's own units (about 2.0 to
8.6, larger looks further up), and there is no calibration to radians.
"""

from mecanumbot_bt_config.blackboard import Tunables

FETCH_DEFAULTS = {
    # ===== Seeing the ball ====================================================
    # The confidence a detection needs to be acted on. Flat, unlike
    # mecanumbot_seek's, which takes its threshold from the SEEKING circuit --
    # see the tree's module docstring for why this game is deliberately not
    # modelled as SEEKING.
    "fetch_detection_threshold": 0.4,
    # How old a detection may be and still count as "in sight" [s].
    "fetch_detection_timeout": 1.0,
    # How long the ball has to be continuously in sight before the robot
    # commits to it, so one flickering frame does not abandon a search leg.
    "fetch_sighting_dwell": 0.4,
    # What the located ball is labelled with on `ball_detections`. Has to match
    # `ball.class_id` in the perception layer's config.
    "fetch_ball_class": "sports ball",

    # ===== What the body does while searching =================================
    # spin | circles. `spin` turns a full revolution where the robot stands, at
    # the movement library's `full_scan_spin_speed`, drives to the next spot
    # (`fetch_spot_*` below), and turns again -- so every bearing from each spot
    # is looked at, at every tilt of the head sweep. `circles` drives the
    # widening circles below without stopping, which only ever looks along the
    # direction of travel and so can drive past a ball lying beside the robot.
    # A research choice about what the robot attends to, like
    # `fetch_circle_facing`.
    "fetch_search_strategy": "spin",

    # ===== The spots the `spin` search turns at ===============================
    # Rings of spots around where the search began, nearest ring first, the
    # same layout as the circles but sparser: a spot is looked round from, so
    # neighbouring spots need only be about two detection ranges apart [m].
    "fetch_spot_first": 1.5,
    "fetch_spot_step": 1.5,
    "fetch_spot_max": 3.0,
    "fetch_spot_spacing": 3.0,
    # Bounds on the spots per ring. Three is enough when each one is a full
    # turn; the ceiling keeps a lap from becoming a quarter of an hour.
    "fetch_spot_min_stops": 3,
    "fetch_spot_max_stops": 8,

    # ===== The circling search ================================================
    # Radius of the first circle and how much each lap adds [m].
    "fetch_circle_first": 1.2,
    "fetch_circle_step": 1.2,
    # How far out the search goes before it gives up [m].
    "fetch_circle_max": 4.8,
    # How far apart the stops on a circle stand [m], and the bounds on how many
    # there may be. Six is the floor because a circle of three stops is a
    # triangle the camera looks along the sides of.
    "fetch_circle_spacing": 1.2,
    "fetch_circle_min_stops": 6,
    "fetch_circle_max_stops": 16,
    # Which way the robot faces at each stop: tangent | inward | outward.
    # `tangent` sweeps the camera over unlooked-at floor as the robot goes
    # round, which is the point of circling when you do not know where the ball
    # is. This is a research choice about what the robot attends to while
    # searching, not a tuning constant.
    "fetch_circle_facing": "tangent",
    # How close counts as having visited a stop [m].
    "fetch_waypoint_reached": 0.5,
    # How long one stop may take before it is written off and the search moves
    # on [s]. A stop nav2 cannot reach is inside a table; the search has more
    # places to be.
    "fetch_waypoint_timeout": 20.0,
    # Laps of the whole pattern before the search gives up, and the backstop in
    # seconds. Each lap rebuilds the circles from the innermost radius, because
    # a ball that was not there a minute ago may have been thrown there since.
    # With `spin`, a lap is every spot once, each with its full turn -- several
    # minutes, which the backstop has to allow for.
    "fetch_search_laps": 3,
    "fetch_search_timeout": 300.0,

    # ===== The head sweep =====================================================
    # Neck positions the sweep runs between, and how long one down-and-up lap
    # takes [s]. The low end looks at the floor immediately in front of the
    # robot, the high end at the far side of the room; a ball outside the
    # current band is invisible however good the detector is, which is why the
    # tilt moves at all.
    "fetch_head_low": 3.0,
    "fetch_head_high": 6.5,
    "fetch_head_sweep_period": 6.0,
    # How often the neck is actually commanded during the sweep [s]. The sweep
    # is continuous; the commands are not, because every one is a serial write
    # to the accessory board.
    "fetch_head_command_interval": 0.3,
    # Where the head is held once the robot has committed to a ball: down, so
    # the ball stays in frame as the robot closes on it and the box stays big
    # enough to range from.
    "fetch_head_approach": 2.6,

    # ===== Approaching and gripping ===========================================
    # Where the approach stops short of the ball [m]. Far enough that the ball
    # is still in the camera's view, close enough that the last move into the
    # grabbers is short.
    "fetch_approach_stop": 0.45,
    # How long one approach may run before it is given up [s].
    "fetch_approach_timeout": 45.0,
    # How close the ball has to be before the grab is attempted [m].
    "fetch_grasp_distance": 0.28,
    # The height band the grabbers can close on [m]. Hardware, not taste: the
    # shafts sit at z ~ 0.034 with a 0.116 m clear gap and there is no lift, so
    # a ball outside this band is one the robot cannot have however close it
    # gets. This is what the located ball's z is *for*.
    "fetch_grasp_height_min": 0.0,
    "fetch_grasp_height_max": 0.12,
    # Gripper positions, how long the close takes to settle, and how long to
    # wait for `/mecanumbot/has_object` to confirm it [s].
    "fetch_gripper_open_left": 6.83,
    "fetch_gripper_open_right": 3.36,
    "fetch_gripper_closed_left": 5.2,
    "fetch_gripper_closed_right": 5.0,
    "fetch_grasp_settle": 1.5,
    "fetch_grasp_confirm_timeout": 3.0,
    # Attempts at the grab before the episode gives up on this ball. A miss is
    # cheap and common -- the ball rolls off the shafts -- and re-approaching
    # from 30 cm is a couple of seconds.
    "fetch_grasp_attempts": 3,

    # ===== Bringing it to somebody ============================================
    # How long the robot looks for a person to give the ball to [s].
    "fetch_find_person_timeout": 45.0,
    # How long it holds still facing them before opening the grabbers [s]. This
    # is the handover pause: long enough that a person registers the robot has
    # stopped and is addressing them.
    "fetch_offer_hold": 2.0,
    # How long one turn onto the person may take [s].
    "fetch_offer_turn_timeout": 15.0,
    # How long the grabbers stay open after the release, before the robot backs
    # away [s].
    "fetch_release_hold": 1.5,
    # How far the robot reverses after releasing [m] and how long it may take
    # [s]. Backing off is what makes the release read as giving rather than
    # dropping: the robot puts the ball down and leaves it to the person.
    "fetch_withdraw_distance": 0.4,
    "fetch_withdraw_timeout": 6.0,
    "fetch_withdraw_speed": 0.12,

    # ===== Between episodes ===================================================
    # How long the robot waits after a handover before starting to look for the
    # ball again [s]. The person has to be able to throw it.
    "fetch_rest": 5.0,
}

TUNABLES = Tunables(FETCH_DEFAULTS)

constant = TUNABLES.constant
resolve = TUNABLES.resolve
file_constant = TUNABLES.file_constant
register_param_keys = TUNABLES.register_param_keys

PARAM_KEYS = TUNABLES.keys
