"""
The tunables of the seek tree, and the ones that have no default at all.

The split is the same one `mecanumbot_movement_behaviours/defaults.py` explains.
A key belongs here when the value describes *how the machinery works* and a
constants file written before the key existed should keep behaving as it did. A
key is `required` instead when it describes *what the experiment is*, because a
run with no defined grasp distance is not a run with a plausible one.

The SEEKING constants are all here rather than required, and deliberately: they
are a model, and a model has a shape that holds across runs. A trial that wants
a different one says so in its own file; a trial that says nothing gets the
shape documented in `seeking.py`.

Angles are declared in the YAML in degrees with a `_deg` suffix and reach the
blackboard in radians under the name without it -- the loader does that for any
key spelled that way, so no list of which ones they are exists anywhere.
"""

from mecanumbot_bt_config.blackboard import Tunables

SEEK_DEFAULTS = {
    # ===== The SEEKING circuit ================================================
    # Time constant of the short-term layer, in seconds. Long on purpose: it is
    # Panksepp's "positive feedback that sustains arousal after the
    # precipitating event has passed", and it is why the robot keeps looking
    # where the object was for a while after losing sight of it.
    "seek_arousal_tau": 8.0,
    # Time constant of the medium-term layer -- the expectancy that survives a
    # minute of fruitless searching and still remembers the robot was told the
    # object is here. An order of magnitude slower than the short-term one.
    "seek_expectancy_tau": 90.0,
    # How strongly sustained arousal feeds expectancy, per second.
    "seek_expectancy_gain": 0.02,
    # SEEKING is never entirely off in a waking animal; this is the floor.
    "seek_baseline": 0.1,
    # What each event is worth. A sighting is the unconditional input and the
    # strongest; being told where the object was is weaker; arriving there is
    # the anticipation peaking just before the question is answered.
    "seek_sight_drive": 1.0,
    "seek_cue_drive": 0.5,
    "seek_arrival_drive": 0.6,
    # Frustrative non-reward: what one completed sweep that found nothing costs
    # expectancy. At 0.25 from a starting 0.8, roughly three fruitless sweeps
    # extinguish the episode.
    "seek_non_reward_cost": 0.25,
    # Expectancy an episode starts with, when the robot has been told where the
    # object is. Not 1.0: the information came from a scan of a place that has
    # since had people walking about in it.
    "seek_initial_expectancy": 0.8,
    # Below this, the episode is over. The robot has looked, and looked, and the
    # thing is not there.
    "seek_extinction_threshold": 0.15,

    # ===== What the drive is for ==============================================
    # The search widens as expectancy falls, between these two radii [m].
    "seek_search_radius_min": 1.5,
    "seek_search_radius_max": 6.0,
    # Incentive salience: the confidence a detection needs to be acted on, at
    # zero arousal and at full arousal. A strongly seeking robot accepts a
    # weaker detection of the thing it is looking for -- bounded at both ends,
    # because a robot that accepts anything is not motivated but broken.
    "seek_detection_threshold_high": 0.7,
    "seek_detection_threshold_low": 0.35,

    # ===== Seeing the object ==================================================
    # How old a detection may be and still count as "in sight" [s].
    "seek_detection_timeout": 1.0,
    # How long the object has to be seen before the robot commits to approaching
    # it, so one flickering frame does not abandon a search leg.
    "seek_sighting_dwell": 0.4,
    # How long to wait for the server to name a target before giving up on the
    # episode [s]. Generous: the server has to localize the object in the cloud.
    "seek_target_timeout": 120.0,

    # ===== Approaching and grasping ===========================================
    # Where the approach stops short of the object [m]. Far enough that the
    # object is still in the camera's view, close enough that the last move into
    # the grabbers is short.
    "seek_approach_stop": 0.55,
    # How long one approach may run before it is given up [s].
    "seek_approach_timeout": 60.0,
    # How close the object has to be before a grasp is attempted [m].
    "seek_grasp_distance": 0.30,
    # Gripper positions for open and closed, and how long the close takes.
    "seek_gripper_open_left": 6.83,
    "seek_gripper_open_right": 3.36,
    "seek_gripper_closed_left": 5.2,
    "seek_gripper_closed_right": 5.0,
    "seek_grasp_settle": 1.5,
    # How long to wait for `/mecanumbot/has_object` to confirm a grasp [s].
    "seek_grasp_confirm_timeout": 3.0,
    # The height band the grabbers can actually close on [m]. Hardware, not
    # taste: the shafts sit at z ~ 0.034 with a 0.116 m clear gap, and there is
    # no lift, so an object outside this band is one the robot cannot have
    # however close it gets. This is what turns the point cloud's z into a
    # decision -- see `reachability.py`.
    "seek_grasp_height_min": 0.03,
    "seek_grasp_height_max": 0.15,

    # ===== Alerting somebody to an object it cannot have ======================
    # How long the robot looks for a person to tell before giving up on the
    # alert and publishing it unheard [s].
    "seek_alert_find_timeout": 30.0,
    # How long the robot holds each orientation of the showing gesture [s].
    # Long enough for a person to follow where it is pointing itself.
    "seek_show_hold": 1.2,
    # How many times it alternates between the object and the person. Read from
    # the file before the tree is built, so it cannot come off the blackboard.
    "seek_show_alternations": 3,
    # How long one turn of the gesture may take before the alert gives up [s].
    "seek_show_turn_timeout": 15.0,
    # LED mode and colour for the alert (see mecanumbot_led's README:
    # 5 = FAST_BLINK, 7 = YELLOW). Mode 0 leaves the LEDs alone.
    "seek_alert_led_mode": 5,
    "seek_alert_led_color": 7,

    # ===== The expanding ring search ==========================================
    # Radius of the first ring and how much each lap adds [m].
    "seek_ring_first": 1.5,
    "seek_ring_step": 1.5,
    # How far apart the stops on a ring stand [m], and the bounds on how many
    # there may be.
    "seek_ring_spacing": 1.5,
    "seek_ring_min_stops": 4,
    "seek_ring_max_stops": 16,
    # How close counts as having visited a search waypoint [m].
    "seek_waypoint_reached": 0.6,
    # How long one search waypoint may take before it is written off and the
    # search moves on [s].
    "seek_waypoint_timeout": 25.0,
    # How long the whole search may run before the episode is given up, however
    # the drive is doing [s]. The backstop, not the mechanism.
    "seek_search_timeout": 600.0,
}

TUNABLES = Tunables(SEEK_DEFAULTS)

constant = TUNABLES.constant
resolve = TUNABLES.resolve
file_constant = TUNABLES.file_constant
register_param_keys = TUNABLES.register_param_keys

PARAM_KEYS = TUNABLES.keys


def drive_settings(blackboard):
    """
    Read the SEEKING constants off the blackboard as `SeekingDrive` arguments.

    One place where the `seek_` prefix the YAML uses is stripped, so the model
    itself is free of this package's naming and stays testable on its own.
    """
    names = (
        "arousal_tau",
        "expectancy_tau",
        "expectancy_gain",
        "baseline",
        "sight_drive",
        "cue_drive",
        "arrival_drive",
        "non_reward_cost",
        "search_radius_min",
        "search_radius_max",
        "detection_threshold_high",
        "detection_threshold_low",
        "extinction_threshold",
        "initial_expectancy",
    )
    return {name: constant(blackboard, f"seek_{name}") for name in names}
