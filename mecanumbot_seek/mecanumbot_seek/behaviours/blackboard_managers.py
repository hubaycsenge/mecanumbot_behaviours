"""
Loading the seek constants, and owning the state one episode carries.

The loading is `mecanumbot_bt_config`'s and knows no key names: it writes
whatever the YAML declares, turns a `_deg` parameter into radians under the name
without the suffix, and fills in a packaged default for anything left out. This
module only says the two things a file cannot say about itself -- which keys a
run must not be missing, and which keys are the run's own state rather than
configuration.

Almost everything the seek tree reads has a default, which is the opposite of
the ostensive package's flat all-required schema, and for a reason: most of what
this tree is configured by is a *model* -- the SEEKING circuit's time constants
and event weights -- and a model has a shape that holds across runs. What is
required is the small set of numbers that describe this robot's body and this
trial's object: how close it may come, how close counts as grasped, and what the
grabbers are set to.

The `SeekingDrive` itself lives on the blackboard rather than being rebuilt per
behaviour, because it is one circuit. Several behaviours push events into it and
several read it, and a copy per behaviour would be several robots' worth of
motivation in one robot.
"""

import py_trees

from mecanumbot_bt_config.blackboard import ParamsToBlackboard
from mecanumbot_bt_config.tree_runner import RUNTIME_DEFAULTS
from mecanumbot_movement_behaviours.defaults import (
    MOVEMENT_DEFAULTS,
    configure_accessories,
)

from mecanumbot_seek.defaults import SEEK_DEFAULTS, drive_settings
from mecanumbot_seek.seeking import SeekingDrive

# What describes this robot and this trial rather than the model, and so has no
# sensible stand-in. A run with an invented grasp distance is not a run.
SEEK_REQUIRED = (
    "seek_approach_stop",
    "seek_grasp_distance",
    "seek_gripper_open_left",
    "seek_gripper_open_right",
    "seek_gripper_closed_left",
    "seek_gripper_closed_right",
    # How close the robot may come to a *person* when it goes to tell them about
    # something it cannot reach. The movement library spells these
    # `robot_approach_distance` / `robot_closeness_threshold` and every other
    # experiment here declares them; there is no defensible default for how
    # close a robot may drive to somebody.
    "robot_approach_distance",
    "robot_closeness_threshold",
)

LOADED_DEFAULTS = (RUNTIME_DEFAULTS, MOVEMENT_DEFAULTS, SEEK_DEFAULTS)

# The episode's own state, seeded so no behaviour has to test whether a key
# exists yet, and cleared on every entry so an episode interrupted halfway
# through does not resume against a stale target.
#
# `seek_target_position` is the spelling `keys.SEEK_KEYS` binds onto the
# movement library's `target_position` field, which is what lets `SeekApproach`
# drive to the object with no navigation code in this package.
RUN_STATE = {
    "seek_object_class": "",
    "seek_target_position": None,
    "seek_last_known": None,
    "seek_sighted_position": None,
    "seek_search_visited": set,
    "seek_search_waypoints": list,
    "seek_grasped": False,
    # --- the alert ----------------------------------------------------------
    # Why the object could not be had, the height that decided it, who was told
    # and how many times the showing gesture completed. All cleared per episode:
    # an alert carrying the previous object's reason is worse than no alert.
    "seek_unreachable_reason": "",
    "seek_object_height": None,
    "seek_person_position": None,
    "seek_alternations": 0,
    # Belongs to the movement library's `InPlaceTurn`, which the scan borrows.
    "search_spin_sign": 0,
}

# Written by the loader hook rather than read from the file.
EXTRA_KEYS = ("seek_drive",)

PARAM_KEYS = tuple(SEEK_DEFAULTS)


def register_param_keys(blackboard):
    """Give a blackboard client read access to every seek constant."""
    for key in PARAM_KEYS:
        blackboard.register_key(key=key, access=py_trees.common.Access.READ)


def build_drive(node, blackboard, values):
    """
    Put one `SeekingDrive` on the blackboard, built from the loaded constants.

    A `ParamsToBlackboard` load hook, so the circuit exists before any behaviour
    is ticked and every behaviour that pushes an event into it is pushing into
    the same one.
    """
    blackboard.seek_drive = SeekingDrive(**drive_settings(blackboard))
    node.get_logger().info(
        f"SEEKING circuit built: arousal tau {values['seek_arousal_tau']:.0f} s, "
        f"expectancy tau {values['seek_expectancy_tau']:.0f} s, "
        f"extinction below {values['seek_extinction_threshold']:.2f}"
    )


def check_search_geometry(node, blackboard, values):
    """Warn rather than fail when the search radii cannot produce a search."""
    if values["seek_search_radius_max"] <= values["seek_search_radius_min"]:
        node.get_logger().warn(
            f"seek_search_radius_max ({values['seek_search_radius_max']}) is not "
            f"above seek_search_radius_min ({values['seek_search_radius_min']}); "
            "the search will not widen as expectancy falls"
        )
    if values["seek_detection_threshold_low"] > values["seek_detection_threshold_high"]:
        node.get_logger().warn(
            "seek_detection_threshold_low is above _high, so incentive salience "
            "runs backwards: a strongly seeking robot will be harder to convince"
        )
    if values["seek_grasp_distance"] >= values["seek_approach_stop"]:
        node.get_logger().warn(
            f"seek_grasp_distance ({values['seek_grasp_distance']}) is not below "
            f"seek_approach_stop ({values['seek_approach_stop']}); the approach "
            "parks further away than the grasp needs and the grasp will fail"
        )
    if values["seek_grasp_height_min"] >= values["seek_grasp_height_max"]:
        node.get_logger().warn(
            f"seek_grasp_height_min ({values['seek_grasp_height_min']}) is not "
            f"below seek_grasp_height_max ({values['seek_grasp_height_max']}); "
            "nothing will ever be judged graspable and every episode will end "
            "in an alert"
        )


class SeekParamsToBlackboard(ParamsToBlackboard):
    """Load the seek constants and build the episode's SEEKING circuit."""

    def __init__(self, name, yaml_path):
        super().__init__(
            name=name,
            yaml_path=yaml_path,
            defaults=LOADED_DEFAULTS,
            required=SEEK_REQUIRED,
            state=RUN_STATE,
            on_loaded=(configure_accessories, build_drive, check_search_geometry),
            extra_keys=EXTRA_KEYS,
        )


class ClearSeekEpisode(py_trees.behaviour.Behaviour):
    """
    Forget the target and reset the circuit, and return SUCCESS.

    Used twice in the tree: after an episode that ended in a grasp, and as the
    fallback that absorbs one that failed. Both go through here because the
    answer to either is the same -- let the object go and wait to be asked for
    the next one.

    The reset is what makes SEEKING switch off between episodes. A circuit still
    carrying the last object's expectancy would start the next episode already
    convinced, which is the model being wrong rather than the robot being keen.
    """

    def __init__(self, name="ClearSeekEpisode", reason=""):
        super().__init__(name)
        self.reason = reason
        self.blackboard = self.attach_blackboard_client(name=name)
        for key in (
            "seek_object_class",
            "seek_target_position",
            "seek_last_known",
            "seek_sighted_position",
            "seek_search_visited",
            "seek_search_waypoints",
            "seek_grasped",
            "seek_unreachable_reason",
            "seek_object_height",
            "seek_person_position",
            "seek_alternations",
        ):
            self.blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(
            key="seek_drive", access=py_trees.common.Access.READ
        )

    def setup(self, **kwargs):
        """Keep the node handle for logging."""
        self.node = kwargs["node"]
        self.logger.info(f"{self.name}: Setup complete")
        return True

    def update(self):
        """Drop the target, clear the search, and reset the circuit."""
        drive = self.blackboard.seek_drive
        if self.blackboard.seek_object_class:
            self.node.get_logger().info(
                f"{self.name}: episode over for "
                f"'{self.blackboard.seek_object_class}'"
                + (f" ({self.reason})" if self.reason else "")
                + f"; grasped={self.blackboard.seek_grasped}, "
                f"expectancy ended at {drive.expectancy:.2f} after "
                f"{drive.non_rewards} fruitless sweep(s)"
            )
        self.blackboard.seek_object_class = ""
        self.blackboard.seek_target_position = None
        self.blackboard.seek_last_known = None
        self.blackboard.seek_sighted_position = None
        self.blackboard.seek_search_visited = set()
        self.blackboard.seek_search_waypoints = []
        self.blackboard.seek_grasped = False
        self.blackboard.seek_unreachable_reason = ""
        self.blackboard.seek_object_height = None
        self.blackboard.seek_person_position = None
        self.blackboard.seek_alternations = 0
        drive.reset()
        return py_trees.common.Status.SUCCESS
