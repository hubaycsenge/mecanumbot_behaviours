"""
Loading the fetch constants, and owning the state one episode carries.

The loading is `mecanumbot_bt_config`'s and knows no key names: it writes
whatever the YAML declares, turns a `_deg` parameter into radians under the name
without the suffix, and fills in a packaged default for anything left out. This
module only says the two things a file cannot say about itself -- which keys a
run must not be missing, and which keys are the run's own state rather than
configuration.

What is required here is small and is about the robot's body and the people in
the room: how close it may drive to a person, where the grabbers sit open and
shut, how close counts as close enough to grip. Everything else -- the search
pattern, the head sweep, the timeouts -- has a default, because a file written
before a key existed should keep behaving as it did.
"""

import py_trees

from mecanumbot_bt_config.blackboard import ParamsToBlackboard
from mecanumbot_bt_config.tree_runner import RUNTIME_DEFAULTS
from mecanumbot_movement_behaviours.defaults import (
    MOVEMENT_DEFAULTS,
    configure_accessories,
)

from mecanumbot_fetch_behaviour.behaviours.ros_interfaces import GripperCommander
from mecanumbot_fetch_behaviour.defaults import FETCH_DEFAULTS, constant
from mecanumbot_fetch_behaviour.gaze import SEARCH_GAZES
from mecanumbot_fetch_behaviour.search_patterns import FACINGS

# What describes this robot and the people in the room rather than the
# machinery, and so has no defensible stand-in.
FETCH_REQUIRED = (
    "fetch_approach_stop",
    "fetch_grasp_distance",
    "fetch_gripper_open_left",
    "fetch_gripper_open_right",
    "fetch_gripper_closed_left",
    "fetch_gripper_closed_right",
    # The movement library spells these `robot_approach_distance` /
    # `robot_closeness_threshold`, and every experiment here declares them:
    # there is no defensible default for how close a robot may drive to
    # somebody, and this tree drives at a person holding out a ball.
    "robot_approach_distance",
    "robot_closeness_threshold",
)

LOADED_DEFAULTS = (RUNTIME_DEFAULTS, MOVEMENT_DEFAULTS, FETCH_DEFAULTS)

# The episode's own state, seeded so no behaviour has to test whether a key
# exists yet, and cleared on every entry so an episode interrupted halfway
# through does not resume against a stale ball.
#
# `fetch_ball_position` is the spelling `keys.FETCH_KEYS` binds onto the
# movement library's `target_position` field, which is what lets `FetchApproach`
# drive to the ball with no navigation code in this package.
RUN_STATE = {
    "fetch_ball_position": None,
    "fetch_ball_score": 0.0,
    "fetch_ball_height": None,
    "fetch_search_waypoints": list,
    "fetch_search_visited": set,
    "fetch_laps_done": 0,
    "fetch_grasped": False,
    "fetch_person_position": None,
    "fetch_delivered": False,
    # The neck position the fetch behaviours last commanded, so the one that
    # takes the head over starts from where it is. Not cleared between
    # episodes: it describes the head, not the ball.
    "fetch_head_position": None,
    # Belongs to the movement library's `InPlaceTurn`, which the scan borrows.
    "search_spin_sign": 0,
}

PARAM_KEYS = tuple(FETCH_DEFAULTS)


def register_param_keys(blackboard):
    """Give a blackboard client read access to every fetch constant."""
    for key in PARAM_KEYS:
        blackboard.register_key(key=key, access=py_trees.common.Access.READ)


def check_fetch_geometry(node, blackboard, values):
    """Warn rather than fail when the constants cannot produce the behaviour."""
    if values["fetch_grasp_distance"] >= values["fetch_approach_stop"]:
        node.get_logger().warn(
            f"fetch_grasp_distance ({values['fetch_grasp_distance']}) is not below "
            f"fetch_approach_stop ({values['fetch_approach_stop']}); the approach "
            "parks further away than the grab needs and every grab will miss"
        )
    if values["fetch_grasp_height_min"] >= values["fetch_grasp_height_max"]:
        node.get_logger().warn(
            f"fetch_grasp_height_min ({values['fetch_grasp_height_min']}) is not "
            f"below fetch_grasp_height_max ({values['fetch_grasp_height_max']}); "
            "no ball will ever be judged reachable"
        )
    if values["fetch_circle_max"] < values["fetch_circle_first"]:
        node.get_logger().warn(
            f"fetch_circle_max ({values['fetch_circle_max']}) is inside "
            f"fetch_circle_first ({values['fetch_circle_first']}); the search "
            "will walk one circle and never widen"
        )
    if values["fetch_circle_facing"] not in FACINGS:
        node.get_logger().warn(
            f"fetch_circle_facing '{values['fetch_circle_facing']}' is not one of "
            f"{FACINGS}; the search will fail as soon as it lays out a circle"
        )
    if values["fetch_head_low"] > values["fetch_head_high"]:
        node.get_logger().warn(
            f"fetch_head_low ({values['fetch_head_low']}) is above fetch_head_high "
            f"({values['fetch_head_high']}); the head cannot follow a ball, and "
            "'low' and 'high' now mean the opposite of what they say"
        )
    if not values["fetch_head_low"] <= values["fetch_head_search"] <= values["fetch_head_high"]:
        node.get_logger().warn(
            f"fetch_head_search ({values['fetch_head_search']}) is outside "
            f"fetch_head_low..fetch_head_high; the first step after a sighting "
            "will jump the head into that range"
        )
    if values["fetch_head_search_mode"] not in SEARCH_GAZES:
        node.get_logger().warn(
            f"fetch_head_search_mode '{values['fetch_head_search_mode']}' is not "
            f"one of {SEARCH_GAZES}; the tree will refuse to build"
        )


class FetchParamsToBlackboard(ParamsToBlackboard):
    """Load the fetch constants and hand the accessory poses to the commander."""

    def __init__(self, name, yaml_path):
        super().__init__(
            name=name,
            yaml_path=yaml_path,
            defaults=LOADED_DEFAULTS,
            required=FETCH_REQUIRED,
            state=RUN_STATE,
            on_loaded=(configure_accessories, check_fetch_geometry),
        )


class ClearFetchEpisode(py_trees.behaviour.Behaviour):
    """
    Forget the ball and the person, and return SUCCESS.

    Used twice in the tree: after an episode that ended in a handover, and as
    the fallback that absorbs one that failed. Both go through here because the
    answer to either is the same -- let this ball go and start looking again.
    An episode that kept the last ball's position would send the robot straight
    back to where the ball used to be, which is exactly where it is not.

    It also hands the open grippers back to every head command, so a round
    abandoned with the ball held does not leave the next search commanding
    closed grabbers.
    """

    def __init__(self, name="ClearFetchEpisode", reason=""):
        super().__init__(name)
        self.reason = reason
        self.blackboard = self.attach_blackboard_client(name=name)
        for key in ("fetch_gripper_open_left", "fetch_gripper_open_right"):
            self.blackboard.register_key(key=key, access=py_trees.common.Access.READ)
        for key in (
            "fetch_ball_position",
            "fetch_ball_score",
            "fetch_ball_height",
            "fetch_search_waypoints",
            "fetch_search_visited",
            "fetch_laps_done",
            "fetch_grasped",
            "fetch_person_position",
            "fetch_delivered",
        ):
            self.blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        """Keep the node handle for logging and the state publisher."""
        self.node = kwargs["node"]
        self.logger.info(f"{self.name}: Setup complete")
        return True

    def update(self):
        """Drop the ball, the person and the search, and report the outcome."""
        self.node.get_logger().info(
            f"{self.name}: episode over"
            + (f" ({self.reason})" if self.reason else "")
            + f"; grasped={self.blackboard.fetch_grasped}, "
            f"delivered={self.blackboard.fetch_delivered}"
        )
        self.blackboard.fetch_ball_position = None
        self.blackboard.fetch_ball_score = 0.0
        self.blackboard.fetch_ball_height = None
        self.blackboard.fetch_search_waypoints = []
        self.blackboard.fetch_search_visited = set()
        self.blackboard.fetch_laps_done = 0
        self.blackboard.fetch_grasped = False
        self.blackboard.fetch_person_position = None
        self.blackboard.fetch_delivered = False
        GripperCommander.keep_grippers(
            constant(self.blackboard, "fetch_gripper_open_left"),
            constant(self.blackboard, "fetch_gripper_open_right"),
        )
        return py_trees.common.Status.SUCCESS
