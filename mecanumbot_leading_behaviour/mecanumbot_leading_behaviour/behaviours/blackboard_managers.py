"""
Loading a leading constants file, and the live measurements that go with it.

The loading itself is `mecanumbot_bt_config`'s and knows no key names: it writes
whatever the YAML declares, turns `_deg` into radians and the structured strings
into `Point`, `SetLedStatus` and `AccessMotorCmd` messages, and fills in the
packaged default for anything the file leaves out. What is declared *here* is
only what a constants file cannot say about itself:

* which keys are the experiment and so may not be missing (`defaults.REQUIRED`
  plus the signalling scripts of whichever condition is running);
* what the checkpoint list means -- the first entry is where the run starts, the
  last is where the human should end up, and the rest is the route the robot
  drives;
* which keys are the run's own state rather than configuration, so restarting
  the tree starts them over.
"""

import py_trees

from mecanumbot_bt_config.blackboard import ConfiguredTimer as _ConfiguredTimer
from mecanumbot_bt_config.blackboard import ParamsToBlackboard
from mecanumbot_msgs.srv import SetLedStatus
from mecanumbot_movement_behaviours.defaults import configure_accessories
from mecanumbot_movement_behaviours.geometry import distance_xy
from mecanumbot_movement_behaviours.ros_interfaces import (
    AccessoryCommander,
    HEAD_SEEK,
    RobotPoseTracker,
    SubjectPoseTracker,
)
from mecanumbot_movement_behaviours.routes import SEARCH_BACKWARDS

from mecanumbot_leading_behaviour.behaviours.defaults import (
    LOADED_DEFAULTS,
    REQUIRED,
    TUNABLES,
)

LED_SERVICE = "/mecanumbot/set_led_status"

# The signalling scripts, as pairs of "the steps" and "how long each one lasts".
# A tree that signals declares the ones it plays; a tree that does not (the
# control condition) declares none, and its constants file need not carry them.
LED_SCRIPTS = (
    "LED_indicate_target",
    "LED_indicate_close_target",
    "LED_catch_attention",
    "LED_thank",
)

GESTURE_SCRIPTS = (
    "Dog_indicate_target",
    "Dog_catch_attention",
    "Dog_thank",
)


def script_keys(*scripts):
    """`("LED_thank",)` -> `("LED_thank_seq", "LED_thank_times")`."""
    return tuple(f"{script}_{part}" for script in scripts for part in ("seq", "times"))


# Written by `split_route` rather than read from the file, so the loader has to
# be told to claim them.
ROUTE_KEYS = (
    "start_position",
    "target_position",
    "Dog_current_checkpoint",
    "Dog_max_checkpoint",
    "patrol_checkpoints",
)

# The run's own state, seeded every time the loader is entered.
RUN_STATE = {
    "patrol_current_checkpoint": 0,
    "patrol_direction": SEARCH_BACKWARDS,
    "patrol_initialized": False,
    # Handedness of the last turn made while looking for a person; the turns
    # back to the route unwind it. 0 until the robot has turned at all.
    "search_spin_sign": 0,
    # Pacing of the look backs: checkpoints driven since the last one, and when
    # it was. 0.0 means "not started yet" -- `DogCheckInDue` starts the clock
    # the first time it is asked, which is when leading begins rather than when
    # this file is read.
    "check_in_checkpoints_since": 0,
    "check_in_last_time": 0.0,
    "last_distance": 200.0,
}


def split_route(node, blackboard, values):
    """
    Read the checkpoint list as a route with a start and an end.

    The first entry is where the run starts, the last is where the human should
    end up, and the entries in between -- plus the first -- are the route the
    robot drives. `patrol_checkpoints` is that same list, walked separately when
    the robot has lost its human and is searching for them.
    """
    checkpoints = list(values["Dog_checkpoints"])
    route = checkpoints[:-1]

    blackboard.start_position = checkpoints[0]
    blackboard.target_position = checkpoints[-1]
    blackboard.Dog_checkpoints = route
    blackboard.Dog_current_checkpoint = 0
    blackboard.Dog_max_checkpoint = len(route) - 1
    blackboard.patrol_checkpoints = list(route)

    node.get_logger().info(
        f"{len(route)} route checkpoints: "
        f"{[(round(cp.x, 2), round(cp.y, 2)) for cp in route]}"
    )


class ConstantParamsToBlackboard(ParamsToBlackboard):
    """
    Load the leading constants onto the blackboard and set the robot's start state.

    `scripts` names the signalling sequences the tree that loads this actually
    plays, and they are required of the constants file: a condition that signals
    with LEDs and finds no LED sequence has nothing to signal with. The control
    condition passes none.

    `defaults` and `required` extend the two declarations rather than replacing
    them, for a package that builds on these trees with constants of its own --
    `mecanumbot_demo_behaviours` does.
    """

    def __init__(
        self,
        name,
        yaml_path,
        scripts=LED_SCRIPTS + GESTURE_SCRIPTS,
        defaults=(),
        required=(),
    ):
        self.scripts = tuple(scripts)
        super().__init__(
            name=name,
            yaml_path=yaml_path,
            defaults=LOADED_DEFAULTS + tuple(defaults),
            required=(
                REQUIRED
                + ("LED_start_setting",)
                + script_keys(*self.scripts)
                + tuple(required)
            ),
            state=RUN_STATE,
            on_loaded=(configure_accessories, split_route),
            extra_keys=ROUTE_KEYS,
        )

    def setup(self, **kwargs):
        """Load the constants, then take the handles the start state needs."""
        loaded = super().setup(**kwargs)
        self.accessories = AccessoryCommander(self.node)
        self.led_client = self.node.create_client(SetLedStatus, LED_SERVICE)
        return loaded

    def initialise(self):
        """Seed the run state, put the LEDs in their start setting, lift the head."""
        super().initialise()
        self.led_client.call_async(_one(self.blackboard.LED_start_setting))
        # Start with the head lifted: ready to greet, and the pose detector sees
        # whole people rather than just their knees.
        self.accessories.look(HEAD_SEEK)


def _one(setting):
    """Take the single entry out of a one-entry sequence, or pass a value through."""
    return setting[0] if isinstance(setting, list) else setting


class ConfiguredTimer(_ConfiguredTimer):
    """A timer whose duration is one of this package's constants."""

    def __init__(self, name, key):
        super().__init__(name=name, key=key, tunables=TUNABLES)


class DistanceToBlackboard(py_trees.behaviour.Behaviour):
    """Publish robot-subject, robot-target and subject-target distances on the blackboard."""

    def __init__(self, name="ComputeDistance"):
        super().__init__(name)

        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            "target_position", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            "robot_closeness_threshold", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            "target_reached_threshold", access=py_trees.common.Access.READ
        )

        for key in (
            "robot_distance_from_subject",
            "subject_within_robot_threshold",
            "distance_from_target",
            "target_within_robot_threshold",
            "target_distance_from_subject",
            "target_within_subject_threshold",
        ):
            self.blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.pose = RobotPoseTracker(self.node)
        self.subject = SubjectPoseTracker(self.node)
        self.logger.info(f"{self.name}: Setup complete")
        return True

    def update(self):
        if self.pose.position is None or self.subject.position is None:
            self.feedback_message = "waiting for robot and subject poses"
            self.node.get_logger().info(f"{self.name}: {self.feedback_message}")
            return py_trees.common.Status.RUNNING

        target = self.blackboard.target_position
        robot_to_subject = distance_xy(self.pose.position, self.subject.position)
        robot_to_target = distance_xy(self.pose.position, target)
        subject_to_target = distance_xy(self.subject.position, target)

        closeness = self.blackboard.robot_closeness_threshold
        reached = self.blackboard.target_reached_threshold

        self.blackboard.robot_distance_from_subject = robot_to_subject
        self.blackboard.subject_within_robot_threshold = robot_to_subject < closeness
        self.blackboard.distance_from_target = robot_to_target
        self.blackboard.target_within_robot_threshold = robot_to_target < closeness
        self.blackboard.target_distance_from_subject = subject_to_target
        self.blackboard.target_within_subject_threshold = subject_to_target < reached

        self.feedback_message = (
            f"d(R, S) = {robot_to_subject:.2f} | d(R, T) = {robot_to_target:.2f} | "
            f"d(S, T) = {subject_to_target:.2f}"
        )
        self.node.get_logger().info(f"{self.name}: {self.feedback_message}")
        return py_trees.common.Status.SUCCESS
