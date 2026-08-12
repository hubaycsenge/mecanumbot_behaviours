"""
Behaviours that put configuration and live measurements on the blackboard.

The constants themselves -- which tunables exist, what they default to, how the
YAML is read -- live in `constants.py`; this module is what writes them onto the
blackboard, alongside the experiment parameters and the runtime state.
"""

import ast
import math

import py_trees
from geometry_msgs.msg import Point

from mecanumbot_msgs.msg import AccessMotorCmd
from mecanumbot_msgs.srv import SetLedStatus

from mecanumbot_leading_behaviour.behaviours.constants import (  # noqa: F401  (re-export)
    ANGLE_PARAMS,
    INTEGER_TUNABLES,
    PARAM_KEYS,
    TUNABLE_DEFAULTS,
    YAML_ROOT_KEYS,
    constant,
    file_constant,
    load_params,
    register_param_keys,
    resolve,
)
from mecanumbot_leading_behaviour.behaviours.geometry import distance_xy
from mecanumbot_leading_behaviour.behaviours.ros_interfaces import (
    AccessoryCommander,
    HEAD_SEEK,
    RobotPoseTracker,
    SubjectPoseTracker,
)
from mecanumbot_leading_behaviour.behaviours.searching import SEARCH_BACKWARDS

LED_SERVICE = "/mecanumbot/set_led_status"


def parse_led(entry):
    """`"{'fl': {'color': .., 'mode': ..}, ...}"` -> SetLedStatus request."""
    corners = ast.literal_eval(entry)
    request = SetLedStatus.Request()
    request.fl_color, request.fl_mode = corners["fl"]["color"], corners["fl"]["mode"]
    request.fr_color, request.fr_mode = corners["fr"]["color"], corners["fr"]["mode"]
    request.bl_color, request.bl_mode = corners["bl"]["color"], corners["bl"]["mode"]
    request.br_color, request.br_mode = corners["br"]["color"], corners["br"]["mode"]
    return request


def parse_gesture(entry):
    """`"{'n_pos': .., 'gl_pos': .., 'gr_pos': ..}"` -> AccessMotorCmd."""
    positions = ast.literal_eval(entry)
    cmd = AccessMotorCmd()
    cmd.n_pos = float(positions["n_pos"])
    cmd.gl_pos = float(positions["gl_pos"])
    cmd.gr_pos = float(positions["gr_pos"])
    return cmd


def parse_checkpoint(entry):
    """`"{'X': .., 'Y': .., 'Z': ..}"` -> Point."""
    coordinates = ast.literal_eval(entry)
    point = Point()
    point.x = coordinates["X"]
    point.y = coordinates["Y"]
    point.z = coordinates["Z"]
    return point


class ConstantParamsToBlackboard(py_trees.behaviour.Behaviour):
    """Load the YAML constants onto the blackboard and set the robot's start state.

    The checkpoint list is split up: the first entry is `start_position`, the last
    is `target_position` (where the human should end up), and the entries in
    between -- plus the first -- stay in `Dog_checkpoints` as the route the robot
    drives. `patrol_checkpoints` is the same list, walked separately when the
    robot has lost its human and is searching for them.

    Keys written:

    * thresholds: `robot_closeness_threshold`, `robot_approach_distance`,
      `target_reached_threshold`, `visibility_time_threshold`,
      `Dog_following_max_threshold`, `Dog_max_wander_allowed`, `init_delay`
    * route: `Dog_checkpoints`, `Dog_current_checkpoint`, `Dog_max_checkpoint`,
      `start_position`, `target_position`
    * search state: `patrol_checkpoints`, `patrol_current_checkpoint`,
      `patrol_direction`, `patrol_initialized`, `search_spin_sign`
    * signalling scripts: `LED_*_seq` / `_times`, `Dog_*_seq` / `_times`
    * tunables: every key of `TUNABLE_DEFAULTS`, defaulted when the YAML file
      does not declare it
    """

    SCALAR_PARAMS = (
        "init_delay",
        "robot_closeness_threshold",
        "robot_approach_distance",
        "target_reached_threshold",
        "visibility_time_threshold",
        "Dog_following_max_threshold",
    )

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

    ROUTE_KEYS = (
        "Dog_checkpoints",
        "Dog_current_checkpoint",
        "Dog_max_checkpoint",
        "start_position",
        "target_position",
    )

    SEARCH_STATE_KEYS = (
        "patrol_checkpoints",
        "patrol_current_checkpoint",
        "patrol_direction",
        "patrol_initialized",
        "search_spin_sign",
    )

    def __init__(self, name, yaml_path):
        super().__init__(name=name)
        self.yaml_path = yaml_path
        self.blackboard = self.attach_blackboard_client(name=name)

        for key in (
            self.SCALAR_PARAMS
            + self.ROUTE_KEYS
            + self.SEARCH_STATE_KEYS
            + PARAM_KEYS
            + ("Dog_max_wander_allowed", "LED_start_setting", "last_distance")
        ):
            self.blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)

        for script in self.LED_SCRIPTS + self.GESTURE_SCRIPTS:
            self.blackboard.register_key(
                key=f"{script}_seq", access=py_trees.common.Access.WRITE
            )
            self.blackboard.register_key(
                key=f"{script}_times", access=py_trees.common.Access.WRITE
            )

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.accessories = AccessoryCommander(self.node)
        self.led_client = self.node.create_client(SetLedStatus, LED_SERVICE)

        params = load_params(self.yaml_path)
        self._write_thresholds(params)
        self._write_tunables(params)
        self._write_scripts(params)
        self._write_route(params)
        self._write_search_state()

        self.feedback_message = "constants loaded"
        self.node.get_logger().info(
            f"{self.name}: constants loaded from {self.yaml_path}"
        )
        return True

    def initialise(self):
        self.led_client.call_async(self.blackboard.LED_start_setting)
        # Start with the head lifted: ready to greet, and the pose detector sees
        # whole people rather than just their knees.
        self.accessories.look(HEAD_SEEK)

    def update(self):
        return py_trees.common.Status.SUCCESS

    # --- internals ----------------------------------------------------------

    def _write_thresholds(self, params):
        for key in self.SCALAR_PARAMS:
            setattr(self.blackboard, key, float(params[key]))
        self.blackboard.Dog_max_wander_allowed = params["Dog_max_wander_allowed"]
        self.blackboard.last_distance = 200.0

    def _write_tunables(self, params):
        """Write every tunable, filling in the packaged default for missing ones."""
        missing = []
        for yaml_key, blackboard_key in ANGLE_PARAMS.items():
            default = math.degrees(TUNABLE_DEFAULTS[blackboard_key])
            if yaml_key not in params:
                missing.append(yaml_key)
            setattr(
                self.blackboard,
                blackboard_key,
                math.radians(float(params.get(yaml_key, default))),
            )

        for key, default in TUNABLE_DEFAULTS.items():
            if key in ANGLE_PARAMS.values():
                continue
            if key not in params:
                missing.append(key)
            value = params.get(key, default)
            setattr(
                self.blackboard,
                key,
                int(value) if key in INTEGER_TUNABLES else float(value),
            )

        # The neck and gripper poses belong to the commander rather than to any
        # one behaviour, so they are handed over once here instead of being
        # threaded through every constructor that creates one.
        AccessoryCommander.configure(
            seek_pos=self.blackboard.neck_seek_pos,
            level_pos=self.blackboard.neck_level_pos,
            gripper_left=self.blackboard.gripper_left_neutral,
            gripper_right=self.blackboard.gripper_right_neutral,
        )

        if missing:
            self.node.get_logger().info(
                f"{self.name}: {len(missing)} tunable(s) not in the YAML, using the "
                f"packaged defaults: {', '.join(sorted(missing))}"
            )

    def _write_scripts(self, params):
        self.blackboard.LED_start_setting = parse_led(params["LED_start_setting"][0])
        for script in self.LED_SCRIPTS:
            setattr(
                self.blackboard,
                f"{script}_seq",
                [parse_led(entry) for entry in params[f"{script}_seq"]],
            )
            setattr(self.blackboard, f"{script}_times", params[f"{script}_times"])
        for script in self.GESTURE_SCRIPTS:
            setattr(
                self.blackboard,
                f"{script}_seq",
                [parse_gesture(entry) for entry in params[f"{script}_seq"]],
            )
            setattr(self.blackboard, f"{script}_times", params[f"{script}_times"])

    def _write_route(self, params):
        checkpoints = [parse_checkpoint(entry) for entry in params["Dog_checkpoints"]]
        self.blackboard.start_position = checkpoints[0]
        self.blackboard.target_position = checkpoints[-1]
        self.blackboard.Dog_checkpoints = checkpoints[:-1]
        self.blackboard.Dog_current_checkpoint = 0
        self.blackboard.Dog_max_checkpoint = len(self.blackboard.Dog_checkpoints) - 1
        self.node.get_logger().info(
            f"{self.name}: {len(self.blackboard.Dog_checkpoints)} route checkpoints: "
            f"{[(round(cp.x, 2), round(cp.y, 2)) for cp in self.blackboard.Dog_checkpoints]}"
        )

    def _write_search_state(self):
        self.blackboard.patrol_checkpoints = list(self.blackboard.Dog_checkpoints)
        self.blackboard.patrol_current_checkpoint = 0
        self.blackboard.patrol_direction = SEARCH_BACKWARDS
        self.blackboard.patrol_initialized = False
        # Handedness of the last turn made while looking for a person; the turns
        # back to the route unwind it. 0 until the robot has turned at all.
        self.blackboard.search_spin_sign = 0


class ConfiguredTimer(py_trees.timers.Timer):
    """
    A `py_trees` timer whose duration is a blackboard constant.

    `py_trees.timers.Timer` takes its duration at construction, which is while
    the tree is being built and therefore before any YAML has been read. This
    subclass looks the duration up when the timer starts instead, so a pause
    between two gestures is configured in the same file as the gestures.
    """

    def __init__(self, name, key):
        super().__init__(name=name, duration=0.0)
        self.key = key
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key=key, access=py_trees.common.Access.READ)

    def initialise(self):
        """Take the duration from the blackboard, then start the timer."""
        self.duration = float(constant(self.blackboard, self.key))
        super().initialise()


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
