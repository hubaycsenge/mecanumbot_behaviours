"""Navigation behaviours, plus the public face of the behaviour library.

`Approach` (the only behaviour that actually drives somewhere) lives here and
goes through nav2. The in-place turns live in `turning.py`, the recovery patrol
in `searching.py`, and the geometry/ROS helpers in `geometry.py` /
`ros_interfaces.py`; they are re-exported at the bottom of this module so
`from ...behaviours.movement_managers import X` keeps working for every tree and
for the `mecanumbot_demo_behaviours` package.
"""

import py_trees

from mecanumbot_leading_behaviour.behaviours.geometry import (
    calculate_facing_orientation,
    distance_xy,
    normalize_angle,
    pose_to_goal,
    quaternion_from_yaw,
    yaw_from_quaternion,
)
from mecanumbot_leading_behaviour.behaviours.ros_interfaces import (
    BallTracker,
    GOAL_ACTIVE_STATUSES,
    GOAL_FAILED_STATUSES,
    Nav2GoalMonitor,
    PeopleTracker,
    RobotPoseTracker,
    STATUS_ABORTED,
    STATUS_ACCEPTED,
    STATUS_CANCELED,
    STATUS_CANCELING,
    STATUS_EXECUTING,
    STATUS_SUCCEEDED,
    STATUS_UNKNOWN,
)
from mecanumbot_leading_behaviour.behaviours.searching import (  # noqa: F401  (re-export)
    ManageSearchCheckpoint,
    WaitForPerson,
)
from mecanumbot_leading_behaviour.behaviours.targets import (
    CHECKPOINT,
    SUBJECT,
    is_human,
    register_target_keys,
    resolve_target_position,
)
from mecanumbot_leading_behaviour.behaviours.turning import (  # noqa: F401  (re-export)
    FindPeople,
    InPlaceTurn,
    RelativeTurnPattern,
    ScanSpin,
    SmoothTurner,
    Spin360,
    TurnToward,
)


class Approach(py_trees.behaviour.Behaviour):
    """Drive to a target with nav2, resending goals that nav2 gives up on.

    `mode="exact"` aims for the target minus `robot_closeness_threshold`,
    `mode="fixed_distance"` only steps `robot_approach_distance` closer per run,
    which is how the robot walks up to a human in stages.
    """

    def __init__(
        self,
        name="Approach",
        target_type=SUBJECT,
        mode="exact",
        target_timeout=3.0,
        goal_timeout=10.0,
        sight_timeout=1.0,
    ):
        super().__init__(name)
        self.target_type = target_type
        self.mode = mode
        self.target_timeout = float(target_timeout)
        self.goal_timeout = float(goal_timeout)
        self.sight_timeout = float(sight_timeout)

        self.blackboard = self.attach_blackboard_client(name=name)
        register_target_keys(self.blackboard)
        self.blackboard.register_key(
            "robot_closeness_threshold", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            "robot_approach_distance", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            "Dog_max_checkpoint", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            "Dog_current_checkpoint", access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.pose = RobotPoseTracker(self.node)
        self.nav2 = Nav2GoalMonitor(self.node)
        self.people = (
            PeopleTracker(self.node, self.sight_timeout)
            if is_human(self.target_type)
            else None
        )
        self.logger.info(f"{self.name}: Setup complete")

    def initialise(self):
        self.nav2.reset()
        self._target_position = None
        self._start_time = self.node.get_clock().now()
        self.node.get_logger().info(f"{self.name}: approaching the {self.target_type}")

    def update(self):
        if self.pose.pose is None:
            self.feedback_message = "waiting for AMCL pose"
            return py_trees.common.Status.RUNNING

        if self._target_position is None:
            self._target_position = self._look_up_target()
        if self._target_position is None:
            if self._elapsed() > self.target_timeout:
                self.node.get_logger().info(
                    f"{self.name}: no {self.target_type} to approach after "
                    f"{self.target_timeout:.0f} s, giving up"
                )
                return py_trees.common.Status.FAILURE
            self.feedback_message = f"waiting for a {self.target_type} to approach"
            return py_trees.common.Status.RUNNING

        if not self.nav2.goal_sent:
            if self.nav2.busy():
                self.feedback_message = "waiting for the previous nav2 goal to finish"
                return py_trees.common.Status.RUNNING
            self._send_goal()
            return py_trees.common.Status.RUNNING

        status = self.nav2.status()
        if status is None:
            if self.nav2.seconds_since_send() > self.goal_timeout:
                self.node.get_logger().info(
                    f"{self.name}: nav2 never reported our goal within "
                    f"{self.goal_timeout:.0f} s, giving up"
                )
                return py_trees.common.Status.FAILURE
            self.feedback_message = "waiting for nav2 to accept the goal"
            return py_trees.common.Status.RUNNING

        if status == STATUS_SUCCEEDED:
            return self._goal_reached()
        if status in GOAL_ACTIVE_STATUSES:
            self.feedback_message = f"driving to the {self.target_type}"
            return py_trees.common.Status.RUNNING

        self.node.get_logger().info(
            f"{self.name}: nav2 dropped the goal, sending it again"
        )
        self.nav2.reset()
        return py_trees.common.Status.RUNNING

    # --- internals ----------------------------------------------------------

    def _elapsed(self):
        return (self.node.get_clock().now() - self._start_time).nanoseconds / 1e9

    def _look_up_target(self):
        subject_position = None
        if self.people is not None:
            if self.people.last_seen_pose is None:
                return None
            subject_position = self.people.last_seen_pose.position
        return resolve_target_position(
            self.blackboard, self.target_type, subject_position
        )

    def _send_goal(self):
        go_threshold = (
            self.blackboard.robot_approach_distance
            if is_human(self.target_type)
            else 1.0
        )
        goal = pose_to_goal(
            self._target_position,
            self.pose.pose,
            stop_threshold=self.blackboard.robot_closeness_threshold,
            mode=self.mode,
            go_threshold=go_threshold,
        )
        self.nav2.send(goal)
        self.node.get_logger().info(
            f"{self.name}: goal for the {self.target_type} at "
            f"x={goal.position.x:.2f} y={goal.position.y:.2f}"
        )

    def _goal_reached(self):
        if self.target_type == CHECKPOINT:
            current = self.blackboard.Dog_current_checkpoint
            if current < self.blackboard.Dog_max_checkpoint:
                self.blackboard.Dog_current_checkpoint = current + 1
            self.node.get_logger().info(
                f"{self.name}: checkpoint {current + 1}/"
                f"{self.blackboard.Dog_max_checkpoint + 1} reached"
            )
        else:
            self.node.get_logger().info(f"{self.name}: reached the {self.target_type}")
        return py_trees.common.Status.SUCCESS


class CheckSubjectTargetSuccess(py_trees.behaviour.Behaviour):
    """SUCCESS once the human stands within `target_reached_threshold` of the target."""

    def __init__(self, name="CheckSubjectTargetSuccess", sight_timeout=1.0):
        super().__init__(name)
        self.sight_timeout = float(sight_timeout)

        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            "target_position", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            "target_reached_threshold", access=py_trees.common.Access.READ
        )

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.people = PeopleTracker(self.node, self.sight_timeout)
        self.logger.info(f"{self.name}: Setup complete")

    def update(self):
        subject_pose = self.people.last_seen_pose
        if subject_pose is None:
            self.feedback_message = "no person seen yet"
            return py_trees.common.Status.RUNNING

        distance = distance_xy(subject_pose.position, self.blackboard.target_position)
        threshold = self.blackboard.target_reached_threshold
        if distance <= threshold:
            self.node.get_logger().info(
                f"{self.name}: person is at the target ({distance:.2f} m <= {threshold:.2f} m)"
            )
            return py_trees.common.Status.SUCCESS

        self.node.get_logger().info(
            f"{self.name}: person is {distance:.2f} m from the target (needs {threshold:.2f} m)"
        )
        return py_trees.common.Status.FAILURE


class CheckRobotHasBall(py_trees.behaviour.Behaviour):
    """SUCCESS while `/mecanumbot/has_object` reports the ball is held."""

    def __init__(self, name="CheckRobotHasBall"):
        super().__init__(name)

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.ball = BallTracker(self.node)
        self.logger.info(f"{self.name}: Setup complete")

    def update(self):
        if self.ball.has_ball is None:
            self.feedback_message = "no ball state received yet"
            return py_trees.common.Status.RUNNING
        return (
            py_trees.common.Status.SUCCESS
            if self.ball.has_ball
            else py_trees.common.Status.FAILURE
        )


class CheckRobotAtLastCheckpoint(py_trees.behaviour.Behaviour):
    """SUCCESS when the robot reached the last checkpoint of the route."""

    def __init__(self, name="CheckRobotAtLastCheckpoint"):
        super().__init__(name)

        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            "Dog_current_checkpoint", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            "Dog_max_checkpoint", access=py_trees.common.Access.READ
        )

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.logger.info(f"{self.name}: Setup complete")

    def update(self):
        current = self.blackboard.Dog_current_checkpoint
        last = self.blackboard.Dog_max_checkpoint
        self.feedback_message = f"checkpoint {current}/{last}"
        if current >= last:
            self.node.get_logger().info(f"{self.name}: robot is at the last checkpoint")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


__all__ = [
    # behaviours defined here
    "Approach",
    "CheckRobotAtLastCheckpoint",
    "CheckRobotHasBall",
    "CheckSubjectTargetSuccess",
    # turning behaviours
    "FindPeople",
    "InPlaceTurn",
    "RelativeTurnPattern",
    "ScanSpin",
    "SmoothTurner",
    "Spin360",
    "TurnToward",
    # search behaviours
    "ManageSearchCheckpoint",
    "WaitForPerson",
    # helpers kept for the demo package and older imports
    "calculate_facing_orientation",
    "distance_xy",
    "normalize_angle",
    "pose_to_goal",
    "quaternion_from_yaw",
    "yaw_from_quaternion",
    "GOAL_ACTIVE_STATUSES",
    "GOAL_FAILED_STATUSES",
    "STATUS_ABORTED",
    "STATUS_ACCEPTED",
    "STATUS_CANCELED",
    "STATUS_CANCELING",
    "STATUS_EXECUTING",
    "STATUS_SUCCEEDED",
    "STATUS_UNKNOWN",
]
