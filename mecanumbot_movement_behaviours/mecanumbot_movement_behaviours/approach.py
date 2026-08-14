"""
Driving to one place with nav2, and the checks that go with having got there.

`Approach` is the single-goal move: it navigates to whatever a target type
points at and stops short of it. Driving a *route* -- several checkpoints in one
waypoint goal -- is `routes.FollowRoute`, and the in-place turns are
`turning.py`; translations all go through nav2's actions, so obstacle avoidance
is untouched for anything that actually drives somewhere.
"""

import py_trees

from mecanumbot_movement_behaviours.defaults import (
    constant,
    register_param_keys,
    resolve,
)
from mecanumbot_movement_behaviours.geometry import distance_xy, pose_to_goal
from mecanumbot_movement_behaviours.keys import DEFAULT_KEYS
from mecanumbot_movement_behaviours.ros_interfaces import (
    BallTracker,
    GOAL_ACTIVE_STATUSES,
    Nav2PoseNavigator,
    PeopleTracker,
    RobotPoseTracker,
    STATUS_SUCCEEDED,
)
from mecanumbot_movement_behaviours.targets import (
    CHECKPOINT,
    SUBJECT,
    is_human,
    register_target_keys,
    resolve_target_position,
)


class Approach(py_trees.behaviour.Behaviour):
    """
    Drive to a target with nav2, resending goals that nav2 gives up on.

    `mode="exact"` aims for the target minus the closeness threshold,
    `mode="fixed_distance"` only steps the approach distance closer per run,
    which is how the robot walks up to a human in stages.

    The goal goes through the `NavigateToPose` action rather than the
    `/goal_pose` topic, so the outcome belongs to this goal rather than being
    guessed from a status array, and the drive is cancelled when the behaviour
    stops -- a turn that follows owns `/cmd_vel` straight away instead of
    waiting for nav2 to notice it is finished.
    """

    KEYS = DEFAULT_KEYS

    def __init__(
        self,
        name="Approach",
        target_type=SUBJECT,
        mode="exact",
        target_timeout=None,
        goal_timeout=None,
        sight_timeout=None,
        retries=None,
        keys=None,
    ):
        super().__init__(name)
        self.target_type = target_type
        self.mode = mode
        self.target_timeout = target_timeout
        self.goal_timeout = goal_timeout
        self.sight_timeout = sight_timeout
        self.retries = retries
        self.keys = keys or self.KEYS

        self.blackboard = self.attach_blackboard_client(name=name)
        register_target_keys(self.blackboard, self.keys)
        register_param_keys(self.blackboard)
        self.keys.register(
            self.blackboard,
            py_trees.common.Access.READ,
            "closeness_threshold",
            "approach_distance",
            "max_checkpoint",
        )
        self.keys.register(
            self.blackboard, py_trees.common.Access.WRITE, "current_checkpoint"
        )

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.target_timeout = float(
            resolve(self.target_timeout, self.blackboard, "approach_target_timeout")
        )
        self.goal_timeout = float(
            resolve(self.goal_timeout, self.blackboard, "approach_goal_timeout")
        )
        self.sight_timeout = float(
            resolve(self.sight_timeout, self.blackboard, "sight_timeout")
        )
        self.retries = int(resolve(self.retries, self.blackboard, "nav_goal_retries"))
        self.pose = RobotPoseTracker(self.node)
        self.nav2 = Nav2PoseNavigator(self.node)
        self.people = (
            PeopleTracker(self.node, self.sight_timeout)
            if is_human(self.target_type)
            else None
        )
        self.logger.info(f"{self.name}: Setup complete")

    def initialise(self):
        self.nav2.reset()
        self._target_position = None
        self._resends = 0
        self._start_time = self.node.get_clock().now()
        self.node.get_logger().info(f"{self.name}: approaching the {self.target_type}")

    def terminate(self, new_status):
        """Stop driving when the behaviour stops, whatever stopped it."""
        if new_status != py_trees.common.Status.RUNNING:
            self.nav2.cancel()

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
            if not self.nav2.server_ready():
                if self._elapsed() > self.goal_timeout:
                    self.node.get_logger().info(
                        f"{self.name}: no nav2 action server within "
                        f"{self.goal_timeout:.0f} s, giving up"
                    )
                    return py_trees.common.Status.FAILURE
                self.feedback_message = "waiting for the nav2 action server"
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

        if self._resends >= self.retries:
            self.node.get_logger().info(
                f"{self.name}: nav2 dropped the goal {self._resends} time(s), giving up"
            )
            return py_trees.common.Status.FAILURE
        self._resends += 1
        self.node.get_logger().info(
            f"{self.name}: nav2 dropped the goal, sending it again "
            f"({self._resends}/{self.retries})"
        )
        self._send_goal()
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
            self.blackboard, self.target_type, subject_position, keys=self.keys
        )

    def _send_goal(self):
        # Walking up to a human is stepped, and how close the robot may come to
        # one is an experiment parameter; a route target only needs a sane
        # stride and to be parked near, so it has its own pair of numbers.
        human = is_human(self.target_type)
        go_threshold = (
            self.blackboard.get(self.keys.approach_distance)
            if human
            else constant(self.blackboard, "route_step_distance")
        )
        stop_threshold = (
            self.blackboard.get(self.keys.closeness_threshold)
            if human
            else constant(self.blackboard, "route_stop_distance")
        )
        goal = pose_to_goal(
            self._target_position,
            self.pose.pose,
            stop_threshold=stop_threshold,
            mode=self.mode,
            go_threshold=go_threshold,
        )
        self.nav2.go_to(goal)
        self.node.get_logger().info(
            f"{self.name}: goal for the {self.target_type} at "
            f"x={goal.position.x:.2f} y={goal.position.y:.2f}"
        )

    def _goal_reached(self):
        if self.target_type == CHECKPOINT:
            current = self.blackboard.get(self.keys.current_checkpoint)
            last = self.blackboard.get(self.keys.max_checkpoint)
            if current < last:
                self.blackboard.set(self.keys.current_checkpoint, current + 1)
            self.node.get_logger().info(
                f"{self.name}: checkpoint {current + 1}/{last + 1} reached"
            )
        else:
            self.node.get_logger().info(f"{self.name}: reached the {self.target_type}")
        return py_trees.common.Status.SUCCESS


class CheckSubjectTargetSuccess(py_trees.behaviour.Behaviour):
    """SUCCESS once the human stands within the reached threshold of the target."""

    KEYS = DEFAULT_KEYS

    def __init__(self, name="CheckSubjectTargetSuccess", sight_timeout=None, keys=None):
        super().__init__(name)
        self.sight_timeout = sight_timeout
        self.keys = keys or self.KEYS

        self.blackboard = self.attach_blackboard_client(name=name)
        register_param_keys(self.blackboard, "sight_timeout")
        self.keys.register(
            self.blackboard,
            py_trees.common.Access.READ,
            "target_position",
            "reached_threshold",
        )

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.sight_timeout = float(
            resolve(self.sight_timeout, self.blackboard, "sight_timeout")
        )
        self.people = PeopleTracker(self.node, self.sight_timeout)
        self.logger.info(f"{self.name}: Setup complete")

    def update(self):
        subject_pose = self.people.last_seen_pose
        if subject_pose is None:
            self.feedback_message = "no person seen yet"
            return py_trees.common.Status.RUNNING

        distance = distance_xy(
            subject_pose.position, self.blackboard.get(self.keys.target_position)
        )
        threshold = self.blackboard.get(self.keys.reached_threshold)
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
