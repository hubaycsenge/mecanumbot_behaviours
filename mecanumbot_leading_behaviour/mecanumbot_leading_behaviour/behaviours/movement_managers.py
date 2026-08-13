"""Navigation behaviours, plus the public face of the behaviour library.

The two behaviours that actually drive somewhere live here and both go through
nav2's actions: `Approach` navigates to a single place, `FollowRoute` runs a leg
of route checkpoints as one waypoint goal. The in-place turns live in
`turning.py`, the recovery patrol in `searching.py`, the pacing of the look
backs in `pacing.py`, and the geometry/ROS helpers in `geometry.py` /
`ros_interfaces.py`; they are re-exported at the bottom of this module so
`from ...behaviours.movement_managers import X` keeps working for every tree and
for the `mecanumbot_demo_behaviours` package.
"""

import py_trees

from mecanumbot_leading_behaviour.behaviours.constants import (
    constant,
    register_param_keys,
    resolve,
)
from mecanumbot_leading_behaviour.behaviours.geometry import (
    calculate_facing_orientation,
    distance_xy,
    normalize_angle,
    pose_to_goal,
    quaternion_from_yaw,
    route_poses,
    yaw_from_quaternion,
)
from mecanumbot_leading_behaviour.behaviours.pacing import route_leg
from mecanumbot_leading_behaviour.behaviours.ros_interfaces import (
    BallTracker,
    GOAL_ACTIVE_STATUSES,
    GOAL_FAILED_STATUSES,
    FollowedSubjectTracker,
    Nav2GoalMonitor,
    Nav2PoseNavigator,
    Nav2RouteNavigator,
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
    GlanceBack,
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

    The goal goes through the `NavigateToPose` action rather than the
    `/goal_pose` topic, so the outcome belongs to this goal rather than being
    guessed from a status array, and the drive is cancelled when the behaviour
    stops -- a turn that follows owns `/cmd_vel` straight away instead of
    waiting for nav2 to notice it is finished.
    """

    def __init__(
        self,
        name="Approach",
        target_type=SUBJECT,
        mode="exact",
        target_timeout=None,
        goal_timeout=None,
        sight_timeout=None,
        retries=None,
    ):
        super().__init__(name)
        self.target_type = target_type
        self.mode = mode
        self.target_timeout = target_timeout
        self.goal_timeout = goal_timeout
        self.sight_timeout = sight_timeout
        self.retries = retries

        self.blackboard = self.attach_blackboard_client(name=name)
        register_target_keys(self.blackboard)
        register_param_keys(self.blackboard)
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
            self.blackboard, self.target_type, subject_position
        )

    def _send_goal(self):
        # Walking up to a human is stepped, and how close the robot may come to
        # one is an experiment parameter; a route target only needs a sane
        # stride and to be parked near, so it has its own pair of numbers.
        human = is_human(self.target_type)
        go_threshold = (
            self.blackboard.robot_approach_distance
            if human
            else constant(self.blackboard, "route_step_distance")
        )
        stop_threshold = (
            self.blackboard.robot_closeness_threshold
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


class FollowRoute(py_trees.behaviour.Behaviour):
    """Lead one leg of the route: several checkpoints in a single nav2 goal.

    Leading used to be one nav2 goal per checkpoint, which meant the robot came
    to a full stop, waited for the goal to be reported done, turned, and set off
    again at every one of them. A leg goes through `NavigateThroughPoses`
    instead: nav2 plans through all the checkpoints of the leg at once and
    drives them without stopping, and because `route_poses()` orients every
    waypoint along the route the robot is already pointing the right way when it
    passes one.

    How long a leg is comes from `pacing.route_leg()` -- as far as the look-back
    pacing allows, so the robot never plans past a check-in it is about to owe,
    and never further than `route_lookahead` checkpoints.

    The leg is cut short, with SUCCESS, when the human stops following: they
    have been out of sight for longer than `visibility_time_threshold`, or
    further away than `Dog_following_max_threshold`, for `check_in_grace`
    seconds together. SUCCESS rather than FAILURE because stopping the drive is
    not a failure of it -- the check-in that comes next in the cycle is due by
    exactly the same measurements, and it is the look back that decides whether
    this is a human who fell behind or a human who is gone.

    Checkpoints count as reached by distance (`checkpoint_reached_distance`)
    rather than by nav2's own arrival, so `Dog_current_checkpoint` moves along
    while the robot drives through and does not depend on which nav2 behaviour
    tree is loaded.

    A waypoint run asks more of the route than a single goal does.
    `ComputePathThroughPoses` plans robot -> first checkpoint, then *from* that
    checkpoint to the next, so every checkpoint has to be somewhere the planner
    will both aim at and set off from -- and a checkpoint pressed against a wall
    is neither, once the costmap has inflated it. `Approach` never asked that:
    it aims `robot_closeness_threshold` short of its target and stops there.

    So a leg nav2 gives up on is not retried as-is. It falls back to exactly
    that older move -- one `NavigateToPose` goal stopping short of the next
    checkpoint -- and the leg carries on checkpoint by checkpoint from there.
    Leading is then no smoother than it used to be, but it never stops dead over
    one awkwardly placed checkpoint.
    """

    def __init__(
        self,
        name="FollowRoute",
        goal_timeout=None,
        retries=None,
        grace=None,
        sight_timeout=None,
    ):
        super().__init__(name)
        self.goal_timeout = goal_timeout
        self.retries = retries
        self.grace = grace
        self.sight_timeout = sight_timeout

        self.blackboard = self.attach_blackboard_client(name=name)
        register_param_keys(self.blackboard)
        for key in (
            "Dog_checkpoints",
            "Dog_max_checkpoint",
            "target_position",
            "Dog_following_max_threshold",
            "visibility_time_threshold",
        ):
            self.blackboard.register_key(key, access=py_trees.common.Access.READ)
        self.blackboard.register_key(
            "Dog_current_checkpoint", access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            "check_in_checkpoints_since", access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.goal_timeout = float(
            resolve(self.goal_timeout, self.blackboard, "approach_goal_timeout")
        )
        self.retries = int(resolve(self.retries, self.blackboard, "nav_goal_retries"))
        self.grace = float(resolve(self.grace, self.blackboard, "check_in_grace"))
        self.sight_timeout = float(
            resolve(self.sight_timeout, self.blackboard, "sight_timeout")
        )
        self.pose = RobotPoseTracker(self.node)
        self.subject = FollowedSubjectTracker(self.node, self.sight_timeout)
        self.route = Nav2RouteNavigator(self.node)
        self.single = Nav2PoseNavigator(self.node)
        self.logger.info(f"{self.name}: Setup complete")

    def initialise(self):
        self.route.reset()
        self.single.reset()
        self.nav2 = self.route
        self._leg = None
        self._step = 0
        self._resends = 0
        self._lagging_since = None
        self._start_time = self.node.get_clock().now()

    def terminate(self, new_status):
        if new_status != py_trees.common.Status.RUNNING:
            self.route.cancel()
            self.single.cancel()

    def update(self):
        if self.pose.position is None:
            self.feedback_message = "waiting for AMCL pose"
            return py_trees.common.Status.RUNNING

        if self._leg is None:
            if not self.route.server_ready() or not self.single.server_ready():
                if self._elapsed() > self.goal_timeout:
                    self.node.get_logger().info(
                        f"{self.name}: no nav2 waypoint action server within "
                        f"{self.goal_timeout:.0f} s, giving up"
                    )
                    return py_trees.common.Status.FAILURE
                self.feedback_message = "waiting for the nav2 waypoint action server"
                return py_trees.common.Status.RUNNING
            return self._begin_leg()

        self._mark_checkpoints_passed()
        if self._step >= len(self._leg):
            return self._leg_done("leg driven")

        lagging = self._lagging_reason()
        if lagging is not None:
            return self._leg_done(f"stopping the leg early: {lagging}")

        status = self.nav2.status()
        if status is None:
            if self.nav2.seconds_since_send() > self.goal_timeout:
                self.node.get_logger().info(
                    f"{self.name}: nav2 never reported our route within "
                    f"{self.goal_timeout:.0f} s, giving up"
                )
                return py_trees.common.Status.FAILURE
            self.feedback_message = "waiting for nav2 to accept the route"
            return py_trees.common.Status.RUNNING

        if status == STATUS_SUCCEEDED:
            # Take nav2's word for what it drove, even where the robot ended up
            # further from the checkpoint than our own measure counts as
            # reached: a goal it says it finished must not be sent again, or the
            # same instant success comes straight back.
            if self.nav2 is self.route:
                while self._step < len(self._leg):
                    self._advance()
                return self._leg_done("nav2 finished the route")

            # One checkpoint of a leg being walked a checkpoint at a time.
            self._advance()
            if self._step >= len(self._leg):
                return self._leg_done("leg driven one checkpoint at a time")
            return self._send_single()
        if status in GOAL_ACTIVE_STATUSES:
            self.feedback_message = self._progress()
            return py_trees.common.Status.RUNNING

        # Nav2 gave up on the goal. Sending the same one again only buys the
        # same answer, so the first drop steps down to the single-goal move that
        # stops short of the checkpoint; only that one is worth retrying.
        if self.nav2 is self.route:
            self.node.get_logger().warn(
                f"{self.name}: nav2 could not drive the leg as waypoints -- check "
                "whether the checkpoints themselves are clear of the costmap "
                "inflation; leading to them one at a time instead"
            )
            return self._send_single()

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
        return self._send_single()

    # --- internals ----------------------------------------------------------

    def _elapsed(self):
        return (self.node.get_clock().now() - self._start_time).nanoseconds / 1e9

    def _begin_leg(self):
        """Work out how far this leg goes, and send it."""
        if not self.blackboard.Dog_checkpoints:
            self.node.get_logger().warn(f"{self.name}: no route checkpoints to lead to")
            return py_trees.common.Status.FAILURE

        self._leg = route_leg(
            self.blackboard.Dog_current_checkpoint,
            self.blackboard.Dog_max_checkpoint,
            self.blackboard.check_in_checkpoints_since,
            constant(self.blackboard, "check_in_every_checkpoints"),
            constant(self.blackboard, "route_lookahead"),
        )
        self._step = 0

        # Checkpoints the robot is already standing on are not driven past, so
        # they are dropped from the goal and left out of the look-back pacing:
        # waiting at the end of the route is not covering ground, and counting
        # it as ground covered would have the robot glancing back on the spot.
        self._mark_checkpoints_passed(count=False)
        if self._step >= len(self._leg):
            return self._leg_done("already at the end of this leg")
        return self._send(self._leg[self._step :])

    def _send(self, indices):
        self.nav2 = self.route
        self.route.follow(
            route_poses(
                self.blackboard.Dog_checkpoints,
                indices,
                look_beyond=self.blackboard.target_position,
            )
        )
        self.node.get_logger().info(
            f"{self.name}: leading through checkpoint(s) {indices} of "
            f"0..{self.blackboard.Dog_max_checkpoint}"
        )
        return py_trees.common.Status.RUNNING

    def _send_single(self):
        """Fall back to `Approach`'s move: one goal, stopping short of the checkpoint."""
        index = self._leg[min(self._step, len(self._leg) - 1)]
        checkpoint = self.blackboard.Dog_checkpoints[index]
        self.nav2 = self.single
        self.single.go_to(
            pose_to_goal(
                checkpoint,
                self.pose.pose,
                stop_threshold=constant(self.blackboard, "route_stop_distance"),
                go_threshold=constant(self.blackboard, "route_step_distance"),
            )
        )
        self.node.get_logger().info(
            f"{self.name}: leading to checkpoint {index} on its own"
        )
        return py_trees.common.Status.RUNNING

    def _progress(self):
        index = self._leg[min(self._step, len(self._leg) - 1)]
        if self.nav2 is not self.route:
            return f"leading to checkpoint {index}"
        remaining = self.route.poses_remaining
        return f"leading to checkpoint {index}" + (
            "" if remaining is None else f", {remaining} waypoint(s) left"
        )

    def _mark_checkpoints_passed(self, count=True):
        """Walk the route index on for every checkpoint of the leg driven past."""
        checkpoints = self.blackboard.Dog_checkpoints
        reached = constant(self.blackboard, "checkpoint_reached_distance")
        while self._step < len(self._leg):
            index = self._leg[self._step]
            if distance_xy(self.pose.position, checkpoints[index]) > reached:
                return
            self._advance(count=count)

    def _advance(self, count=True):
        """Put one checkpoint of the leg behind us."""
        index = self._leg[self._step]
        self._step += 1
        if count:
            self.blackboard.check_in_checkpoints_since += 1
        self.blackboard.Dog_current_checkpoint = min(
            index + 1, self.blackboard.Dog_max_checkpoint
        )
        self.node.get_logger().info(
            f"{self.name}: checkpoint {index} "
            f"{'reached' if count else 'already behind us'}, next is "
            f"{self.blackboard.Dog_current_checkpoint}"
        )

    def _lagging_reason(self):
        """Why the human is no longer following, once they have been for `grace`."""
        reason = None
        age = self.subject.age
        if self.subject.position is None or age is None:
            reason = "the human has not been seen yet"
        else:
            distance = distance_xy(self.pose.position, self.subject.position)
            allowed = self.blackboard.Dog_following_max_threshold
            if distance > allowed:
                reason = (
                    f"the human is {distance:.2f} m behind, "
                    f"more than the {allowed:.2f} m allowed"
                )

        if reason is None:
            self._lagging_since = None
            return None

        # A human who steps behind a pillar for a moment is still following, so
        # the reason has to hold for a while before it stops the leg.
        now = self.node.get_clock().now()
        if self._lagging_since is None:
            self._lagging_since = now
            return None
        if (now - self._lagging_since).nanoseconds / 1e9 < self.grace:
            self.feedback_message = f"{reason} (waiting {self.grace:.0f} s)"
            return None
        return reason

    def _leg_done(self, reason):
        self.nav2.cancel()
        self.node.get_logger().info(f"{self.name}: {reason}")
        self.feedback_message = reason
        return py_trees.common.Status.SUCCESS


class CheckSubjectTargetSuccess(py_trees.behaviour.Behaviour):
    """SUCCESS once the human stands within `target_reached_threshold` of the target."""

    def __init__(self, name="CheckSubjectTargetSuccess", sight_timeout=None):
        super().__init__(name)
        self.sight_timeout = sight_timeout

        self.blackboard = self.attach_blackboard_client(name=name)
        register_param_keys(self.blackboard, "sight_timeout")
        self.blackboard.register_key(
            "target_position", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            "target_reached_threshold", access=py_trees.common.Access.READ
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
    "FollowRoute",
    # turning behaviours
    "FindPeople",
    "GlanceBack",
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
