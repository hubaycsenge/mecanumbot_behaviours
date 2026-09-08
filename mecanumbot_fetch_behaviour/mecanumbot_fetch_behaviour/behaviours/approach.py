"""
Driving up to the ball, deciding whether it can be had, and closing on it.

Approaching a *place* is `mecanumbot_movement_behaviours.Approach`, and this
package uses it unchanged through `keys.FetchApproach` wherever a fixed place is
the target. Approaching the *ball* is different in one way that matters, and
that difference is `ApproachBall`: the target moves. Partly because the estimate
refines as the robot gets closer -- a ball ranged from its apparent diameter at
four metres is placed to within a good fraction of a metre, at one metre nearly
exactly -- and partly because in a fetch game the ball genuinely moves. It rolls,
and somebody picks it up.

Everything still goes through nav2's action, so the obstacle avoidance and the
costmaps are untouched: the same division of labour as every other tree here.
"""

import py_trees

from mecanumbot_movement_behaviours.defaults import (
    register_param_keys as register_movement_keys,
)
from mecanumbot_movement_behaviours.geometry import distance_xy, pose_to_goal
from mecanumbot_movement_behaviours.ros_interfaces import (
    GOAL_ACTIVE_STATUSES,
    STATUS_SUCCEEDED,
    BallTracker,
    Nav2PoseNavigator,
    RobotPoseTracker,
)

from mecanumbot_fetch_behaviour.behaviours.ros_interfaces import (
    BallDetectionTracker,
    GripperCommander,
)
from mecanumbot_fetch_behaviour.defaults import constant, register_param_keys

# How far the estimate has to move before the goal is worth re-sending [m].
# Below the approach's own stop tolerance, so a refinement that matters is acted
# on and detector jitter is not.
RETARGET_DISTANCE = 0.2


class ApproachBall(py_trees.behaviour.Behaviour):
    """
    Drive to the ball, re-aiming as the estimate refines and the ball moves.

    SUCCESS once the robot is within grasping range, or once nav2 reports the
    goal reached -- whichever comes first, because parking exactly is nav2's
    business and being close enough to grip is this behaviour's.

    FAILURE when the ball has not been seen for `fetch_approach_timeout` or when
    nav2 gives up. Losing sight of it partway is not immediately fatal: the last
    position is still the best guess and the robot keeps driving to it. That
    matters more here than in a seeking task, because the last few metres are
    exactly where the robot's own body starts to occlude the thing it is driving
    at -- the camera is at shin height and the ball is on the floor.
    """

    def __init__(self, name="ApproachBall", timeout=None):
        super().__init__(name)
        self.timeout = timeout
        self.blackboard = self.attach_blackboard_client(name=name)
        register_param_keys(self.blackboard)
        # `nav_goal_retries` is the movement library's, not this package's.
        register_movement_keys(self.blackboard)
        for key in ("fetch_ball_position", "fetch_ball_score", "fetch_ball_height"):
            self.blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        """Build the pose tracker, the navigator and the ball tracker."""
        self.node = kwargs["node"]
        self.timeout = float(
            self.timeout
            if self.timeout is not None
            else constant(self.blackboard, "fetch_approach_timeout")
        )
        self.stop_distance = float(constant(self.blackboard, "fetch_approach_stop"))
        self.grasp_distance = float(constant(self.blackboard, "fetch_grasp_distance"))
        self.threshold = float(constant(self.blackboard, "fetch_detection_threshold"))
        self.pose = RobotPoseTracker(self.node)
        self.nav2 = Nav2PoseNavigator(self.node)
        self.balls = BallDetectionTracker(
            self.node,
            timeout=constant(self.blackboard, "fetch_detection_timeout"),
            class_id=str(constant(self.blackboard, "fetch_ball_class")) or None,
        )
        self.logger.info(f"{self.name}: Setup complete")
        return True

    def initialise(self):
        """Reset the navigator and start the clock."""
        self.nav2.reset()
        self._goal_position = None
        self._start = self.node.get_clock().now()
        self._resends = 0

    def terminate(self, new_status):
        """Stop driving when the behaviour stops, whatever stopped it."""
        if new_status != py_trees.common.Status.RUNNING:
            self.nav2.cancel()

    def update(self):
        """Drive to the ball, re-aiming as its estimated position moves."""
        if self.pose.pose is None:
            self.feedback_message = "waiting for AMCL pose"
            return py_trees.common.Status.RUNNING

        target = self._current_target()
        if target is None:
            self.node.get_logger().info(f"{self.name}: nothing to approach")
            return py_trees.common.Status.FAILURE

        remaining = distance_xy(self.pose.pose.position, target)
        if remaining <= self.grasp_distance:
            self.node.get_logger().info(
                f"{self.name}: within grasping range ({remaining:.2f} m)"
            )
            return py_trees.common.Status.SUCCESS

        if self._elapsed() > self.timeout:
            self.node.get_logger().info(
                f"{self.name}: still {remaining:.2f} m away after "
                f"{self.timeout:.0f} s, giving up"
            )
            return py_trees.common.Status.FAILURE

        if self._needs_new_goal(target):
            if not self.nav2.server_ready():
                self.feedback_message = "waiting for the nav2 action server"
                return py_trees.common.Status.RUNNING
            self._send_goal(target)
            return py_trees.common.Status.RUNNING

        status = self.nav2.status()
        if status == STATUS_SUCCEEDED:
            self.node.get_logger().info(f"{self.name}: nav2 says we are there")
            return py_trees.common.Status.SUCCESS
        if status is None or status in GOAL_ACTIVE_STATUSES:
            self.feedback_message = f"{remaining:.2f} m to go"
            return py_trees.common.Status.RUNNING

        retries = int(constant(self.blackboard, "nav_goal_retries"))
        if self._resends >= retries:
            self.node.get_logger().info(
                f"{self.name}: nav2 dropped the goal {self._resends} time(s), "
                "giving up"
            )
            return py_trees.common.Status.FAILURE
        self._resends += 1
        self._send_goal(target)
        return py_trees.common.Status.RUNNING

    # --- internals ------------------------------------------------------------

    def _elapsed(self):
        return (self.node.get_clock().now() - self._start).nanoseconds / 1e9

    def _current_target(self):
        """
        Return the freshest estimate of where the ball is.

        A live detection wins over the stored one, and the stored one survives
        the ball going briefly out of view -- which happens constantly as the
        robot turns towards it and as its own bumper starts to occlude it.
        """
        self.balls.look_from(self.pose.pose.position)
        if self.balls.visible(self.threshold):
            hypothesis = self.balls.hypothesis
            self.blackboard.fetch_ball_position = hypothesis.position
            self.blackboard.fetch_ball_score = float(hypothesis.score)
            self.blackboard.fetch_ball_height = float(hypothesis.position.z)
            return hypothesis.position
        return self.blackboard.fetch_ball_position

    def _needs_new_goal(self, target):
        if not self.nav2.goal_sent or self._goal_position is None:
            return True
        return distance_xy(self._goal_position, target) > RETARGET_DISTANCE

    def _send_goal(self, target):
        goal = pose_to_goal(
            target,
            self.pose.pose,
            stop_threshold=self.stop_distance,
            mode="exact",
        )
        self.nav2.go_to(goal)
        self._goal_position = target
        self.feedback_message = (
            f"goal at x={goal.position.x:.2f} y={goal.position.y:.2f}"
        )


class CheckBallReachable(py_trees.behaviour.Behaviour):
    """
    SUCCESS while the ball is in the height band the grabbers can close on.

    The grabbers are a horizontal pincer whose shafts sit at about z = 0.034
    with a 0.116 m clear gap, and there is no lift. So a ball on a table, on a
    chair, or in somebody's hand is not a ball the robot can have, however
    close it drives -- and the difference between that and "the ball is not
    here" is a number only the located ball's `z` supplies. This is the same
    argument `mecanumbot_seek` makes about the point cloud's height, one sensor
    cheaper: the ball's apparent size gives a range, the range gives a ray
    length, and the ray gives a height.

    FAILURE ends the episode rather than starting an alert. That is a real
    difference from the seek tree, and it is deliberate: seek's whole point is
    that an object found and not obtained is worth *telling somebody about*,
    because the robot was sent to find that specific thing. A ball out of reach
    in a fetch game is not news -- the person can see it, they are in the room
    playing -- so the honest response is to go and look for another one.
    """

    def __init__(self, name="CheckBallReachable"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        register_param_keys(self.blackboard)
        self.blackboard.register_key(
            key="fetch_ball_height", access=py_trees.common.Access.READ
        )

    def setup(self, **kwargs):
        """Read the height band the grabbers can close on."""
        self.node = kwargs["node"]
        self.height_min = float(constant(self.blackboard, "fetch_grasp_height_min"))
        self.height_max = float(constant(self.blackboard, "fetch_grasp_height_max"))
        self.logger.info(f"{self.name}: Setup complete")
        return True

    def update(self):
        """Compare the ball's height against what the pincer can reach."""
        height = self.blackboard.fetch_ball_height
        if height is None:
            self.feedback_message = "no height for the ball"
            return py_trees.common.Status.FAILURE
        if self.height_min <= height <= self.height_max:
            self.feedback_message = f"{height * 100:.0f} cm up, within reach"
            return py_trees.common.Status.SUCCESS
        self.node.get_logger().info(
            f"{self.name}: the ball is {height * 100:.0f} cm up, outside the "
            f"{self.height_min * 100:.0f}-{self.height_max * 100:.0f} cm the "
            "grabbers can close on -- it is not on the floor"
        )
        return py_trees.common.Status.FAILURE


class GraspBall(py_trees.behaviour.Behaviour):
    """
    Close the grabbers on the ball and check whether anything is held.

    SUCCESS when `/mecanumbot/has_object` says something is between the shafts.
    FAILURE when nothing is after `fetch_grasp_confirm_timeout`, and the
    grabbers are re-opened on the way out so the robot does not drive off with
    them shut. A miss is cheap and common -- the ball rolls off the shafts as
    they close -- which is why the tree wraps the approach and the grab in a
    `Retry` rather than treating one miss as a failed episode.

    The neck is held at `fetch_head_approach` rather than sent to a neutral
    pose, which is why this uses `GripperCommander` and not the movement
    library's `AccessoryCommander`: lifting the head mid-grasp loses sight of
    the ball at exactly the wrong moment.
    """

    def __init__(self, name="GraspBall"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        register_param_keys(self.blackboard)
        self.blackboard.register_key(
            key="fetch_grasped", access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs):
        """Build the gripper commander and the held-object tracker."""
        self.node = kwargs["node"]
        self.settle = float(constant(self.blackboard, "fetch_grasp_settle"))
        self.confirm_timeout = float(
            constant(self.blackboard, "fetch_grasp_confirm_timeout")
        )
        self.neck = float(constant(self.blackboard, "fetch_head_approach"))
        self.open_left = float(constant(self.blackboard, "fetch_gripper_open_left"))
        self.open_right = float(constant(self.blackboard, "fetch_gripper_open_right"))
        self.closed_left = float(constant(self.blackboard, "fetch_gripper_closed_left"))
        self.closed_right = float(
            constant(self.blackboard, "fetch_gripper_closed_right")
        )
        self.gripper = GripperCommander(self.node)
        self.held = BallTracker(self.node)
        self.logger.info(f"{self.name}: Setup complete")
        return True

    def initialise(self):
        """Open the grabbers and start the close."""
        self._start = self.node.get_clock().now()
        self._closed = False
        self.gripper.send(self.neck, self.open_left, self.open_right)

    def terminate(self, new_status):
        """Re-open the grabbers unless something is actually held."""
        if new_status == py_trees.common.Status.RUNNING:
            return
        if not self.blackboard.fetch_grasped:
            self.gripper.send(self.neck, self.open_left, self.open_right)

    def update(self):
        """Close, wait for the shafts to settle, then read the object sensor."""
        elapsed = (self.node.get_clock().now() - self._start).nanoseconds / 1e9

        if not self._closed:
            self.gripper.send(self.neck, self.closed_left, self.closed_right)
            self._closed = True
            self.feedback_message = "closing"
            return py_trees.common.Status.RUNNING

        if elapsed < self.settle:
            self.feedback_message = f"settling ({elapsed:.1f}/{self.settle:.1f} s)"
            return py_trees.common.Status.RUNNING

        if self.held.has_ball:
            self.blackboard.fetch_grasped = True
            self.node.get_logger().info(f"{self.name}: got the ball")
            return py_trees.common.Status.SUCCESS

        if elapsed > self.settle + self.confirm_timeout:
            self.node.get_logger().info(
                f"{self.name}: nothing held after {elapsed:.1f} s, re-opening"
            )
            return py_trees.common.Status.FAILURE

        self.feedback_message = "waiting for the object sensor"
        return py_trees.common.Status.RUNNING
