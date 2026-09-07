"""
Driving up to the object once it has been seen, and closing on it.

Approaching a *place* is `mecanumbot_movement_behaviours.Approach`, and this
package uses it unchanged through `keys.SeekApproach` for the drive to the
remembered location. Approaching the *object* is different in one way that
matters, and that difference is this module: the target moves.

Not because the object moves -- it is a mug on a floor -- but because the
estimate of where it is refines as the robot gets closer. A 3D detection from
five metres away places it to within a good fraction of a metre; from one metre
it is nearly exact. A single goal sent on the first sighting parks the robot
next to where the object looked like it was, which is not close enough to grip
anything. So the goal is re-sent whenever the estimate moves more than a
threshold, and the threshold exists so the robot is not re-planning at the
detector's frame rate.

Everything still goes through nav2's action, so the obstacle avoidance and the
costmaps are untouched -- the same division of labour as every other tree here.
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

from mecanumbot_seek.behaviours.ros_interfaces import (
    GripperCommander,
    ObjectDetectionTracker,
)
from mecanumbot_seek.defaults import constant, register_param_keys

# How far the estimate has to move before the goal is worth re-sending [m].
# Below the approach's own stop tolerance, so a refinement that matters is acted
# on and detector jitter is not.
RETARGET_DISTANCE = 0.25


class ApproachObject(py_trees.behaviour.Behaviour):
    """
    Drive to the sighted object, re-aiming as the estimate refines.

    SUCCESS once nav2 reports the goal reached, or once the robot is within the
    grasp distance -- whichever comes first, because parking exactly is nav2's
    business and being close enough to grip is this behaviour's.

    FAILURE when the object has not been seen for `seek_approach_timeout`, or
    when nav2 gives up. Losing sight of it partway is not immediately fatal: the
    last position is still the best guess and the robot keeps driving to it,
    which is the sustained-arousal property doing something useful.
    """

    def __init__(self, name="ApproachObject", timeout=None):
        super().__init__(name)
        self.timeout = timeout
        self.blackboard = self.attach_blackboard_client(name=name)
        register_param_keys(self.blackboard)
        # `nav_goal_retries` is the movement library's, not this package's.
        register_movement_keys(self.blackboard)
        self.blackboard.register_key(
            key="seek_drive", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="seek_object_class", access=py_trees.common.Access.READ
        )
        for key in ("seek_target_position", "seek_sighted_position"):
            self.blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        """Build the pose tracker, the navigator and the detector."""
        self.node = kwargs["node"]
        self.timeout = float(
            self.timeout
            if self.timeout is not None
            else constant(self.blackboard, "seek_approach_timeout")
        )
        self.stop_distance = float(constant(self.blackboard, "seek_approach_stop"))
        self.grasp_distance = float(constant(self.blackboard, "seek_grasp_distance"))
        self.pose = RobotPoseTracker(self.node)
        self.nav2 = Nav2PoseNavigator(self.node)
        self.detections = ObjectDetectionTracker(
            self.node, timeout=constant(self.blackboard, "seek_detection_timeout")
        )
        self.logger.info(f"{self.name}: Setup complete")
        return True

    def initialise(self):
        """Reset the navigator and start the clock."""
        self.nav2.reset()
        self._goal_position = None
        self._start = self.node.get_clock().now()
        self._resends = 0
        self.node.get_logger().info(
            f"{self.name}: approaching '{self.blackboard.seek_object_class}'"
        )

    def terminate(self, new_status):
        """Stop driving when the behaviour stops, whatever stopped it."""
        if new_status != py_trees.common.Status.RUNNING:
            self.nav2.cancel()

    def update(self):
        """Drive to the object, re-aiming as its estimated position refines."""
        if self.pose.pose is None:
            self.feedback_message = "waiting for AMCL pose"
            return py_trees.common.Status.RUNNING

        target = self._current_target()
        if target is None:
            self.node.get_logger().info(f"{self.name}: nothing to approach")
            return py_trees.common.Status.FAILURE

        reached = distance_xy(self.pose.pose.position, target)
        if reached <= self.grasp_distance:
            self.node.get_logger().info(
                f"{self.name}: within grasping range ({reached:.2f} m)"
            )
            return py_trees.common.Status.SUCCESS

        if self._elapsed() > self.timeout:
            self.node.get_logger().info(
                f"{self.name}: still {reached:.2f} m away after "
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
            self.feedback_message = f"{reached:.2f} m to go"
            return py_trees.common.Status.RUNNING

        retries = int(constant(self.blackboard, "nav_goal_retries"))
        if self._resends >= retries:
            self.node.get_logger().info(
                f"{self.name}: nav2 dropped the goal {self._resends} time(s), "
                "giving up"
            )
            return py_trees.common.Status.FAILURE
        self._resends += 1
        self.node.get_logger().info(
            f"{self.name}: nav2 dropped the goal, sending it again "
            f"({self._resends}/{retries})"
        )
        self._send_goal(target)
        return py_trees.common.Status.RUNNING

    # --- internals ------------------------------------------------------------

    def _elapsed(self):
        return (self.node.get_clock().now() - self._start).nanoseconds / 1e9

    def _current_target(self):
        """
        Return the freshest estimate of where the object is.

        A live detection wins over the stored one, and the stored one survives
        the object going briefly out of view -- which happens constantly as the
        robot turns towards it.
        """
        drive = self.blackboard.seek_drive
        self.detections.class_id = self.blackboard.seek_object_class or None
        if self.detections.visible(drive.detection_threshold()):
            position = self.detections.hypothesis.position
            self.blackboard.seek_sighted_position = position
            self.blackboard.seek_target_position = position
            return position
        return self.blackboard.seek_target_position

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
        self.node.get_logger().info(
            f"{self.name}: goal at x={goal.position.x:.2f} y={goal.position.y:.2f}"
        )


class GraspObject(py_trees.behaviour.Behaviour):
    """
    Close the grabbers on the object and check whether anything is held.

    The gripper is a horizontal pincer with no lift, so a successful grasp is
    the object between the shafts and `/mecanumbot/has_object` saying so -- a
    gripped object is carried along the floor, not picked up.

    FAILURE when nothing is held after `seek_grasp_confirm_timeout`, and the
    grabbers are re-opened on the way out so the robot does not drive off with
    them shut. Failing here is a real outcome and not an error: the tree's
    caller treats "found but could not grip" as a completed episode, because the
    robot did find the thing.

    The neck is held where it is rather than sent to a neutral pose, which is
    why this uses `GripperCommander` and not the movement library's
    `AccessoryCommander` -- moving the head mid-grasp loses sight of the object
    at exactly the wrong moment.
    """

    def __init__(self, name="GraspObject"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        register_param_keys(self.blackboard)
        # `neck_level_pos` is the movement library's: the grasp holds the head
        # where that package's commander would put it, without moving it.
        register_movement_keys(self.blackboard)
        self.blackboard.register_key(
            key="seek_drive", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="seek_object_class", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="seek_grasped", access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs):
        """Build the gripper commander and the held-object tracker."""
        self.node = kwargs["node"]
        self.settle = float(constant(self.blackboard, "seek_grasp_settle"))
        self.confirm_timeout = float(
            constant(self.blackboard, "seek_grasp_confirm_timeout")
        )
        self.neck = float(constant(self.blackboard, "neck_level_pos"))
        self.open_left = float(constant(self.blackboard, "seek_gripper_open_left"))
        self.open_right = float(constant(self.blackboard, "seek_gripper_open_right"))
        self.closed_left = float(constant(self.blackboard, "seek_gripper_closed_left"))
        self.closed_right = float(
            constant(self.blackboard, "seek_gripper_closed_right")
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
        self.node.get_logger().info(
            f"{self.name}: closing on '{self.blackboard.seek_object_class}'"
        )

    def terminate(self, new_status):
        """Re-open the grabbers unless something is actually held."""
        if new_status == py_trees.common.Status.RUNNING:
            return
        if not self.blackboard.seek_grasped:
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
            self.blackboard.seek_grasped = True
            # Consummation. SEEKING is appetitive and switches off on receipt --
            # a robot still seeking with the thing in its grabbers is the model
            # being wrong rather than the robot being keen.
            self.blackboard.seek_drive.consummate()
            self.node.get_logger().info(
                f"{self.name}: got '{self.blackboard.seek_object_class}'"
            )
            return py_trees.common.Status.SUCCESS

        if elapsed > self.settle + self.confirm_timeout:
            self.node.get_logger().info(
                f"{self.name}: nothing held after {elapsed:.1f} s, re-opening"
            )
            return py_trees.common.Status.FAILURE

        self.feedback_message = "waiting for the object sensor"
        return py_trees.common.Status.RUNNING
