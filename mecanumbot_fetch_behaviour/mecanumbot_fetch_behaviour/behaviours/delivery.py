"""
Taking the ball to somebody and giving it to them.

Four behaviours, and between them they are the half of fetch that makes it a
game rather than a retrieval task. The robot has the ball; what it does next is
addressed to a person.

`SomeoneToGiveTo` picks the audience -- the **first** person the robot can see,
which is what "brings it to somebody" means when nobody has been nominated. It
records where they are for the record; the turns and the drive each take the
person from their own live detection, because a person the robot is walking
towards moves and a position frozen when they were first noticed would have the
robot delivering to where they used to stand.

`ReleaseBall` opens the grabbers. `BackAway` then puts a little distance between
the robot and the ball, and it is the step that makes the release read as
*giving* rather than as dropping: the robot lets go, moves off, and leaves the
ball to the person. Backing away is a nav2 goal rather than a `/cmd_vel` reverse
on purpose -- this repository's trees drive the wheels directly only for
in-place rotation, because that is the one movement where bypassing the local
costmap is safe and the handedness is the point. Reversing blind at a person's
feet is neither.

There is no showing gesture here, unlike `mecanumbot_seek`'s alert. Seek
alternates its orientation between an object and a person because it is telling
them about something it cannot reach, and the gesture is the message. Here the
robot is holding the ball and standing in front of them: the message is the
object, delivered.
"""

import math

import py_trees
from geometry_msgs.msg import Pose

from mecanumbot_movement_behaviours.defaults import (
    register_param_keys as register_movement_keys,
)
from mecanumbot_movement_behaviours.geometry import (
    distance_xy,
    quaternion_from_yaw,
    yaw_from_quaternion,
)
from mecanumbot_movement_behaviours.ros_interfaces import (
    GOAL_ACTIVE_STATUSES,
    STATUS_SUCCEEDED,
    HEAD_SEEK,
    AccessoryCommander,
    Nav2PoseNavigator,
    PeopleTracker,
    RobotPoseTracker,
)

from mecanumbot_fetch_behaviour.behaviours.ros_interfaces import GripperCommander
from mecanumbot_fetch_behaviour.defaults import constant, register_param_keys


class SomeoneToGiveTo(py_trees.behaviour.Behaviour):
    """
    SUCCESS while somebody is visible to give the ball to, and records where.

    The condition half of finding an audience; the scan that runs when it fails
    is the movement library's `FindPeople`, bound to this package's key spelling
    in `keys.py`.

    "The first person around" is taken literally: whoever `people_fusion` is
    reporting, not the nearest and not the most confidently tracked. In a fetch
    game the person who is there is the person you give it to, and choosing
    between them would be inventing a social preference the robot has no basis
    for.
    """

    def __init__(self, name="SomeoneToGiveTo"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        register_param_keys(self.blackboard)
        register_movement_keys(self.blackboard)
        self.blackboard.register_key(
            key="fetch_person_position", access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs):
        """Subscribe to the fused people detections."""
        self.node = kwargs["node"]
        self.sight_timeout = float(constant(self.blackboard, "sight_timeout"))
        self.people = PeopleTracker(self.node, self.sight_timeout)
        self.logger.info(f"{self.name}: Setup complete")
        return True

    def update(self):
        """Succeed as soon as there is somebody to give the ball to."""
        if not self.people.has_fresh_detection():
            self.feedback_message = "nobody here"
            return py_trees.common.Status.FAILURE
        position = self.people.last_seen_pose.position
        self.blackboard.fetch_person_position = position
        self.node.get_logger().info(
            f"{self.name}: somebody at x={position.x:.2f} y={position.y:.2f}"
        )
        return py_trees.common.Status.SUCCESS


class ReleaseBall(py_trees.behaviour.Behaviour):
    """
    Open the grabbers and let the ball go, with the head up.

    Always SUCCESS: once the robot has decided to hand the ball over there is
    no failure mode worth propagating -- the grabbers open or the hardware is
    broken, and a tree that refuses to let go of a ball because it cannot
    confirm the release is worse than one that opens and moves on.

    The head is lifted first, and that is the social half of the act. Through
    the whole approach and grab the neck has been down watching the floor;
    coming up as the ball is released is what turns the robot from a machine
    fetching an object into one addressing a person. It also puts the person
    back in the camera's frame, which the delivery's own logging wants.
    """

    def __init__(self, name="ReleaseBall"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        register_param_keys(self.blackboard)
        register_movement_keys(self.blackboard)
        for key in ("fetch_grasped", "fetch_delivered"):
            self.blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        """Build the gripper and accessory commanders."""
        self.node = kwargs["node"]
        self.open_left = float(constant(self.blackboard, "fetch_gripper_open_left"))
        self.open_right = float(constant(self.blackboard, "fetch_gripper_open_right"))
        self.neck = float(constant(self.blackboard, "neck_seek_pos"))
        self.gripper = GripperCommander(self.node)
        self.accessories = AccessoryCommander(self.node)
        self.logger.info(f"{self.name}: Setup complete")
        return True

    def update(self):
        """Look up, open the grabbers, and record the handover."""
        # From here on every head command, the library's included, carries the
        # open grippers again.
        self.gripper.keep_grippers(self.open_left, self.open_right)
        self.accessories.look(HEAD_SEEK)
        self.gripper.send(self.neck, self.open_left, self.open_right)
        self.blackboard.fetch_grasped = False
        self.blackboard.fetch_delivered = True
        self.node.get_logger().info(f"{self.name}: ball released")
        return py_trees.common.Status.SUCCESS


class BackAway(py_trees.behaviour.Behaviour):
    """
    Put a little distance between the robot and the ball it has just given up.

    SUCCESS once the robot has moved `fetch_withdraw_distance` from where it
    released, or once nav2 reports the goal reached. FAILURE only on the
    timeout, and the tree treats that as harmless -- the ball has already
    changed hands, and an episode is not undone by the robot failing to step
    back.

    The goal is a place behind the robot, sent to nav2, so the local costmap
    still applies. It may well turn round rather than reverse, which is fine and
    arguably better: a robot that gives you a ball and then turns to go and look
    for it again is legible, and legibility is the whole point of the gesture.
    """

    def __init__(self, name="BackAway", timeout=None):
        super().__init__(name)
        self.timeout = timeout
        self.blackboard = self.attach_blackboard_client(name=name)
        register_param_keys(self.blackboard)

    def setup(self, **kwargs):
        """Build the pose tracker and the navigator."""
        self.node = kwargs["node"]
        self.timeout = float(
            self.timeout
            if self.timeout is not None
            else constant(self.blackboard, "fetch_withdraw_timeout")
        )
        self.distance = float(constant(self.blackboard, "fetch_withdraw_distance"))
        self.pose = RobotPoseTracker(self.node)
        self.nav2 = Nav2PoseNavigator(self.node)
        self.logger.info(f"{self.name}: Setup complete")
        return True

    def initialise(self):
        """Remember where the handover happened and start the clock."""
        self.nav2.reset()
        self._start = self.node.get_clock().now()
        self._from = None
        self._sent = False

    def terminate(self, new_status):
        """Stop driving when the behaviour stops, whatever stopped it."""
        if new_status != py_trees.common.Status.RUNNING:
            self.nav2.cancel()

    def update(self):
        """Send one goal behind the robot and wait until it is far enough away."""
        if self.pose.pose is None:
            self.feedback_message = "waiting for AMCL pose"
            return py_trees.common.Status.RUNNING

        if self._from is None:
            self._from = self.pose.pose.position

        moved = distance_xy(self.pose.pose.position, self._from)
        if moved >= self.distance:
            return py_trees.common.Status.SUCCESS

        elapsed = (self.node.get_clock().now() - self._start).nanoseconds / 1e9
        if elapsed > self.timeout:
            self.node.get_logger().info(
                f"{self.name}: only got {moved:.2f} m away in {elapsed:.0f} s; "
                "the ball has changed hands anyway"
            )
            return py_trees.common.Status.FAILURE

        if not self._sent:
            if not self.nav2.server_ready():
                self.feedback_message = "waiting for the nav2 action server"
                return py_trees.common.Status.RUNNING
            self.nav2.go_to(self._goal_behind())
            self._sent = True
            return py_trees.common.Status.RUNNING

        status = self.nav2.status()
        if status == STATUS_SUCCEEDED:
            return py_trees.common.Status.SUCCESS
        if status is not None and status not in GOAL_ACTIVE_STATUSES:
            self.node.get_logger().info(
                f"{self.name}: nav2 would not take us back; carrying on"
            )
            return py_trees.common.Status.FAILURE
        self.feedback_message = f"{moved:.2f}/{self.distance:.2f} m away"
        return py_trees.common.Status.RUNNING

    def _goal_behind(self):
        """Return a pose one withdraw-distance behind the robot, facing the same way."""
        yaw = yaw_from_quaternion(self.pose.pose.orientation)
        goal = Pose()
        goal.position.x = self._from.x - self.distance * math.cos(yaw)
        goal.position.y = self._from.y - self.distance * math.sin(yaw)
        goal.orientation = quaternion_from_yaw(yaw)
        return goal
