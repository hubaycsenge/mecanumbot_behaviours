"""Small ROS helper objects the behaviours own instead of re-implementing.

Every leading behaviour used to create its own subscriptions, its own QoS
profile and its own copy of the nav2 goal-status bookkeeping. The helpers here
hold that plumbing once; a behaviour just instantiates the ones it needs in
`setup()` and reads their properties in `update()`.
"""

import numpy as np
import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseArray, PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool

from mecanumbot_msgs.msg import AccessMotorCmd

from mecanumbot_leading_behaviour.behaviours.geometry import yaw_from_quaternion

# --- topics -----------------------------------------------------------------
AMCL_TOPIC = "/amcl_pose"
PEOPLE_TOPIC = "/mecanumbot/people_fusion"
SUBJECT_TOPIC = "/mecanumbot/subject_pose"
HAS_OBJECT_TOPIC = "/mecanumbot/has_object"
GOAL_POSE_TOPIC = "/goal_pose"
CMD_VEL_TOPIC = "/cmd_vel"
ACCESSORY_TOPIC = "/cmd_accessory_pos"
NAV2_STATUS_TOPIC = "/navigate_to_pose/_action/status"
NAV2_ROUTE_STATUS_TOPIC = "/navigate_through_poses/_action/status"

# --- nav2 actions ------------------------------------------------------------
# Absolute, like every topic above: the trees run in the `mecanumbot` namespace
# and nav2 does not, so a relative name would be looked for under the tree's own
# namespace and never found.
NAV2_TO_POSE_ACTION = "/navigate_to_pose"
NAV2_THROUGH_POSES_ACTION = "/navigate_through_poses"

# --- nav2 goal statuses (action_msgs/GoalStatus) -----------------------------
STATUS_UNKNOWN = 0
STATUS_ACCEPTED = 1
STATUS_EXECUTING = 2
STATUS_CANCELING = 3
STATUS_SUCCEEDED = 4
STATUS_CANCELED = 5
STATUS_ABORTED = 6

GOAL_ACTIVE_STATUSES = (STATUS_ACCEPTED, STATUS_EXECUTING)
GOAL_FAILED_STATUSES = (
    STATUS_ABORTED,
    STATUS_CANCELED,
    STATUS_CANCELING,
    STATUS_UNKNOWN,
)

# --- accessory (neck / gripper) poses ---------------------------------------
# n_pos is the neck-mounted camera tilt: larger means the head looks further up
# (see MECANUMBOT_{MIN,MAX}_CAM_POS = 2.0 .. 8.6 in the teleop scripts).
#
# NECK_SEEK_POS serves double duty: the lifted head reads as the robot seeking
# contact with a human, and it gives the YOLO26n-pose detector a view of the
# whole body instead of just the knees of nearby people, which it often misses.
# So the head stays lifted for as long as the robot is looking for or at
# someone, not just for a moment.
#
# These are the fallbacks. The constants YAML declares them as `neck_seek_pos`,
# `neck_level_pos` and `gripper_{left,right}_neutral`, and the parameter loader
# hands them to `AccessoryCommander.configure()` before any tree is ticked.
NECK_SEEK_POS = 7.0
NECK_LEVEL_POS = 6.0
GRIPPER_LEFT_NEUTRAL = 6.83
GRIPPER_RIGHT_NEUTRAL = 3.36

HEAD_SEEK = "seek"
HEAD_LEVEL = "level"


def amcl_qos():
    """QoS profile matching the AMCL pose publisher."""
    return QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )


class RobotPoseTracker:
    """Latest AMCL pose of the robot."""

    def __init__(self, node):
        self.pose = None
        self.updates = 0
        self._subscription = node.create_subscription(
            PoseWithCovarianceStamped, AMCL_TOPIC, self._callback, amcl_qos()
        )

    @property
    def position(self):
        return None if self.pose is None else self.pose.position

    @property
    def yaw(self):
        return None if self.pose is None else yaw_from_quaternion(self.pose.orientation)

    def _callback(self, msg):
        self.pose = msg.pose.pose
        self.updates += 1


class PeopleTracker:
    """Fused people detections, with an age check for "is somebody visible now"."""

    def __init__(self, node, sight_timeout=1.0):
        self.node = node
        self.sight_timeout = float(sight_timeout)
        self.poses = []
        self.last_seen_time = None
        self.last_seen_pose = None
        self._subscription = node.create_subscription(
            PoseArray, PEOPLE_TOPIC, self._callback, 10
        )

    @property
    def age(self):
        """Seconds since the last detection, or None if nobody was ever seen."""
        if self.last_seen_time is None:
            return None
        return (self.node.get_clock().now() - self.last_seen_time).nanoseconds / 1e9

    def has_fresh_detection(self):
        age = self.age
        return bool(self.poses) and age is not None and age <= self.sight_timeout

    def _callback(self, msg):
        self.poses = list(msg.poses)
        if not self.poses:
            return
        self.last_seen_pose = self.poses[0]
        stamp = msg.header.stamp
        if stamp.sec or stamp.nanosec:
            self.last_seen_time = Time.from_msg(stamp)
        else:
            self.last_seen_time = self.node.get_clock().now()


class SubjectPoseTracker:
    """Latest pose on `/mecanumbot/subject_pose` (the tracked human)."""

    def __init__(self, node):
        self.node = node
        self.pose = None
        self.stamp = None
        self._subscription = node.create_subscription(
            PoseStamped, SUBJECT_TOPIC, self._callback, 10
        )

    @property
    def position(self):
        return None if self.pose is None else self.pose.position

    @property
    def age(self):
        """Seconds since the last subject pose, or None if there was none yet."""
        if self.stamp is None:
            return None
        return (
            self.node.get_clock().now() - Time.from_msg(self.stamp)
        ).nanoseconds / 1e9

    def _callback(self, msg):
        self.pose = msg.pose
        self.stamp = msg.header.stamp


class FollowedSubjectTracker:
    """Where the human being led is, and how long ago that was known.

    `/mecanumbot/subject_pose` is the better answer of the two -- it is one
    tracked person rather than whoever happened to be detected -- but it is not
    always published, and a tracker can hold on to a pose after the person
    behind it has gone. So both sources are kept and the fresher one is used;
    with neither, `age` is None, which every caller reads as "not seen at all".
    """

    def __init__(self, node, sight_timeout=1.0):
        self.subject = SubjectPoseTracker(node)
        self.people = PeopleTracker(node, sight_timeout)

    def _fresher(self):
        subject_age, people_age = self.subject.age, self.people.age
        if subject_age is None:
            return self.people.last_seen_pose, people_age
        if people_age is None or subject_age <= people_age:
            return self.subject.pose, subject_age
        return self.people.last_seen_pose, people_age

    @property
    def pose(self):
        return self._fresher()[0]

    @property
    def position(self):
        pose = self.pose
        return None if pose is None else pose.position

    @property
    def age(self):
        """Seconds since the human was last placed anywhere, or None."""
        return self._fresher()[1]


class BallTracker:
    """Latest `/mecanumbot/has_object` state; None until the first message."""

    def __init__(self, node):
        self.has_ball = None
        self._subscription = node.create_subscription(
            Bool, HAS_OBJECT_TOPIC, self._callback, 10
        )

    def _callback(self, msg):
        self.has_ball = msg.data


class Nav2GoalMonitor:
    """Publishes `/goal_pose` goals and follows their outcome.

    Nav2 does not tell us the goal id of a pose we published, so a goal is
    matched by taking the newest entry in the action status array that is not
    older than our publish time -- the same trick as before, in one place.

    `Nav2Navigator` below drives nav2 through its actions instead and is what
    the leading behaviours use; this one stays for the behaviours that publish a
    goal pose and only want to know how it went (`mecanumbot_ostensive_behaviour`
    does), and for `busy()`, which is the question "is nav2 driving right now"
    and is answered from the status topics rather than from a goal of our own.
    """

    def __init__(self, node, history=5):
        self.node = node
        self._history = history
        self._statuses = []
        self._route_statuses = []
        self._publisher = node.create_publisher(PoseStamped, GOAL_POSE_TOPIC, 10)
        self._status_subscription = node.create_subscription(
            GoalStatusArray, NAV2_STATUS_TOPIC, self._status_callback, 10
        )
        # A waypoint run reports on its own action, so watching only the
        # single-goal one would let a turn take /cmd_vel mid-route.
        self._route_status_subscription = node.create_subscription(
            GoalStatusArray, NAV2_ROUTE_STATUS_TOPIC, self._route_status_callback, 10
        )
        self.reset()

    def reset(self):
        """Forget the goal we were following (call from `initialise()`)."""
        self._send_time = None
        self._goal_uuid = None
        self._last_status = None

    @property
    def goal_sent(self):
        return self._send_time is not None

    def send(self, pose, frame_id="map"):
        """Publish a goal pose and start following it."""
        goal = PoseStamped()
        goal.header.frame_id = frame_id
        self._send_time = self.node.get_clock().now()
        goal.header.stamp = self._send_time.to_msg()
        goal.pose = pose
        self._goal_uuid = None
        self._last_status = None
        self._publisher.publish(goal)

    def busy(self):
        """True while nav2 is executing some goal (ours or a leftover one)."""
        return any(
            status.status == STATUS_EXECUTING
            for status in self._statuses + self._route_statuses
        )

    def seconds_since_send(self):
        if self._send_time is None:
            return 0.0
        return (self.node.get_clock().now() - self._send_time).nanoseconds / 1e9

    def status(self):
        """Status of our goal, or None while nav2 has not reported it yet.

        The last known status is kept, so a goal that scrolls out of the status
        array does not look like it disappeared.
        """
        if not self.goal_sent:
            return None
        if self._goal_uuid is None:
            self._lock_onto_goal()
        if self._goal_uuid is None:
            return None
        for goal_status in self._statuses:
            if np.array_equal(goal_status.goal_info.goal_id.uuid, self._goal_uuid):
                self._last_status = goal_status.status
                break
        return self._last_status

    def _lock_onto_goal(self):
        send_time = Time.from_msg(self._send_time.to_msg())
        for goal_status in reversed(self._statuses):  # newest first
            if Time.from_msg(goal_status.goal_info.stamp) >= send_time:
                self._goal_uuid = goal_status.goal_info.goal_id.uuid
                return

    def _status_callback(self, msg):
        # Nav2 appends new goals, so only the tail is interesting.
        self._statuses = list(msg.status_list[-self._history :])

    def _route_status_callback(self, msg):
        self._route_statuses = list(msg.status_list[-self._history :])


class Nav2Navigator:
    """One nav2 navigation goal at a time, sent as an action.

    A goal pose published on `/goal_pose` is a message shouted into the dark:
    nav2 never says which goal id it became, so the outcome has to be guessed
    from the status array, and there is no way to take it back. The action gives
    all three -- the goal handle identifies the goal, the result says how it
    ended, and `cancel()` stops it, which is what lets a behaviour hand
    `/cmd_vel` over cleanly when the robot has to turn instead of drive.

    Everything is asynchronous: a behaviour only ever reads `status()` during a
    tick, and the callbacks run between ticks on the same executor, so nothing
    here ever waits.

    Subclasses fill in which action this is; `Nav2PoseNavigator` drives to one
    pose, `Nav2RouteNavigator` runs a leg of waypoints in a single goal.
    """

    ACTION_TYPE = None
    ACTION_NAME = ""

    def __init__(self, node):
        self.node = node
        self._client = ActionClient(node, self.ACTION_TYPE, self.ACTION_NAME)
        self.reset()

    # --- lifecycle -----------------------------------------------------------

    def reset(self):
        """Forget the goal we were following (call from `initialise()`)."""
        self._send_time = None
        self._goal_handle = None
        self._status = None
        self._rejected = False
        self._cancel_requested = False
        self.feedback = None

    def server_ready(self):
        """True once the nav2 action server is up."""
        return self._client.server_is_ready()

    @property
    def goal_sent(self):
        return self._send_time is not None

    def send(self, goal):
        """Start a goal; `goal` is the action's own goal message."""
        self.reset()
        self._send_time = self.node.get_clock().now()
        self._client.send_goal_async(goal, feedback_callback=self._on_feedback).add_done_callback(
            self._on_goal_response
        )

    def cancel(self):
        """Ask nav2 to stop driving; safe to call at any point of a goal."""
        if not self.goal_sent:
            return
        if self._goal_handle is None:
            # The goal has not been accepted yet -- cancel it once it is.
            self._cancel_requested = True
            return
        if self._status in (None, STATUS_ACCEPTED, STATUS_EXECUTING):
            self._goal_handle.cancel_goal_async()
            self._status = STATUS_CANCELING

    def seconds_since_send(self):
        if self._send_time is None:
            return 0.0
        return (self.node.get_clock().now() - self._send_time).nanoseconds / 1e9

    def status(self):
        """Status of our goal, or None while nav2 has not reported on it yet."""
        if self._rejected:
            return STATUS_ABORTED
        return self._status

    # --- callbacks -----------------------------------------------------------

    def _on_goal_response(self, future):
        # A goal that never reached nav2 at all is reported the same way as one
        # nav2 turned down: the behaviour retries it or gives up on it.
        try:
            handle = future.result()
            accepted = handle.accepted
        except Exception as error:  # the server went away mid-request
            self._rejected = True
            self.node.get_logger().warn(f"{self.ACTION_NAME}: goal never landed ({error})")
            return
        if not accepted:
            self._rejected = True
            self.node.get_logger().warn(f"{self.ACTION_NAME}: nav2 rejected the goal")
            return
        self._goal_handle = handle
        if self._status is None:
            self._status = STATUS_ACCEPTED
        handle.get_result_async().add_done_callback(self._on_result)
        if self._cancel_requested:
            self._cancel_requested = False
            self.cancel()

    def _on_result(self, future):
        try:
            self._status = future.result().status
        except Exception as error:
            self._status = STATUS_ABORTED
            self.node.get_logger().warn(f"{self.ACTION_NAME}: no result ({error})")

    def _on_feedback(self, message):
        self.feedback = message.feedback
        if self._status in (None, STATUS_ACCEPTED):
            self._status = STATUS_EXECUTING


class Nav2PoseNavigator(Nav2Navigator):
    """`NavigateToPose`: drive to one pose."""

    ACTION_TYPE = NavigateToPose
    ACTION_NAME = NAV2_TO_POSE_ACTION

    def go_to(self, pose, frame_id="map"):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = frame_id
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose = pose
        self.send(goal)


class Nav2RouteNavigator(Nav2Navigator):
    """`NavigateThroughPoses`: drive a run of waypoints without stopping at each."""

    ACTION_TYPE = NavigateThroughPoses
    ACTION_NAME = NAV2_THROUGH_POSES_ACTION

    def follow(self, poses, frame_id="map"):
        goal = NavigateThroughPoses.Goal()
        stamp = self.node.get_clock().now().to_msg()
        for pose in poses:
            stamped = PoseStamped()
            stamped.header.frame_id = frame_id
            stamped.header.stamp = stamp
            stamped.pose = pose
            goal.poses.append(stamped)
        self.send(goal)

    @property
    def poses_remaining(self):
        """Waypoints nav2 says are still ahead, or None before the first feedback."""
        if self.feedback is None:
            return None
        return int(self.feedback.number_of_poses_remaining)


class VelocityCommander:
    """Thin `/cmd_vel` publisher."""

    def __init__(self, node):
        self._publisher = node.create_publisher(Twist, CMD_VEL_TOPIC, 10)

    def turn(self, angular_z):
        cmd = Twist()
        cmd.angular.z = float(angular_z)
        self._publisher.publish(cmd)

    def stop(self):
        self._publisher.publish(Twist())


class AccessoryCommander:
    """Neck (camera tilt) and gripper commands.

    The poses are class state, like the last commanded neck position, because
    there is only one robot: every behaviour that builds a commander means the
    same head. `configure()` is called once by the parameter loader, so the
    poses come from the same YAML as the gesture sequences that move between
    them, and a commander built later in `setup()` still sees them.
    """

    _last_neck_pos = None

    seek_pos = NECK_SEEK_POS
    level_pos = NECK_LEVEL_POS
    gripper_left = GRIPPER_LEFT_NEUTRAL
    gripper_right = GRIPPER_RIGHT_NEUTRAL

    @classmethod
    def configure(cls, seek_pos=None, level_pos=None, gripper_left=None, gripper_right=None):
        """Set the named poses from configuration; None leaves one as it is."""
        if seek_pos is not None:
            cls.seek_pos = float(seek_pos)
        if level_pos is not None:
            cls.level_pos = float(level_pos)
        if gripper_left is not None:
            cls.gripper_left = float(gripper_left)
        if gripper_right is not None:
            cls.gripper_right = float(gripper_right)

    def __init__(self, node):
        self.node = node
        self._publisher = node.create_publisher(AccessMotorCmd, ACCESSORY_TOPIC, 10)

    def send(self, n_pos, gl_pos=None, gr_pos=None):
        cmd = AccessMotorCmd()
        cmd.n_pos = float(n_pos)
        cmd.gl_pos = float(self.gripper_left if gl_pos is None else gl_pos)
        cmd.gr_pos = float(self.gripper_right if gr_pos is None else gr_pos)
        self._publisher.publish(cmd)
        AccessoryCommander._last_neck_pos = cmd.n_pos

    def look(self, where):
        """Move the head to a named pose, skipping redundant commands.

        `HEAD_SEEK` lifts the head (contact seeking + better people detection),
        `HEAD_LEVEL` returns it to the neutral driving gaze.
        """
        if where is None:
            return
        n_pos = self.seek_pos if where == HEAD_SEEK else self.level_pos
        if self._last_neck_pos is not None and abs(self._last_neck_pos - n_pos) < 1e-3:
            return
        self.send(n_pos)
        self.node.get_logger().info(f"Head -> {where} (n_pos={n_pos})")


def duration(seconds):
    """`rclpy` duration from seconds, spelled out once."""
    return rclpy.duration.Duration(seconds=float(seconds))


def now_seconds(node):
    """Node clock as plain seconds, for the timestamps kept on the blackboard."""
    return node.get_clock().now().nanoseconds / 1e9
