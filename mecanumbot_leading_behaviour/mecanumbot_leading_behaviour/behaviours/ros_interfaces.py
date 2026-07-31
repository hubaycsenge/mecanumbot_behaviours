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
    """

    def __init__(self, node, history=5):
        self.node = node
        self._history = history
        self._statuses = []
        self._publisher = node.create_publisher(PoseStamped, GOAL_POSE_TOPIC, 10)
        self._status_subscription = node.create_subscription(
            GoalStatusArray, NAV2_STATUS_TOPIC, self._status_callback, 10
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
        return any(status.status == STATUS_EXECUTING for status in self._statuses)

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

    The last commanded neck position is shared by every commander in the process
    -- there is only one robot -- so a gesture sequence that moves the neck does
    not leave another behaviour believing the head is still where it put it.
    """

    _last_neck_pos = None

    def __init__(self, node):
        self.node = node
        self._publisher = node.create_publisher(AccessMotorCmd, ACCESSORY_TOPIC, 10)

    def send(self, n_pos, gl_pos=GRIPPER_LEFT_NEUTRAL, gr_pos=GRIPPER_RIGHT_NEUTRAL):
        cmd = AccessMotorCmd()
        cmd.n_pos = float(n_pos)
        cmd.gl_pos = float(gl_pos)
        cmd.gr_pos = float(gr_pos)
        self._publisher.publish(cmd)
        AccessoryCommander._last_neck_pos = cmd.n_pos

    def look(self, where):
        """Move the head to a named pose, skipping redundant commands.

        `HEAD_SEEK` lifts the head (contact seeking + better people detection),
        `HEAD_LEVEL` returns it to the neutral driving gaze.
        """
        if where is None:
            return
        n_pos = NECK_SEEK_POS if where == HEAD_SEEK else NECK_LEVEL_POS
        if self._last_neck_pos is not None and abs(self._last_neck_pos - n_pos) < 1e-3:
            return
        self.send(n_pos)
        self.node.get_logger().info(f"Head -> {where} (n_pos={n_pos})")


def duration(seconds):
    """`rclpy` duration from seconds, spelled out once."""
    return rclpy.duration.Duration(seconds=float(seconds))
