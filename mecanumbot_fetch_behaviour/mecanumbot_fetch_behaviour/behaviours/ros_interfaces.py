"""
The ROS plumbing the fetch behaviours own.

The movement library already holds the trackers every tree needs -- the robot's
pose, nav2's goal state, the fused people, `/mecanumbot/has_object`, the
accessory commander. What is here is only what is specific to playing fetch:

    /mecanumbot/ball_detections  (in)   vision_msgs/Detection3DArray
        where the ball is, in the map frame, from
        `mecanumbot_locate_detections`. A standard type, and the same one the
        Deep3R seeking system passes object hypotheses around in: a labelled
        pose with a confidence is exactly what Detection3DArray carries. The
        `z` is not decoration -- it is what separates a ball on the floor from
        a ball in somebody's hand, and the grabbers have no lift.

    /mecanumbot/fetch/state      (out)  std_msgs/String
        what the robot is doing, one word per transition, for the record and
        for anybody watching a trial without a terminal on the tree. A String
        rather than a new message type: mecanumbot_msgs is deliberately small
        and is for data no standard type covers, and this is a label.

`GripperCommander` is duplicated from `mecanumbot_seek` rather than shared,
which is worth saying out loud. It is eight lines, and the alternative is a
dependency from one experiment package to another -- which is the arrangement
this repository's library/experiment split exists to avoid. If a third tree
needs it, it belongs in `mecanumbot_movement_behaviours` next to
`AccessoryCommander`, not in whichever experiment wrote it first.
"""

from rclpy.time import Time
from std_msgs.msg import String
from vision_msgs.msg import Detection3DArray

from mecanumbot_msgs.msg import AccessMotorCmd

# Absolute, like every topic the other trees use: the tree runs in the
# `mecanumbot` namespace and nav2 does not, so a relative name would be looked
# for under the tree's own namespace and never found.
BALL_DETECTIONS_TOPIC = "/mecanumbot/ball_detections"
FETCH_STATE_TOPIC = "/mecanumbot/fetch/state"
ACCESSORY_TOPIC = "/cmd_accessory_pos"


def class_filter(value):
    """
    Return the label a ball has to carry, or None to accept any label.

    `fetch_ball_class: None` in a constants file reaches here as the *string*
    'None' -- YAML's null is `null` or `~` -- and so does a YAML null after the
    call sites' `str()`. Taken literally, either one means "only balls labelled
    None", which silently ignores every ball there is. Nothing is labelled
    None, so all of these mean "no filter".
    """
    if value is None:
        return None
    text = str(value).strip()
    if text in ("", "None", "none", "null", "~"):
        return None
    return text


class BallHypothesis:
    """One located ball, reduced to what the tree acts on."""

    def __init__(self, class_id, score, position):
        self.class_id = str(class_id)
        self.score = float(score)
        self.position = position

    def __repr__(self):
        """Show the label, the score and where it is."""
        return (
            f"BallHypothesis({self.class_id!r}, score={self.score:.2f}, "
            f"x={self.position.x:.2f}, y={self.position.y:.2f}, "
            f"z={self.position.z:.2f})"
        )


class BallDetectionTracker:
    """
    Where the ball is right now, with an age and a confidence gate.

    The nearest ball wins rather than the most confident one, which is the
    opposite of `mecanumbot_seek`'s choice and is right for a different reason.
    Seek is told to find one particular object and picks the hypothesis most
    likely to *be* it. In a fetch game every ball on the floor is equally the
    ball; the one to go for is the one the robot can reach soonest, and picking
    by score instead makes the robot cross the room past a ball at its feet
    because a further one is better lit.
    """

    def __init__(self, node, timeout=1.0, class_id=None, topic=BALL_DETECTIONS_TOPIC):
        self.node = node
        self.timeout = float(timeout)
        self.class_id = class_filter(class_id)
        self.hypothesis = None
        self.last_seen = None
        # When the last message of any kind arrived, by this node's clock, and
        # what it carried that was not used -- so a ball that is published but
        # never acted on can say why instead of vanishing.
        self.last_received = None
        self.last_ignored_classes = ()
        self._robot_position = None
        self._subscription = node.create_subscription(
            Detection3DArray, topic, self._callback, 10
        )

    @property
    def age(self):
        """Seconds since the last detection, or None if there has been none."""
        if self.last_seen is None:
            return None
        return (self.node.get_clock().now() - self.last_seen).nanoseconds / 1e9

    @property
    def position(self):
        """Position of the newest detection, or None."""
        return None if self.hypothesis is None else self.hypothesis.position

    def look_from(self, position):
        """Tell the tracker where the robot is, so "nearest" means anything."""
        self._robot_position = position

    def fresh(self):
        """Say whether a detection arrived recently enough to still mean anything."""
        age = self.age
        return self.hypothesis is not None and age is not None and age <= self.timeout

    def visible(self, threshold):
        """Say whether a ball is in sight at or above a confidence threshold."""
        return self.fresh() and self.hypothesis.score >= float(threshold)

    def received_age(self):
        """Seconds since any message arrived, by this node's clock, or None."""
        if self.last_received is None:
            return None
        return (self.node.get_clock().now() - self.last_received).nanoseconds / 1e9

    def _callback(self, msg):
        self.last_received = self.node.get_clock().now()
        best = None
        best_distance = None
        ignored = set()
        for detection in msg.detections:
            for result in detection.results:
                if self.class_id and result.hypothesis.class_id != self.class_id:
                    ignored.add(result.hypothesis.class_id)
                    continue
                candidate = BallHypothesis(
                    result.hypothesis.class_id,
                    result.hypothesis.score,
                    detection.bbox.center.position,
                )
                distance = self._distance_to(candidate.position)
                if best is None or distance < best_distance:
                    best, best_distance = candidate, distance
        self.last_ignored_classes = tuple(sorted(ignored))
        if best is None:
            return
        self.hypothesis = best
        stamp = msg.header.stamp
        if stamp.sec or stamp.nanosec:
            self.last_seen = Time.from_msg(stamp)
        else:
            self.last_seen = self.node.get_clock().now()

    def _distance_to(self, position):
        """Distance from the robot, or 0.0 while the robot has no pose yet."""
        if self._robot_position is None:
            return 0.0
        return (
            (position.x - self._robot_position.x) ** 2
            + (position.y - self._robot_position.y) ** 2
        ) ** 0.5


class GripperCommander:
    """
    Open and close the grabbers, leaving the neck where it is.

    The movement library's `AccessoryCommander` owns the neck and sends a
    neutral gripper pose with every command, which is right for a tree that
    never grips anything. Closing on a ball needs the opposite: the gripper
    positions are the message and the neck must not move, or the camera loses
    sight of the ball at exactly the wrong moment.
    """

    def __init__(self, node):
        self.node = node
        self._publisher = node.create_publisher(AccessMotorCmd, ACCESSORY_TOPIC, 10)

    def send(self, neck, left, right):
        """Command a neck tilt and both gripper positions together."""
        cmd = AccessMotorCmd()
        cmd.n_pos = float(neck)
        cmd.gl_pos = float(left)
        cmd.gr_pos = float(right)
        self._publisher.publish(cmd)


class FetchStatePublisher:
    """
    Publishes what the robot is doing, one label per transition.

    Latched-ish by convention rather than by QoS: every phase publishes on
    entry, so a subscriber that joins late sees the next transition rather than
    the current state. That is enough for a trial recording, which is what this
    is for.
    """

    def __init__(self, node, topic=FETCH_STATE_TOPIC):
        self.node = node
        self._publisher = node.create_publisher(String, topic, 10)
        self._last = None

    def publish(self, phase):
        """Publish a phase label, skipping a repeat of the current one."""
        if phase == self._last:
            return
        self._last = phase
        self._publisher.publish(String(data=str(phase)))
