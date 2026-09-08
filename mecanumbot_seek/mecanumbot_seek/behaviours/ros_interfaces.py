"""
The ROS plumbing the seek behaviours own, and the contract with the server.

The movement library already holds the trackers every tree needs -- the robot's
pose, nav2's goal state, the neck and gripper commander. What is here is only
what is specific to seeking an object, which is the robot's end of the T2 link:

    /mecanumbot/seek/request    (in)   std_msgs/String
        what to look for. **Free text**, from the operator or the web GUI --
        not a class label. It is handed to the server's open-vocabulary
        detector as the query verbatim, so "the red mug on the desk" finds what
        "mug" does not, and the half of the description that makes the object
        findable is exactly the half a class label would throw away. It travels
        unchanged into SeekingState.object_class and SeekAlert.object_class,
        which is what the trial record should show.

    /mecanumbot/seek/target     (in)   vision_msgs/Detection3DArray
        where the server found it in the point cloud, transformed into the
        robot's own `map` frame. Standard type on purpose: it is a labelled
        pose with a confidence, which is exactly what Detection3DArray carries,
        and mecanumbot_msgs is kept for data no standard type covers.

    /mecanumbot/seek/detections (in)   vision_msgs/Detection3DArray
        live onboard detections of the same object, in `map`. This is the
        incentive-salience channel -- what the robot can see for itself right
        now, as against what it was told.

    /mecanumbot/seek/state      (out)  mecanumbot_msgs/SeekingState
        the modelled circuit, published every tick for the record.

    /mecanumbot/seek/alert      (out)  mecanumbot_msgs/SeekAlert
        the robot found the thing and cannot have it. One per episode that ends
        that way, including the ones where there was nobody to tell.

Why two detection topics rather than one. The server's answer is a *memory*: a
single hypothesis, computed once from the T1 cloud, that stays put until the
server sends another. The onboard detections are *perception*: many per second,
each one only briefly true. Merging them would mean the tree could not tell
"where it was" from "where it is", which is the distinction the whole two-branch
search is built on.
"""

import rclpy
from rclpy.time import Time
from std_msgs.msg import String
from vision_msgs.msg import Detection3DArray

from mecanumbot_msgs.msg import AccessMotorCmd, SeekAlert, SeekingState
from mecanumbot_msgs.srv import SetLedStatus

# --- topics -----------------------------------------------------------------
# Absolute, like every topic the other trees use: the tree runs in the
# `mecanumbot` namespace and nav2 does not, so a relative name would be looked
# for under the tree's own namespace and never found.
SEEK_REQUEST_TOPIC = "/mecanumbot/seek/request"
SEEK_TARGET_TOPIC = "/mecanumbot/seek/target"
SEEK_DETECTIONS_TOPIC = "/mecanumbot/seek/detections"
SEEK_STATE_TOPIC = "/mecanumbot/seek/state"
SEEK_ALERT_TOPIC = "/mecanumbot/seek/alert"
ACCESSORY_TOPIC = "/cmd_accessory_pos"
LED_SERVICE = "/mecanumbot/set_led_status"


class ObjectHypothesis:
    """One detection, reduced to what the tree acts on."""

    def __init__(self, class_id, score, position, stamp=None):
        self.class_id = str(class_id)
        self.score = float(score)
        self.position = position
        self.stamp = stamp

    def __repr__(self):
        """Show the label, the score and where it is."""
        return (
            f"ObjectHypothesis({self.class_id!r}, score={self.score:.2f}, "
            f"x={self.position.x:.2f}, y={self.position.y:.2f})"
        )


def _best_hypothesis(msg, class_id=None):
    """Highest-scoring detection in the array, optionally of one class."""
    best = None
    for detection in msg.detections:
        for result in detection.results:
            if class_id and result.hypothesis.class_id != class_id:
                continue
            candidate = ObjectHypothesis(
                result.hypothesis.class_id,
                result.hypothesis.score,
                result.pose.pose.position,
                msg.header.stamp,
            )
            if best is None or candidate.score > best.score:
                best = candidate
    return best


class SeekRequestTracker:
    """What the robot has been asked to look for; None until it is told."""

    def __init__(self, node):
        self.object_class = None
        self._subscription = node.create_subscription(
            String, SEEK_REQUEST_TOPIC, self._callback, 10
        )

    def _callback(self, msg):
        self.object_class = msg.data.strip() or None


class SeekTargetTracker:
    """
    Where the server says the object was, in the map frame.

    A memory, not a perception: one hypothesis that stays put until the server
    replaces it. `updates` counts replacements, so the tree can tell a second
    scan's answer from the first one's without comparing poses.
    """

    def __init__(self, node):
        self.node = node
        self.hypothesis = None
        self.updates = 0
        self._subscription = node.create_subscription(
            Detection3DArray, SEEK_TARGET_TOPIC, self._callback, 10
        )

    @property
    def position(self):
        """Position of the target hypothesis, or None."""
        return None if self.hypothesis is None else self.hypothesis.position

    def _callback(self, msg):
        best = _best_hypothesis(msg)
        if best is None:
            return
        self.hypothesis = best
        self.updates += 1


class ObjectDetectionTracker:
    """
    Live onboard detections of the sought object, with an age and a gate.

    The gate is the incentive-salience one: `visible()` takes the confidence
    threshold from the SEEKING circuit rather than holding one of its own, so a
    strongly seeking robot acts on a weaker detection. That is the model
    changing what the robot perceives, which is what Panksepp's third attribute
    says an emotional system does.
    """

    def __init__(self, node, timeout=1.0, class_id=None):
        self.node = node
        self.timeout = float(timeout)
        self.class_id = class_id
        self.hypothesis = None
        self.last_seen = None
        self._subscription = node.create_subscription(
            Detection3DArray, SEEK_DETECTIONS_TOPIC, self._callback, 10
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

    def fresh(self):
        """Say whether a detection arrived recently enough to still mean anything."""
        age = self.age
        return self.hypothesis is not None and age is not None and age <= self.timeout

    def visible(self, threshold):
        """Say whether the object is in sight at or above a confidence threshold."""
        return self.fresh() and self.hypothesis.score >= float(threshold)

    def _callback(self, msg):
        best = _best_hypothesis(msg, self.class_id)
        if best is None:
            return
        self.hypothesis = best
        stamp = msg.header.stamp
        if stamp.sec or stamp.nanosec:
            self.last_seen = Time.from_msg(stamp)
        else:
            self.last_seen = self.node.get_clock().now()


class SeekingStatePublisher:
    """Publishes the modelled circuit, so a trial can be replayed against it."""

    def __init__(self, node, frame_id="map"):
        self.node = node
        self.frame_id = frame_id
        self._publisher = node.create_publisher(SeekingState, SEEK_STATE_TOPIC, 10)

    def publish(self, drive):
        """Publish one snapshot of the drive."""
        state = drive.snapshot()
        msg = SeekingState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.object_class = state["object_class"]
        msg.arousal = float(state["arousal"])
        msg.expectancy = float(state["expectancy"])
        msg.object_in_sight = bool(state["object_in_sight"])
        msg.seconds_since_cue = float(state["seconds_since_cue"])
        msg.search_radius = float(state["search_radius"])
        msg.phase = state["phase"]
        self._publisher.publish(msg)


class GripperCommander:
    """
    Open and close the grabbers, leaving the neck where it is.

    The movement library's `AccessoryCommander` owns the neck and sends a
    neutral gripper pose with every command, which is right for a tree that
    never grips anything. Closing on an object needs the opposite: the gripper
    positions are the message and the neck must not move, or the camera loses
    sight of the thing being picked up at exactly the wrong moment.
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


class AlertPublisher:
    """Publishes the record of an object the robot found and could not have."""

    def __init__(self, node, frame_id="map"):
        self.node = node
        self.frame_id = frame_id
        self._publisher = node.create_publisher(SeekAlert, SEEK_ALERT_TOPIC, 10)

    def publish(
        self,
        object_class,
        position,
        height,
        reason,
        person=None,
        alternations=0,
    ):
        """Publish one alert; `person` None means there was nobody to tell."""
        msg = SeekAlert()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.object_class = object_class or ""
        if position is not None:
            msg.object_position = position
        msg.object_height = float(0.0 if height is None else height)
        msg.reason = reason
        msg.person_informed = person is not None
        if person is not None:
            msg.person_position = person
        msg.gaze_alternations = min(255, max(0, int(alternations)))
        self._publisher.publish(msg)


class LedSignaller:
    """
    Fire-and-forget LED calls, skipped when there is no LED controller.

    The alert should be salient, and the LEDs are the robot's loudest channel.
    But this tree has to run on a bench with no Arduino attached, so a service
    that is not up is a reason to do nothing rather than to fail the alert --
    the alert is the message and the publication, and the flash is decoration.
    """

    def __init__(self, node):
        self.node = node
        self._client = node.create_client(SetLedStatus, LED_SERVICE)
        self._warned = False

    def flash(self, mode, color):
        """Set all four panels; mode 0 means leave the LEDs alone."""
        if not int(mode):
            return False
        if not self._client.service_is_ready():
            if not self._warned:
                self.node.get_logger().warn(
                    f"{LED_SERVICE} is not up; alerting without the LEDs"
                )
                self._warned = True
            return False
        request = SetLedStatus.Request()
        for panel in ("fl", "fr", "br", "bl"):
            setattr(request, f"{panel}_mode", int(mode))
            setattr(request, f"{panel}_color", int(color))
        # Not awaited: the alert is already published and the tree has no reason
        # to hold a tick open for a serial write to an LED board.
        self._client.call_async(request)
        return True


def duration(seconds):
    """`rclpy` duration from seconds, spelled out once."""
    return rclpy.duration.Duration(seconds=float(seconds))
