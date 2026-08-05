"""
The one subscription this package adds on top of the leading behaviour library.

Everything else the ostensive tree needs -- the AMCL pose, the fused people, the
nav2 goal monitor, the velocity and neck commanders -- already exists in
`mecanumbot_leading_behaviour.behaviours.ros_interfaces` and is re-exported at
the bottom of this module so the behaviours have a single import to reach for.

What is new is `cam_people_detections`. The fused `people_fusion` poses say
*where* people are in the map but nothing about what they are doing with their
arms, and the gestures are the whole point here, so the camera detector's
keypoint output is subscribed to directly.
"""

from mecanumbot_msgs.msg import CamPersonDetectionArray
from rclpy.time import Time

from mecanumbot_leading_behaviour.behaviours.ros_interfaces import (  # noqa: F401  (re-export)
    AccessoryCommander,
    GOAL_ACTIVE_STATUSES,
    HEAD_SEEK,
    Nav2GoalMonitor,
    PeopleTracker,
    RobotPoseTracker,
    STATUS_SUCCEEDED,
)

from mecanumbot_ostensive_behaviour.behaviours.keypoints import (
    body_scale,
    image_centroid_x,
    keypoints_from_msg,
    torso_centre,
)

CAM_DETECTIONS_TOPIC = "/mecanumbot/cam_people_detections"


class PersonObservation:
    """One person on one camera frame: their joints and the values read off them."""

    __slots__ = ("keypoints", "torso", "centroid_x", "scale")

    def __init__(self, keypoints):
        self.keypoints = keypoints
        self.torso = torso_centre(keypoints)
        self.centroid_x = image_centroid_x(keypoints)
        self.scale = body_scale(keypoints)

    @property
    def usable(self):
        """Return True when the person can be tracked and measured at all."""
        return self.torso is not None and self.scale is not None


class CamDetectionTracker:
    """
    Latest `cam_people_detections` frame, converted into `PersonObservation`s.

    The conversion happens in the subscription callback rather than on demand
    because several behaviours read the same frame within one tick, and the
    keypoint dictionaries would otherwise be rebuilt for each of them.

    Frames are identified by their header stamp, which is what lets the tracking
    in `target_lock` run once per frame no matter how many behaviours ask for it.
    """

    def __init__(self, node, sight_timeout=1.0):
        self.node = node
        self.sight_timeout = float(sight_timeout)
        self.people = []
        self.stamp = None
        self._time = None
        self._subscription = node.create_subscription(
            CamPersonDetectionArray, CAM_DETECTIONS_TOPIC, self._callback, 10
        )

    @property
    def age(self):
        """Seconds since the last camera frame, or None if none has arrived."""
        if self._time is None:
            return None
        return (self.node.get_clock().now() - self._time).nanoseconds / 1e9

    def fresh(self):
        """Return True when a camera frame arrived recently enough to act on."""
        age = self.age
        return age is not None and age <= self.sight_timeout

    def _callback(self, message):
        self.people = [
            observation
            for observation in (
                PersonObservation(keypoints_from_msg(person.keypoints))
                for person in message.people
            )
            if observation.usable
        ]
        stamp = message.header.stamp
        self.stamp = (stamp.sec, stamp.nanosec)
        if stamp.sec or stamp.nanosec:
            self._time = Time.from_msg(stamp)
        else:
            self._time = self.node.get_clock().now()


def seconds(node):
    """Return the node clock as a float, for the pure gesture code."""
    return node.get_clock().now().nanoseconds / 1e9
