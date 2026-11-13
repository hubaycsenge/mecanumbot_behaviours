"""
The ROS plumbing the object-choice tree adds to the two trees it borrows from.

    /mecanumbot/brick_detections            (in)  vision_msgs/Detection3DArray
    /mecanumbot/cam_brick_boxes             (in)  vision_msgs/Detection2DArray
        every brick, labelled with its colour, from `mecanumbot_detect_bricks`.

    /mecanumbot/object_choice/target_detections   (out, relayed)
    /mecanumbot/object_choice/target_boxes        (out, relayed)
        **only the chosen brick**. See `TargetRelay`.

    /mecanumbot/object_choice/state         (out) std_msgs/String
        one label per transition, as `/mecanumbot/fetch/state` is for fetch --
        and the fetch behaviours' own labels land here too, by remapping.

    /mecanumbot/object_choice/trial         (out) std_msgs/String, JSON
        one record per trial, whatever its outcome. See `trial.FinishTrial`.

    /mecanumbot/object_choice/next          (in)  std_msgs/Empty
        the experimenter's "the bricks are set, go".

## Why a relay, and why the launch file remaps

The approach, the centring turn, the creep and the grab are the fetch tree's
behaviours, used unchanged -- they are tuned on the robot, and re-deriving them
for a brick would be a second copy of the hardest-won code in the repository.
But they are single-target by design: each picks *the* ball, the nearest in the
map and the biggest in the image. With several bricks on the floor they would
wander to whichever brick was nearest, which is exactly the decision the trial
is about.

So the launch file remaps the fetch behaviours' two inputs,
`/mecanumbot/ball_detections` and `/mecanumbot/cam_ball_boxes`, onto the
target topics above, and `TargetRelay` forwards onto them only the brick of the
chosen colour. To the fetch behaviours, the chosen brick is the only ball there
is. `TargetRelay.check_remapping` says so loudly at startup if the remapping is
missing -- without it the approach drives to the remembered position with no
live correction and the centring turn sees nothing, which runs but scores wrong.
"""

import json

from geometry_msgs.msg import Point
from std_msgs.msg import Empty, String
from vision_msgs.msg import Detection2DArray, Detection3DArray

BRICK_DETECTIONS_TOPIC = "/mecanumbot/brick_detections"
BRICK_BOXES_TOPIC = "/mecanumbot/cam_brick_boxes"
TARGET_DETECTIONS_TOPIC = "/mecanumbot/object_choice/target_detections"
TARGET_BOXES_TOPIC = "/mecanumbot/object_choice/target_boxes"
STATE_TOPIC = "/mecanumbot/object_choice/state"
TRIAL_TOPIC = "/mecanumbot/object_choice/trial"
GO_TOPIC = "/mecanumbot/object_choice/next"

# What the fetch behaviours subscribe to, and must be remapped away from.
FETCH_BALL_DETECTIONS_TOPIC = "/mecanumbot/ball_detections"
FETCH_BALL_BOXES_TOPIC = "/mecanumbot/cam_ball_boxes"


def _label(detection):
    """Return a detection's colour: its first result's class, or its id."""
    for result in detection.results:
        if result.hypothesis.class_id:
            return result.hypothesis.class_id
    return detection.id


def _score(detection):
    return max([float(r.hypothesis.score) for r in detection.results] or [0.0])


class BrickSighting:
    """One colour's newest located brick."""

    __slots__ = ("colour", "position", "score", "stamp")

    def __init__(self, colour, position, score, stamp):
        self.colour = colour
        self.position = position
        self.score = float(score)
        self.stamp = stamp


class BrickTracker:
    """The newest `brick_detections` position of every colour, by this node's clock."""

    def __init__(self, node, topic=BRICK_DETECTIONS_TOPIC):
        self.node = node
        self.sightings = {}
        self._subscription = node.create_subscription(
            Detection3DArray, topic, self._callback, 10
        )

    def fresh(self, timeout, threshold=0.0):
        """Return the sightings no older than `timeout` [s] and scored >= threshold."""
        now = self.node.get_clock().now()
        return {
            colour: sighting
            for colour, sighting in self.sightings.items()
            if (now - sighting.stamp).nanoseconds / 1e9 <= timeout
            and sighting.score >= threshold
        }

    def _callback(self, msg):
        now = self.node.get_clock().now()
        for detection in msg.detections:
            centre = detection.bbox.center.position
            colour = _label(detection)
            self.sightings[colour] = BrickSighting(
                colour,
                Point(x=centre.x, y=centre.y, z=centre.z),
                _score(detection),
                now,
            )


class TargetRelay:
    """
    Forward only the chosen brick onto the topics the fetch behaviours read.

    One per process, started by the parameter loader, because the target is
    one fact about the trial: `ChooseBrick` sets it and `FinishTrial` clears
    it, and with no target nothing at all is forwarded -- an approach that
    somehow ran without a choice would find no ball, rather than the nearest
    brick.
    """

    _instance = None
    target = None

    @classmethod
    def start(cls, node):
        """Create the relay on `node`, once."""
        if cls._instance is None:
            cls._instance = cls(node)
            cls._instance.check_remapping()
        return cls._instance

    @classmethod
    def set_target(cls, colour):
        """Forward this colour from now on; None forwards nothing."""
        cls.target = colour

    def __init__(self, node):
        self.node = node
        self._detections_pub = node.create_publisher(
            Detection3DArray, TARGET_DETECTIONS_TOPIC, 10
        )
        self._boxes_pub = node.create_publisher(Detection2DArray, TARGET_BOXES_TOPIC, 10)
        node.create_subscription(
            Detection3DArray, BRICK_DETECTIONS_TOPIC, self._on_detections, 10
        )
        node.create_subscription(Detection2DArray, BRICK_BOXES_TOPIC, self._on_boxes, 10)

    def check_remapping(self):
        """Say whether the fetch behaviours' inputs reach the relay's outputs."""
        problems = []
        for fetch_topic, target_topic, kind in (
            (FETCH_BALL_DETECTIONS_TOPIC, TARGET_DETECTIONS_TOPIC, Detection3DArray),
            (FETCH_BALL_BOXES_TOPIC, TARGET_BOXES_TOPIC, Detection2DArray),
        ):
            probe = self.node.create_subscription(kind, fetch_topic, lambda _: None, 1)
            resolved = probe.topic_name
            self.node.destroy_subscription(probe)
            if resolved != target_topic:
                problems.append(f"{fetch_topic} -> {resolved} (expected {target_topic})")
        if problems:
            self.node.get_logger().error(
                "The fetch behaviours are NOT reading the chosen brick: "
                + "; ".join(problems)
                + ". Start the tree with launch_object_choice.launch.py, which "
                "remaps them; without it the robot drives to the remembered "
                "brick position with no live correction."
            )
        else:
            self.node.get_logger().info(
                "fetch behaviours read the chosen brick through the relay"
            )
        return not problems

    def _on_detections(self, msg):
        if self.target is None:
            return
        kept = [d for d in msg.detections if _label(d) == self.target]
        if not kept:
            return
        out = Detection3DArray()
        out.header = msg.header
        out.detections = kept
        self._detections_pub.publish(out)

    def _on_boxes(self, msg):
        out = Detection2DArray()
        out.header = msg.header
        if self.target is not None:
            out.detections = [d for d in msg.detections if _label(d) == self.target]
        # Forwarded even when empty: the centring turn has to see the brick
        # leave the frame, not keep acting on its last box.
        self._boxes_pub.publish(out)


class StatePublisher:
    """
    Publish a phase label every time a phase is entered.

    No de-duplication, unlike the fetch tree's publisher: each label belongs to
    one behaviour that is ticked once per trial, so remembering the last label
    per instance would silence every label from the second trial on.
    """

    def __init__(self, node, topic=STATE_TOPIC):
        self._publisher = node.create_publisher(String, topic, 10)

    def publish(self, phase):
        """Publish `phase`."""
        self._publisher.publish(String(data=str(phase)))


class TrialRecordPublisher:
    """Publish one JSON record per trial."""

    def __init__(self, node, topic=TRIAL_TOPIC):
        self._publisher = node.create_publisher(String, topic, 10)

    def publish(self, record):
        """Publish `record` as one line of JSON."""
        self._publisher.publish(String(data=json.dumps(record, sort_keys=True)))


class GoSignal:
    """Count the experimenter's go messages."""

    def __init__(self, node, topic=GO_TOPIC):
        self.count = 0
        self._subscription = node.create_subscription(Empty, topic, self._callback, 10)

    def _callback(self, _msg):
        self.count += 1
