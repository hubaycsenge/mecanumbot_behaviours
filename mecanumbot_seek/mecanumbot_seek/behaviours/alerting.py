"""
Telling a person about something the robot found and cannot have.

A seeking episode that ends with the object located but out of reach is not a
failure -- the robot knows something a person would want to know, and the
apparatus for saying so is here.

## What the robot does, and why that

The gesture is **orienting alternation**: the robot faces the object, then faces
the person, then the object again, a few times. In dogs this is the canonical
*showing* behaviour, and gaze alternation between a human and a referent is its
core. The robot has no eyes, but it has a neck and a body, and orienting is the
functional analogue -- the wiki's `ostensive-signalling` page makes exactly that
argument for the outbound direction, which is the direction almost nothing in
the corpus tests.

It comes for free from the movement library, and better than it would have if
written here: `TurnToward` picks the lifted "seeking" head pose for a human
target and the level pose for a place, so the neck rises to address the person
and drops to indicate the object without a line of code saying so.

> **This is apparatus for a question, not an answer to it.** The one piece of
> evidence the corpus holds on robot-produced ostension is negative: infants
> followed a robot's gaze but did not learn from it, while following *and*
> learning from an equivalent human. Whether a person reads this alternation as
> the robot showing them something is the thing to measure, and the
> `SeekAlert` message exists so a trial can be scored against what the robot
> actually did -- including how many alternations it managed before the person
> walked off.

## What is here

`CheckObjectGraspable` is the condition that diverts the tree: it fails when the
object is out of the grasp band or the robot never got near it, recording *why*.
`RecordUnreachable` absorbs a failed approach into the same channel.
`SomeoneToTell` and `AnnounceUnreachable` are the ends of the alert; the
alternation itself is built in the tree out of `TurnToward`, because it is a
shape rather than a behaviour.
"""

import py_trees

from mecanumbot_movement_behaviours.defaults import (
    register_param_keys as register_movement_keys,
)
from mecanumbot_movement_behaviours.geometry import distance_xy
from mecanumbot_movement_behaviours.ros_interfaces import (
    PeopleTracker,
    RobotPoseTracker,
)

from mecanumbot_seek import reachability
from mecanumbot_seek.behaviours.ros_interfaces import (
    AlertPublisher,
    LedSignaller,
    ObjectDetectionTracker,
)
from mecanumbot_seek.defaults import constant, register_param_keys


class CheckObjectGraspable(py_trees.behaviour.Behaviour):
    """
    SUCCESS when the robot may go ahead and grip; FAILURE with a reason if not.

    The gate between the approach and the grasp, and the branch point between
    "pick it up" and "go and tell someone". Height is judged before distance --
    a robot parked against a table is close enough and still cannot have the mug
    on it, and calling that a distance problem sends a person looking for an
    obstacle that is not there.
    """

    def __init__(self, name="CheckObjectGraspable"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        register_param_keys(self.blackboard)
        self.blackboard.register_key(
            key="seek_drive", access=py_trees.common.Access.READ
        )
        for key in ("seek_object_class", "seek_target_position"):
            self.blackboard.register_key(key=key, access=py_trees.common.Access.READ)
        for key in ("seek_unreachable_reason", "seek_object_height"):
            self.blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        """Build the pose tracker and the detector."""
        self.node = kwargs["node"]
        self.height_min = float(constant(self.blackboard, "seek_grasp_height_min"))
        self.height_max = float(constant(self.blackboard, "seek_grasp_height_max"))
        self.grasp_distance = float(constant(self.blackboard, "seek_grasp_distance"))
        self.pose = RobotPoseTracker(self.node)
        self.detections = ObjectDetectionTracker(
            self.node, timeout=constant(self.blackboard, "seek_detection_timeout")
        )
        self.logger.info(
            f"{self.name}: Setup complete; grabbers reach "
            f"{reachability.grasp_band(self.height_min, self.height_max)}"
        )
        return True

    def update(self):
        """Assess the object against the robot's own geometry."""
        target = self.blackboard.seek_target_position
        if target is None:
            self.blackboard.seek_unreachable_reason = reachability.LOST
            self.node.get_logger().info(f"{self.name}: nothing to grip")
            return py_trees.common.Status.FAILURE

        height = self._height(target)
        self.blackboard.seek_object_height = height

        distance = None
        if self.pose.pose is not None:
            distance = distance_xy(self.pose.pose.position, target)

        reason = reachability.assess(
            height=height,
            distance=distance,
            grasp_height_min=self.height_min,
            grasp_height_max=self.height_max,
            grasp_distance=self.grasp_distance,
        )
        if reachability.reachable(reason):
            self.node.get_logger().info(
                f"{self.name}: graspable"
                + (f" at {height:.2f} m" if height is not None else "")
            )
            return py_trees.common.Status.SUCCESS

        self.blackboard.seek_unreachable_reason = reason
        self.node.get_logger().info(
            f"{self.name}: not graspable -- "
            + reachability.describe(
                reason, self.blackboard.seek_object_class, height
            )
        )
        return py_trees.common.Status.FAILURE

    def _height(self, target):
        """
        Height of the object above the floor, from the freshest source.

        A live detection wins over the server's memory, because the robot is
        standing next to the thing by now. `None` where neither placed it in z,
        which `assess` reads as "do not refuse on this basis".
        """
        self.detections.class_id = self.blackboard.seek_object_class or None
        drive = self.blackboard.seek_drive
        if self.detections.visible(drive.detection_threshold()):
            return float(self.detections.hypothesis.position.z)
        return None if target is None else float(target.z)


class RecordUnreachable(py_trees.behaviour.Behaviour):
    """
    Record why the object could not be had, and return SUCCESS.

    Sits as the fallback under the approach, so a drive that fails becomes a
    reason rather than a failed episode. SUCCESS on purpose: the point is to
    carry on into the alert, not to end the run.
    """

    def __init__(self, name="RecordUnreachable", reason=reachability.NO_ROUTE):
        super().__init__(name)
        self.reason = reason
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            key="seek_object_class", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="seek_unreachable_reason", access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs):
        """Keep the node handle for logging."""
        self.node = kwargs["node"]
        self.logger.info(f"{self.name}: Setup complete")
        return True

    def update(self):
        """Write the reason down and carry on."""
        self.blackboard.seek_unreachable_reason = self.reason
        self.node.get_logger().info(
            f"{self.name}: "
            + reachability.describe(self.reason, self.blackboard.seek_object_class)
        )
        return py_trees.common.Status.SUCCESS


class SomeoneToTell(py_trees.behaviour.Behaviour):
    """
    SUCCESS while somebody is visible to be told, and records where they are.

    The condition half of finding an audience; the scan that runs when it fails
    is the movement library's `FindPeople`.

    The position it records is **for the alert message, not for the turns**.
    `TurnToward(SUBJECT)` and `Approach(SUBJECT)` each hold their own
    `PeopleTracker` and take the person from the live detection, which is right
    -- a person the robot is addressing moves, and a position frozen at the
    moment they were first noticed would have the robot showing the object to
    where they used to stand. What goes on the blackboard is where they were
    when the robot decided to tell them, which is what the record wants.
    """

    def __init__(self, name="SomeoneToTell"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        register_param_keys(self.blackboard)
        register_movement_keys(self.blackboard)
        self.blackboard.register_key(
            key="seek_person_position", access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs):
        """Subscribe to the fused people detections."""
        self.node = kwargs["node"]
        self.sight_timeout = float(constant(self.blackboard, "sight_timeout"))
        self.people = PeopleTracker(self.node, self.sight_timeout)
        self.logger.info(f"{self.name}: Setup complete")
        return True

    def update(self):
        """Succeed as soon as there is somebody to address."""
        if not self.people.has_fresh_detection():
            self.feedback_message = "nobody to tell"
            return py_trees.common.Status.FAILURE
        position = self.people.last_seen_pose.position
        self.blackboard.seek_person_position = position
        self.node.get_logger().info(
            f"{self.name}: somebody at x={position.x:.2f} y={position.y:.2f}"
        )
        return py_trees.common.Status.SUCCESS


class CountAlternation(py_trees.behaviour.Behaviour):
    """
    Count one completed alternation of the showing gesture.

    A leaf rather than a counter inside the turns, because what is worth
    recording is how many times the robot got all the way round -- object,
    person, object -- and a person who walks off halfway through has been shown
    fewer times than one who stayed. That number goes in the alert message and
    is the thing a trial is scored on.
    """

    def __init__(self, name="CountAlternation"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        # WRITE covers reading it back too, which this behaviour has to do.
        self.blackboard.register_key(
            key="seek_alternations", access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs):
        """Keep the node handle for logging."""
        self.node = kwargs["node"]
        self.logger.info(f"{self.name}: Setup complete")
        return True

    def update(self):
        """Increment the count and carry on."""
        count = int(self.blackboard.seek_alternations) + 1
        self.blackboard.seek_alternations = count
        self.feedback_message = f"alternation {count}"
        return py_trees.common.Status.SUCCESS


class AnnounceUnreachable(py_trees.behaviour.Behaviour):
    """
    Publish the alert, and flash the LEDs while doing it.

    The last leaf of the alert. It always succeeds, including when nobody was
    found to tell -- the record has to distinguish "told somebody" from "had
    nobody to tell", and both are outcomes worth having in the bag rather than
    reasons to fail an episode.

    The LED call is fire-and-forget and skipped entirely when the service is not
    up, so the tree runs on a robot with no LED controller.
    """

    def __init__(self, name="AnnounceUnreachable"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        register_param_keys(self.blackboard)
        for key in (
            "seek_object_class",
            "seek_target_position",
            "seek_object_height",
            "seek_unreachable_reason",
            "seek_person_position",
            "seek_alternations",
        ):
            self.blackboard.register_key(key=key, access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        """Build the alert publisher and the LED client."""
        self.node = kwargs["node"]
        self.alerts = AlertPublisher(self.node)
        self.leds = LedSignaller(self.node)
        self.led_mode = int(constant(self.blackboard, "seek_alert_led_mode"))
        self.led_color = int(constant(self.blackboard, "seek_alert_led_color"))
        self.logger.info(f"{self.name}: Setup complete")
        return True

    def update(self):
        """Publish the alert and signal it."""
        reason = self.blackboard.seek_unreachable_reason or reachability.LOST
        object_class = self.blackboard.seek_object_class
        height = self.blackboard.seek_object_height
        person = self.blackboard.seek_person_position

        self.alerts.publish(
            object_class=object_class,
            position=self.blackboard.seek_target_position,
            height=height,
            reason=reason,
            person=person,
            alternations=int(self.blackboard.seek_alternations),
        )
        self.leds.flash(self.led_mode, self.led_color)

        line = reachability.describe(reason, object_class, height)
        if person is None:
            self.node.get_logger().warn(f"{self.name}: {line} -- and nobody to tell")
        else:
            self.node.get_logger().info(
                f"{self.name}: told somebody -- {line} "
                f"(after {self.blackboard.seek_alternations} alternation(s))"
            )
        return py_trees.common.Status.SUCCESS
