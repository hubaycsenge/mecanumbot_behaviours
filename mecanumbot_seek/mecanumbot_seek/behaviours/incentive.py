"""
Being told what to look for, and noticing it.

Two behaviours, and they are the two halves of how a SEEKING episode is engaged.
`AcquireSeekTarget` takes the *memory* -- what the server found in the T1 point
cloud, transformed into the robot's own map frame -- and turns it into the
blackboard keys the rest of the tree drives to. `WatchForObject` takes the
*perception*: it watches the live detections and succeeds the moment the object
is actually in view, which is what interrupts everything else the robot was
doing.

`WatchForObject` is the branch that makes the tree work. It runs in parallel
with the whole goal-directed search, so wherever the robot happens to be in that
search -- driving to the remembered place, spinning at it, three rings out into
the expanding search -- seeing the object ends the parallel and the tree drops
into the approach. That is why the robot "stops along the way if it finds the
item": nothing has to check for it at each step, because one branch is doing
nothing else.
"""

import py_trees

from mecanumbot_movement_behaviours.ros_interfaces import HEAD_LEVEL, AccessoryCommander

from mecanumbot_seek.behaviours.ros_interfaces import (
    ObjectDetectionTracker,
    SeekRequestTracker,
    SeekTargetTracker,
)
from mecanumbot_seek.defaults import constant, register_param_keys


class AcquireSeekTarget(py_trees.behaviour.Behaviour):
    """
    Wait for a target from the server, and put it on the blackboard.

    SUCCESS once both are known: what to look for (`seek/request`) and where the
    server last saw it (`seek/target`). FAILURE if the server has said nothing
    within `seek_target_timeout` -- a robot standing still waiting for an answer
    that is not coming is not a state this tree should be able to get stuck in,
    the same reason the ostensive tree's attention watch times out.

    The position is written to **two** keys, and the difference matters for the
    rest of the episode. `seek_target_position` is what `SeekApproach` drives to
    and is updated as the robot learns more; `seek_last_known` is the memory the
    search rings are centred on and is not.
    """

    def __init__(self, name="AcquireSeekTarget", timeout=None):
        super().__init__(name)
        self.timeout = timeout
        self.blackboard = self.attach_blackboard_client(name=name)
        register_param_keys(self.blackboard)
        for key in (
            "seek_object_class",
            "seek_target_position",
            "seek_last_known",
        ):
            self.blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(
            key="seek_drive", access=py_trees.common.Access.READ
        )

    def setup(self, **kwargs):
        """Subscribe to the request and the server's answer."""
        self.node = kwargs["node"]
        self.timeout = float(
            self.timeout
            if self.timeout is not None
            else constant(self.blackboard, "seek_target_timeout")
        )
        self.request = SeekRequestTracker(self.node)
        self.target = SeekTargetTracker(self.node)
        self.accessories = AccessoryCommander(self.node)
        self.logger.info(f"{self.name}: Setup complete")
        return True

    def initialise(self):
        """Start the clock, and put the head where the camera can see the floor."""
        self._start = self.node.get_clock().now()
        self._announced = False
        # Level rather than the lifted seeking pose the social trees use: the
        # thing being sought is an object on the floor, not a person's face.
        self.accessories.look(HEAD_LEVEL)

    def update(self):
        """Wait for a labelled target, then hand it to the rest of the tree."""
        object_class = self.request.object_class
        if object_class is None:
            self.feedback_message = "waiting to be told what to look for"
            return self._or_timeout()

        if not self._announced:
            self.node.get_logger().info(f"{self.name}: asked to find '{object_class}'")
            self._announced = True

        hypothesis = self.target.hypothesis
        if hypothesis is None:
            self.feedback_message = f"waiting for the server to place '{object_class}'"
            return self._or_timeout()

        self.blackboard.seek_object_class = object_class
        self.blackboard.seek_target_position = hypothesis.position
        self.blackboard.seek_last_known = hypothesis.position

        # Engaging the circuit: being told where something is is an incentive,
        # and a weaker one than seeing it. `reset` here rather than in the
        # loader, because this is where the episode's object becomes known.
        drive = self.blackboard.seek_drive
        drive.reset(object_class)
        drive.cue(hypothesis.score)

        self.node.get_logger().info(
            f"{self.name}: '{object_class}' was at x={hypothesis.position.x:.2f} "
            f"y={hypothesis.position.y:.2f} (server confidence "
            f"{hypothesis.score:.2f}); expectancy {drive.expectancy:.2f}"
        )
        return py_trees.common.Status.SUCCESS

    def _or_timeout(self):
        elapsed = (self.node.get_clock().now() - self._start).nanoseconds / 1e9
        if elapsed > self.timeout:
            self.node.get_logger().info(
                f"{self.name}: no target within {self.timeout:.0f} s, giving up"
            )
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING


class WatchForObject(py_trees.behaviour.Behaviour):
    """
    Watch for the object; SUCCESS once it has been in sight long enough.

    The dwell is short but not zero. One frame of a marginal detection is not a
    reason to abandon a search leg the robot is halfway through, and at a low
    detection threshold -- which is what a strongly seeking robot runs at -- a
    single frame is exactly what noise looks like. Holding the sighting for
    `seek_sighting_dwell` costs a fraction of a second and removes almost all of
    that.

    Never FAILURE. Running as the selected child of a `SuccessOnSelected`
    parallel, a failure here would fail the whole seek, and "I have not seen it
    yet" is not a reason to stop looking -- extinction is, and that belongs to
    the search branch.
    """

    def __init__(self, name="WatchForObject", dwell=None):
        super().__init__(name)
        self.dwell = dwell
        self.blackboard = self.attach_blackboard_client(name=name)
        register_param_keys(self.blackboard)
        self.blackboard.register_key(
            key="seek_drive", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="seek_object_class", access=py_trees.common.Access.READ
        )
        for key in ("seek_sighted_position", "seek_target_position"):
            self.blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        """Subscribe to the live detections."""
        self.node = kwargs["node"]
        self.dwell = float(
            self.dwell
            if self.dwell is not None
            else constant(self.blackboard, "seek_sighting_dwell")
        )
        self.detections = ObjectDetectionTracker(
            self.node, timeout=constant(self.blackboard, "seek_detection_timeout")
        )
        self.logger.info(f"{self.name}: Setup complete")
        return True

    def initialise(self):
        """Forget any sighting from a previous episode."""
        self._sighted_since = None

    def update(self):
        """Succeed once the object has been continuously visible for the dwell."""
        drive = self.blackboard.seek_drive
        self.detections.class_id = self.blackboard.seek_object_class or None
        threshold = drive.detection_threshold()

        if not self.detections.visible(threshold):
            if self._sighted_since is not None:
                self.node.get_logger().info(f"{self.name}: lost sight of it again")
            self._sighted_since = None
            self.feedback_message = f"nothing at or above {threshold:.2f}"
            return py_trees.common.Status.RUNNING

        now = self.node.get_clock().now()
        if self._sighted_since is None:
            self._sighted_since = now
            self.node.get_logger().info(
                f"{self.name}: possible sighting, score "
                f"{self.detections.hypothesis.score:.2f} >= {threshold:.2f}"
            )

        held = (now - self._sighted_since).nanoseconds / 1e9
        if held < self.dwell:
            self.feedback_message = f"held for {held:.1f}/{self.dwell:.1f} s"
            return py_trees.common.Status.RUNNING

        position = self.detections.hypothesis.position
        self.blackboard.seek_sighted_position = position
        # The approach drives to `seek_target_position`, so pointing it at what
        # the robot can see replaces the server's memory with the live view --
        # which is the more recent of the two and, unlike the memory, moves.
        self.blackboard.seek_target_position = position
        self.node.get_logger().info(
            f"{self.name}: '{self.blackboard.seek_object_class}' in sight at "
            f"x={position.x:.2f} y={position.y:.2f}"
        )
        return py_trees.common.Status.SUCCESS
