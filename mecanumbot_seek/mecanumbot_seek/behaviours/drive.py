"""
Ticking the SEEKING circuit, and publishing what it says.

One leaf, which runs beside everything else in the seek parallel and never
succeeds. It advances both layers of the circuit by the elapsed time, feeds it
the one event nothing else is placed to feed it -- whether the object is in
sight right now -- and publishes the state.

It has to be its own behaviour rather than something folded into the monitor,
because the circuit must keep running while the robot is driving to a place, not
only while it is watching for something. A decay that stops when the robot stops
looking is not a decay.
"""

import py_trees

from mecanumbot_seek.behaviours.ros_interfaces import (
    ObjectDetectionTracker,
    SeekingStatePublisher,
)
from mecanumbot_seek.defaults import constant, register_param_keys


class TickSeekingDrive(py_trees.behaviour.Behaviour):
    """
    Advance the SEEKING circuit every tick; always RUNNING.

    Always RUNNING is the point: in a `SuccessOnSelected` parallel this branch
    neither ends the parallel nor can fail it, so the circuit runs for exactly
    as long as the episode does.
    """

    def __init__(self, name="TickSeekingDrive"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        register_param_keys(self.blackboard)
        self.blackboard.register_key(
            key="seek_drive", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="seek_object_class", access=py_trees.common.Access.READ
        )

    def setup(self, **kwargs):
        """Build the detector gate and the state publisher."""
        self.node = kwargs["node"]
        self.detections = ObjectDetectionTracker(
            self.node, timeout=constant(self.blackboard, "seek_detection_timeout")
        )
        self.state = SeekingStatePublisher(self.node)
        self._last_tick = None
        self.logger.info(f"{self.name}: Setup complete")
        return True

    def initialise(self):
        """Start the clock fresh, so a restarted episode does not decay a gap."""
        self._last_tick = self.node.get_clock().now()

    def update(self):
        """Step the circuit by the elapsed time and publish it."""
        drive = self.blackboard.seek_drive
        now = self.node.get_clock().now()
        dt = (now - self._last_tick).nanoseconds / 1e9
        self._last_tick = now

        # The detector gate takes its threshold from the circuit, so the more
        # strongly the robot is seeking the weaker a detection it will act on.
        # That is incentive salience, and this is where the loop closes: a
        # sighting raises arousal, which lowers the threshold, which makes the
        # next marginal frame count as a sighting too.
        self.detections.class_id = self.blackboard.seek_object_class or None
        if self.detections.visible(drive.detection_threshold()):
            drive.sight(self.detections.hypothesis.score)
        else:
            drive.out_of_sight()

        drive.step(dt)
        self.state.publish(drive)
        self.feedback_message = (
            f"{drive.phase}: arousal {drive.arousal:.2f}, "
            f"expectancy {drive.expectancy:.2f}"
        )
        return py_trees.common.Status.RUNNING
