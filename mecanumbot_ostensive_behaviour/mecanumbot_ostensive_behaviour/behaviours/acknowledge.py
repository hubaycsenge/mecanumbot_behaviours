"""
Showing the addressee that their bid for attention landed.

Ostensive communication is two-sided: the person makes their signal deliberately
visible, and the robot has to make its uptake visible back, or the person cannot
tell whether to go on and point at something. Between locking on and reading the
cue there is otherwise nothing to see -- the robot has already turned to face
them, and it turned to face them while they were still waving.

The nod is done with the neck the camera is mounted on, which is the only
expressive channel this condition uses: no LEDs and no gripper, so the
acknowledgement cannot be confused with the light or gesture signalling the
leading conditions in `mecanumbot_leading_behaviour` are comparing.
"""

import py_trees

from mecanumbot_ostensive_behaviour.behaviours.blackboard_managers import (
    register_param_keys,
)
from mecanumbot_ostensive_behaviour.behaviours.ros_interfaces import (
    AccessoryCommander,
    seconds,
)


class NodAcknowledge(py_trees.behaviour.Behaviour):
    """
    Nod the camera head through the configured sequence, then return SUCCESS.

    `ack_neck_seq` is a list of neck tilt positions and `ack_neck_times` how long
    to hold each of them. The last entry decides where the head is left, so it
    should be the seeking pose the rest of the exchange expects.
    """

    def __init__(self, name="NodAcknowledge"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        register_param_keys(self.blackboard)

    def setup(self, **kwargs):
        """Build the neck commander."""
        self.node = kwargs["node"]
        self.accessories = AccessoryCommander(self.node)
        self.logger.info(f"{self.name}: Setup complete")
        return True

    def initialise(self):
        """Start before the first step of the nod."""
        self._index = -1
        self._step_started = None
        self._steps = min(
            len(self.blackboard.ack_neck_seq), len(self.blackboard.ack_neck_times)
        )

    def update(self):
        """Hold each position for its time, then move on to the next."""
        now = seconds(self.node)

        if self._index >= 0:
            held = now - self._step_started
            if held < self.blackboard.ack_neck_times[self._index]:
                self.feedback_message = f"nod step {self._index + 1}/{self._steps}"
                return py_trees.common.Status.RUNNING

        self._index += 1
        if self._index >= self._steps:
            self.node.get_logger().info(f"{self.name}: acknowledged the addressee")
            return py_trees.common.Status.SUCCESS

        self.accessories.send(self.blackboard.ack_neck_seq[self._index])
        self._step_started = now
        return py_trees.common.Status.RUNNING
