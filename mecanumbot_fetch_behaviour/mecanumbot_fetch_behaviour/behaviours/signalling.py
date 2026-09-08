"""
Saying out loud what phase of the game the robot is in.

One behaviour, and it exists for the recording rather than for the robot. A
fetch trial is watched from outside -- a camera on a tripod, a person with a
clipboard -- and afterwards somebody has to line up what the robot did with what
the person did. `/mecanumbot/fetch/state` is that alignment: one label per
transition, on a `std_msgs/String`.

A String and not a new message type on purpose. `mecanumbot_msgs` is
deliberately small and is for data no standard type covers; a phase name is a
label, and the day it needs a timestamp it already has a header's worth of one
from the recording.
"""

import py_trees

from mecanumbot_fetch_behaviour.behaviours.ros_interfaces import FetchStatePublisher


class AnnouncePhase(py_trees.behaviour.Behaviour):
    """
    Publish one phase label, and return SUCCESS.

    Placed as the first child of each phase's sequence, so the label goes out
    when the phase is entered rather than when it ends. Always SUCCESS: it is a
    note in the margin and must never be able to change what the tree does.
    """

    def __init__(self, name, phase):
        super().__init__(name)
        self.phase = phase

    def setup(self, **kwargs):
        """Build the state publisher."""
        self.node = kwargs["node"]
        self.state = FetchStatePublisher(self.node)
        self.logger.info(f"{self.name}: Setup complete")
        return True

    def update(self):
        """Announce the phase this behaviour marks."""
        self.state.publish(self.phase)
        self.feedback_message = self.phase
        return py_trees.common.Status.SUCCESS
