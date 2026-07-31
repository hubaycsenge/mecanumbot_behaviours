"""Behaviours for the "where did my human go" recovery patrol."""

import py_trees

from mecanumbot_leading_behaviour.behaviours.geometry import (
    closest_checkpoint_index,
    path_progress_sign,
)
from mecanumbot_leading_behaviour.behaviours.ros_interfaces import (
    AccessoryCommander,
    HEAD_SEEK,
    PeopleTracker,
    RobotPoseTracker,
    VelocityCommander,
)

SEARCH_BACKWARDS = -1  # back down the route, where the human was last following
SEARCH_FORWARDS = 1


class WaitForPerson(py_trees.behaviour.Behaviour):
    """SUCCESS as soon as somebody is visible again.

    Runs as the interrupt branch of the recovery parallel: the patrol keeps
    searching in the other branch until this one succeeds. The head is lifted
    for the whole wait -- it reads as the robot seeking contact, and it gives the
    pose detector a full-body view instead of a pair of knees.

    On success it also records which way along the route the person turned up,
    so a later patrol starts searching in that direction.
    """

    def __init__(self, name="WaitForPerson", sight_timeout=1.0):
        super().__init__(name)
        self.sight_timeout = float(sight_timeout)

        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            "Dog_checkpoints", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            "patrol_direction", access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            "patrol_initialized", access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.pose = RobotPoseTracker(self.node)
        self.people = PeopleTracker(self.node, self.sight_timeout)
        self.velocity = VelocityCommander(self.node)
        self.accessories = AccessoryCommander(self.node)
        self.logger.info(f"{self.name}: Setup complete")

    def initialise(self):
        self.accessories.look(HEAD_SEEK)
        self.node.get_logger().info(f"{self.name}: watching for a person, head lifted")

    def update(self):
        if not self.people.has_fresh_detection():
            self.feedback_message = "nobody visible yet"
            return py_trees.common.Status.RUNNING

        self.velocity.stop()  # the patrol branch is about to be cancelled
        self._remember_search_direction()
        self.node.get_logger().info(
            f"{self.name}: person found, interrupting the search"
        )
        return py_trees.common.Status.SUCCESS

    def _remember_search_direction(self):
        """Note whether the person showed up ahead of or behind the robot."""
        # Next patrol re-snaps to the checkpoint nearest wherever we end up.
        self.blackboard.patrol_initialized = False

        checkpoints = self.blackboard.Dog_checkpoints
        person_pose = self.people.last_seen_pose
        if self.pose.position is None or person_pose is None or not checkpoints:
            self.blackboard.patrol_direction = SEARCH_BACKWARDS
            self.node.get_logger().warn(
                f"{self.name}: no pose or checkpoints, searching backwards by default"
            )
            return

        self.blackboard.patrol_direction = path_progress_sign(
            checkpoints, self.pose.position, person_pose.position
        )
        self.node.get_logger().info(
            f"{self.name}: person is "
            f"{'ahead' if self.blackboard.patrol_direction > 0 else 'behind'} on the route"
        )


class ManageSearchCheckpoint(py_trees.behaviour.Behaviour):
    """Walk the patrol index along the checkpoint list.

    The first run (and every run after a person was found) snaps to the
    checkpoint nearest the robot, then each run steps one checkpoint in
    `patrol_direction`, reversing at either end of the route.
    """

    def __init__(self, name="ManageSearchCheckpoint"):
        super().__init__(name)

        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            "Dog_checkpoints", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            "Dog_max_checkpoint", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            "patrol_current_checkpoint", access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            "patrol_direction", access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            "patrol_initialized", access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.pose = RobotPoseTracker(self.node)
        self.logger.info(f"{self.name}: Setup complete")

    def update(self):
        if self.pose.position is None:
            self.feedback_message = "waiting for AMCL pose"
            return py_trees.common.Status.RUNNING

        if not self.blackboard.patrol_initialized:
            index = closest_checkpoint_index(
                self.blackboard.Dog_checkpoints,
                self.pose.position.x,
                self.pose.position.y,
            )
            self.blackboard.patrol_current_checkpoint = index
            self.blackboard.patrol_initialized = True
            self.node.get_logger().info(
                f"{self.name}: search starts at checkpoint {index}"
            )
            return py_trees.common.Status.SUCCESS

        last_index = self.blackboard.Dog_max_checkpoint
        index = (
            self.blackboard.patrol_current_checkpoint + self.blackboard.patrol_direction
        )

        if index <= 0:
            index = 0
            self.blackboard.patrol_direction = SEARCH_FORWARDS
            self.node.get_logger().info(
                f"{self.name}: reached the start, searching forwards"
            )
        elif index >= last_index:
            index = last_index
            self.blackboard.patrol_direction = SEARCH_BACKWARDS
            self.node.get_logger().info(
                f"{self.name}: reached the end, searching backwards"
            )

        self.blackboard.patrol_current_checkpoint = index
        self.node.get_logger().info(f"{self.name}: next search checkpoint is {index}")
        return py_trees.common.Status.SUCCESS
