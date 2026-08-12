"""Dog-inspired gesture and following-check behaviours."""

import py_trees

from mecanumbot_leading_behaviour.behaviours.constants import (
    constant,
    register_param_keys,
    resolve,
)
from mecanumbot_leading_behaviour.behaviours.geometry import (
    closest_checkpoint_index,
    distance_xy,
    route_progress,
)
from mecanumbot_leading_behaviour.behaviours.ros_interfaces import (
    AccessoryCommander,
    PeopleTracker,
    RobotPoseTracker,
    SubjectPoseTracker,
    duration,
)

# Gesture mode -> the pair of blackboard keys holding its commands and timings.
GESTURE_KEYS = {
    "indicate_target": ("Dog_indicate_target_seq", "Dog_indicate_target_times"),
    "catch_attention": ("Dog_catch_attention_seq", "Dog_catch_attention_times"),
    "thank": ("Dog_thank_seq", "Dog_thank_times"),
}


class DogBehaviourSequence(py_trees.behaviour.Behaviour):
    """Play one timed accessory-gesture sequence from the blackboard."""

    def __init__(self, name="DogBehaviourSequence", mode="indicate_target"):
        super().__init__(name)
        if mode not in GESTURE_KEYS:
            raise ValueError(
                f"unknown Dog behaviour mode '{mode}', expected one of {sorted(GESTURE_KEYS)}"
            )
        self.mode = mode
        self.seq_key, self.times_key = GESTURE_KEYS[mode]

        self.blackboard = self.attach_blackboard_client(name=name)
        for seq_key, times_key in GESTURE_KEYS.values():
            self.blackboard.register_key(
                key=seq_key, access=py_trees.common.Access.READ
            )
            self.blackboard.register_key(
                key=times_key, access=py_trees.common.Access.READ
            )

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.accessories = AccessoryCommander(self.node)
        self.logger.info(f"{self.name}: Setup complete")

    def initialise(self):
        self.index = 0
        self.next_send_time = None
        self.gesture = getattr(self.blackboard, self.seq_key)
        self.delays = getattr(self.blackboard, self.times_key)

    def update(self):
        if not self.gesture or not self.delays:
            self.feedback_message = f"no commands configured for '{self.mode}'"
            return py_trees.common.Status.RUNNING

        now = self.node.get_clock().now()

        if self.next_send_time is None:
            self.node.get_logger().info(f"{self.name}: gesture '{self.mode}' started")
            self._send_command(now)
            return py_trees.common.Status.RUNNING

        if now < self.next_send_time:
            self.feedback_message = f"step {self.index}/{len(self.gesture)}"
            return py_trees.common.Status.RUNNING

        if self.index < len(self.gesture):
            self._send_command(now)
            return py_trees.common.Status.RUNNING

        self.node.get_logger().info(f"{self.name}: gesture '{self.mode}' completed")
        return py_trees.common.Status.SUCCESS

    def _send_command(self, now):
        cmd = self.gesture[self.index]
        self.accessories.send(cmd.n_pos, cmd.gl_pos, cmd.gr_pos)
        self.next_send_time = now + duration(self.delays[self.index])
        self.index += 1


class DogCheckFollowing(py_trees.behaviour.Behaviour):
    """The dog looking over its shoulder: is the human still coming along?

    FAILURE (which sends the tree into recovery) when the human dropped further
    behind than `Dog_following_max_threshold`, or when their pose went stale
    `Dog_max_wander_allowed` checks in a row.
    """

    def __init__(self, name="DogCheckFollowing"):
        super().__init__(name)

        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            "visibility_time_threshold", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            "Dog_following_max_threshold", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            "Dog_max_wander_allowed", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            "last_distance", access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.pose = RobotPoseTracker(self.node)
        self.subject = SubjectPoseTracker(self.node)
        self.wanders = 0
        self.logger.info(f"{self.name}: Setup complete")

    def update(self):
        if self.pose.position is None or self.subject.position is None:
            self.feedback_message = "waiting for robot and subject poses"
            return py_trees.common.Status.RUNNING

        age = self.subject.age
        if age is not None and age > self.blackboard.visibility_time_threshold:
            self.wanders += 1
            if self.wanders > self.blackboard.Dog_max_wander_allowed:
                self.node.get_logger().info(
                    f"{self.name}: subject out of sight {self.wanders} checks in a row, lost"
                )
                self.wanders = 0
                return py_trees.common.Status.FAILURE

        distance = distance_xy(self.pose.position, self.subject.position)
        max_distance = self.blackboard.Dog_following_max_threshold
        self.blackboard.last_distance = distance

        if distance > max_distance:
            self.node.get_logger().info(
                f"{self.name}: subject {distance:.2f} m away, more than the "
                f"{max_distance:.2f} m allowed"
            )
            return py_trees.common.Status.FAILURE

        self.wanders = 0
        self.node.get_logger().info(
            f"{self.name}: following at {distance:.2f} m (max {max_distance:.2f} m)"
        )
        return py_trees.common.Status.SUCCESS


class DogResumeLeading(py_trees.behaviour.Behaviour):
    """Pick the checkpoint to lead to now that the human has been found again.

    Take the checkpoint nearest the robot and ask whether the human is already
    past it: if they are, that stretch of the route is walked and leading
    resumes at the checkpoint after it, otherwise the pair still has to get
    there. Without this the robot would carry on towards whichever checkpoint it
    was heading for when it lost the human, which the search may well have left
    behind.
    """

    # How far past a checkpoint (as a fraction of the stretch to the next one)
    # the human has to be before it counts as walked is `resume_passed_margin`,
    # which keeps somebody standing right on a checkpoint from flipping the
    # decision back and forth.

    def __init__(self, name="DogResumeLeading", sight_timeout=None):
        super().__init__(name)
        self.sight_timeout = sight_timeout

        self.blackboard = self.attach_blackboard_client(name=name)
        register_param_keys(self.blackboard, "sight_timeout", "resume_passed_margin")
        self.blackboard.register_key(
            "Dog_checkpoints", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            "Dog_max_checkpoint", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            "Dog_current_checkpoint", access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.sight_timeout = float(
            resolve(self.sight_timeout, self.blackboard, "sight_timeout")
        )
        self.pose = RobotPoseTracker(self.node)
        self.people = PeopleTracker(self.node, self.sight_timeout)
        self.logger.info(f"{self.name}: Setup complete")

    def update(self):
        if self.pose.position is None:
            self.feedback_message = "waiting for AMCL pose"
            return py_trees.common.Status.RUNNING

        checkpoints = self.blackboard.Dog_checkpoints
        nearest = closest_checkpoint_index(
            checkpoints, self.pose.position.x, self.pose.position.y
        )
        index, reason = self._resume_index(checkpoints, nearest)
        index = max(0, min(index, self.blackboard.Dog_max_checkpoint))

        self.blackboard.Dog_current_checkpoint = index
        self.feedback_message = f"leading on to checkpoint {index}"
        self.node.get_logger().info(
            f"{self.name}: {reason}, leading on to checkpoint {index}"
        )
        return py_trees.common.Status.SUCCESS

    def _resume_index(self, checkpoints, nearest):
        """Checkpoint to head for, and why -- see the class docstring."""
        person_pose = self.people.last_seen_pose
        if person_pose is None or not checkpoints:
            return nearest, "nobody to place on the route"

        progress = route_progress(checkpoints, person_pose.position)
        margin = constant(self.blackboard, "resume_passed_margin")
        if progress > nearest + margin:
            return (
                nearest + 1,
                f"human is past checkpoint {nearest} (at {progress:.2f})",
            )
        return (
            nearest,
            f"human has not reached checkpoint {nearest} yet (at {progress:.2f})",
        )
