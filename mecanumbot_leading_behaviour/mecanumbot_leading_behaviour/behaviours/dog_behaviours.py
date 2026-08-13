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
from mecanumbot_leading_behaviour.behaviours.pacing import check_in_due
from mecanumbot_leading_behaviour.behaviours.ros_interfaces import (
    AccessoryCommander,
    FollowedSubjectTracker,
    HEAD_SEEK,
    PeopleTracker,
    RobotPoseTracker,
    duration,
    now_seconds,
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

    def __init__(self, name="DogCheckFollowing", sight_timeout=None):
        super().__init__(name)
        self.sight_timeout = sight_timeout

        self.blackboard = self.attach_blackboard_client(name=name)
        register_param_keys(self.blackboard, "sight_timeout")
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
        self.sight_timeout = float(
            resolve(self.sight_timeout, self.blackboard, "sight_timeout")
        )
        self.pose = RobotPoseTracker(self.node)
        self.subject = FollowedSubjectTracker(self.node, self.sight_timeout)
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


class DogCheckInDue(py_trees.behaviour.Behaviour):
    """Decide whether it is time to look back for the human.

    SUCCESS when a look back is due, FAILURE while leading may simply carry on.
    The tree wraps it in an inverter so that "not due" is the branch that lets
    the robot keep driving; `pacing.check_in_due()` holds the rule itself, and
    this behaviour only measures the world for it -- how long since the last
    look back, how many checkpoints since, how old the subject pose is and how
    far away they are.

    The clock starts the first time this is asked rather than when the constants
    are loaded, because leading begins with the robot face to face with its
    human: it has just caught their attention, so the interval is counted from
    there and the first look back comes a leg or two later, not immediately.
    """

    def __init__(self, name="DogCheckInDue", sight_timeout=None):
        super().__init__(name)
        self.sight_timeout = sight_timeout

        self.blackboard = self.attach_blackboard_client(name=name)
        register_param_keys(
            self.blackboard,
            "check_in_every_checkpoints",
            "check_in_interval",
            "sight_timeout",
        )
        for key in (
            "Dog_following_max_threshold",
            "visibility_time_threshold",
            "check_in_checkpoints_since",
        ):
            self.blackboard.register_key(key, access=py_trees.common.Access.READ)
        self.blackboard.register_key(
            "check_in_last_time", access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.sight_timeout = float(
            resolve(self.sight_timeout, self.blackboard, "sight_timeout")
        )
        self.pose = RobotPoseTracker(self.node)
        self.subject = FollowedSubjectTracker(self.node, self.sight_timeout)
        self.logger.info(f"{self.name}: Setup complete")

    def update(self):
        now = now_seconds(self.node)
        if not self.blackboard.check_in_last_time:
            self.blackboard.check_in_last_time = now

        due, reason = check_in_due(
            checkpoints_since=self.blackboard.check_in_checkpoints_since,
            seconds_since=now - self.blackboard.check_in_last_time,
            subject_age=self.subject.age,
            subject_distance=self._subject_distance(),
            every_checkpoints=constant(self.blackboard, "check_in_every_checkpoints"),
            interval=constant(self.blackboard, "check_in_interval"),
            follow_threshold=self.blackboard.Dog_following_max_threshold,
            stale_after=self.blackboard.visibility_time_threshold,
        )

        self.feedback_message = reason
        if due:
            self.node.get_logger().info(f"{self.name}: looking back -- {reason}")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

    def _subject_distance(self):
        if self.pose.position is None or self.subject.position is None:
            return None
        return distance_xy(self.pose.position, self.subject.position)


class DogWaitForCatchUp(py_trees.behaviour.Behaviour):
    """Stand still, head up, and give a lagging human a moment to catch up.

    A dog that has looked back and found its human trailing waits for them
    before it does anything else, and only goes back to fetch them when waiting
    did not help. SUCCESS as soon as they are within
    `Dog_following_max_threshold` again, FAILURE after `check_in_catch_up_timeout`
    seconds -- which is what sends the tree off to walk back to them.
    """

    def __init__(self, name="DogWaitForCatchUp", timeout=None, sight_timeout=None):
        super().__init__(name)
        self.timeout = timeout
        self.sight_timeout = sight_timeout

        self.blackboard = self.attach_blackboard_client(name=name)
        register_param_keys(
            self.blackboard, "check_in_catch_up_timeout", "sight_timeout"
        )
        self.blackboard.register_key(
            "Dog_following_max_threshold", access=py_trees.common.Access.READ
        )

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.timeout = float(
            resolve(self.timeout, self.blackboard, "check_in_catch_up_timeout")
        )
        self.sight_timeout = float(
            resolve(self.sight_timeout, self.blackboard, "sight_timeout")
        )
        self.pose = RobotPoseTracker(self.node)
        self.subject = FollowedSubjectTracker(self.node, self.sight_timeout)
        self.accessories = AccessoryCommander(self.node)
        self.logger.info(f"{self.name}: Setup complete")

    def initialise(self):
        self._start_time = self.node.get_clock().now()
        self.accessories.look(HEAD_SEEK)
        self.node.get_logger().info(f"{self.name}: waiting for the human to catch up")

    def update(self):
        elapsed = (self.node.get_clock().now() - self._start_time).nanoseconds / 1e9

        if self.pose.position is not None and self.subject.position is not None:
            distance = distance_xy(self.pose.position, self.subject.position)
            allowed = self.blackboard.Dog_following_max_threshold
            self.feedback_message = f"{distance:.2f} m away, waiting {elapsed:.0f} s"
            if distance <= allowed:
                self.node.get_logger().info(
                    f"{self.name}: the human caught up ({distance:.2f} m)"
                )
                return py_trees.common.Status.SUCCESS

        if elapsed > self.timeout:
            self.node.get_logger().info(
                f"{self.name}: the human did not catch up in {self.timeout:.0f} s, "
                "going back to them"
            )
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING


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
        for key in (
            "Dog_current_checkpoint",
            "check_in_checkpoints_since",
            "check_in_last_time",
        ):
            self.blackboard.register_key(key, access=py_trees.common.Access.WRITE)

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
        # Leading resumes straight out of a face-to-face, so the look backs are
        # paced from here rather than from the one before the human was lost.
        self.blackboard.check_in_checkpoints_since = 0
        self.blackboard.check_in_last_time = now_seconds(self.node)
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
