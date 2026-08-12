"""
Reading where the addressee is pointing, and driving there.

Two behaviours, deliberately split at the point where the decision is made.
`DecodeDirectionCue` only watches and decides; `FollowDirectionCue` only acts on
a decision already taken. That is what lets the decoding run in a parallel next
to `KeepTargetInFocus` -- the robot is turning while it watches -- and the
driving run afterwards, on its own, once the direction is settled.

A cue has to survive a dwell before it is believed: the same arm, pointing
within `cue_stability` of the same azimuth, for `cue_dwell` seconds. An arm
sweeping up into a point passes through every angle on the way, and without the
dwell the robot would commit to whichever of them it happened to sample.

Driving goes through nav2 like every other translation on this robot, so the
local costmap supervises it and the goal monitor resends a goal nav2 gives up
on. The one thing worth knowing is that the goal is a point in open space rather
than a place on a route: it is `cue_distance` metres from the person along the
direction they indicated, and nothing guarantees it is reachable or even inside
the map. Nav2 rejecting it is a normal outcome and ends the exchange.
"""

import math

import py_trees
from geometry_msgs.msg import Pose

from mecanumbot_leading_behaviour.behaviours.geometry import quaternion_from_yaw

from mecanumbot_ostensive_behaviour.behaviours.blackboard_managers import (
    register_param_keys,
)
from mecanumbot_ostensive_behaviour.behaviours.cue_geometry import (
    angular_difference,
    cue_bearing,
    cue_goal_position,
)
from mecanumbot_ostensive_behaviour.behaviours.gestures import pointing_cue
from mecanumbot_ostensive_behaviour.behaviours.ros_interfaces import (
    GOAL_ACTIVE_STATUSES,
    Nav2GoalMonitor,
    RobotPoseTracker,
    STATUS_SUCCEEDED,
    seconds,
)
from mecanumbot_ostensive_behaviour.behaviours.target_lock import TargetFollower


class DirectionCue:
    """A settled direction cue: where it was given from, and which way it points."""

    __slots__ = ("bearing", "origin", "azimuth", "side", "decided_at")

    def __init__(self, bearing, origin, azimuth, side, decided_at):
        self.bearing = float(bearing)
        self.origin = origin
        self.azimuth = float(azimuth)
        self.side = side
        self.decided_at = float(decided_at)

    def __repr__(self):
        """Return a debugging representation naming the direction and its origin."""
        return (
            f"DirectionCue({math.degrees(self.bearing):+.0f} deg from "
            f"({self.origin[0]:.2f}, {self.origin[1]:.2f}), {self.side} arm)"
        )


class DecodeDirectionCue(py_trees.behaviour.Behaviour):
    """
    RUNNING until the addressee holds a pointing gesture, then record the direction.

    FAILURE when they stop being visible, or when `cue_timeout` passes with no
    cue held long enough -- somebody who caught the robot's attention and then
    did not ask for anything should not hold it for ever.
    """

    def __init__(self, name="DecodeDirectionCue"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        register_param_keys(self.blackboard)
        self.blackboard.register_key(
            "ostensive_target", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            "ostensive_cue", access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs):
        """Build the trackers that follow the addressee."""
        self.node = kwargs["node"]
        self.follower = TargetFollower(
            self.node,
            hfov=self.blackboard.camera_hfov,
            sight_timeout=self.blackboard.detection_timeout,
            max_image_jump=self.blackboard.target_max_image_jump,
            bearing_tolerance=self.blackboard.target_bearing_tolerance,
            mirror=self.blackboard.mirror_image_x,
        )
        self.logger.info(f"{self.name}: Setup complete")
        return True

    def initialise(self):
        """Start watching for a cue with no partial gesture remembered."""
        self._reset_dwell()
        self._stamp = None
        self._start_time = seconds(self.node)
        self._warned_about_position = False
        self.blackboard.ostensive_cue = None
        self.node.get_logger().info(f"{self.name}: watching for a direction cue")

    def update(self):
        """Sample the pointing arm once per frame and commit when it settles."""
        lock = self.blackboard.ostensive_target
        if lock is None:
            self.node.get_logger().info(f"{self.name}: there is no addressee")
            return py_trees.common.Status.FAILURE

        self.follower.refresh(lock)
        now = seconds(self.node)

        if lock.seconds_since_seen(now) > self.blackboard.track_loss_timeout:
            self.node.get_logger().info(
                f"{self.name}: lost the addressee before they pointed anywhere"
            )
            return py_trees.common.Status.FAILURE

        stamp = self.follower.camera.stamp
        if stamp is not None and stamp != self._stamp:
            self._stamp = stamp
            settled = self._sample(lock, now)
            if settled is not None:
                cue = self._build_cue(lock, settled, now)
                if cue is not None:
                    return self._commit(cue)

        elapsed = now - self._start_time
        if elapsed > self.blackboard.cue_timeout:
            self.node.get_logger().info(
                f"{self.name}: no direction cue held in {elapsed:.0f} s"
            )
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING

    # --- internals ------------------------------------------------------------

    def _reset_dwell(self):
        self._side = None
        self._azimuths = []
        self._since = None

    def _sample(self, lock, now):
        """Add this frame's cue to the dwell; return the mean azimuth once settled."""
        if lock.observation is None:
            return None

        cue = pointing_cue(
            lock.observation.keypoints,
            min_extension_ratio=self.blackboard.cue_min_extension,
            full_extension_ratio=self.blackboard.cue_full_extension,
            min_lateral_ratio=self.blackboard.cue_min_lateral,
        )
        if cue is None:
            self._reset_dwell()
            self.feedback_message = "the addressee is not pointing anywhere"
            return None

        drifted = bool(self._azimuths) and (
            abs(angular_difference(self._mean_azimuth(), cue.azimuth))
            > self.blackboard.cue_stability
        )
        if cue.side != self._side or drifted:
            # The arm is still moving, or swapped: start the dwell here.
            self._side = cue.side
            self._azimuths = [cue.azimuth]
            self._since = now
            self.feedback_message = f"{cue.side} arm, settling"
            return None

        self._azimuths.append(cue.azimuth)
        held = now - self._since
        self.feedback_message = (
            f"{cue.side} arm at {math.degrees(self._mean_azimuth()):+.0f} deg, "
            f"held {held:.1f}/{self.blackboard.cue_dwell:.1f} s"
        )
        if held < self.blackboard.cue_dwell:
            return None
        return self._mean_azimuth()

    def _mean_azimuth(self):
        return sum(self._azimuths) / len(self._azimuths)

    def _build_cue(self, lock, azimuth, now):
        """Turn a settled azimuth into a map direction, or None if not yet possible."""
        robot = self.follower.robot_xy()
        if lock.map_position is None or robot is None:
            if not self._warned_about_position:
                self._warned_about_position = True
                self.node.get_logger().warn(
                    f"{self.name}: the addressee is pointing, but there is no "
                    "people_fusion pose to anchor the direction to yet"
                )
            return None

        bearing = cue_bearing(lock.map_position, robot, azimuth)
        return DirectionCue(bearing, lock.map_position, azimuth, self._side, now)

    def _commit(self, cue):
        """Publish the cue on the blackboard and return SUCCESS."""
        self.blackboard.ostensive_cue = cue
        self.node.get_logger().info(
            f"{self.name}: direction cue read -- {cue.side} arm, "
            f"{math.degrees(cue.azimuth):+.0f} deg from where they face, "
            f"i.e. {math.degrees(cue.bearing):+.0f} deg in the map"
        )
        return py_trees.common.Status.SUCCESS


class FollowDirectionCue(py_trees.behaviour.Behaviour):
    """
    Drive to the place the addressee indicated, through nav2.

    The goal is `cue_distance` metres from where the person stood, along the
    direction they pointed, and the robot arrives facing that same way -- so it
    ends up looking at whatever it was sent to look at rather than back at the
    person who sent it.
    """

    def __init__(self, name="FollowDirectionCue"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        register_param_keys(self.blackboard)
        self.blackboard.register_key(
            "ostensive_cue", access=py_trees.common.Access.READ
        )

    def setup(self, **kwargs):
        """Build the pose tracker and the nav2 goal monitor."""
        self.node = kwargs["node"]
        self.pose = RobotPoseTracker(self.node)
        self.nav2 = Nav2GoalMonitor(self.node)
        self.logger.info(f"{self.name}: Setup complete")
        return True

    def initialise(self):
        """Forget the previous goal and start the clock."""
        self.nav2.reset()
        self._start_time = seconds(self.node)

    def update(self):
        """Send the cued goal and follow it to completion."""
        cue = self.blackboard.ostensive_cue
        if cue is None:
            self.node.get_logger().info(f"{self.name}: there is no cue to follow")
            return py_trees.common.Status.FAILURE

        if self.pose.pose is None:
            self.feedback_message = "waiting for AMCL pose"
            return py_trees.common.Status.RUNNING

        elapsed = seconds(self.node) - self._start_time
        if elapsed > self.blackboard.cue_goal_timeout:
            self.node.get_logger().info(
                f"{self.name}: gave up on the cued goal after {elapsed:.0f} s"
            )
            return py_trees.common.Status.FAILURE

        if not self.nav2.goal_sent:
            if self.nav2.busy():
                self.feedback_message = "waiting for the previous nav2 goal to finish"
                return py_trees.common.Status.RUNNING
            self._send_goal(cue)
            return py_trees.common.Status.RUNNING

        status = self.nav2.status()
        if status is None:
            self.feedback_message = "waiting for nav2 to accept the goal"
            return py_trees.common.Status.RUNNING
        if status == STATUS_SUCCEEDED:
            self.node.get_logger().info(f"{self.name}: reached the indicated place")
            return py_trees.common.Status.SUCCESS
        if status in GOAL_ACTIVE_STATUSES:
            self.feedback_message = "driving where the addressee pointed"
            return py_trees.common.Status.RUNNING

        self.node.get_logger().info(
            f"{self.name}: nav2 dropped the cued goal, sending it again"
        )
        self.nav2.reset()
        return py_trees.common.Status.RUNNING

    # --- internals ------------------------------------------------------------

    def _send_goal(self, cue):
        x, y = cue_goal_position(cue.origin, cue.bearing, self.blackboard.cue_distance)
        goal = Pose()
        goal.position.x = x
        goal.position.y = y
        goal.orientation = quaternion_from_yaw(cue.bearing)
        self.nav2.send(goal)
        self.node.get_logger().info(
            f"{self.name}: goal {self.blackboard.cue_distance:.1f} m along "
            f"{math.degrees(cue.bearing):+.0f} deg, at x={x:.2f} y={y:.2f}"
        )
