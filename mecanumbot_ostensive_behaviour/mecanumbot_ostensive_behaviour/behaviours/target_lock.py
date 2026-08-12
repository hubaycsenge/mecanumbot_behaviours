"""
Holding on to the one person who asked for the robot's attention.

An ostensive exchange has an addressee: once somebody has signalled, everything
that follows -- the turn that keeps them centred, the arm the direction cue is
read from -- must be about *them*, and not about whoever else walks past. The
`TargetLock` is that commitment, and it lives on the blackboard so every
behaviour in the tree is talking about the same person.

Two separate identities have to be kept aligned, because the two things the
robot needs come from two different places:

* the person's **image identity**, carried frame to frame by the small step
  their torso makes between frames. The camera detector does not publish track
  IDs, so proximity is what there is;
* the person's **map position**, which the camera cannot give at all -- it has
  no depth. That comes from `people_fusion`, matched to the image identity by
  bearing (see `cue_geometry.match_by_bearing`).

The refresh is idempotent per camera frame: several behaviours tick against the
same lock in one pass of the tree, and the frame stamp is what stops them
re-running the association for each of them, or worse, disagreeing.
"""

import math

from mecanumbot_ostensive_behaviour.behaviours.cue_geometry import (
    bearing_from_image_x,
    match_by_bearing,
    nearest_index,
)
from mecanumbot_ostensive_behaviour.behaviours.ros_interfaces import (
    CamDetectionTracker,
    PeopleTracker,
    RobotPoseTracker,
    seconds,
)


def nearest_observation(observations, image_torso, max_jump):
    """Return the index of the observation whose torso is nearest, or None."""
    if image_torso is None:
        return None
    return nearest_index(
        [observation.torso for observation in observations], image_torso, max_jump
    )


class TargetLock:
    """The addressee: who they are in the image, where they are in the map."""

    __slots__ = (
        "image_torso",
        "observation",
        "map_position",
        "map_position_time",
        "signal",
        "locked_at",
        "last_seen",
        "stamp",
    )

    def __init__(self, image_torso, observation, signal, now):
        self.image_torso = image_torso
        self.observation = observation
        self.map_position = None
        self.map_position_time = None
        self.signal = signal
        self.locked_at = float(now)
        self.last_seen = float(now)
        self.stamp = None

    @property
    def visible(self):
        """Return True when the addressee was found on the most recent camera frame."""
        return self.observation is not None

    def seconds_since_seen(self, now):
        """Return how long the addressee has been missing from the camera."""
        return float(now) - self.last_seen

    def __repr__(self):
        """Return a debugging representation naming the signal and the position."""
        where = (
            "no map position"
            if self.map_position is None
            else f"map=({self.map_position[0]:.2f}, {self.map_position[1]:.2f})"
        )
        return f"TargetLock({self.signal}, {where})"


class TargetFollower:
    """
    The subscriptions and the per-frame bookkeeping behind a `TargetLock`.

    Each behaviour that needs the addressee builds its own follower -- and so
    its own subscriptions, which is how the leading behaviour library does it
    too -- but they all refresh the single lock on the blackboard, and the frame
    stamp keeps that work down to once per camera frame.
    """

    def __init__(
        self,
        node,
        hfov,
        sight_timeout=1.0,
        max_image_jump=0.25,
        bearing_tolerance=math.radians(20.0),
        mirror=False,
    ):
        self.node = node
        self.hfov = float(hfov)
        self.max_image_jump = float(max_image_jump)
        self.bearing_tolerance = float(bearing_tolerance)
        self.camera = CamDetectionTracker(node, sight_timeout, mirror)
        self.people = PeopleTracker(node, sight_timeout)
        self.pose = RobotPoseTracker(node)

    def refresh(self, lock):
        """Bring the lock up to date with the newest camera frame, once per frame."""
        if lock is None:
            return
        stamp = self.camera.stamp
        if stamp is None or stamp == lock.stamp:
            return
        lock.stamp = stamp

        index = nearest_observation(
            self.camera.people, lock.image_torso, self.max_image_jump
        )
        if index is None:
            # Do not clear `image_torso`: the addressee is expected back near
            # where they were last seen, and that is what re-acquires them.
            lock.observation = None
            return

        observation = self.camera.people[index]
        lock.observation = observation
        lock.image_torso = observation.torso
        lock.last_seen = seconds(self.node)
        self._update_map_position(lock, observation)

    def bearing_of(self, observation):
        """Return the map-frame bearing of an observed person, or None."""
        yaw = self.pose.yaw
        if yaw is None or observation.centroid_x is None:
            return None
        return bearing_from_image_x(observation.centroid_x, self.hfov, yaw)

    def robot_xy(self):
        """Return the robot's (x, y) in the map, or None before AMCL reports."""
        position = self.pose.position
        return None if position is None else (position.x, position.y)

    def fused_positions(self):
        """Return the fused people detections as plain (x, y) tuples."""
        return [(pose.position.x, pose.position.y) for pose in self.people.poses]

    def locate(self, observation):
        """Return the map (x, y) of an observed person, or None if unmatched."""
        bearing = self.bearing_of(observation)
        robot = self.robot_xy()
        if bearing is None or robot is None:
            return None
        candidates = self.fused_positions()
        index = match_by_bearing(bearing, robot, candidates, self.bearing_tolerance)
        return None if index is None else candidates[index]

    def _update_map_position(self, lock, observation):
        located = self.locate(observation)
        if located is None:
            # Keep the previous position. The fused array drops out regularly --
            # the LiDAR detector suppresses stationary tracks -- and a person
            # who is standing still to point at something is exactly the case
            # that produces the dropout.
            return
        lock.map_position = located
        lock.map_position_time = seconds(self.node)
