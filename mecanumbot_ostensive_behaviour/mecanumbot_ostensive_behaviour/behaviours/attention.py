"""
Waiting for somebody to ask for the robot's attention, and committing to them.

This is the only behaviour in the package that looks at every person in the
frame. Everything after it is about the one person it picks, which is the point
of the ostensive framing: the exchange has an addressee, and the addressee is
whoever bid for it first.

Candidates are followed across frames the same way the lock is -- by how far
their torso moved -- because the camera detector publishes no track IDs, and a
wave has to be recognised over a second or so of frames rather than in one.
A candidate the tracker loses is dropped along with their part-completed dwell,
so an arm raised, hidden behind somebody, and raised again does not add up to a
signal it never sustained.
"""

import math

import py_trees

from mecanumbot_ostensive_behaviour.behaviours.blackboard_managers import (
    register_param_keys,
)
from mecanumbot_ostensive_behaviour.behaviours.gestures import (
    WaveTracker,
    attention_signal,
)
from mecanumbot_ostensive_behaviour.behaviours.ros_interfaces import (
    AccessoryCommander,
    HEAD_SEEK,
    seconds,
)
from mecanumbot_ostensive_behaviour.behaviours.target_lock import (
    TargetFollower,
    TargetLock,
    nearest_observation,
)


class _Candidate:
    """One person being watched for an attention bid, and how long they have held it."""

    __slots__ = ("torso", "waves", "signal", "signal_since")

    def __init__(self, torso, waves):
        self.torso = torso
        self.waves = waves
        self.signal = None
        self.signal_since = None

    def held_for(self, now):
        """Return how long the current signal has been held, or 0.0 if there is none."""
        if self.signal_since is None:
            return 0.0
        return float(now) - self.signal_since


class WaitForAttentionSignal(py_trees.behaviour.Behaviour):
    """
    RUNNING until somebody holds an attention signal, then lock onto them.

    FAILURE after `attention_timeout` with nobody having signalled, which is
    what lets the tree fall through to looking around instead of waiting for
    ever at whatever the robot happens to be pointed at.
    """

    def __init__(self, name="WaitForAttentionSignal"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        register_param_keys(self.blackboard)
        self.blackboard.register_key(
            "ostensive_target", access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs):
        """Build the trackers and the neck commander."""
        self.node = kwargs["node"]
        self.follower = TargetFollower(
            self.node,
            hfov=self.blackboard.camera_hfov,
            sight_timeout=self.blackboard.detection_timeout,
            max_image_jump=self.blackboard.target_max_image_jump,
            bearing_tolerance=self.blackboard.target_bearing_tolerance,
            mirror=self.blackboard.mirror_image_x,
        )
        self.accessories = AccessoryCommander(self.node)
        self.logger.info(f"{self.name}: Setup complete")
        return True

    def initialise(self):
        """Start a fresh watch with the head lifted."""
        self._candidates = []
        self._stamp = None
        self._start_time = seconds(self.node)
        # The lifted head both reads as the robot seeking contact and gives the
        # pose detector whole bodies instead of knees -- which is what the
        # gestures are decoded from, so it matters twice over here.
        self.accessories.look(HEAD_SEEK)
        self.node.get_logger().info(f"{self.name}: watching for an attention signal")

    def update(self):
        """Follow every candidate and lock onto the first one to hold a signal."""
        now = seconds(self.node)
        camera = self.follower.camera

        if camera.stamp is not None and camera.stamp != self._stamp:
            self._stamp = camera.stamp
            signalled = self._step_candidates(camera.people, now)
            if signalled is not None:
                return self._lock_onto(signalled, now)

        elapsed = now - self._start_time
        if elapsed > self.blackboard.attention_timeout:
            self.node.get_logger().info(
                f"{self.name}: nobody signalled in {elapsed:.0f} s"
            )
            return py_trees.common.Status.FAILURE

        if not camera.fresh():
            self.feedback_message = "waiting for camera people detections"
        else:
            self.feedback_message = f"watching {len(camera.people)} person(s)"
        return py_trees.common.Status.RUNNING

    # --- internals ------------------------------------------------------------

    def _step_candidates(self, observations, now):
        """Advance every candidate one frame; return the observation that signalled."""
        available = list(range(len(observations)))
        kept = []
        signalled = None

        for candidate in self._candidates:
            index = nearest_observation(
                [observations[i] for i in available],
                candidate.torso,
                self.blackboard.target_max_image_jump,
            )
            if index is None:
                continue  # lost this candidate, and their dwell with them
            observation = observations[available.pop(index)]
            candidate.torso = observation.torso
            kept.append(candidate)
            if self._advance(candidate, observation, now) and signalled is None:
                signalled = (candidate, observation)

        for index in available:
            observation = observations[index]
            candidate = _Candidate(observation.torso, self._new_wave_tracker())
            kept.append(candidate)
            self._advance(candidate, observation, now)

        self._candidates = kept
        return signalled

    def _advance(self, candidate, observation, now):
        """Update one candidate's signal state; return True once the dwell is met."""
        signal = attention_signal(
            observation.keypoints,
            candidate.waves,
            now,
            mode=self.blackboard.attention_signal_mode,
            margin_ratio=self.blackboard.wrist_above_shoulder_margin,
        )
        if signal is None:
            candidate.signal = None
            candidate.signal_since = None
            return False

        if candidate.signal_since is None:
            candidate.signal_since = now
            self.node.get_logger().info(
                f"{self.name}: {signal.kind} seen, holding it for "
                f"{self.blackboard.attention_dwell:.1f} s to confirm"
            )
        candidate.signal = signal
        return candidate.held_for(now) >= self.blackboard.attention_dwell

    def _lock_onto(self, signalled, now):
        """Write the new addressee to the blackboard and return SUCCESS."""
        candidate, observation = signalled
        lock = TargetLock(observation.torso, observation, candidate.signal, now)
        lock.stamp = self._stamp
        lock.map_position = self.follower.locate(observation)
        lock.map_position_time = None if lock.map_position is None else now

        self.blackboard.ostensive_target = lock
        bearing = self.follower.bearing_of(observation)
        self.node.get_logger().info(
            f"{self.name}: addressee locked on a {candidate.signal.kind} "
            f"({candidate.signal.side} hand) at bearing "
            f"{'unknown' if bearing is None else f'{math.degrees(bearing):+.0f} deg'}"
        )
        if lock.map_position is None:
            self.node.get_logger().warn(
                f"{self.name}: no people_fusion pose matched the addressee yet; "
                "the direction cue cannot be acted on until one does"
            )
        return py_trees.common.Status.SUCCESS

    def _new_wave_tracker(self):
        return WaveTracker(
            window=self.blackboard.wave_window,
            min_amplitude=self.blackboard.wave_min_amplitude,
            min_reversals=self.blackboard.wave_min_reversals,
        )
