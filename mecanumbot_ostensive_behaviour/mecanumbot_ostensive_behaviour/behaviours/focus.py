"""
Keeping the addressee in the middle of the camera image.

The robot turns on where the person appears in the *image*, not on where they
are in the map. That is the cheaper signal -- it needs no fused pose, so it
still works in the moment the fusion drops a stationary person -- and it is also
the correct one: what has to stay true is that the person remains inside the
frame the gestures are being read from.

Rotation goes through the leading package's `SmoothTurner`, so it is profiled
and it bypasses nav2 exactly as every other in-place turn on this robot does.
Two things keep it from hunting:

* a deadband (`focus_tolerance`), so the robot ignores the person shifting their
  weight and only answers a real drift towards the edge of the frame;
* acting once per camera frame, and never on a frame captured while the robot
  was still turning. Without that the behaviour would command a second turn from
  a pre-turn image and swing past centre.

`settle=True` gives the one-shot version used to face somebody who has just
signalled; `settle=False` gives the version that runs alongside the cue decoder
and never finishes on its own.
"""

import math

import py_trees

from mecanumbot_leading_behaviour.behaviours.turning import InPlaceTurn

from mecanumbot_ostensive_behaviour.behaviours.blackboard_managers import (
    register_param_keys,
)
from mecanumbot_ostensive_behaviour.behaviours.cue_geometry import (
    bearing_offset_from_image_x,
)
from mecanumbot_ostensive_behaviour.behaviours.ros_interfaces import (
    HEAD_SEEK,
    seconds,
)
from mecanumbot_ostensive_behaviour.behaviours.target_lock import TargetFollower


class KeepTargetInFocus(InPlaceTurn):
    """
    Turn in place to keep the locked addressee centred in the camera image.

    With `settle` set it returns SUCCESS as soon as they are inside the
    deadband; without it, it stays RUNNING for as long as it can see them and is
    meant to be run in a parallel next to whatever is reading their gestures.
    Either way it returns FAILURE when the addressee has been out of sight for
    longer than `track_loss_timeout`, which is how the exchange is abandoned.
    """

    def __init__(self, name="KeepTargetInFocus", settle=False, timeout=20.0, **kwargs):
        super().__init__(
            name,
            head=HEAD_SEEK,
            timeout=float(timeout) if settle else float("inf"),
            **kwargs,
        )
        self.settle = bool(settle)
        register_param_keys(self.blackboard)
        self.blackboard.register_key(
            "ostensive_target", access=py_trees.common.Access.READ
        )

    def setup(self, **kwargs):
        """Take the turn speed from the constants, then build the usual handles."""
        self._turner_kwargs.setdefault(
            "max_speed", self.blackboard.focus_turn_speed
        )
        super().setup(**kwargs)
        self.follower = TargetFollower(
            self.node,
            hfov=self.blackboard.camera_hfov,
            sight_timeout=self.blackboard.detection_timeout,
            max_image_jump=self.blackboard.target_max_image_jump,
            bearing_tolerance=self.blackboard.target_bearing_tolerance,
        )

    def initialise(self):
        """Lift the head and start with no turn in progress."""
        super().initialise()
        self._turning = False
        self._acted_stamp = None
        self.turner.stop()

    def update(self):
        """Follow the addressee and answer any drift out of the deadband."""
        if self._exit_status is not None:
            return self.ramp_down()

        lock = self.blackboard.ostensive_target
        if lock is None:
            return self.finish(
                py_trees.common.Status.FAILURE, "there is no addressee to keep in focus"
            )

        self.follower.refresh(lock)
        now = seconds(self.node)
        if lock.seconds_since_seen(now) > self.blackboard.track_loss_timeout:
            return self.finish(
                py_trees.common.Status.FAILURE,
                f"lost the addressee for {lock.seconds_since_seen(now):.1f} s",
            )

        if self.pose.pose is None:
            self.feedback_message = "waiting for AMCL pose"
            return py_trees.common.Status.RUNNING

        if self.elapsed() > self.timeout:
            return self.finish(
                py_trees.common.Status.FAILURE, "gave up trying to face the addressee"
            )

        if self._turning:
            if not self.turner.step():
                return py_trees.common.Status.RUNNING
            self._turning = False
            # Frames captured mid-turn describe a view the robot has already
            # left; wait for one taken from where it now stands.
            self._acted_stamp = self.follower.camera.stamp

        return self._answer_new_frame()

    # --- internals ------------------------------------------------------------

    def _answer_new_frame(self):
        """Act on the newest camera frame, at most once per frame."""
        stamp = self.follower.camera.stamp
        if stamp is None or stamp == self._acted_stamp:
            self.feedback_message = "waiting for a fresh camera frame"
            return py_trees.common.Status.RUNNING

        error = self._bearing_error()
        if error is None:
            self.feedback_message = "addressee not in this frame, holding still"
            return py_trees.common.Status.RUNNING
        self._acted_stamp = stamp

        if abs(error) <= self.blackboard.focus_tolerance:
            self.feedback_message = f"centred ({math.degrees(error):+.0f} deg)"
            if self.settle:
                return self.finish(
                    py_trees.common.Status.SUCCESS, "facing the addressee"
                )
            return py_trees.common.Status.RUNNING

        if not self.nav2_handed_over():
            return py_trees.common.Status.RUNNING

        self.turner.begin(error)
        self._turning = True
        self.record_spin_sign(error)
        self.node.get_logger().info(
            f"{self.name}: turning {math.degrees(error):+.0f} deg to re-centre "
            "the addressee"
        )
        return py_trees.common.Status.RUNNING

    def _bearing_error(self):
        """Return the turn [rad] that would centre the addressee, or None."""
        lock = self.blackboard.ostensive_target
        if lock is None or lock.observation is None:
            return None
        centroid_x = lock.observation.centroid_x
        if centroid_x is None:
            return None
        return bearing_offset_from_image_x(centroid_x, self.blackboard.camera_hfov)


class FaceTarget(KeepTargetInFocus):
    """Turn once to face the addressee, and succeed as soon as they are centred."""

    def __init__(self, name="FaceTarget", timeout=20.0, **kwargs):
        super().__init__(name, settle=True, timeout=timeout, **kwargs)
