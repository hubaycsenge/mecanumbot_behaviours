"""
Keeping the ball in the middle of the picture once it has been found.

Two behaviours, one per axis, and both close the loop on where the ball sits
*in the image* rather than on where the fusion node placed it in the map. An
image offset becomes an angle with nothing but the lens's field of view; a map
position also needs the neck's calibration, which is the number that has been
wrong. So both converge on the ball whatever the neck's zero is.

`TrackBallWithHead` is the vertical axis. It runs beside the approach for as
long as the robot closes in, tilting the neck by a fraction of the ball's
elevation each time a frame taken *after* the last move arrives. As the robot
nears the ball it sinks in the frame and the head follows it down; when the
ball drops out under the camera's chin close to the robot, the head goes to
`fetch_head_low`, the pose that looks at the floor just beyond the lens.

`FaceBall` is the horizontal axis: an in-place turn on `/cmd_vel` until the ball
is centred left-to-right. nav2's rotation shim already faces a ball the approach
drives to, but not one that is closer than the approach's stop distance --
`pose_to_goal` hands back the robot's own pose then, nav2 reports the goal
reached at once, and the robot grabs at a ball off to one side. An in-place
turn is the one movement this repository's trees command on `/cmd_vel`, and
this is one.
"""

import py_trees

from mecanumbot_movement_behaviours.geometry import distance_xy
from mecanumbot_movement_behaviours.ros_interfaces import (
    Nav2GoalMonitor,
    RobotPoseTracker,
    VelocityCommander,
)

from mecanumbot_fetch_behaviour.behaviours.ros_interfaces import (
    BallBoxTracker,
    GripperCommander,
)
from mecanumbot_fetch_behaviour.defaults import constant, register_param_keys
from mecanumbot_fetch_behaviour.gaze import image_offset, neck_step, turn_rate


def _camera(blackboard):
    """Return `(width, height, hfov)` of the frame the ball boxes are in."""
    return (
        float(constant(blackboard, "fetch_camera_width")),
        float(constant(blackboard, "fetch_camera_height")),
        float(constant(blackboard, "fetch_camera_hfov")),
    )


class TrackBallWithHead(py_trees.behaviour.Behaviour):
    """
    Tilt the neck to keep the ball centred vertically; always RUNNING.

    A modifier on the approach beside it, like the search's head behaviour on
    the search: it must not be able to end or fail the parallel it sits in.

    One step per settled frame. After each neck command the boxes stamped in
    the next `fetch_head_track_settle` seconds are ignored, because they were
    taken while the head was still on its way and their offsets describe a
    tilt the head no longer has. Correcting on them is how a visual servo
    oscillates.

    When no ball has been in view for `fetch_head_track_lost` seconds, the head
    stays where it is -- the ball is most likely a frame or two from coming
    back -- unless the robot is within `fetch_head_close_range` of where the
    ball was last placed. Then the ball has gone out under the lens, and the
    head drops to `fetch_head_low` to find it just in front of the grabbers.
    """

    def __init__(self, name="TrackBallWithHead"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        register_param_keys(self.blackboard)
        self.blackboard.register_key(
            key="fetch_ball_position", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="fetch_head_position", access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs):
        """Read the loop's constants and build the box tracker and commander."""
        self.node = kwargs["node"]
        self.width, self.height, self.hfov = _camera(self.blackboard)
        self.low = float(constant(self.blackboard, "fetch_head_low"))
        self.high = float(constant(self.blackboard, "fetch_head_high"))
        self.rad_per_unit = float(constant(self.blackboard, "fetch_neck_rad_per_unit"))
        self.gain = float(constant(self.blackboard, "fetch_head_track_gain"))
        self.deadband = float(constant(self.blackboard, "fetch_head_track_deadband"))
        self.settle = float(constant(self.blackboard, "fetch_head_track_settle"))
        self.lost = float(constant(self.blackboard, "fetch_head_track_lost"))
        self.close_range = float(constant(self.blackboard, "fetch_head_close_range"))
        self.threshold = float(constant(self.blackboard, "fetch_detection_threshold"))
        self.boxes = BallBoxTracker(
            self.node, timeout=constant(self.blackboard, "fetch_detection_timeout")
        )
        self.pose = RobotPoseTracker(self.node)
        self.neck = GripperCommander(self.node)
        self.logger.info(f"{self.name}: Setup complete")
        return True

    def initialise(self):
        """Start from wherever the head was left: that is where the ball was seen."""
        self._position = self.blackboard.fetch_head_position
        if self._position is None:
            self._position = constant(self.blackboard, "fetch_head_search")
        self._moved_at = None
        self._seen_at = self._now()

    def update(self):
        """Step the neck towards the ball, or down when it went under the lens."""
        now = self._now()
        newer_than = None if self._moved_at is None else self._moved_at + self.settle
        box, _ = self.boxes.best(self.threshold, newer_than=newer_than)

        if box is None:
            if self._moved_at is not None and now < self._moved_at + self.settle:
                self.feedback_message = "letting the head settle"
            elif now - self._seen_at > self.lost and self._ball_is_close():
                if self._position > self.low + 1e-3:
                    self.node.get_logger().info(
                        f"{self.name}: ball out of view close by, looking down "
                        f"just beyond the lens ({self.low:.2f})"
                    )
                    self._move(self.low, now)
                self.feedback_message = "looking just beyond the lens"
            else:
                self.feedback_message = f"holding at {self._position:.2f}"
            return py_trees.common.Status.RUNNING

        self._seen_at = now
        _, elevation = image_offset(box[0], box[1], self.width, self.height, self.hfov)
        target = neck_step(
            self._position,
            elevation,
            self.rad_per_unit,
            self.gain,
            self.deadband,
            self.low,
            self.high,
        )
        if target is None:
            self.feedback_message = f"ball centred, head at {self._position:.2f}"
        else:
            self._move(target, now)
            self.feedback_message = f"head to {target:.2f}"
        return py_trees.common.Status.RUNNING

    def _move(self, position, now):
        self._position = float(position)
        self.blackboard.fetch_head_position = self._position
        self.neck.look(self._position)
        self._moved_at = now

    def _ball_is_close(self):
        ball = self.blackboard.fetch_ball_position
        if ball is None or self.pose.pose is None:
            return False
        return distance_xy(self.pose.pose.position, ball) <= self.close_range

    def _now(self):
        return self.node.get_clock().now().nanoseconds / 1e9


class FaceBall(py_trees.behaviour.Behaviour):
    """
    Turn in place until the ball is centred left-to-right; always SUCCESS.

    Never FAILURE, because not managing to centre the ball is not a reason to
    give it up: the approach already drove at it, and the grab after this is
    what finds out whether it is between the grabbers. So the turn ends
    SUCCESS when the ball is within `fetch_face_tolerance` of the centre, when
    it has not been in view for `fetch_face_lost` seconds (under the lens, most
    likely, and the approach's heading is the best there is), or on
    `fetch_face_timeout`.
    """

    def __init__(self, name="FaceBall"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        register_param_keys(self.blackboard)

    def setup(self, **kwargs):
        """Read the turn's constants and build the velocity commander."""
        self.node = kwargs["node"]
        self.width, self.height, self.hfov = _camera(self.blackboard)
        self.tolerance = float(constant(self.blackboard, "fetch_face_tolerance"))
        self.gain = float(constant(self.blackboard, "fetch_face_gain"))
        self.max_rate = float(constant(self.blackboard, "fetch_face_max_rate"))
        self.min_rate = float(constant(self.blackboard, "fetch_face_min_rate"))
        self.lost = float(constant(self.blackboard, "fetch_face_lost"))
        self.timeout = float(constant(self.blackboard, "fetch_face_timeout"))
        self.threshold = float(constant(self.blackboard, "fetch_detection_threshold"))
        self.boxes = BallBoxTracker(
            self.node, timeout=constant(self.blackboard, "fetch_detection_timeout")
        )
        self.velocity = VelocityCommander(self.node)
        self.nav2 = Nav2GoalMonitor(self.node)
        self.logger.info(f"{self.name}: Setup complete")
        return True

    def initialise(self):
        """Start the clock; nothing is commanded until nav2 has let go."""
        self._start = self._now()
        self._seen_at = self._start
        self._centred_frames = 0
        self._last_stamp = None

    def terminate(self, new_status):
        """Stop turning, whatever ended the behaviour."""
        if new_status != py_trees.common.Status.RUNNING:
            self.velocity.stop()

    def update(self):
        """Turn by the ball's bearing in the image until it is centred."""
        now = self._now()
        if now - self._start > self.timeout:
            return self._done(f"not centred after {self.timeout:.0f} s, grabbing anyway")
        # The approach cancelled its goal on the way out; give nav2 a moment
        # to stop commanding before this takes /cmd_vel.
        if self.nav2.busy() and now - self._start < 1.0:
            self.feedback_message = "waiting for nav2 to let go"
            return py_trees.common.Status.RUNNING

        box, stamp = self.boxes.best(self.threshold)
        if box is None:
            self.velocity.stop()
            if now - self._seen_at > self.lost:
                return self._done("ball not in view, keeping the approach's heading")
            self.feedback_message = "waiting for the ball"
            return py_trees.common.Status.RUNNING

        self._seen_at = now
        bearing, _ = image_offset(box[0], box[1], self.width, self.height, self.hfov)
        rate = turn_rate(bearing, self.gain, self.max_rate, self.min_rate, self.tolerance)
        if rate == 0.0:
            self.velocity.stop()
            # Two frames and not one: the first centred frame is often one the
            # robot was still turning through.
            if stamp != self._last_stamp:
                self._centred_frames += 1
                self._last_stamp = stamp
            if self._centred_frames >= 2:
                return self._done("ball centred")
            self.feedback_message = "centred, confirming"
            return py_trees.common.Status.RUNNING

        self._centred_frames = 0
        self.velocity.turn(rate)
        self.feedback_message = f"turning {rate:+.2f} rad/s, ball {bearing:+.2f} rad off"
        return py_trees.common.Status.RUNNING

    def _done(self, reason):
        self.velocity.stop()
        self.node.get_logger().info(f"{self.name}: {reason}")
        return py_trees.common.Status.SUCCESS

    def _now(self):
        return self.node.get_clock().now().nanoseconds / 1e9
