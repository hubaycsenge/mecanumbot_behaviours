"""
UncertaintyMonitor and FinishDetector behaviours.

Both follow the same interface as the autoslam behaviours: setup(), update(),
terminate().  exploration_node.py ticks them in order.

UncertaintyMonitor
------------------
When slam_toolbox's pose covariance trace exceeds the threshold the robot has
drifted far enough from where the map says it is that the next frontier goal
is likely to be planned against a stale map.  The right answer is to return
somewhere the robot has already mapped well -- the origin -- so slam_toolbox
gets another look at familiar features and can close the loop.

The full sequence:
  1. pause explore_lite (/explore/resume → False)
  2. navigate to (revisit_x, revisit_y) via NavigateToPose
  3. resume explore_lite (/explore/resume → True)

All three steps are async; the behaviour tracks its own state so the node's
timer callback does not have to wait.

FinishDetector
--------------
explore_lite does not publish a "done" topic.  When all frontiers are
exhausted it simply stops sending nav2 goals, and the robot stops moving.
FinishDetector declares the pass done when the robot has stood still for
frontier_quiet_time seconds -- excluding periods when UncertaintyMonitor has
its own revisit in flight.  A hard max_duration budget overrides this if set.
"""

import math


class Context:
    """Snapshot of measured state passed to each behaviour tick."""

    def __init__(self, now, elapsed, pose_xy, covariance_trace, distance):
        self.now = now
        self.elapsed = elapsed
        self.pose_xy = pose_xy
        self.covariance_trace = covariance_trace
        self.distance = distance


class UncertaintyMonitor:
    """Pause explore_lite, return to origin, resume when covariance is high."""

    def setup(self, node, params):
        from mecanumbot_exploration.behaviours.ros_interfaces import (
            ExploreClient, NavClient)
        self._node = node
        self._threshold = params["uncertainty_threshold"]
        self._revisit_x = params["revisit_x"]
        self._revisit_y = params["revisit_y"]
        self._revisiting = False
        self._explore = ExploreClient(node, params["explore_resume_service"])
        self._nav = NavClient(node, params["nav2_action"])

    @property
    def revisiting(self):
        return self._revisiting

    def update(self, context):
        if context.covariance_trace > self._threshold and not self._revisiting:
            self._node.get_logger().warn(
                "covariance trace {:.4f} exceeds threshold {:.4f}; pausing "
                "explore_lite and returning to ({:.2f}, {:.2f}) for a loop "
                "closure".format(
                    context.covariance_trace, self._threshold,
                    self._revisit_x, self._revisit_y))
            self._revisiting = True
            self._explore.pause(callback=self._send_goal)

    def terminate(self):
        pass

    def _send_goal(self):
        self._nav.navigate_to(
            self._revisit_x, self._revisit_y,
            on_done=self._on_done)

    def _on_done(self, success):
        if success:
            self._node.get_logger().info(
                "revisit complete; resuming explore_lite")
        else:
            self._node.get_logger().warn(
                "revisit goal did not complete; resuming explore_lite anyway")
        self._revisiting = False
        self._explore.resume()


class FinishDetector:
    """Declare the pass done when the robot has stood still long enough."""

    def setup(self, node, params):
        self._quiet_time = params["frontier_quiet_time"]
        self._movement_threshold = params["movement_threshold"]
        self._max_duration = params["max_duration"]
        self._last_move_time = None
        self._last_xy = None
        self.finished = False

    @property
    def quiet_for(self):
        """Seconds since the robot last moved significantly; 0 if never polled."""
        if self._last_move_time is None:
            return 0.0
        return max(0.0, self._last_now - self._last_move_time)

    def update(self, context, monitor_revisiting):
        if self.finished:
            return
        self._last_now = context.now

        if self._last_move_time is None:
            self._last_move_time = context.now

        if context.pose_xy is not None:
            if self._last_xy is None:
                self._last_xy = context.pose_xy
            else:
                moved = math.hypot(
                    context.pose_xy[0] - self._last_xy[0],
                    context.pose_xy[1] - self._last_xy[1])
                if moved > self._movement_threshold:
                    self._last_move_time = context.now
                    self._last_xy = context.pose_xy

        # Hard budget overrides quality criteria.
        if self._max_duration > 0.0 and context.elapsed >= self._max_duration:
            self.finished = True
            return

        quiet_for = context.now - self._last_move_time
        if (quiet_for >= self._quiet_time
                and not monitor_revisiting
                and context.elapsed >= self._quiet_time):
            self.finished = True

    def terminate(self):
        pass
