"""
The exploration pass orchestrator.

This is the m-explore-ros2 based alternative to mecanumbot_autoslam.  Where
autoslam implements its own RRT frontier detector and exit criteria, this pass
delegates frontier detection and navigation entirely to explore_lite
(m-explore-ros2) and orchestrates two things:

    /pose ──────────────────► UncertaintyMonitor ──► /explore/resume (pause/resume)
    (PoseWithCovarianceStamped)                   ──► navigate_to_pose (loop closure)

    /pose ──────────────────► FinishDetector ──► exploration/finished (latched)

    exploration/state ◄── (one-line summary every tick)

Like autoslam, this is not a py_trees tree: the pass has no branch to select.
It looks (is covariance high?), acts (pause, drive, resume), and decides
whether it is done (has the robot stopped?).  Three steps in a fixed order,
so the behaviours are plain objects with setup() / update() / terminate() and
this node is the tick.

**What this pass does NOT do.**  It does not detect frontiers -- explore_lite
does that.  It does not plan navigation -- nav2 does that, started by
launch_mecanumbot_base with the study AMCL stack replaced by slam_toolbox
after the preflight runs.  It does not integrate with the Deep3R
reconstruction server -- that is autoslam's job and T2.

**When to use this instead of autoslam.**  When the goal is a 2D map only,
without a 3D reconstruction, and without the full RRT scoring.  The
uncertainty monitor keeps the map consistent enough for downstream use;
explore_lite handles the frontier strategy.
"""

import rclpy
from rclpy.node import Node

from mecanumbot_exploration.behaviours.monitoring import (
    Context, FinishDetector, UncertaintyMonitor)
from mecanumbot_exploration.behaviours.ros_interfaces import (
    ExplorationSignals, PoseTracker)
from mecanumbot_exploration.defaults import all_defaults

NODE_NAME = "exploration_node"


class ExplorationNode(Node):
    """Hold trackers, tick behaviours, report the pass."""

    def __init__(self):
        super().__init__(NODE_NAME)
        params = self._parameters_from_defaults()

        self.pose = PoseTracker(self, params["pose_topic"])
        self.signals = ExplorationSignals(
            self, params["finished_topic"], params["state_topic"])

        self.monitor = UncertaintyMonitor()
        self.finish = FinishDetector()
        for behaviour in (self.monitor, self.finish):
            behaviour.setup(self, params)

        self.started = self._now()
        self.done = False
        self.timer = self.create_timer(
            1.0 / float(params["rate"]), self._tick)
        self.get_logger().info(
            "exploration_node up -- explore_lite drives, covariance monitored "
            "on {} (threshold {:.4f})".format(
                params["pose_topic"], params["uncertainty_threshold"]))

    # --- parameters -----------------------------------------------------------

    def _parameters_from_defaults(self):
        defaults = all_defaults()
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        return {name: self.get_parameter(name).value for name in defaults}

    # --- the tick -------------------------------------------------------------

    def _tick(self):
        if self.done:
            return
        now = self._now()
        context = Context(
            now=now,
            elapsed=now - self.started,
            pose_xy=self.pose.xy,
            covariance_trace=self.pose.covariance_trace,
            distance=self.pose.distance,
        )
        self.monitor.update(context)
        self.finish.update(context, monitor_revisiting=self.monitor.revisiting)

        if self.finish.finished:
            self._finish(context)
            return

        self.signals.state(
            "distance={:.1f}m cov={:.4f} quiet={:.0f}s revisiting={} elapsed={:.0f}s".format(
                context.distance,
                context.covariance_trace,
                self.finish.quiet_for,
                self.monitor.revisiting,
                context.elapsed,
            )
        )

    def _finish(self, context):
        self.done = True
        self.monitor.terminate()
        self.finish.terminate()
        self.signals.finish()
        self.signals.state(
            "finished: drove {:.1f}m in {:.0f}s".format(
                context.distance, context.elapsed))
        self.get_logger().info(
            "exploration done; drove {:.1f}m in {:.0f}s".format(
                context.distance, context.elapsed))

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for step in (node.monitor.terminate, node.destroy_node,
                     rclpy.try_shutdown):
            try:
                step()
            except KeyboardInterrupt:
                pass


if __name__ == "__main__":
    main()
