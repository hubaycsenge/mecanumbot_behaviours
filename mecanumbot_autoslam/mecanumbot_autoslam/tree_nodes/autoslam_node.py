"""
The autoslam pass: run the exploration behaviours until the place is scanned.

This is T1. It is the node the launch file starts, and all it does is hold the
trackers, tick four behaviours in order and publish what happened:

    /map ─────────────────► DetectFrontiers ──┐
    map -> base_link (tf) ─►                  │
                                              ▼
    deep3r/map_agreement ─► FinishExploration ─► exploration/finished (latched)
    deep3r/revisit_regions ──────────────┐    │
                                         ▼    ▼
                                     DriveToGoal ──► nav2 NavigateToPose
                                         │
              exploration/state, exploration/frontiers (markers)

**Why it is not a `py_trees` tree.** Every other experiment in this repository
is, because every other experiment has branches -- a person to commit to or not,
an object reachable or not, a condition that changes what signalling happens.
An exploration pass has none: it looks, it decides whether it is done, and if it
is not it drives somewhere. Four steps in a fixed order. Wrapping that in a tree
would add a runner, a blackboard and a tick policy without making a single
decision clearer, so the behaviours here are plain objects with the same shape
(`setup`, `update`, `terminate`) and this node is the tick.

**Why it is in `mecanumbot_behaviours` at all.** Because it sends the robot
somewhere. Everything it reasons *with* -- the RRT, the frontier scoring, the
occupancy model, the exit criteria, the 2D/3D comparison -- stayed in
`mecanumbot_custom_nav2`, which is now a library of judgements about maps and
one node that turns the server's verdict into a nav2 keepout mask. Nothing in
that package commands motion any more.
"""

import rclpy
from rclpy.node import Node

from mecanumbot_autoslam.behaviours.base import Context
from mecanumbot_autoslam.behaviours.detecting import DetectFrontiers
from mecanumbot_autoslam.behaviours.driving import DriveToGoal
from mecanumbot_autoslam.behaviours.finishing import FinishExploration
from mecanumbot_autoslam.behaviours.ros_interfaces import (
    AgreementTracker,
    ExplorationSignals,
    MapTracker,
    PoseSource,
    RevisitTracker,
)
from mecanumbot_autoslam.defaults import all_defaults

#: The node name, which is also the root key the constants file is written
#: under. Unlike the trees next door there is no blackboard loader here: the
#: constants arrive as ROS parameters, so the file and this name have to agree.
NODE_NAME = "autoslam_node"


class AutoslamNode(Node):
    """Hold the trackers, tick the behaviours, report the pass."""

    def __init__(self):
        """Declare the constants, build everything and start the timer."""
        super().__init__(NODE_NAME)
        params = self._parameters_from_defaults()

        self.map = MapTracker(
            self, params["map_topic"],
            params["free_threshold"], params["occupied_threshold"],
        )
        self.pose = PoseSource(
            self, params["pose_source"], params["pose_topic"],
            params["map_frame"], params["base_frame"],
        )
        self.agreement = AgreementTracker(
            self, params["agreement_topic"], params["cloud_window"]
        )
        self.revisit = RevisitTracker(self, params["revisit_topic"])
        self.signals = ExplorationSignals(
            self, params["finished_topic"], params["state_topic"],
            params["marker_topic"],
        )

        self.detect = DetectFrontiers()
        self.finish = FinishExploration()
        self.drive = DriveToGoal()
        for behaviour in (self.detect, self.finish, self.drive):
            behaviour.setup(self, params)

        self.started = self._now()
        self.done = False
        self.timer = self.create_timer(1.0 / float(params["rate"]), self._tick)
        self.get_logger().info(
            "autoslam up; waiting for a map and a pose "
            "(source: {})".format(params["pose_source"])
        )

    # --- parameters -----------------------------------------------------------

    def _parameters_from_defaults(self):
        """Declare every constant this package has, and read it back."""
        defaults = all_defaults()
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        return {name: self.get_parameter(name).value for name in defaults}

    # --- the tick -------------------------------------------------------------

    def _tick(self):
        """Run one cycle of the pass."""
        if self.done:
            return
        self.pose.poll()
        if self.map.grid is None or self.pose.xy is None:
            return

        context = Context(
            now=self._now(),
            grid=self.map.grid,
            robot_xy=self.pose.xy,
            distance=self.pose.distance,
            loop_closed=self.map.take_loop_closure(),
        )
        if context.loop_closed:
            # The trees were grown against free space that no longer exists.
            self.detect.restart(context.robot_xy)
            self.get_logger().info(
                "map re-rasterised (loop closure); trees restarted")

        self.detect.update(context, current_goal=self.drive.goal)
        self.finish.update(
            context,
            cloud=self.agreement.progress,
            elapsed=context.now - self.started,
        )

        if self.finish.finished:
            self._finish(context)
            return

        self.drive.update(context, revisit_points=self.revisit.points)
        self._report(context)

    def _finish(self, context):
        """Stop driving, latch `finished`, and say how the pass went."""
        self.done = True
        self.drive.terminate()
        self.signals.finish()
        self._report(context)
        self.get_logger().info(
            "drove {:.1f} m in {:.0f} s".format(
                context.distance, context.now - self.started)
        )

    def _report(self, context):
        """Publish the state line and the frontier markers."""
        summary = context.verdict.summary() if context.verdict else "starting"
        self.signals.state(
            "frontiers={} distance={:.1f}m goal={} | {}".format(
                len(context.scored), context.distance,
                context.goal_source or "none", summary)
        )
        self.signals.frontiers(context.scored, context.goal)

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None):
    """Run the autoslam pass."""
    rclpy.init(args=args)
    node = AutoslamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.drive.terminate()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
