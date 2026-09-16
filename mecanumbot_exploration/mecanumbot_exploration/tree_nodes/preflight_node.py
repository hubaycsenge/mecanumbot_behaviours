"""
Stop the localization stack before the exploration pass, then exit.

Runs first in launch_exploration.launch.py and exits; the rest of the launch
is held behind its exit so slam_toolbox comes up into a graph where AMCL is
no longer publishing map -> odom.

**What this stops, and why.**

AMCL and slam_toolbox both publish the map -> odom transform.  With both
running, the navigation stack sees two competing estimates of where the robot
is: every subscription that reads a pose will get whichever publisher won the
last race, and the transforms will jump.  The fix is to stop the localization
lifecycle manager before slam_toolbox starts; its manage_nodes SHUTDOWN call
takes AMCL and map_server down cleanly in one call.

**What this deliberately does not stop.**

Nav2 itself -- the controller, planner, bt_navigator -- stays up.  Unlike
autoslam, this pass does not restart the navigation stack; it reuses the one
already running (started by launch_mecanumbot_base) and only replaces the
localization source.  Nav2 will continue to accept NavigateToPose goals from
explore_lite while slam_toolbox provides the map -> odom transform and the
live /map topic.

The joystick is left alone, as in the autoslam preflight: it is the human
override and disabling it is a worse failure than anything it might cause.
"""

import sys
import time

import rclpy
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.node import Node


class ExplorationPreflight(Node):
    """Stop the localization lifecycle manager, then exit."""

    def __init__(self):
        super().__init__("exploration_preflight")
        self.declare_parameter("preflight_discovery", 2.0)
        self.declare_parameter("preflight_timeout", 15.0)
        self.declare_parameter("preflight_manager_timeout", 60.0)

    def _live_node_names(self):
        """Return all fully-qualified node names currently on the graph."""
        deadline = time.time() + float(
            self.get_parameter("preflight_discovery").value)
        names = []
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            names = [
                "{}/{}".format(ns.rstrip("/"), name)
                for name, ns in self.get_node_names_and_namespaces()
            ]
        return names

    def run(self):
        manager_timeout = float(
            self.get_parameter("preflight_manager_timeout").value)

        nodes = self._live_node_names()
        localization_manager = next(
            (n for n in nodes
             if n.endswith("/lifecycle_manager_localization")),
            None)

        if localization_manager is None:
            self.get_logger().info(
                "lifecycle_manager_localization not on the graph -- "
                "AMCL is not running, nothing to stop")
            return True

        self.get_logger().info(
            "stopping {} (up to {:.0f}s -- it answers once every "
            "managed node has finished its transitions)".format(
                localization_manager, manager_timeout))

        # Strip the leading '/'.
        manager_name = localization_manager.lstrip("/")
        client = self.create_client(
            ManageLifecycleNodes,
            "/{}/manage_nodes".format(manager_name))
        if not client.wait_for_service(timeout_sec=manager_timeout):
            self.get_logger().error(
                "manage_nodes service not available on {}".format(
                    localization_manager))
            return False

        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.SHUTDOWN
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=manager_timeout)
        if not future.done():
            self.get_logger().error(
                "manage_nodes did not answer within {:.0f}s".format(
                    manager_timeout))
            return False

        if not future.result().success:
            self.get_logger().warn(
                "manage_nodes answered but reported failure; "
                "AMCL may still be running")
            return False

        # Poll until AMCL deregisters from the graph. The lifecycle manager
        # reports success once all transition callbacks have returned, but the
        # node may still be visible briefly while its process tears down.
        post_timeout = float(self.get_parameter("preflight_timeout").value)
        deadline = time.time() + post_timeout
        amcl_alive = True
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            names = [
                "{}/{}".format(ns.rstrip("/"), name)
                for name, ns in self.get_node_names_and_namespaces()
            ]
            if not any(n.endswith("/amcl") for n in names):
                amcl_alive = False
                break

        if amcl_alive:
            self.get_logger().error(
                "AMCL is still on the graph after {:.0f}s; "
                "slam_toolbox will conflict with it".format(post_timeout))
            return False

        self.get_logger().info(
            "localization stack stopped; slam_toolbox may now own map -> odom")
        return True


def main(args=None):
    """Stop the localization stack, then exit so the launch can continue."""
    rclpy.init(args=args)
    node = ExplorationPreflight()
    success = False
    try:
        success = node.run()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    if not success:
        sys.exit(1)


if __name__ == "__main__":
    main()
