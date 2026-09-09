"""
Clear the way for an autoslam pass, then get out of the way.

Runs first in `launch_autoslam.launch.py` and exits; the rest of the launch is
held behind its exit, so slam_toolbox and nav2 come up into a graph that has
already been cleared rather than racing whatever was still running.

What counts as contradicting, and why each thing does, is `preflight.py` -- this
module only performs the plan. Three methods, tried in the order that is
gentlest:

1. **nav2 lifecycle managers.** `manage_nodes` with SHUTDOWN retires every
   server under a manager in dependency order. One call for the whole study
   stack, and the only method here nav2 itself considers supported.
2. **A lifecycle node's own `change_state`**, for a stack whose manager is
   already gone. The current state is read first, because the shutdown
   transition is a different id from each of unconfigured, inactive and active.
3. **SIGTERM, then SIGKILL**, for everything whose *name* autoslam re-uses --
   the whole study nav2 navigation stack, slam_toolbox, an old pass -- and for
   the plain nodes that are not lifecycle-managed at all, like the behaviour
   trees. Matched on the executable name or the `__node:=` remap in a process's
   command line, never a free-text search, and never against this process or
   anything in its own process group.

**Why the nav2 navigation stack is signalled rather than shut down.** A
lifecycle shutdown does not free a node name: a finalized node is still on the
graph and still answers `<name>/change_state`. `navigation_launch.py` registers
`controller_server`, `bt_navigator` and six more under exactly the names the
study stack already holds, so a surviving old server answers our own manager's
`configure` -- and an *active* one rejects it, at which point nav2 logs "Failed
to bring up all requested nodes. Aborting bringup" and tears down the servers it
had just started. The pass then sits there logging "nav2 action server not
ready" with no nav2 at all. Stopping those nodes is not enough; they have to be
gone.

**It verifies, and it refuses to start over a collision.** Two different
outcomes are worth telling apart. A tree on the operator PC that cannot be
signalled from here *degrades* a pass and is a warning. A study nav2 server
still holding a name we need *prevents* one, so the preflight exits non-zero and
`launch_autoslam.launch.py` stops rather than starting a stack that provably
cannot come up. `preflight_strict:=false` starts anyway.
"""

import os
import signal
import sys
import time

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.node import Node

from mecanumbot_autoslam import preflight

#: Which shutdown transition takes a lifecycle node out of each state.
SHUTDOWN_TRANSITIONS = {
    State.PRIMARY_STATE_UNCONFIGURED: Transition.TRANSITION_UNCONFIGURED_SHUTDOWN,
    State.PRIMARY_STATE_INACTIVE: Transition.TRANSITION_INACTIVE_SHUTDOWN,
    State.PRIMARY_STATE_ACTIVE: Transition.TRANSITION_ACTIVE_SHUTDOWN,
}

#: Seconds between SIGTERM and SIGKILL for a process that will not go.
KILL_GRACE = 2.0


class AutoslamPreflight(Node):
    """Find what contradicts an autoslam pass and stop it."""

    def __init__(self):
        """Declare the preflight constants."""
        super().__init__("autoslam_preflight")
        self.declare_parameter("preflight_discovery", 2.0)
        self.declare_parameter("preflight_timeout", 15.0)
        # Shutting a whole nav2 stack down is several sequential lifecycle
        # transitions per node plus their bonds, and it does not answer until
        # it has finished all of them. 15 s was not enough for a seven-node
        # stack -- the call timed out, the stack stayed up, and the pass could
        # not start.
        self.declare_parameter("preflight_manager_timeout", 60.0)
        self.declare_parameter("preflight_kill_processes", True)
        self.blocking = []

    # --- discovery ------------------------------------------------------------

    def live_nodes(self):
        """Return every node on the graph, fully qualified."""
        # Discovery is not instant and an empty list would read as "nothing to
        # do", which is the one wrong answer this node can give.
        deadline = time.time() + float(
            self.get_parameter("preflight_discovery").value)
        names = []
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            names = [
                "{}/{}".format(namespace.rstrip("/"), name)
                for name, namespace in self.get_node_names_and_namespaces()
            ]
        return names

    # --- the three methods ----------------------------------------------------

    def shutdown_manager(self, name, timeout):
        """Ask a nav2 lifecycle manager to shut every node it manages down."""
        client = self.create_client(
            ManageLifecycleNodes, "/{}/manage_nodes".format(name))
        if not client.wait_for_service(timeout_sec=timeout):
            return False, "no manage_nodes service"
        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.SHUTDOWN
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            return False, "manage_nodes did not answer"
        return bool(future.result().success), "shut down its managed nodes"

    def shutdown_lifecycle(self, name, timeout):
        """Shut one lifecycle node down through its own change_state."""
        state_client = self.create_client(GetState, "/{}/get_state".format(name))
        if not state_client.wait_for_service(timeout_sec=timeout):
            return False, "not a lifecycle node (no get_state)"
        future = state_client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            return False, "get_state did not answer"

        transition = SHUTDOWN_TRANSITIONS.get(future.result().current_state.id)
        if transition is None:
            return True, "already finalized"

        change_client = self.create_client(
            ChangeState, "/{}/change_state".format(name))
        if not change_client.wait_for_service(timeout_sec=timeout):
            return False, "no change_state service"
        request = ChangeState.Request()
        request.transition.id = transition
        future = change_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            return False, "change_state did not answer"
        return bool(future.result().success), "shut down"

    def signal_processes(self, tokens):
        """SIGTERM every local process matching one of `tokens`."""
        keep_pids = (os.getpid(), os.getppid())
        keep_groups = (os.getpgid(0),)
        hits = preflight.processes_to_signal(
            _local_processes(), tokens,
            keep_pids=keep_pids, keep_groups=keep_groups)
        for pid, token in hits:
            self.get_logger().info("stopping {} (pid {})".format(token, pid))
            _signal(pid, signal.SIGTERM)
        if not hits:
            return []
        time.sleep(KILL_GRACE)
        for pid, token in hits:
            if _alive(pid):
                self.get_logger().warn(
                    "{} (pid {}) ignored SIGTERM; sending SIGKILL".format(
                        token, pid))
                _signal(pid, signal.SIGKILL)
        # A launch file started with respawn=true puts its node back a couple
        # of seconds later, so what was signalled is not the same question as
        # what is now gone. _verify asks the second one.
        return [token for _, token in hits]

    # --- the pass -------------------------------------------------------------

    def run(self):
        """Find the contradictions, clear what can be cleared, report the rest."""
        timeout = float(self.get_parameter("preflight_timeout").value)
        nodes = self.live_nodes()
        steps = preflight.plan(nodes)

        found = sum(len(group) for group in steps.values())
        if not found:
            self.get_logger().info(
                "nothing on the graph contradicts an autoslam pass "
                "({} node(s) seen)".format(len(nodes)))
            return
        for line in preflight.summary(steps):
            self.get_logger().info("contradiction: {}".format(line))

        manager_timeout = float(
            self.get_parameter("preflight_manager_timeout").value)
        for rule in steps[preflight.BY_MANAGER]:
            self.get_logger().info(
                "asking {} to shut its nodes down (up to {:.0f} s -- it answers "
                "only once every one of them has gone)".format(
                    rule.node, manager_timeout))
            _, detail = self.shutdown_manager(rule.node, manager_timeout)
            self._record(rule, "manager", detail)
        for rule in steps[preflight.BY_LIFECYCLE]:
            _, detail = self.shutdown_lifecycle(rule.node, timeout)
            self._record(rule, "lifecycle", detail)

        tokens = preflight.tokens_to_signal(steps)
        may_signal = bool(self.get_parameter("preflight_kill_processes").value)
        if tokens and may_signal:
            self.signal_processes(tokens)
        elif tokens:
            self.get_logger().warn(
                "preflight_kill_processes is false, so nothing was signalled")

        self._verify(steps)

    def _verify(self, steps):
        """
        Look at the graph again, and separate "not tidy" from "cannot start".

        The graph and not the process table: a name collision is about whether
        the name is still taken. A cleanly shut-down lifecycle node still holds
        its name, and a node composed into a container never had a process of
        its own -- asking about processes reported eight live nav2 nodes as "not
        running here" and let the pass start into the collision.
        """
        remaining = self.live_nodes()
        left = preflight.survivors(steps, remaining)
        self.blocking = preflight.blocking(steps, remaining)
        cleared = [
            rule for group in steps.values() for rule in group
            if rule not in left
        ]
        for rule in cleared:
            self.get_logger().info("{}: gone".format(rule.node))

        warnings = [rule for rule in left if not rule.collides]
        if warnings:
            self.get_logger().warn(
                "{} contradiction(s) still on the graph. They do not stop the "
                "pass starting, but they will compete with it. If they are on "
                "another machine, stop them there:".format(len(warnings)))
            for rule in warnings:
                self.get_logger().warn("  {}: {}".format(rule.node, rule.why))

        if self.blocking:
            self.get_logger().error(
                "{} node(s) still hold names autoslam needs. nav2 cannot come "
                "up alongside them: its bringup aborts on the collision and "
                "takes our own servers down with it, so the pass would get no "
                "navigation at all. Refusing to start.".format(
                    len(self.blocking)))
            for rule in self.blocking:
                self.get_logger().error("  {}: {}".format(rule.node, rule.why))
            self.get_logger().error(
                "Stop them by hand -- Ctrl-C the base launch, or restart it "
                "with use_nav2:=false -- or start with preflight_strict:=false "
                "to try anyway.")
            return
        if not warnings:
            self.get_logger().info("the graph is clear; starting the pass")

    def _record(self, rule, how, detail):
        """
        Log what was attempted on one node.

        Only what was *tried* -- whether it worked is `_verify`'s question, and
        it asks the graph rather than believing a service that answered "yes".
        """
        self.get_logger().info("{} ({}): {}".format(rule.node, how, detail))


def _local_processes():
    """Return `(pid, process group, cmdline)` for every readable process."""
    processes = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        pid = int(entry)
        try:
            with open("/proc/{}/cmdline".format(pid), "rb") as handle:
                cmdline = handle.read().decode(errors="replace").split("\0")
            group = os.getpgid(pid)
        except (OSError, ProcessLookupError):
            continue
        if any(cmdline):
            processes.append((pid, group, [part for part in cmdline if part]))
    return processes


def _signal(pid, number):
    """Send one signal, ignoring a process that has already gone."""
    try:
        os.kill(pid, number)
    except (OSError, ProcessLookupError):
        pass


def _alive(pid):
    """Report whether a pid still exists."""
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    return True


def main(args=None):
    """
    Clear the graph, then exit so the rest of the launch can start.

    Exit 0 means the pass may start -- possibly with a warning about something
    on another machine. Exit 1 means a node autoslam re-registers is still
    running here, so nav2 cannot come up and the launch stops instead of
    spending a trial finding that out.
    """
    rclpy.init(args=args)
    node = AutoslamPreflight()
    blocked = []
    try:
        node.run()
        blocked = node.blocking
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    if blocked:
        sys.exit(1)


if __name__ == "__main__":
    main()
