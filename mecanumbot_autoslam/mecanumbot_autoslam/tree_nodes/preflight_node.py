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
3. **SIGTERM, then SIGKILL**, for the plain nodes -- behaviour trees,
   slam_toolbox, an old explorer. Matched on the executable name in a process's
   command line, never a free-text search, and never against this process or
   anything in its own process group.

**It reports what it could not do.** A node on the operator PC cannot be
signalled from here and often cannot be shut down either; saying so in the log
is the difference between a pass that fails mysteriously and one that fails with
its cause on the screen. That is also why the exit code is always 0: a
contradiction that could not be cleared is a warning to a person, not a reason
to refuse to explore.
"""

import os
import signal
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
        self.declare_parameter("preflight_kill_processes", True)
        self.unresolved = []

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

    def signal_processes(self, executables):
        """SIGTERM every local process running one of `executables`."""
        keep_pids = (os.getpid(), os.getppid())
        keep_groups = (os.getpgid(0),)
        hits = preflight.processes_to_signal(
            _local_processes(), executables,
            keep_pids=keep_pids, keep_groups=keep_groups)
        for pid, executable in hits:
            self.get_logger().info(
                "stopping {} (pid {})".format(executable, pid))
            _signal(pid, signal.SIGTERM)
        if not hits:
            return []
        time.sleep(KILL_GRACE)
        for pid, executable in hits:
            if _alive(pid):
                self.get_logger().warn(
                    "{} (pid {}) ignored SIGTERM; sending SIGKILL".format(
                        executable, pid))
                _signal(pid, signal.SIGKILL)
        return [executable for _, executable in hits]

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

        for rule in steps[preflight.BY_MANAGER]:
            ok, detail = self.shutdown_manager(rule.node, timeout)
            self._record(rule, ok, detail)
        for rule in steps[preflight.BY_LIFECYCLE]:
            ok, detail = self.shutdown_lifecycle(rule.node, timeout)
            self._record(rule, ok, detail)

        executables = preflight.executables_to_signal(steps)
        may_signal = bool(self.get_parameter("preflight_kill_processes").value)
        stopped = (
            self.signal_processes(executables)
            if executables and may_signal else []
        )
        for rule in steps[preflight.BY_PROCESS]:
            if not may_signal:
                self._record(rule, False, "preflight_kill_processes is false")
                continue
            candidates = preflight.candidate_executables(rule)
            hit = [name for name in stopped if name in candidates]
            self._record(
                rule, bool(hit),
                "signalled {}".format(", ".join(hit)) if hit
                else "no local process running it -- another machine?",
            )

        if self.unresolved:
            self.get_logger().warn(
                "{} contradiction(s) still running -- an autoslam pass may "
                "fight them. If they are on another machine, stop them "
                "there:".format(len(self.unresolved)))
            for line in self.unresolved:
                self.get_logger().warn("  {}".format(line))
        else:
            self.get_logger().info("the graph is clear; starting the pass")

    def _record(self, rule, ok, detail):
        """Log one outcome, and remember it if the node is still there."""
        if ok:
            self.get_logger().info("{}: {}".format(rule.node, detail))
        else:
            self.unresolved.append("{}: {}".format(rule.node, detail))


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
    """Clear the graph, then exit so the rest of the launch can start."""
    rclpy.init(args=args)
    node = AutoslamPreflight()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
