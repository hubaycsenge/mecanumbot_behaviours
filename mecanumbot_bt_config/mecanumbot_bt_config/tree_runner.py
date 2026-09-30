"""
Finding a tree's constants file, and spinning the tree on it.

Every executable tree in this repository needs the same three things: work out
which YAML to load, pre-read the handful of values that are settled while the
tree is still being built, and spin. Both behaviour packages used to carry their
own copy of this, and the copies differed only in the package they fall back to
and the node name they register under -- so those are arguments here.

The two runtime values are the only key names this package knows, and they are
here because they belong to the runner rather than to any behaviour: a tree that
has not been built yet has no blackboard to read them from.
"""

import argparse
import os
import signal
import time

import py_trees
import py_trees_ros
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseArray, Twist
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions

from mecanumbot_bt_config.blackboard import Tunables
from mecanumbot_bt_config.params import load_params

RUNTIME_DEFAULTS = {
    "tick_period_ms": 100.0,
    "setup_timeout": 15.0,
    # Seconds to wait for the people detector before the first tick. 0.0 does
    # not wait at all, which is the behaviour every tree had until 2026-09-23.
    "perception_wait_timeout": 20.0,
}

RUNTIME = Tunables(RUNTIME_DEFAULTS)

# Where the trees' in-place turns go (`VelocityCommander` in the movement
# library). Spelled here too because this package sits below that one.
CMD_VEL_TOPIC = "/cmd_vel"
STOP_REPEATS = 5

# What the LiDAR people detector publishes on every scan, empty or not, once it
# is running -- so its first message is exactly "I am up". `people_fusion` is
# the wrong thing to wait for: it is only published when somebody is actually
# being tracked, so in an empty room it never arrives.
PERCEPTION_TOPIC = "/mecanumbot/dets"


def resolve_yaml_path(tree_name, package_name, default_filename):
    """`--yaml_path`, else `YAML_PATH` / `BEHAVIOUR_YAML_PATH`, else the packaged file."""
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--yaml_path", type=str, default=None)
    parsed, _ = parser.parse_known_args()
    yaml_path = (
        parsed.yaml_path or os.getenv("YAML_PATH") or os.getenv("BEHAVIOUR_YAML_PATH")
    )
    if yaml_path:
        print(f"[{tree_name}] Using YAML_PATH: {yaml_path}")
        return yaml_path

    fallback = os.path.join(
        get_package_share_directory(package_name), "config", default_filename
    )
    print(f"[{tree_name}] YAML_PATH unset, fallback to: {fallback}")
    return fallback


def build_params(yaml_path, root_keys=None):
    """
    Read the constants file for the values needed while building the tree.

    The tick period, the setup timeout and a decorator's retry count are all
    settled before the first tick, which is before `ParamsToBlackboard` has run,
    so they come from the file rather than from the blackboard. A file that
    cannot be read is not fatal here -- the loader behaviour reports it properly
    a moment later, and until then the packaged defaults apply.
    """
    try:
        return load_params(yaml_path, root_keys)
    except Exception as error:  # unreadable, malformed, or not a param file
        print(f"[tree_runner] could not pre-read {yaml_path} ({error}); using defaults")
        return {}


def wait_for_perception(node, timeout, topic=PERCEPTION_TOPIC):
    """
    Block until the people detector has published once, or `timeout` passes.

    DR-SPAAM spends about seven seconds loading torch and warming CUDA up, and
    the tree used to start ticking straight through it. In the runs of
    2026-09-23 that meant the opening behaviour -- three seconds of looking for
    the subject to approach -- ran and gave up entirely before the detector
    existed: the tree asked whether anybody was there while nothing was in a
    position to answer, and then acted on the silence.

    Waiting for the detector's first message is enough, because it publishes on
    every scan whether or not it found anyone. Returns whether it arrived: a
    timeout is not fatal, since a run with `use_perception:=false` is a
    legitimate thing to do and the tree should still start.
    """
    if timeout <= 0.0:
        return True

    arrived = []
    subscription = node.create_subscription(
        PoseArray, topic, lambda _msg: arrived.append(True), 10
    )
    try:
        deadline = time.monotonic() + timeout
        while not arrived and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_subscription(subscription)
    return bool(arrived)


def run_tree(
    create_root,
    tree_name,
    package_name,
    node_name,
    default_yaml,
    args=None,
    tick_period_ms=None,
    root_keys=None,
):
    """
    Build, set up and spin one behaviour tree.

    `tick_period_ms` defaults to the constants file's `tick_period_ms`; passing
    one overrides it.
    """
    # rclpy's own SIGINT handler shuts the context down before `spin` returns,
    # and a dead context cannot publish -- so the tree could not stop the robot
    # on Ctrl-C, and the last turn command kept the wheels going. Take the
    # signals ourselves; `stop_robot` runs while the context is still alive.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, signal.default_int_handler)

    yaml_path = resolve_yaml_path(tree_name, package_name, default_yaml)
    params = build_params(yaml_path, root_keys)
    if tick_period_ms is None:
        tick_period_ms = RUNTIME.file_constant(params, "tick_period_ms")

    tree_node = py_trees_ros.trees.BehaviourTree(root=create_root(yaml_path=yaml_path))
    tree_node.setup(
        timeout=float(RUNTIME.file_constant(params, "setup_timeout")),
        node_name=node_name,
    )
    # Before the first tick, not after: the opening behaviour of a leading tree
    # asks whether it can see anybody, and must not be answered by a detector
    # that has not started.
    wait_timeout = float(RUNTIME.file_constant(params, "perception_wait_timeout"))
    if wait_for_perception(tree_node.node, wait_timeout):
        if wait_timeout > 0.0:
            tree_node.node.get_logger().info(
                f"people detector is publishing on {PERCEPTION_TOPIC}; starting the tree"
            )
    else:
        tree_node.node.get_logger().warn(
            f"nothing on {PERCEPTION_TOPIC} after {wait_timeout:g}s -- starting the "
            "tree anyway, but every behaviour that looks for a person will fail "
            "until the detector comes up. Is perception running?"
        )

    print(f"Starting {tree_name} behaviour tree using YAML: {yaml_path}")

    tree_node.tick_tock(period_ms=float(tick_period_ms))
    try:
        rclpy.spin(tree_node.node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        stop_robot(tree_node)
        rclpy.try_shutdown()


def stop_robot(tree_node):
    """
    Halt whatever the tree set moving, before the process exits.

    Stopping the root terminates every running behaviour, which is where the
    nav2 goals are cancelled; the zero twists then end an in-place turn, which
    nothing else would. Best effort throughout: a failure in one behaviour's
    `terminate` must not keep the stop command from going out.
    """
    node = tree_node.node
    try:
        if tree_node.timer is not None:
            tree_node.timer.cancel()
        tree_node.root.stop(py_trees.common.Status.INVALID)
    except Exception as error:  # shutting down regardless
        print(f"[tree_runner] stopping the tree failed ({error}); stopping the wheels anyway")
    try:
        publisher = node.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        for _ in range(STOP_REPEATS):
            publisher.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.05)  # also flushes the nav2 cancels
    except Exception as error:
        print(f"[tree_runner] could not send the stop command: {error}")
