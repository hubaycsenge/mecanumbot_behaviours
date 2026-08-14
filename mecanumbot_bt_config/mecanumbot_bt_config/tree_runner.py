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

import py_trees_ros
import rclpy
from ament_index_python.packages import get_package_share_directory

from mecanumbot_bt_config.blackboard import Tunables
from mecanumbot_bt_config.params import load_params

RUNTIME_DEFAULTS = {
    "tick_period_ms": 100.0,
    "setup_timeout": 15.0,
}

RUNTIME = Tunables(RUNTIME_DEFAULTS)


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
    rclpy.init(args=args)

    yaml_path = resolve_yaml_path(tree_name, package_name, default_yaml)
    params = build_params(yaml_path, root_keys)
    if tick_period_ms is None:
        tick_period_ms = RUNTIME.file_constant(params, "tick_period_ms")

    tree_node = py_trees_ros.trees.BehaviourTree(root=create_root(yaml_path=yaml_path))
    tree_node.setup(
        timeout=float(RUNTIME.file_constant(params, "setup_timeout")),
        node_name=node_name,
    )
    print(f"Starting {tree_name} behaviour tree using YAML: {yaml_path}")

    tree_node.tick_tock(period_ms=float(tick_period_ms))
    rclpy.spin(tree_node.node)
