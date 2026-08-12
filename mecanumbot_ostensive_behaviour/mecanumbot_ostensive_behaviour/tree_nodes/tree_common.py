"""
YAML resolution and the tree runner for the ostensive trees.

Deliberately a copy of the leading package's `tree_common` rather than an import
of it: that one hard-codes `bottom_up_tree_node` as both the ROS node name and
the root key of the constants YAML, which is why every leading executable
registers under the same node name. This package uses its own name, so the two
trees can run side by side and `ros2 node list` says which is which.
"""

import argparse
import os

import py_trees_ros
import rclpy
from ament_index_python.packages import get_package_share_directory

from mecanumbot_leading_behaviour.behaviours.constants import (
    file_constant,
    load_params,
)

PACKAGE_NAME = "mecanumbot_ostensive_behaviour"

# The ROS node name, and the root key the constants YAML is nested under.
NODE_NAME = "ostensive_bt_node"

YAML_ROOT_KEYS = (NODE_NAME, "ros__parameters")


def resolve_yaml_path(tree_name, default_filename):
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
        get_package_share_directory(PACKAGE_NAME), "config", default_filename
    )
    print(f"[{tree_name}] YAML_PATH unset, fallback to: {fallback}")
    return fallback


def build_params(yaml_path):
    """
    Read the constants file for the values needed before the first tick.

    The tick period and the setup timeout are settled while the tree is being
    built, which is before `OstensiveParamsToBlackboard` runs, so they come from
    the file. An unreadable file is not reported here -- the loader behaviour
    does that properly a moment later.
    """
    try:
        return load_params(yaml_path, YAML_ROOT_KEYS)
    except Exception as error:  # unreadable, malformed, or missing the root key
        print(f"[tree_common] could not pre-read {yaml_path} ({error}); using defaults")
        return {}


def run_tree(create_root, tree_name, default_yaml, args=None, tick_period_ms=None):
    """
    Build, set up and spin one behaviour tree.

    `tick_period_ms` defaults to the constants file's `tick_period_ms`; passing
    one overrides it.
    """
    rclpy.init(args=args)

    yaml_path = resolve_yaml_path(tree_name, default_yaml)
    params = build_params(yaml_path)
    if tick_period_ms is None:
        tick_period_ms = file_constant(params, "tick_period_ms")

    tree_node = py_trees_ros.trees.BehaviourTree(root=create_root(yaml_path=yaml_path))
    tree_node.setup(
        timeout=float(file_constant(params, "setup_timeout")), node_name=NODE_NAME
    )
    print(f"Starting {tree_name} behaviour tree using YAML: {yaml_path}")

    tree_node.tick_tock(period_ms=float(tick_period_ms))
    rclpy.spin(tree_node.node)
