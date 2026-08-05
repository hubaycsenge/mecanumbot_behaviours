"""Scaffolding shared by the executable behaviour trees.

Every tree needs the same three things: find its YAML, build a lost-human
recovery branch, and spin. They live here so the tree files only contain the
behaviour composition that makes each condition different.
"""

import argparse
import os

import py_trees
import py_trees_ros
import rclpy
from ament_index_python.packages import get_package_share_directory

from mecanumbot_leading_behaviour.behaviours.movement_managers import Approach
from mecanumbot_leading_behaviour.behaviours.searching import (
    ManageSearchCheckpoint,
    WaitForPerson,
)
from mecanumbot_leading_behaviour.behaviours.targets import PATROL
from mecanumbot_leading_behaviour.behaviours.turning import Spin360

PACKAGE_NAME = "mecanumbot_leading_behaviour"

# All trees register under this node name because it is also the root key the
# constants YAML files are nested under.
NODE_NAME = "bottom_up_tree_node"

TICK_PERIOD_MS = 100.0
SETUP_TIMEOUT_S = 15.0


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


def create_recover_lost_sequence(ID=""):
    """Search the route for a human who stopped following.
    scan a full circle, step to the next search checkpoint, drive there, repeat.
    The parallel ends the moment somebody is spotted.
    """
    patrol = py_trees.composites.Sequence(name=ID + "SearchSequence", memory=True)
    patrol.add_children(
        [
            Spin360(name=ID + "FullCircleScan"),
            ManageSearchCheckpoint(name=ID + "UpdateSearchIndex"),
            Approach(name=ID + "GoToSearchCheckpoint", target_type=PATROL),
        ]
    )

    recovery = py_trees.composites.Parallel(
        name=ID + "HandleLostParallel",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne(),
    )
    recovery.add_children(
        [
            WaitForPerson(name=ID + "InterruptOnPersonFound", sight_timeout=1.0),
            py_trees.decorators.Repeat(
                name=ID + "SearchLoop", child=patrol, num_success=-1
            ),
        ]
    )
    return recovery


def run_tree(
    create_root, tree_name, default_yaml, args=None, tick_period_ms=TICK_PERIOD_MS
):
    """Build, set up and spin one behaviour tree."""
    rclpy.init(args=args)

    yaml_path = resolve_yaml_path(tree_name, default_yaml)
    tree_node = py_trees_ros.trees.BehaviourTree(root=create_root(yaml_path=yaml_path))
    tree_node.setup(timeout=SETUP_TIMEOUT_S, node_name=NODE_NAME)
    print(f"Starting {tree_name} behaviour tree using YAML: {yaml_path}")

    tree_node.tick_tock(period_ms=tick_period_ms)
    rclpy.spin(tree_node.node)
