"""
Scaffolding shared by the executable leading trees.

Finding the constants file and spinning the tree are `mecanumbot_bt_config`'s
job; what is left here is what belongs to *these* trees -- the package their
constants are packaged in, the node name they register under, and the lost-human
recovery branch every one of them builds.
"""

import py_trees

from mecanumbot_bt_config import tree_runner
from mecanumbot_movement_behaviours.targets import PATROL
from mecanumbot_movement_behaviours.turning import Spin360

from mecanumbot_leading_behaviour.behaviours.route_behaviours import (
    Approach,
    ManageSearchCheckpoint,
    WaitForPerson,
)

PACKAGE_NAME = "mecanumbot_leading_behaviour"

# All trees register under this node name, which is also the root key the
# constants files happen to be nested under -- though nothing depends on the two
# matching any more, because the loader reads the root key off the file.
NODE_NAME = "bottom_up_tree_node"


def resolve_yaml_path(tree_name, default_filename):
    """`--yaml_path`, else `YAML_PATH` / `BEHAVIOUR_YAML_PATH`, else the packaged file."""
    return tree_runner.resolve_yaml_path(tree_name, PACKAGE_NAME, default_filename)


def build_params(yaml_path):
    """Read the constants file for the values needed while building the tree."""
    return tree_runner.build_params(yaml_path)


def create_recover_lost_sequence(ID=""):
    """
    Search the route for a human who stopped following.

    Scan a full circle, step to the next search checkpoint, drive there,
    repeat.
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
            WaitForPerson(name=ID + "InterruptOnPersonFound"),
            py_trees.decorators.Repeat(
                name=ID + "SearchLoop", child=patrol, num_success=-1
            ),
        ]
    )
    return recovery


def run_tree(create_root, tree_name, default_yaml, args=None, tick_period_ms=None):
    """
    Build, set up and spin one leading tree.

    `tick_period_ms` defaults to the constants file's `tick_period_ms`; passing
    one overrides it.
    """
    tree_runner.run_tree(
        create_root,
        tree_name,
        PACKAGE_NAME,
        NODE_NAME,
        default_yaml,
        args=args,
        tick_period_ms=tick_period_ms,
    )
