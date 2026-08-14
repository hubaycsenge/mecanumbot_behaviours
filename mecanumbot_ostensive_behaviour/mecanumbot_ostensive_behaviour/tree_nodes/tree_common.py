"""
YAML resolution and the tree runner for the ostensive trees.

Both are `mecanumbot_bt_config`'s now. This used to be a copy of the leading
package's `tree_common`, because that one hard-coded `bottom_up_tree_node` as
both the ROS node name and the root key of the constants YAML. The runner takes
both as arguments and the loader reads the root key off the file, so what is
left here is the two names that really are this package's: where its constants
are packaged, and what its node is called -- which is why the ostensive tree can
run alongside a leading one and `ros2 node list` says which is which.
"""

from mecanumbot_bt_config import tree_runner

PACKAGE_NAME = "mecanumbot_ostensive_behaviour"

# The ROS node name the ostensive tree registers under.
NODE_NAME = "ostensive_bt_node"


def resolve_yaml_path(tree_name, default_filename):
    """`--yaml_path`, else `YAML_PATH` / `BEHAVIOUR_YAML_PATH`, else the packaged file."""
    return tree_runner.resolve_yaml_path(tree_name, PACKAGE_NAME, default_filename)


def build_params(yaml_path):
    """Read the constants file for the values needed before the first tick."""
    return tree_runner.build_params(yaml_path)


def run_tree(create_root, tree_name, default_yaml, args=None, tick_period_ms=None):
    """
    Build, set up and spin the ostensive tree.

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
