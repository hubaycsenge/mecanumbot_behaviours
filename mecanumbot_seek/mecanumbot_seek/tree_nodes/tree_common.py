"""
YAML resolution and the tree runner for the seek tree.

Both are `mecanumbot_bt_config`'s. What is left here is the two names that
really are this package's: where its constants are packaged, and what its node
is called -- which is why the seek tree can run alongside a leading or ostensive
one and `ros2 node list` says which is which.
"""

from mecanumbot_bt_config import tree_runner

PACKAGE_NAME = "mecanumbot_seek"

# The ROS node name the seek tree registers under.
NODE_NAME = "seek_bt_node"


def resolve_yaml_path(tree_name, default_filename):
    """`--yaml_path`, else `YAML_PATH` / `BEHAVIOUR_YAML_PATH`, else the packaged file."""
    return tree_runner.resolve_yaml_path(tree_name, PACKAGE_NAME, default_filename)


def build_params(yaml_path):
    """Read the constants file for the values needed before the first tick."""
    return tree_runner.build_params(yaml_path)


def run_tree(create_root, tree_name, default_yaml, args=None, tick_period_ms=None):
    """
    Build, set up and spin the seek tree.

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
