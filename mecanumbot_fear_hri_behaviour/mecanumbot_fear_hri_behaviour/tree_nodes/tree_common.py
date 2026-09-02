"""YAML resolution and tree runner for Fear HRI."""

from mecanumbot_bt_config import tree_runner

PACKAGE_NAME = "mecanumbot_fear_hri_behaviour"
NODE_NAME = "fear_hri_bt_node"


def resolve_yaml_path(tree_name, default_filename):
    return tree_runner.resolve_yaml_path(tree_name, PACKAGE_NAME, default_filename)


def build_params(yaml_path):
    return tree_runner.build_params(yaml_path)


def run_tree(create_root, tree_name, default_yaml, args=None, tick_period_ms=None):
    tree_runner.run_tree(
        create_root,
        tree_name,
        PACKAGE_NAME,
        NODE_NAME,
        default_yaml,
        args=args,
        tick_period_ms=tick_period_ms,
    )
