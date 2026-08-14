import rclpy
import py_trees
import py_trees_ros

from mecanumbot_demo_behaviours.behaviours.movement_managers import (
    DEMO_DEFAULTS,
    FindPeople,
    GoToRandomPerson,
)
from mecanumbot_leading_behaviour.behaviours.LED_behaviours import LEDBehaviourSequence
from mecanumbot_leading_behaviour.behaviours.blackboard_managers import (
    LED_SCRIPTS,
    ConstantParamsToBlackboard,
)
from mecanumbot_leading_behaviour.behaviours.defaults import file_constant
from mecanumbot_leading_behaviour.tree_nodes.tree_common import build_params
from ament_index_python.packages import get_package_share_directory

leading_pkg_share_dir = get_package_share_directory("mecanumbot_leading_behaviour")
DEFAULT_YAML_FILENAME = "Eto_behaviour_setting_constants.yaml"


def get_yaml_path():
    import argparse
    import os

    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--yaml_path", type=str, default=None)
    parsed, _ = parser.parse_known_args()

    yaml_path = parsed.yaml_path
    if not yaml_path:
        yaml_path = os.getenv("YAML_PATH") or os.getenv("BEHAVIOUR_YAML_PATH")

    if yaml_path:
        print(f"[LED_tree] Using YAML_PATH: {yaml_path}")
        return yaml_path

    fallback = os.path.join(leading_pkg_share_dir, "config", DEFAULT_YAML_FILENAME)
    print(f"[LED_tree] YAML_PATH unset, fallback to: {fallback}")
    return fallback


def create_root(yaml_path=None):
    if yaml_path is None:
        yaml_path = get_yaml_path()
    params_loader = ConstantParamsToBlackboard(
        name="LoadConstantParams",
        yaml_path=yaml_path,
        scripts=LED_SCRIPTS,
        defaults=(DEMO_DEFAULTS,),
    )
    LED_catch_attention = LEDBehaviourSequence("LCatch", "catch_attention")

    going_to_random_person_seq = py_trees.composites.Sequence(
        name="GoingToRandomPerson", memory=True
    )
    going_to_random_person_seq.add_children(
        [GoToRandomPerson(name="GoToRandomPerson"), LED_catch_attention]
    )

    root = py_trees.composites.Sequence("ROOT", memory=True)
    root.add_children(
        [params_loader, FindPeople(name="FindPeople"), going_to_random_person_seq]
    )
    return root


def main(args=None):
    rclpy.init(args=args)

    yaml_path = get_yaml_path()
    params = build_params(yaml_path)

    tree_node = py_trees_ros.trees.BehaviourTree(root=create_root(yaml_path))
    tree_node.setup(
        timeout=float(file_constant(params, "setup_timeout")),
        node_name="wander_between_people_node",
    )
    print("Starting wander-between-people behaviour tree")

    # This tree ticks faster than the leading ones: it drives the spin on
    # /cmd_vel itself, unprofiled, rather than handing a turn to SmoothTurner.
    tree_node.tick_tock(
        period_ms=float(file_constant(params, "demo_tick_period_ms", DEMO_DEFAULTS))
    )
    rclpy.spin(tree_node.node)


if __name__ == "__main__":
    main()
