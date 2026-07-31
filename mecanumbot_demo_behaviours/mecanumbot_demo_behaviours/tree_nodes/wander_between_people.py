import rclpy
import py_trees
import py_trees_ros

from mecanumbot_demo_behaviours.behaviours.movement_managers import (
    FindPeople,
    GoToRandomPerson,
)
from mecanumbot_leading_behaviour.behaviours.LED_behaviours import LEDBehaviourSequence
from mecanumbot_leading_behaviour.behaviours.blackboard_managers import (
    ConstantParamsToBlackboard,
)
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


def create_root():
    yaml_path = get_yaml_path()
    params_loader = ConstantParamsToBlackboard(
        name="LoadConstantParams", yaml_path=yaml_path
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

    tree = create_root()
    tree_node = py_trees_ros.trees.BehaviourTree(root=tree)

    tree_node.setup(timeout=15.0, node_name="wander_between_people_node")
    find_people = FindPeople(name="FindPeople")
    go_to_random_person = GoToRandomPerson(name="GoToRandomPerson")
    greet = LEDBehaviourSequence(name="Greet")
    print("Starting wander-between-people behaviour tree")

    tree_node.tick_tock(period_ms=10.0)
    rclpy.spin(tree_node.node)


if __name__ == "__main__":
    main()
