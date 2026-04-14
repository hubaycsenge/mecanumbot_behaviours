import os 
import rclpy 
import py_trees 
import py_trees_ros 
from ament_index_python.packages import get_package_share_directory 
from mecanumbot_leading_behaviour.behaviours.movement_managers import Approach, \
                                                                      TurnToward, CheckRobotAtLastCheckpoint
from mecanumbot_leading_behaviour.behaviours.blackboard_managers import ConstantParamsToBlackboard

leading_pkg_share_dir = get_package_share_directory('mecanumbot_leading_behaviour')
DEFAULT_YAML_FILENAME = "behaviour_setting_constants.yaml"


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
        print(f"[ctrl_tree] Using YAML_PATH: {yaml_path}")
        return yaml_path

    fallback = os.path.join(leading_pkg_share_dir, "config", DEFAULT_YAML_FILENAME)
    print(f"[ctrl_tree] YAML_PATH unset, fallback to: {fallback}")
    return fallback


def create_root(yaml_path=None):
    if yaml_path is None:
        yaml_path = get_yaml_path()

    root = py_trees.composites.Sequence("ROOT", memory=True)

    params_loader = ConstantParamsToBlackboard(name="LoadConstantParams", yaml_path=yaml_path)
    turn_toward_target = TurnToward(name="TurnTowardTarget", target_type="target")
    turn_toward_start = TurnToward(name="TurnTowardStart", target_type="start")
    approach_target = Approach(name="ApproachTarget", target_type="target")    
    approach_start = Approach(name="ApproachStart", target_type="start")
    go_to_last_checkpoint_seq = py_trees.composites.Sequence(
        name="GoToLastCheckpoint",
        memory=True
    )
    approach_ckpt = Approach(name="ApproachCheckpoint", target_type="checkpoint")
    check_if_at_last_checkpoint = CheckRobotAtLastCheckpoint(name="CheckAtLastCheckpoint")
    go_to_last_checkpoint_seq.add_children([
        approach_ckpt,
        check_if_at_last_checkpoint
    ])
    approach_target_loop = py_trees.decorators.Retry(
        name="ApproachTargetLoop",
        child=go_to_last_checkpoint_seq,
        num_failures=-1 
    )


    root.add_children([params_loader, turn_toward_start, approach_start, turn_toward_target, approach_target])


    return root 

def main(args=None):
    rclpy.init(args=args)

    yaml_path = get_yaml_path()
    tree = create_root(yaml_path=yaml_path)

    tree_node = py_trees_ros.trees.BehaviourTree(root=tree)
    tree_node.setup(timeout=15.0, node_name="bottom_up_tree_node")
    print(f"Starting control behaviour tree using YAML: {yaml_path}")

    tree_node.tick_tock(period_ms=10.0)
    rclpy.spin(tree_node.node)     # <--- keeps node alive


if __name__ == "__main__":
    main()