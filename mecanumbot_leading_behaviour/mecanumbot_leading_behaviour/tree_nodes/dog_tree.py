import os 
import rclpy 
import py_trees 
import py_trees_ros 
from ament_index_python.packages import get_package_share_directory 
from mecanumbot_leading_behaviour.behaviours.dog_behaviours import DogBehaviourSequence, DogCheckFollowing, DogSelectTarget
from mecanumbot_leading_behaviour.behaviours.movement_managers import Approach, \
                                                                      TurnToward, CheckSubjectTargetSuccess
from mecanumbot_leading_behaviour.behaviours.blackboard_managers import ConstantParamsToBlackboard

leading_pkg_share_dir = get_package_share_directory('mecanumbot_leading_behaviour')
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
        print(f"[dog_tree] Using YAML_PATH: {yaml_path}")
        return yaml_path

    fallback = os.path.join(leading_pkg_share_dir, "config", DEFAULT_YAML_FILENAME)
    print(f"[dog_tree] YAML_PATH unset, fallback to: {fallback}")
    return fallback


def create_root(yaml_path=None):
    if yaml_path is None:
        yaml_path = get_yaml_path()

    root = py_trees.composites.Sequence("ROOT", memory=True)

    params_loader = ConstantParamsToBlackboard(name="LoadConstantParams", yaml_path=yaml_path)


    delay_timer = py_trees.timers.Timer(name="DelayTimer", duration=1)

    Dog_show_target = DogBehaviourSequence('DShow', 'indicate_target')
    Dog_catch_attention_init = DogBehaviourSequence('DCatchInit', 'catch_attention')
    Dog_catch_attention_show_tgt = DogBehaviourSequence('DCatchShow', 'catch_attention')
    Dog_check_following = DogCheckFollowing(name="DogCheckFollowing")

    approach_target_step = Approach(name="ApproachTarget", target_type="checkpoint")
    approach_subject_init = Approach(name="ApproachSubjectInit", target_type="subject",mode ="fixed_distance")

    turn_toward_subject_init_seek = TurnToward(name="TurnTowardSubjectInitSeek", target_type="subject")
    turn_toward_subject_check_follow = TurnToward(name="TurnTowardSubjectCheckFollow", target_type="subject")
    turn_toward_subject_show_tgt = TurnToward(name="TurnTowardSubjectShowTgt", target_type="subject")
    turn_toward_target_show = TurnToward(name="TurnTowardTargetShow", target_type="target")
    turn_toward_checkpoint_step = TurnToward(name="TurnTowardCheckpointStep", target_type="checkpoint")

    check_subject_near_target = CheckSubjectTargetSuccess(name="CheckSubjectNearTarget")
    
    

    # Approach subject and catch attention
    seek_attention_init = py_trees.composites.Sequence( # seems OK 
        name="SeekAttentionInit",
        memory=True
    )
    seek_attention_init.add_children([
        approach_subject_init,
        turn_toward_subject_init_seek,
        Dog_catch_attention_init
    ])

    # Indicate close target until subject is close enough
    show_while_close_seq = py_trees.composites.Sequence(
        name="ShowWhileSubjectCloseSeq",
        memory=True
    )

    show_while_close_seq.add_children([
        check_subject_near_target,
        turn_toward_subject_show_tgt,
        Dog_catch_attention_show_tgt,
        turn_toward_target_show,
        Dog_show_target
    ]) 


    lead_step_sequence = py_trees.composites.Sequence(
        name="LeadStepSequence",
        memory=True
    )
    lead_step_sequence.add_children([ 
        Dog_check_following,
        turn_toward_checkpoint_step,
        approach_target_step, 
        turn_toward_subject_check_follow,
        delay_timer
    ])

    behaviour_selector = py_trees.composites.Selector("ShowOrLeadSelector",memory=True)
    behaviour_selector.add_children(
        [
            show_while_close_seq,
            lead_step_sequence
        ]
    )

    behaviour_loop = py_trees.decorators.Repeat("ShowOrLeadLoop",
                                                behaviour_selector,
                                                num_success=-1)
    root.add_children(
            [params_loader,
            seek_attention_init,
            behaviour_loop]
    )


    return root

def main(args=None):
    rclpy.init(args=args)

    yaml_path = get_yaml_path()
    tree = create_root(yaml_path=yaml_path)

    tree_node = py_trees_ros.trees.BehaviourTree(root=tree)
    tree_node.setup(timeout=15.0, node_name="bottom_up_tree_node")
    print(f"Starting doglike behaviour tree using YAML: {yaml_path}")

    tree_node.tick_tock(period_ms=100.0)
    rclpy.spin(tree_node.node)     # <--- keeps node alive


if __name__ == "__main__":
    main()

    