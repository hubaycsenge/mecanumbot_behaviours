import os 
import rclpy 
import py_trees 
import py_trees_ros 
from ament_index_python.packages import get_package_share_directory 
from mecanumbot_leading_behaviour.behaviours.dog_behaviours import DogBehaviourSequence, DogCheckFollowing
from mecanumbot_leading_behaviour.behaviours.movement_managers import Approach, FindPeople, TurnToward,\
                                                                     CheckSubjectTargetSuccess, RelativeTurnPattern, CheckRobotHasBall
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
    turn_delay_timer = py_trees.timers.Timer(name="TurnDelayTimer", duration=10)

    Dog_show_target = DogBehaviourSequence('DShow', 'indicate_target')
    Dog_catch_attention_init = DogBehaviourSequence('DCatchInit', 'catch_attention')
    Dog_catch_attention_show_tgt = DogBehaviourSequence('DCatchShow', 'catch_attention')
    Dog_thank = DogBehaviourSequence('DThank', 'thank')

    Dog_check_following = DogCheckFollowing(name="DogCheckFollowing")

    approach_target_step = Approach(name="ApproachTarget", target_type="checkpoint")
    approach_subject_init = Approach(name="ApproachSubjectInit", target_type="subject",mode ="fixed_distance")

    turn_toward_subject_init_seek = TurnToward(name="TurnTowardSubjectInitSeek", target_type="subject")
    turn_toward_target_show = TurnToward(name="TurnTowardTargetShow", target_type="target")
    turn_toward_checkpoint_step = TurnToward(name="TurnTowardCheckpointStep", target_type="checkpoint")
    find_person_init = FindPeople(name="FindPersonInit")
    find_person_ball_reaction = FindPeople(name="FindPersonBallReaction")
    find_person_while_show = FindPeople(name="FindPersonWhileShow")
    find_person_while_lead = FindPeople(name="FindPersonWhileLead")
    attention_turn_pattern_init = RelativeTurnPattern(name="AttentionTurnPatternInit", step_angle_deg=15.0)
    attention_turn_pattern_show = RelativeTurnPattern(name="AttentionTurnPatternShow", step_angle_deg=15.0)
    check_if_ball_given = CheckRobotHasBall(name="CheckIfBallGiven")



    check_subject_near_target = CheckSubjectTargetSuccess(name="CheckSubjectNearTarget")
    
    ball_reaction_seq = py_trees.composites.Sequence(name="BallReactionSeq",memory=True)
    ball_reaction_seq.add_children([
        check_if_ball_given,
        find_person_ball_reaction,
        Dog_thank
    ])
    ball_reaction_repeat = py_trees.decorators.Repeat(name="BallReactionRepeat", child=ball_reaction_seq, num_success=-1)

    # Approach subject and catch attention
    seek_attention_init = py_trees.composites.Sequence( # seems OK 
        name="SeekAttentionInit",
        memory=True
    )
    seek_attention_init.add_children([
        find_person_init,
        approach_subject_init,
        turn_toward_subject_init_seek, # TODO: kell?
        attention_turn_pattern_init,
        Dog_catch_attention_init
    ])

    # Indicate close target until subject is close enough
    show_while_close_seq = py_trees.composites.Sequence(
        name="ShowWhileSubjectCloseSeq",
        memory=True
    )

    show_while_close_seq.add_children([
        find_person_while_show,
        check_subject_near_target,
        turn_delay_timer,
        Dog_catch_attention_show_tgt,
        turn_toward_target_show,
        Dog_show_target
    ]) 


    lead_step_sequence = py_trees.composites.Sequence(
        name="LeadStepSequence",
        memory=True
    )

    lead_step_sequence.add_children([ 
        find_person_while_lead,
        check_subject_near_target,
        Dog_check_following,
        turn_toward_checkpoint_step,
        approach_target_step, 
        delay_timer
    ])

    behaviour_selector = py_trees.composites.Selector("ShowOrLeadSelector",memory=True)

    behaviour_selector.add_children(
        [
            ball_reaction_repeat,
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

    