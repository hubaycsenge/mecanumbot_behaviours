import os 
import rclpy 
import py_trees 
import py_trees_ros 
from ament_index_python.packages import get_package_share_directory 
from mecanumbot_leading_behaviour.behaviours.dog_behaviours import DogBehaviourSequence, DogCheckFollowing, DogCheckIfReached
from mecanumbot_leading_behaviour.behaviours.movement_managers import Approach, \
                                                                      TurnToward, CheckSubjectTargetSuccess
from mecanumbot_leading_behaviour.behaviours.blackboard_managers import ConstantParamsToBlackboard

leading_pkg_share_dir = get_package_share_directory('mecanumbot_leading_behaviour') 
YAML_PATH = os.path.join(leading_pkg_share_dir,'config',"behaviour_setting_constants.yaml") 
def create_root():
    root = py_trees.composites.Sequence("ROOT", memory=True)

    params_loader = ConstantParamsToBlackboard(
        name="LoadConstantParams", yaml_path=YAML_PATH
    )

    delay_timer = py_trees.timers.Timer(name="DelayTimer", duration=1)

    Dog_show_target = DogBehaviourSequence('DShow', 'indicate_target')
    Dog_catch_attention_init = DogBehaviourSequence('DCatchInit', 'catch_attention')
    Dog_catch_attention_recov = DogBehaviourSequence('DCatchRecov', 'catch_attention')
    Dog_catch_attention_show_tgt = DogBehaviourSequence('DCatchShow', 'catch_attention')
    Dog_check_following = DogCheckFollowing(name="DogCheckFollowing")
    Dog_check_if_target_reached = DogCheckIfReached(name="DogCheckIfTargetReached")

    approach_target_step = Approach(name="ApproachTarget", target_type="target",mode="fixed_distance")
    approach_subject_init = Approach(name="ApproachSubjectInit", target_type="subject")
    #approach_subject_recov = Approach(name="ApproachSubjectRecov", target_type="subject")

    turn_toward_subject_init_seek = TurnToward(name="TurnTowardSubjectInitSeek", target_type="subject")
    turn_toward_subject_check_follow = TurnToward(name="TurnTowardSubjectCheckFollow", target_type="subject")
    #turn_toward_subject_recov_seek = TurnToward(name="TurnTowardSubjectRecovSeek", target_type="subject")
    turn_toward_subject_show_tgt = TurnToward(name="TurnTowardSubjectShowTgt", target_type="subject")
    turn_toward_target_show = TurnToward(name="TurnTowardTargetShow", target_type="target")
    turn_toward_target_check_follow = TurnToward(name="TurnTowardTargetCheckFollow", target_type="target")

    check_subject_near_target = CheckSubjectTargetSuccess(name="CheckSubjectNearTarget")
    check_subject_near_target_exit_leading = CheckSubjectTargetSuccess(name="CheckSubjectNearTargetExit")



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

    '''seek_attention_recov = py_trees.composites.Sequence(
        name="SeekAttentionRecov",
        memory=True
    )
    seek_attention_recov.add_children([
        approach_subject_recov,
        turn_toward_subject_recov_seek,
        Dog_catch_attention_recov
    ])'''

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
        approach_target_step, 
        turn_toward_subject_check_follow,
        delay_timer,
        turn_toward_target_check_follow
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
    
    tree = create_root() 
    tree_node = py_trees_ros.trees.BehaviourTree(root=tree)
    tree_node.setup(timeout=15.0, node_name="bottom_up_tree_node")
    print("Starting bottom-up behaviour tree...") 
    # Tick the tree at 10 Hz indefinitely 
    tree_node.tick_tock(period_ms=100.0)
    rclpy.spin(tree_node.node)     # <--- keeps node alive

if __name__ == "__main__": 
    main()

    