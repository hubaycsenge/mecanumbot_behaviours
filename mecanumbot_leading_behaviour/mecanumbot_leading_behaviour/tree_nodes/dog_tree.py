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

    delay_timer = py_trees.timers.Timer(name="DelayTimer", duration=2)

    Dog_show_target = DogBehaviourSequence('DShow', 'indicate_target')
    Dog_catch_attention = DogBehaviourSequence('DCatch', 'catch_attention')
    Dog_check_following = DogCheckFollowing(name="DogCheckFollowing")
    Dog_check_if_target_reached = DogCheckIfReached(name="DogCheckIfTargetReached")

    approach_target_step = Approach(name="ApproachTarget", target_type="target",mode="fixed_distance")
    approach_subject = Approach(name="ApproachSubject", target_type="subject")

    turn_toward_subject = TurnToward(name="TurnTowardSubject", target_type="subject")
    turn_toward_target = TurnToward(name="TurnTowardTarget", target_type="target")

    check_subject_near_target = CheckSubjectTargetSuccess(name="CheckSubjectNearTarget")

    # Approach subject and catch attention
    seek_attention = py_trees.composites.Sequence(
        name="SeekAttention",
        memory=True
    )
    seek_attention.add_children([
        approach_subject,
        turn_toward_subject,
        Dog_catch_attention
    ])

    # Indicate close target until subject is close enough
    show_while_close_seq = py_trees.composites.Sequence(
        name="ShowWhileSubjectCloseSeq",
        memory=True
    )

    show_while_close_seq.add_children([
        check_subject_near_target,
        turn_toward_subject,
        Dog_catch_attention,
        turn_toward_target,
        Dog_show_target
    ]) 

    show_while_close_loop = py_trees.decorators.Repeat(
        name="ShowWhileCloseLoop",
        child=show_while_close_seq,
        num_successes=-1 
    )
    
    # Check following, go a step forward if so
    lead_until_follows_seq = py_trees.composites.Sequence(
        name="LeadUntilFollows",
        memory=True
    )
    lead_until_follows_seq.add_children([
        Dog_check_following,
        approach_target_step,
    ]) 

    lead_until_follows_loop = py_trees.decorators.Repeat(
        name="LeadUntilFollowsLoop",
        child=lead_until_follows_seq,
        num_successes=-1
    )
    lead_to_target_seq = py_trees.composites.Sequence(
        name="LeadToTargetSeq",
        memory=True
    )  
    
    # seek attention, then lead until follows, check if reached target, retry until target reached
    lead_to_target_seq.add_children([
        seek_attention,
        lead_until_follows_loop,
        Dog_check_if_target_reached
    ])

    lead_to_target_loop = py_trees.decorators.Retry(
        name="LeadToTargetLoop",
        child=lead_to_target_seq,
        num_failures=-1
    )

    root.add_children([
        params_loader,
        delay_timer,
        lead_to_target_loop,
        show_while_close_loop
    ])

    return root

def main(args=None):
    rclpy.init(args=args) 
    
    tree = create_root() 
    tree_node = py_trees_ros.trees.BehaviourTree(root=tree)
    tree_node.setup(timeout=15.0, node_name="bottom_up_tree_node")
    print("Starting bottom-up behaviour tree...") 
    # Tick the tree at 10 Hz indefinitely 
    tree_node.tick_tock(period_ms=10.0)
    rclpy.spin(tree_node.node)     # <--- keeps node alive

if __name__ == "__main__": 
    main()

    