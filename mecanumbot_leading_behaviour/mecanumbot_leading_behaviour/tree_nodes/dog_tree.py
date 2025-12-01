import rclpy
import py_trees
import py_trees_ros
from mecanumbot_behaviours.mecanumbot_leading_behaviour.mecanumbot_leading_behaviour.utils.subtrees import create_approach_subject_until_success_subtree,create_indicate_target_subtree
from mecanumbot_leading_behaviour.behaviours.movement_managers import TargetToGoalPose, \
                                                                     TurnTowardsTarget, \
                                                                     TurnTowardsSubject, \
                                                                     CheckSubjectTargetSuccess
from mecanumbot_leading_behaviour.behaviours.dog_behaviours import DogIndicateTarget, \
                                                                   DogLookForFeedback,\
                                                                   DogCatchAttention,\
                                                                   DogCheckIfSubjLed, \
                                                                   DogCheckSubjectTargetSuccess

def create_core_sequence(wait_duration = 1.0,feedback_duration = 2.0):

    root = py_trees.composites.Sequence("CoreLeadingSequence")

    approach_subject_subtree = create_approach_subject_until_success_subtree()
    delay_timer = py_trees.timers.Timer(name="CoreDelayTimer", duration = wait_duration)

    target_to_goal_pose = TargetToGoalPose(name="TargetToGoalPose",timeout_sec = feedback_duration)
    turn_towards_target = TurnTowardsTarget(name="TurnTowardsTarget")
    turn_towards_subject = TurnTowardsSubject(name="TurnTowardsSubject")

    catch_attention = DogCatchAttention(name="DogCatchAttention")
    look_for_feedback = DogLookForFeedback(name="DogLookForFeedback")
    check_if_subj_led = DogCheckIfSubjLed(name="DogCheckIfSubjectLed")
    dog_check_subject_target_success = DogCheckSubjectTargetSuccess(name="DogCheckSubjectTargetSuccess")
    indicate_target = create_indicate_target_subtree(condition='Dog')

    approach_subject_catch_attention_subtree = py_trees.composites.Sequence("ApproachSubjectCatchAttentionSequence")
    approach_subject_catch_attention_subtree.add_children([
        approach_subject_subtree,
        turn_towards_subject,
        catch_attention
    ])
    
    lead_with_feedback_subtree = py_trees.composites.Sequence("LeadWithFeedbackSequence")
    lead_with_feedback_subtree.add_children([
        target_to_goal_pose,
        look_for_feedback,
        check_if_subj_led,
        turn_towards_subject     
    ])
    lead_with_feedback_repeats = py_trees.decorators.Repeat(name="LeadWithFeedbackRepeats", child=lead_with_feedback_subtree, num_iterations=50)

    lead_handlelost_subtree = py_trees.composites.Sequence("LeadHandleLostSequence")
    lead_handlelost_subtree.add_children([
        approach_subject_catch_attention_subtree, 
        lead_with_feedback_repeats,
        dog_check_subject_target_success
    ])
    lead_handlelost_retries = py_trees.decorators.Retry(name="LeadHandleLostRetries", child=lead_handlelost_subtree, num_iterations=100000)

    root.add_children([
        lead_handlelost_retries,
        indicate_target,
        delay_timer
    ])
    return root

def create_leading_ctrl_tree(wait_duration = 10.0,time_switch_duration = 1.0,feedback_duration = 2.0):
    """
    Creates the control tree for the mecanumbot leading behaviour.
    """
    root = py_trees.composites.Sequence("MecanumbotLeadingDogTree")

    core_sequence = create_core_sequence(wait_duration = time_switch_duration,feedback_duration = feedback_duration)
    delay_timer = py_trees.timers.Timer(name="DelayTimer", duration = wait_duration)
    repeated_core_sequence = py_trees.decorators.Repeat(name="RepeatCoreSequence", child = core_sequence, num_iterations = 10000000) # TODO have inf retries

    root.add_children([delay_timer,
                    repeated_core_sequence])

    return root

def main(args=None):
    rclpy.init(args=args)

    # Create the tree
    tree = create_leading_ctrl_tree(wait_duration = 10.0,time_switch_duration = 1.0)

    # Wrap tree in ROS behaviour tree executor
    tree_node = py_trees_ros.trees.BehaviourTree(tree)

    # Setup lifecycle (similar to ROS nodes)
    tree_node.setup(timeout=15.0)

    try:
        tree_node.tick_tock(period_ms=100)  # run tree at 10 Hz
    except KeyboardInterrupt:
        pass

    tree_node.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()