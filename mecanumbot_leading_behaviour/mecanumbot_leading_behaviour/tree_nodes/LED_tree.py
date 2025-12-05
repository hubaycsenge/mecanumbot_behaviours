import py_trees
import rclpy
import py_trees_ros
from mecanumbot_behaviours.mecanumbot_leading_behaviour.mecanumbot_leading_behaviour.utils.subtrees import create_approach_subject_until_success_subtree,create_indicate_target_subtree
from mecanumbot_leading_behaviour.behaviours.movement_managers import TargetToGoalPose,\
                                                                      TurnTowardTarget,\
                                                                      TurnTowardsSubject,\
                                                                      CheckSubjectTargetSuccess

from mecanumbot_leading_behaviour.behaviours.LED_behaviours import LEDIndicateTarget

def create_core_sequence(wait_duration = 1.0):
    """
    Creates the core sequence for leading behaviour.
    """
    root = py_trees.composites.Sequence("CoreLeadingSequence")

    approach_subject_subtree = create_approach_subject_until_success_subtree()
    delay_timer = py_trees.timers.Timer(name="CoreDelayTimer", duration=wait_duration)
    target_to_goal_pose = TargetToGoalPose(name="TargetToGoalPose")
    turn_towards_target = TurnTowardTarget(name="TurnTowardsTarget")
    turn_towards_subject = TurnTowardsSubject(name="TurnTowardsSubject")
    check_subject_target_success = CheckSubjectTargetSuccess(name="CheckSubjectTargetSuccess")

    LED_signal_target = LEDIndicateTarget(name="LEDSignalTarget")
    LED_indicate_target = create_indicate_target_subtree("LEDIndicateTarget")


    gobetween = py_trees.composites.Sequence("GoBetweenItems")
    gobetween.add_children([
        target_to_goal_pose,
        LED_signal_target,
        delay_timer,
        approach_subject_subtree,
        check_subject_target_success
    ])
    gobetween_retries = py_trees.decorators.Retry(name="GoBetweenItemsRetries", child = gobetween, num_iterations=1000)

    root.add_children([
        gobetween_retries,
        LED_indicate_target
    ])

    
    
    return root

def create_leading_ctrl_tree(wait_duration = 10.0,time_switch_duration = 1.0):
    """
    Creates the control tree for the mecanumbot leading behaviour.
    """
    root = py_trees.composites.Sequence("MecanumbotLeadingCtrlTree")

    core_sequence = create_core_sequence(wait_duration = time_switch_duration)
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