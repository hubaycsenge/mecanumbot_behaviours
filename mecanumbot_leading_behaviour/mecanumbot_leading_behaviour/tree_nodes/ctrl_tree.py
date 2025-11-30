import py_trees
from mecanumbot_leading_behaviour.tree_nodes.subtrees import create_approach_subject_subtree,create_indicate_target_subtree
from mecanumbot_leading_behaviour.behaviours.movement_managers import TargetToGoalPose, TurnTowardsTarget,TurnTowardsSubject

def create_core_sequence(wait_duration = 1.0):
    """
    Creates the core sequence for leading behaviour.
    """
    root = py_trees.composites.Sequence("CoreLeadingSequence")

    approach_subject_subtree = create_approach_subject_until_success_subtree()
    delay_timer = py_trees.timers.Timer(name="CoreDelayTimer", duration=wait_duration)
    target_to_goal_pose = TargetToGoalPose(name="TargetToGoalPose")
    turn_towards_target = TurnTowardsTarget(name="TurnTowardsTarget")
    turn_towards_subject = TurnTowardsSubject(name="TurnTowardsSubject")

    root.add_children([
        target_to_goal_pose,
        turn_towards_target,
        delay_timer,
        approach_subject_subtree,
        turn_towards_subject,
        delay_timer
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