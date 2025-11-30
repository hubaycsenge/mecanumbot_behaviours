from mecanumbot_leading_behaviour.behaviours.blackboard_managers import SubjectToBlackboard,
                                                                        RobotToBlackboard,
                                                                        DistanceToBlackboard

from mecanumbot_leading_behaviour.behaviours.movement_managers import SubjectToGoalPose,
                                                                      TargetToGoalPose,
                                                                      TurnTowardsSubject,
                                                                      TurnTowardsTarget,
                                                                      CheckApproachSuccess,
                                                                      CheckSubjectTargetSuccess

from mecanumbot_leading_behaviour.behaviours.dog_behaviours import DogIndicateTarget,
                                                                   DogLookForFeedback,
                                                                   DogCatchAttention

from mecanumbot_leading_behaviour.behaviours.LED_behaviours import LEDIndicateTarget,
                                                                   LEDCatchAttention,
                                                                   LEDStop  

import py_trees

def create_approach_subject_subtree(subject_topic_name = 'subject_pose',robot_topic_name = 'amcl_pose', subj_dist_threshold = 0.5,tgt_reach_threshold = 0.2, approach_distance_threshold = 0.3):
    """
    Creates a subtree for approaching the subject.
    """
    root = py_trees.composites.Sequence("ApproachSubjectSequence")

    subject_to_blackboard = SubjectToBlackboard(name="SubjectToBlackboard", topic_name=subject_topic_name)
    robot_to_blackboard = RobotToBlackboard(name="RobotToBlackboard", topic_name=robot_topic_name)
    distance_to_blackboard = DistanceToBlackboard(name="DistanceToBlackboard", threshold=subj_dist_threshold, target_reach_threshold=tgt_reach_threshold)
    check_approach_success = CheckApproachSuccess(name="CheckApproachSuccess", approach_distance_threshold=approach_distance_threshold)

    root.add_children([
        subject_to_blackboard,
        subject_to_goal_pose,
        robot_to_blackboard,
        distance_to_blackboard,
        check_approach_success
    ])

    return root

def create_approach_subject_until_success_subtree(subject_topic_name = 'subject_pose',robot_topic_name = 'amcl_pose', subj_dist_threshold = 0.5,tgt_reach_threshold = 0.2, approach_distance_threshold = 0.3):
    """
    Creates a subtree for approaching the subject until success.
    """
    root = py_trees.decorators.Retry("ApproachSubjectUntilSuccess", num_attempts = 20)

    approach_subject_subtree = create_approach_subject_subtree(subject_topic_name, robot_topic_name, subj_dist_threshold, tgt_reach_threshold, approach_distance_threshold)

    root.add_child(approach_subject_subtree)

    return root


def create_indicate_target_subtree(condition):
    """
    Creates a subtree for indicating the target with LEDs and dog accessory.
    """
    root = py_trees.composites.Sequence("IndicateTargetSequence")

    turn_towards_subject = TurnTowardsSubject(name="TurnTowardsSubjectIndincate")
    turn_towards_target = TurnTowardsTarget(name="TurnTowardsTargetIndicate")
    led_indicate_target = LEDIndicateTarget(name="LEDIndicateTarget")
    dog_indicate_target = DogIndicateTarget(name="DogIndicateTarget")
    led_catch_attention = LEDCatchAttention(name="LEDCatchAttentionIndicate")
    dog_catch_attention = DogCatchAttention(name="DogCatchAttentionIndicate")
    check_subject_target_success = CheckSubjectTargetSuccess(name="CheckSubjectTargetSuccessIndicate")

    if condition == 'Dog':
        root.add_children([
            turn_towards_subject,
            dog_catch_attention,
            turn_towards_target,
            dog_indicate_target,
            turn_towards_subject,
            dog_catch_attention,
            check_subject_target_success
        ])

    elif condition == 'LED':
        root.add_children([
            turn_towards_subject,
            turn_towards_target,
            led_indicate_target,
            check_subject_target_success
        ])
    else: 
        print("Condition not recognized. Choose 'Dog' or 'LED'.")

    return root

