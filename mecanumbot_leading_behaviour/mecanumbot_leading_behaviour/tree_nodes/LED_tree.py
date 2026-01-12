import os 
import rclpy 
import py_trees 
import py_trees_ros 
from ament_index_python.packages import get_package_share_directory 
from mecanumbot_leading_behaviour.behaviours.dog_behaviours import DogBehaviourSequence
from mecanumbot_leading_behaviour.behaviours.LED_behaviours import LEDBehaviourSequence
from mecanumbot_leading_behaviour.behaviours.movement_managers import Approach, \
                                                                      TurnToward, CheckSubjectTargetSuccess
from mecanumbot_leading_behaviour.behaviours.blackboard_managers import ConstantParamsToBlackboard, \
                                                                        DistanceToBlackboard

leading_pkg_share_dir = get_package_share_directory('mecanumbot_leading_behaviour') 
YAML_PATH = os.path.join(leading_pkg_share_dir,'config',"behaviour_setting_constants.yaml") 
def create_root():
    root = py_trees.composites.Sequence("ROOT", memory=True)

    params_loader = ConstantParamsToBlackboard(
        name="LoadConstantParams", yaml_path=YAML_PATH
    )

    delay_timer = py_trees.timers.Timer(name="DelayTimer", duration=2)

    LED_show_target = LEDBehaviourSequence('LShow', 'indicate_target')
    LED_catch_attention = LEDBehaviourSequence('LCatch', 'catch_attention')
    LED_indicate_near_target = LEDBehaviourSequence('LNear', 'indicate_close_target')

    approach_target = Approach(name="ApproachTarget", target_type="target")
    approach_subject = Approach(name="ApproachSubject", target_type="subject")
    turn_toward_subject = TurnToward(name="TurnTowardSubject", target_type="subject")
    turn_toward_target = TurnToward(name="TurnTowardTarget", target_type="target")

    check_subject_near_target = CheckSubjectTargetSuccess(
        name="CheckSubjectNearTarget"
    )

    # Selector will keep showing target until condition succeeds
    show_until_close = py_trees.composites.Selector(
        name="ShowUntilSubjectClose",
        memory=False
    )

    show_until_close.add_children([
        check_subject_near_target,
        turn_toward_subject,
        LED_catch_attention,
        turn_toward_target,
        LED_indicate_near_target
    ])

    root.add_children([
        params_loader,
        delay_timer,
        approach_subject,
        approach_target,
        LED_show_target,
        show_until_close
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

    