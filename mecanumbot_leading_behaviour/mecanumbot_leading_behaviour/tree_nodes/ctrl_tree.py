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
    root = py_trees.composites.Sequence("ROOT", memory = True) 

    params_loader = ConstantParamsToBlackboard( name="LoadConstantParams", yaml_path=YAML_PATH) 
    delay_timer = py_trees.timers.Timer(name="DelayTimer",  duration=2) 
    ''' 
    Sanity checks, on tested nodes
    '''

    
    dog_show_target = DogBehaviourSequence('DB1', 'indicate_target')
    
    dog_catch_att = DogBehaviourSequence('DB2', 'catch_attention')
    LED_show_target = LEDBehaviourSequence('LB1', 'indicate_target')
    LED_catch_att = LEDBehaviourSequence('LB2', 'catch_attention')
    #oot.add_children([params_loader,delay_timer,dog_catch_att,dog_show_target,LED_catch_att,LED_show_target]) 
    
    distance_to_bb = DistanceToBlackboard(name="DistanceToBB")
    turn_toward_subject = TurnToward(name="TurnTowardSubject",target_type ="subject")
    turn_toward_target = TurnToward(name="TurnTowardTarget",target_type ="target")
    approach_target = Approach(name="ApproachTarget", target_type="target")    
    approach_subject = Approach(name="ApproachSubject", target_type="subject")
    check_subject_target = CheckSubjectTargetSuccess(name="CheckSubjectTargetSuccess")

    root.add_children([params_loader, delay_timer, distance_to_bb, approach_subject, approach_target])


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