import os 
import rclpy 
import py_trees 
import py_trees_ros 
from mecanumbot_leading_behaviour.behaviours.blackboard_managers import ConstantParamsToBlackboard 
from ament_index_python.packages import get_package_share_directory 

leading_pkg_share_dir = get_package_share_directory('mecanumbot_leading_behaviour') 
YAML_PATH = os.path.join(leading_pkg_share_dir,'config',"behaviour_setting_constants.yaml") 
def create_root(): 
    root = py_trees.composites.Sequence("ROOT", memory = False) 

    params_loader = ConstantParamsToBlackboard( name="LoadConstantParams", yaml_path=YAML_PATH) 
    delay_timer = py_trees.timers.Timer(name="DelayTimer",  duration=20) 
    
    root.add_children([params_loader,delay_timer]) 
    return root 

def main(args=None): 
    rclpy.init(args=args) 
    tree = create_root() 
    tree_node = py_trees_ros.trees.BehaviourTree(root=tree) 
    tree_node.setup(timeout=15.0) 
    try: 
        # Tick the tree at 10 Hz indefinitely 
        #tree_node.tick_tock(period_ms=100) 
        rclpy.spin(tree_node.node)     # <--- keeps node alive
    except KeyboardInterrupt: pass 
    finally: 
        tree_node.shutdown() 
        rclpy.shutdown() 

if __name__ == "__main__": 
    main()