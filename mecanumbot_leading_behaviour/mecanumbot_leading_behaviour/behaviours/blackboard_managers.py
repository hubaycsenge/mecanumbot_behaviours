import ast
import rclpy
import math
import py_trees
from  py_trees_ros import subscribers
from geometry_msgs.msg import PoseStamped

import py_trees
import yaml
import py_trees_ros
from geometry_msgs.msg import Point
from mecanumbot_msgs.msg import AccessMotorCmd
from mecanumbot_msgs.srv import SetLedStatus
from rclpy.node import Node


class ConstantParamsToBlackboard(py_trees.behaviour.Behaviour):
    """
    Reads parameters from the ROS 2 node at startup and writes them
    into the py_trees blackboard under well-defined keys.

    universally needed:
        init_delay: [s], float 
        robot_closeness_threshold: [m], float 
        robot_approach_subject_distance: [m], float, desired distance between robot and subject when robot going to subject
        target_reached_threshold: [m], float subject reached the target
        target_position: [m] stored in a dict, converted to geometry_msgs Point message 

    LED condition parameters:
        LED_indicate_target_seq: [color,mode]x4, mecanumbot_msgs, SetLedStatus srv request stored in a list of dicts 
        LED_indicate_target_times: [s], stored in a list with the same number of elements as corresp. seq 
        LED_catch_attention_seq: [color,mode]x4, mecanumbot_msgs, SetLedStatus srv request stored in a list of dicts 
        LED_catch_attention_times: [s],float, stored in a list with the same number of elements as corresp. seq 

    Dog condition parameters:
        Dog following_min_threshold: [m], float, min distance to consider the subject is following the robot
        Dog_indicate_tgt_seq: [rad]->[motor_state] x3, mecanumbot_msgs, AccessMotorCmd msg stored in a list of dicts 
        Dog_catch_attention_seq: [rad]->[motor_state] x3, mecanumbot_msgs, AccessMotorCmd msg stored in a list of dicts 
        Dog_look_for_feedback_delay # [s], float
    """
    def __init__(self, name: str, yaml_path: str):
        super().__init__(name=name)
        self.yaml_path = yaml_path
        self.blackboard = self.attach_blackboard_client(name=name)

        self.blackboard.register_key("init_delay", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("robot_approach_subject_distance", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("robot_closeness_threshold", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("target_reached_threshold", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("target_position", access=py_trees.common.Access.WRITE)

        self.blackboard.register_key("LED_indicate_target_seq", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("LED_catch_attention_seq", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("LED_indicate_target_times", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("LED_catch_attention_times", access=py_trees.common.Access.WRITE)

        self.blackboard.register_key("Dog_look_for_feedback_delay", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("Dog_following_min_threshold", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("Dog_indicate_target_seq", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("Dog_catch_attention_seq", access=py_trees.common.Access.WRITE)
        
    def setup(self, **kwargs):
        with open(self.yaml_path, 'r') as f:
            raw_params = yaml.safe_load(f)

         # LED helper function
        def parse_led(d):
            d = ast.literal_eval(d)
            req = SetLedStatus.Request()
            req.fl_color, req.fl_mode = d["fl"]["color"], d["fl"]["mode"]
            req.fr_color, req.fr_mode = d["fr"]["color"], d["fr"]["mode"]
            req.bl_color, req.bl_mode = d["bl"]["color"], d["bl"]["mode"]
            req.br_color, req.br_mode = d["br"]["color"], d["br"]["mode"]
            return req

        # Dog helper function
        def parse_dog(d):
            d = ast.literal_eval(d)
            cmd = AccessMotorCmd()
            cmd.n_pos = float(d["n_pos"])
            cmd.gl_pos = float(d["gl_pos"])
            cmd.gr_pos = float(d["gr_pos"])
            return cmd
        raw_params = raw_params['bottom_up_tree_node']['ros__parameters']

        self.blackboard.init_delay = float(raw_params['init_delay'])
        self.blackboard.robot_closeness_threshold = float(raw_params['robot_closeness_threshold'])
        self.blackboard.target_within_threshold = float(raw_params['target_within_threshold'])
        self.blackboard.Dog_look_for_feedback_delay = float(raw_params['Dog_look_for_feedback_delay'])

        target_pos_list = raw_params["target_position"]
        self.blackboard.target_position = Point()
        self.blackboard.target_position.x = float(target_pos_list[0])
        self.blackboard.target_position.y = float(target_pos_list[1])
        self.blackboard.target_position.z = float(target_pos_list[2])

        self.blackboard.LED_indicate_target_seq = [parse_led(e) for e in raw_params["LED_indicate_target_seq"]]
        self.blackboard.LED_catch_attention_seq = [parse_led(e) for e in raw_params["LED_catch_attention_seq"]]
        self.blackboard.LED_indicate_target_times = raw_params["LED_indicate_target_times"]
        self.blackboard.LED_catch_attention_times = raw_params["LED_catch_attention_times"]

        self.blackboard.Dog_indicate_target_seq = [parse_dog(e) for e in raw_params["Dog_indicate_target_seq"]]
        self.blackboard.Dog_catch_attention_seq = [parse_dog(e) for e in raw_params["Dog_catch_attention_seq"]]
        self.feedback_message = "ConstantParamsToBlackboard setup complete"
        self.logger.info(self.feedback_message)

        return True
    
    def update(self):
        return py_trees.common.Status.SUCCESS      

class SubjectToBlackboard(subscribers.ToBlackboard):
    def __init__(self, name, topic_name):
        super(SubjectToBlackboard, self).__init__(name=name,
                                           topic_name=topic_name,
                                           topic_type= PoseStamped,
                                           blackboard_variables={"subject_pose": PoseStamped()},
                                           clearing_policy=py_trees.common.ClearingPolicy.NEVER
                                           )

    def setup(self, **kwargs):
        """
        Called once when the behaviour is added to the tree.
        """
        self.blackboard.subject_pose = PoseStamped()
        self.blackboard.subject_pose.header.frame_id = "map"
        self.blackboard.subject_pose.pose.position.x = 0.0
        self.blackboard.subject_pose.pose.position.y = 0.0
        self.blackboard.subject_pose.pose.position.z = 0.0
        self.blackboard.subject_pose.pose.orientation.x = 0.0
        self.blackboard.subject_pose.pose.orientation.y = 0.0
        self.blackboard.subject_pose.pose.orientation.z = 0.0
        self.blackboard.subject_pose.pose.orientation.w = 1.0
        self.logger.info('SubjectToBlackboard behaviour initialised  entries')
        self.feedback_message = "SubjectToBlackboard setup complete"
        self.logger.info(self.feedback_message)

        return True
    
    def update(self):
        """
        Call the parent to write the raw data to the blackboard
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)
        status = super(SubjectToBlackboard, self).update()
        if status != py_trees.common.Status.RUNNING:
            self.node.get_logger().info("SubjectToBlackboard is finished")
            # else don't do anything in between - i.e. avoid the ping pong problems
        else: 
            self.node.get_logger().info("SubjectToBlackboard is running")
        return status

class RobotToBlackboard(subscribers.ToBlackboard):
    def __init__(self, name, topic_name):
        super(RobotToBlackboard, self).__init__(name=name,
                                           topic_name = topic_name,
                                           topic_type = PoseStamped,
                                           blackboard_variables={"robot_pose": PoseStamped()},
                                           clearing_policy=py_trees.common.ClearingPolicy.NEVER
                                           )

    def setup(self, **kwargs):
        """
        Called once when the behaviour is added to the tree.
        """
        self.blackboard.robot_pose = PoseStamped()
        self.blackboard.robot_pose.header.frame_id = "map"
        self.blackboard.robot_pose.pose.position.x = 0.0
        self.blackboard.robot_pose.pose.position.y = 0.0
        self.blackboard.robot_pose.pose.position.z = 0.0
        self.blackboard.robot_pose.pose.orientation.x = 0.0
        self.blackboard.robot_pose.pose.orientation.y = 0.0
        self.blackboard.robot_pose.pose.orientation.z = 0.0
        self.blackboard.robot_pose.pose.orientation.w = 1.0
        self.logger.info('RobotToBlackboard behaviour initialised  entries')
        self.feedback_message = "RobotToBlackboard setup complete"
        self.logger.info(self.feedback_message)

        return True

    def update(self):
        """
        Call the parent to write the raw data to the blackboard
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)
        status = super(RobotToBlackboard, self).update()
        if status != py_trees.common.Status.RUNNING:
            rclpy.loginfo("RobotToBlackboard is finished")
            # else don't do anything in between - i.e. avoid the ping pong problems
        else: 
            rclpy.loginfo("RobotToBlackboard is running")
        return status

class DistanceToBlackboard(py_trees.behaviour.Behaviour):
    """
    Reads blackboard.robot.pose.position, blackboard.target and blackboard.subject.pose.position (Points),
    computes Euclidean distance for d[tgt, subject]; d[robot, subject];d[robot, tgt], writes:  
       - blackboard.distance
       - blackboard.within_threshold (True/False)
    """
    def __init__(self, name="ComputeDistance"):
        super().__init__(name)

        # create a blackboard client
        self.blackboard = py_trees.blackboard.BlackboardClient(name=name)

        # register the keys we READ
        self.blackboard.register_key("robot_pose", py_trees.common.Access.READ)
        self.blackboard.register_key("subject_pose", py_trees.common.Access.READ)
        self.blackboard.register_key("target_position", py_trees.common.Access.READ)

        self.blackboard.register_key("robot_closeness_threshold", py_trees.common.Access.READ)
        self.blackboard.register_key("target_reached_threshold", py_trees.common.Access.READ)

        # register the keys we WRITE
        self.blackboard.register_key("robot_distance_from_subject", py_trees.common.Access.WRITE)
        self.blackboard.register_key("subject_within_robot_threshold", py_trees.common.Access.WRITE)

        self.blackboard.register_key("distance_from_target", py_trees.common.Access.WRITE)
        self.blackboard.register_key("target_within_robot_threshold", py_trees.common.Access.WRITE)

        self.blackboard.register_key("target_distance_from_subject", py_trees.common.Access.WRITE)
        self.blackboard.register_key("target_within_subject_threshold", py_trees.common.Access.WRITE)
    
    def setup(self, **kwargs):
        """
        Called once when the behaviour is added to the tree.
        """
        #TODO: check how to init read-only blackboard variables
        self.feedback_message = "DistanceToBlackboard setup complete"
        self.logger.info(self.feedback_message)
        return True
 
    def calculate_distance(self, position1, position2):
        """
        Compute Euclidean distance between two Points
        """
        x1, y1 = position1.x, position1.y
        x2, y2 =  position2.x, position2.y

        return math.hypot(x2 - x1, y2 - y1)

    def update(self):
        """
        Read both poses, compute distance, update blackboard.
        """
        robot_position = self.blackboard.robot_pose.pose.position #geometry_msgs/PoseStamped -> Point
        subject_position = self.blackboard.subject_pose.pose.position #geometry_msgs/PoseStamped -> Point
        target_pose = self.blackboard.target # geometry_msgs/Point


        if robot_position is None or subject_position is None:
            self.feedback_message = "waiting for poses"
            return py_trees.common.Status.RUNNING

        # compute Euclidean distance
        d_subject = self.calculate_distance(robot_position, subject_position)
        d_target = self.calculate_distance(robot_position, target_pose)
        d_subject_target = self.calculate_distance(subject_position, target_pose)

        # write to blackboard
        self.blackboard.robot_distance_from_subject = d_subject
        self.blackboard.subject_within_robot_threshold = (d_subject < self.blackboard.robot_closeness_threshold)

        # write to blackboard
        self.blackboard.robot_distance_from_target = d_target
        self.blackboard.target_within_robot_threshold = (d_target < self.blackboard.robot_closeness_threshold)

        self.blackboard.target_distance_from_subject = d_subject_target
        self.blackboard.target_within_subject_threshold = (d_subject_target < self.blackboard.target_reached_threshold)

        self.feedback_message = f"""distance from subject = {d_subject:.2f}, within = {self.blackboard.subject_within_threshold}/n 
                                  distance from target = {d_target:.2f}, within = {self.blackboard.target_within_threshold}"""

        return py_trees.common.Status.SUCCESS
