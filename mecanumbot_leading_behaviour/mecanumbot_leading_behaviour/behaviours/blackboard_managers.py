import ast
import math
import py_trees
from  py_trees_ros import subscribers
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import py_trees
import yaml
from geometry_msgs.msg import Point
from mecanumbot_msgs.msg import AccessMotorCmd
from mecanumbot_msgs.srv import SetLedStatus


class ConstantParamsToBlackboard(py_trees.behaviour.Behaviour): # Checks done - works
    """
    Reads parameters from the ROS 2 node at startup and writes them
    into the py_trees blackboard under well-defined keys.

    universally needed:
        init_delay: [s], float 
        robot_closeness_threshold: [m], float robot is considered close to subject/target if within this distance
        robot_approach_distance: [m], float, desired distance between robot and subject when robot going to subject
        target_reached_threshold: [m], float subject reached the target
        target_position: [m] stored in a dict, converted to geometry_msgs Point message 

    LED condition parameters:
        LED_indicate_target_seq: [color,mode]x4, mecanumbot_msgs, SetLedStatus srv request stored in a list of dicts 
        LED_indicate_target_times: [s], stored in a list with the same number of elements as corresp. seq 
        LED_catch_attention_seq: [color,mode]x4, mecanumbot_msgs, SetLedStatus srv request stored in a list of dicts 
        LED_catch_attention_times: [s],float, stored in a list with the same number of elements as corresp. seq 

    Dog condition parameters:
        Dog following_min_threshold: [m], float, min distance to consider the subject is following the robot
        Dog_indicate_target_seq: [rad]->[motor_state] x3, mecanumbot_msgs, AccessMotorCmd msg stored in a list of dicts 
        Dog_indicate_target_times: [s], stored in a list with the same number of elements as corresp. seq 
        Dog_catch_attention_seq: [rad]->[motor_state] x3, mecanumbot_msgs, AccessMotorCmd msg stored in a list of dicts 
        Dog_catch_attention_times: [s], stored in a list with the same number of elements as corresp. seq
        Dog_look_for_feedback_delay # [s], float
    """
    def __init__(self, name: str, yaml_path: str):
        super().__init__(name=name)
        self.yaml_path = yaml_path
        self.blackboard = self.attach_blackboard_client(name=name)

        self.blackboard.register_key("init_delay", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("robot_approach_distance", access=py_trees.common.Access.WRITE)
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
        self.blackboard.register_key("Dog_indicate_target_times", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("Dog_catch_attention_seq", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("Dog_catch_attention_times", access=py_trees.common.Access.WRITE)
        
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
        self.blackboard.target_reached_threshold = float(raw_params['target_reached_threshold'])
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
        self.blackboard.Dog_indicate_target_times = raw_params["Dog_indicate_target_times"]
        self.blackboard.Dog_catch_attention_seq = [parse_dog(e) for e in raw_params["Dog_catch_attention_seq"]]
        self.blackboard.Dog_catch_attention_times = raw_params["Dog_catch_attention_times"]
        self.feedback_message = "ConstantParamsToBlackboard setup complete"
        self.logger.info(self.feedback_message)

        return True
    
    def update(self):
        return py_trees.common.Status.SUCCESS      

class DistanceToBlackboard(py_trees.behaviour.Behaviour): # Checks done - works
    """
    Reads blackboard.subject_pose and blackboard.target_position, 
    subscribes to /amcl_pose, computes Euclidean distances, and writes results 
    to the blackboard.
    """
    def __init__(self, name="ComputeDistance"):
        super().__init__(name)

        # create a blackboard client
        self.blackboard = self.attach_blackboard_client(name=name)

        # register the keys we READ (excluding robot_pose)
        #self.blackboard.register_key("subject_position", py_trees.common.Access.READ)
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
        
        # Internal state for the subscriber
        self.robot_pose = None
        self.subject_position = None
        self.subscriber = None
    
    def setup(self, **kwargs):
        """
        Called once when the behaviour is added to the tree.
        The main py_trees_ros Node must be passed via kwargs.
        """
        # Retrieve the ROS node handle from the setup kwargs
        try:
            node = kwargs['node']
            self.node = node
        except KeyError:
            raise KeyError("The 'node' argument was not passed to setup()")

        # Define QoS profile to match the amcl publisher (TRANSIENT_LOCAL)
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )

        # Create the subscriber using the provided node context
        self.robot_subscriber = node.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            qos_profile=qos_profile
        )
        self.subject_subscriber = node.create_subscription(
            PoseStamped,
            '/mecanumbot/subject_pose',
            self.subject_callback,
            qos_profile=10
        )
        self.feedback_message = "DistanceToBlackboard setup complete"
        self.logger.info(self.feedback_message)
        return True

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        """
        Callback to store the latest robot pose.
        We only need the Pose component (which contains the position Point).
        """
        self.robot_pose = msg.pose.pose # geometry_msgs/Pose

    def subject_callback(self, msg: PoseStamped):
        """
        Callback to store the latest subject position.
        We only need the position Point.
        """
        self.subject_position = msg.pose.position # geometry_msgs/Point

    def calculate_distance(self, position1: Point, position2: Point):
        """
        Compute Euclidean distance between two Points
        """
        x1, y1 = position1.x, position1.y
        x2, y2 =  position2.x, position2.y

        return math.hypot(x2 - x1, y2 - y1)

    def update(self):
        """
        Read poses, compute distances, update blackboard.
        """
        # The robot_pose is now stored internally and is a geometry_msgs/Pose
        robot_pose = self.robot_pose 
        subject_position = self.subject_position
        
        # The subject_pose is still read from the BB, and is a geometry_msgs/PoseStamped
        try:
            target_position = self.blackboard.target_position # geometry_msgs/Point
        except AttributeError:
            self.feedback_message = "waiting for target data on blackboard"
            self.node.get_logger().info(self.feedback_message)
            return py_trees.common.Status.RUNNING


        if robot_pose is None:
            self.feedback_message = "waiting for initial robot pose"
            self.node.get_logger().info(self.feedback_message)
            return py_trees.common.Status.RUNNING
        if subject_position is None:
            self.feedback_message = "waiting for initial subject pose"
            self.node.get_logger().info(self.feedback_message)
            return py_trees.common.Status.RUNNING
        
        # Get the Point object from the internal robot_pose
        robot_position = robot_pose.position 

        # --- Distance Calculations ---
        
        # 1. Robot to Subject
        d_subject = self.calculate_distance(robot_position, subject_position)
        
        # 2. Robot to Target
        d_target = self.calculate_distance(robot_position, target_position)
        
        # 3. Subject to Target
        d_subject_target = self.calculate_distance(subject_position, target_position)

        # --- Write to Blackboard ---
        
        self.blackboard.robot_distance_from_subject = d_subject
        self.blackboard.subject_within_robot_threshold = (d_subject < self.blackboard.robot_closeness_threshold)

        self.blackboard.distance_from_target = d_target
        self.blackboard.target_within_robot_threshold = (d_target < self.blackboard.robot_closeness_threshold)

        self.blackboard.target_distance_from_subject = d_subject_target
        self.blackboard.target_within_subject_threshold = (d_subject_target < self.blackboard.target_reached_threshold)

        self.feedback_message = f"""d(R, S) = {d_subject:.2f} | d(R, T) = {d_target:.2f} | d(S, T) = {d_subject_target:.2f}"""
        self.node.get_logger().info(self.feedback_message)
        return py_trees.common.Status.SUCCESS
