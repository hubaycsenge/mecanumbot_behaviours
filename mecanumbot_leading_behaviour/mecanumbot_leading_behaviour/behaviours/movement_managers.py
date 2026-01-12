import math
import rclpy
import py_trees
import numpy as np
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped,Pose
import numpy as np
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.time import Time
from action_msgs.msg import GoalStatusArray, GoalStatus


#nav2goal statuses
STATUS_UNKNOWN=0
STATUS_ACCEPTED=1
STATUS_EXECUTING=2
STATUS_CANCELING=3
STATUS_SUCCEEDED=4
STATUS_CANCELED=5
STATUS_ABORTED=6


class TurnToward(py_trees.behaviour.Behaviour): # Tested, works

    def __init__(self, name="TurnToward", target_type="subject"):
        super().__init__(name)

        # Create the ROS publisher
        self.publisher = None
        self.subject_pose = None
        self.robot_pose = None
        self.robot_orientation = None
        
        # Blackboard
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key="target_position", access=py_trees.common.Access.READ)
        
        self.turning = False
        self.target_type = target_type
    
        self.cmd_send_time = None 

    def setup(self, **kwargs):
        node = kwargs["node"]
        self.node = node
        
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )

        self.publisher = node.create_publisher(PoseStamped, "/goal_pose", 10)

        if self.target_type == "subject":
            self.subject_subscriber = node.create_subscription(
                PoseStamped,
                "/mecanumbot/subject_pose",
                self.subject_callback,
                10
            )

        self.robot_subscriber = node.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.amcl_callback,
            qos_profile
        )

        self.status_sub = node.create_subscription(
            GoalStatusArray,
            "/navigate_to_pose/_action/status",
            self.goal_status_callback,
            10
        )
        
        self.logger.info(f"{self.name}: Setup complete")

    def initialise(self):
        self.goal_sent = False
        self.cmd_send_time = None
        self.goal_uuid = None
        self.goals_in_sys = None
        self.compare_position = None
        return super().initialise()

    def update(self):
        # Safety Checks
        if self.robot_pose is None:
            return py_trees.common.Status.RUNNING
        if self.compare_position is None:
            if self.target_type == "subject":
                if self.subject_pose is None:
                    return py_trees.common.Status.RUNNING
                self.compare_position = self.subject_pose.pose.position
            else:
                self.compare_position = self.blackboard.target_position
        if self.compare_position is None:
            return py_trees.common.Status.FAILURE

        if not self.goal_sent:
            if self.goals_in_sys is not None and len(self.goals_in_sys) >= 0: # robot goal status array has history
                robot_has_goal_running = self.check_if_running()
                if not robot_has_goal_running:
                    self.send_turn_command()
                    return py_trees.common.Status.RUNNING
                else:
                    self.node.get_logger().info(f"{self.name}: Waiting for previous goal to finish")
                    return py_trees.common.Status.RUNNING
            else:
                self.send_turn_command()
                return py_trees.common.Status.RUNNING
        else:
            if self.goal_uuid is None:
                self.assign_goal_uuid()
            if self.goal_uuid is None:
                self.node.get_logger().info(f"{self.name}: No goal UUID assigned yet")
                return py_trees.common.Status.RUNNING
            for goal in self.goals_in_sys:
                #self.node.get_logger().info(f"{self.name}: Checking goal UUID {goal.goal_info.goal_id.uuid} against {self.goal_uuid}")
                if np.array_equal(goal.goal_info.goal_id.uuid, self.goal_uuid):
                    self.goal_status = goal.status
            if self.goal_status == STATUS_SUCCEEDED:
                self.node.get_logger().info(f"{self.name}: Turn completed successfully")
                return py_trees.common.Status.SUCCESS
            elif self.goal_status in [STATUS_EXECUTING, STATUS_ACCEPTED]:
                #self.node.get_logger().info(f"{self.name}: Turn in progress")
                return py_trees.common.Status.RUNNING
            elif self.goal_status in [STATUS_ABORTED, STATUS_CANCELED, STATUS_CANCELING, STATUS_UNKNOWN]:
                self.node.get_logger().info(f"{self.name}: Turn failed, retrying")
                self.goal_sent = False
                self.goal_uuid = None
                return py_trees.common.Status.RUNNING
        return py_trees.common.Status.RUNNING
    
    def assign_goal_uuid(self):
        """Assign the UUID of the last sent goal from the goals in system."""
        if self.goals_in_sys is None:
            #self.node.get_logger().info(f"{self.name}: No goals in system to assign UUID from")
            return
        for goal_status in self.goals_in_sys:
            goal_time = Time.from_msg(goal_status.goal_info.stamp)
            cmd_time   = Time.from_msg(self.cmd_send_time.to_msg())
            #self.node.get_logger().info(f"{self.name}: Comparing goal time {goal_time.nanoseconds} with cmd time {cmd_time.nanoseconds}")
            if goal_time >= cmd_time:
                self.goal_uuid = goal_status.goal_info.goal_id.uuid
                self.node.get_logger().info(f"{self.name}: Assigned goal UUID {self.goal_uuid}")
                return
    
    def check_if_running(self):
        """Check if there is a goal currently being executed by nav2."""
        for goal_status in self.goals_in_sys:
            if goal_status.status == STATUS_EXECUTING or goal_status.status == STATUS_ACCEPTED:
                return True
        return False
    
    def send_turn_command(self):
        """Helper to create and publish the command with fresh timestamp"""
        desired_orientation = calculate_facing_orientation(self.robot_pose, self.compare_position)
        
        self.turn_cmd = PoseStamped()
        self.turn_cmd.header.frame_id = "map"
        self.cmd_send_time = self.node.get_clock().now()
        self.turn_cmd.header.stamp = self.cmd_send_time.to_msg() 
        self.turn_cmd.pose.position = self.robot_pose.position
        self.turn_cmd.pose.orientation = desired_orientation
        
        self.publisher.publish(self.turn_cmd)
        
        self.goal_sent = True
        
        self.node.get_logger().info(f"{self.name}: Published turn command \n Directions: Z: {desired_orientation.z} W: {desired_orientation.w}")

    def goal_status_callback(self, msg):
        """Callback to monitor nav2goal status updates."""
        self.logger.info(f"{self.name}: Received goal status update with {len(msg.status_list)} entries")
        self.goals_in_sys = msg.status_list
        
    def amcl_callback(self, msg):
        self.robot_pose = msg.pose.pose
        self.robot_orientation = msg.pose.pose.orientation

    def subject_callback(self, msg):
        self.subject_pose = msg

class Approach(py_trees.behaviour.Behaviour): # TODO

    def __init__(self, name="Approach", target_type="subject", mode ="exact"):
        super().__init__(name)

        # Create the ROS publisher
        self.publisher = None
        self.subject_pose = None
        self.robot_pose = None
        self.robot_orientation = None
        self.mode = mode
        # Blackboard
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key="target_position", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="robot_closeness_threshold", access=py_trees.common.Access.READ) #stop_threshold
        self.blackboard.register_key(key="robot_approach_distance", access=py_trees.common.Access.READ) #go_threshold
        
        self.turning = False
        self.target_type = target_type
    
        self.cmd_send_time = None 

    def setup(self, **kwargs):
        node = kwargs["node"]
        self.node = node
        
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )

        self.publisher = node.create_publisher(PoseStamped, "/goal_pose", 10)

        if self.target_type == "subject":
            self.subject_subscriber = node.create_subscription(
                PoseStamped,
                "/mecanumbot/subject_pose",
                self.subject_callback,
                10
            )

        self.robot_subscriber = node.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.amcl_callback,
            qos_profile
        )

        self.status_sub = node.create_subscription(
            GoalStatusArray,
            "/navigate_to_pose/_action/status",
            self.goal_status_callback,
            10
        )
        
        self.logger.info(f"{self.name}: Setup complete")

    def initialise(self):
        self.goal_sent = False
        self.cmd_send_time = None
        self.goal_uuid = None
        self.goals_in_sys = None
        self.compare_position = None
        return super().initialise()

    def update(self):
        # Safety Checks
        if self.robot_pose is None:
            return py_trees.common.Status.RUNNING
        if self.compare_position is None:
            if self.target_type == "subject":
                if self.subject_pose is None:
                    return py_trees.common.Status.RUNNING
                self.compare_position = self.subject_pose.pose.position
            else:
                self.compare_position = self.blackboard.target_position
        if self.compare_position is None:
            return py_trees.common.Status.FAILURE

        if not self.goal_sent:
            if self.goals_in_sys is not None and len(self.goals_in_sys) >= 0: # robot goal status array has history
                robot_has_goal_running = self.check_if_running()
                if not robot_has_goal_running:
                    self.send_goal_command()
                    return py_trees.common.Status.RUNNING

                else:
                    self.node.get_logger().info(f"{self.name}: Waiting for previous goal to finish")
                    return py_trees.common.Status.RUNNING
            else:
                self.send_goal_command()
                return py_trees.common.Status.RUNNING
        else:
            if self.goal_uuid is None:
                self.assign_goal_uuid()
            if self.goal_uuid is None:
                self.node.get_logger().info(f"{self.name}: No goal UUID assigned yet")
                return py_trees.common.Status.RUNNING
            for goal in self.goals_in_sys:
                #self.node.get_logger().info(f"{self.name}: Checking goal UUID {goal.goal_info.goal_id.uuid} against {self.goal_uuid}")
                if np.array_equal(goal.goal_info.goal_id.uuid, self.goal_uuid):
                    self.goal_status = goal.status
            if self.goal_status == STATUS_SUCCEEDED:
                self.node.get_logger().info(f"{self.name}: Approach completed successfully")
                return py_trees.common.Status.SUCCESS
            elif self.goal_status in [STATUS_EXECUTING, STATUS_ACCEPTED]:
                #self.node.get_logger().info(f"{self.name}: Approach in progress")
                return py_trees.common.Status.RUNNING
            elif self.goal_status in [STATUS_ABORTED, STATUS_CANCELED, STATUS_CANCELING, STATUS_UNKNOWN]:
                self.node.get_logger().info(f"{self.name}: Approach failed, retrying")
                self.goal_sent = False
                self.goal_uuid = None
                return py_trees.common.Status.RUNNING
        return py_trees.common.Status.RUNNING
    
    def assign_goal_uuid(self):
        """Assign the UUID of the last sent goal from the goals in system."""
        if self.goals_in_sys is None:
            #self.node.get_logger().info(f"{self.name}: No goals in system to assign UUID from")
            return
        for goal_status in self.goals_in_sys:
            goal_time = Time.from_msg(goal_status.goal_info.stamp)
            cmd_time   = Time.from_msg(self.cmd_send_time.to_msg())
            #self.node.get_logger().info(f"{self.name}: Comparing goal time {goal_time.nanoseconds} with cmd time {cmd_time.nanoseconds}")
            if goal_time >= cmd_time:
                self.goal_uuid = goal_status.goal_info.goal_id.uuid
                self.node.get_logger().info(f"{self.name}: Assigned goal UUID {self.goal_uuid}")
                return
    
    def check_if_running(self):
        """Check if there is a goal currently being executed by nav2."""
        for goal_status in self.goals_in_sys:
            if goal_status.status == STATUS_EXECUTING or goal_status.status == STATUS_ACCEPTED:
                return True
        return False
    
    def send_goal_command(self):
        stop_threshold = self.blackboard.robot_closeness_threshold
        go_threshold = self.blackboard.robot_approach_distance
        """Helper to create and publish the command with fresh timestamp"""
        desired_pose = pose_to_goal(self.compare_position, self.robot_pose, stop_threshold, mode=self.mode, go_threshold)
        
        self.goal_cmd = PoseStamped()
        self.goal_cmd.header.frame_id = "map"
        self.cmd_send_time = self.node.get_clock().now()
        self.goal_cmd.header.stamp = self.cmd_send_time.to_msg() 
        self.goal_cmd.pose = desired_pose
        
        self.publisher.publish(self.goal_cmd)
        
        self.goal_sent = True
        
        self.node.get_logger().info(f"{self.name}: Published goal command \n Directions: X: {desired_pose.position.x} Y: {desired_pose.position.y} Z: {desired_pose.orientation.z} W: {desired_pose.orientation.w}")

    def goal_status_callback(self, msg):
        """Callback to monitor nav2goal status updates."""
        self.logger.info(f"{self.name}: Received goal status update with {len(msg.status_list)} entries")
        self.goals_in_sys = msg.status_list
        
    def amcl_callback(self, msg):
        self.robot_pose = msg.pose.pose
        self.robot_orientation = msg.pose.pose.orientation

    def subject_callback(self, msg):
        self.subject_pose = msg
class CheckSubjectTargetSuccess(py_trees.behaviour.Behaviour): #TODO
    """
    Checks if the subject has successfully reached the target
    based on distance threshold from blackboard entries.
    """

    def __init__(self, name="CheckSubjectTargetSuccess"):
        super().__init__(name)

        # Blackboard keys
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key="target_reached_threshold",access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="target_position",access=py_trees.common.Access.READ)


    def setup(self, **kwargs):
        node = kwargs["node"]
        self.node = node
        
        self.subject_subscriber = node.create_subscription(
            PoseStamped,
            "/mecanumbot/subject_pose",
            self.subject_callback,
            10
        )
        
        self.logger.info(f"{self.name}: Setup complete")

    def update(self):
        if self.subject_pose is None:
            self.node.get_logger().info(f"{self.name}: No subject pose received yet")
            return py_trees.common.Status.RUNNING
        if self.blackboard.target_reached_threshold>=self.dist:
            self.node.get_logger().info(f"{self.name}: Subject reached target: within threshold")
            return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().info(f"{self.name}: Subject has not reached target yet")
            return py_trees.common.Status.FAILURE

    def subject_callback(self, msg):
        self.subject_pose = msg
        target_position = self.blackboard.target_position
        dx = target_position.x - self.subject_pose.pose.position.x
        dy = target_position.y - self.subject_pose.pose.position.y
        self.dist = math.sqrt(dx*dx + dy*dy)

def calculate_facing_orientation(robot_pose, target_position):
    # 1. Vector FROM Robot TO Target (Target - Robot)
    dx = target_position.x - robot_pose.position.x
    dy = target_position.y - robot_pose.position.y

    # 2. Absolute angle in the Map frame
    desired_yaw = np.arctan2(dy, dx)

    # 3. Convert to Quaternion
    # We do NOT subtract robot_yaw. We want the absolute compass direction.
    q = quaternion_from_euler(0, 0, desired_yaw)

    orientation = PoseStamped().pose.orientation
    orientation.x = q[0]
    orientation.y = q[1]
    orientation.z = q[2]
    orientation.w = q[3]
    
    return orientation

def pose_to_goal(object_position,robot_pose,stop_threshold = 0.3, mode="exact", go_threshold = 1.0):
            
            Ox = object_position.x
            Oy = object_position.y
            Rx = robot_pose.position.x
            Ry = robot_pose.position.y
            
            dx = Ox - Rx
            dy = Oy - Ry
            dist = math.sqrt(dx*dx + dy*dy)

            # If already too close, do nothing
            if dist < stop_threshold:
                return robot_pose

            # Compute approach position
            if mode == "exact": # go [stop_threshold] near to object
                X = Rx + dx*(1-stop_threshold/dist) 
                Y = Ry + dy*(1-stop_threshold/dist) 

            elif mode == "fixed_distance": # go [go_threshold] closer to object, except if too close to object, then go to [stop_threshold]
                if dist < go_threshold:
                    X = Rx + dx*(1-stop_threshold/dist) 
                    Y = Ry + dy*(1-stop_threshold/dist)
                else:
                    X = Rx + dx*(go_threshold/dist) 
                    Y = Ry + dy*(go_threshold/dist) 
            # Orientation: face the person
            yaw = math.atan2(Oy - Y, Ox - X)
            q = quaternion_from_euler(0, 0, yaw)

            goal = Pose()
            goal.position.x = X
            goal.position.y = Y
            goal.orientation.x = q[0]
            goal.orientation.y = q[1]
            goal.orientation.z = q[2]
            goal.orientation.w = q[3]
            return goal

def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    return math.atan2(siny_cosp, cosy_cosp)