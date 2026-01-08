import math
import rclpy
import py_trees
import numpy as np
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
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


class SubjectToGoalPose(py_trees.behaviour.Behaviour): #TODO
    """
    Listens to a PoseStamped 'subject' entry,
    and publishes it as a Nav2 goal pose to /goal_pose.
    """

    def __init__(self, name="SubjectToGoalPose"):
        super().__init__(name)

        # Create the ROS publisher
        self.publisher = None

        # Blackboard keys
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key="robot_approach_distance",access=py_trees.common.Access.READ)

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
        self.person_subscriber = node.create_subscription(
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
        self.publisher = node.create_publisher(
            PoseStamped,
            "/goal_pose",
            10
        )

        self.logger.info("Goal pose publisher ready")

    def update(self):
        """
        Read subject pose from blackboard and publish as goal.
        """
        
        
        if self.subject_position is None:
            self.logger.warn("No subject pose being published yet")
            return py_trees.common.Status.RUNNING
        
        # Construct goal message
        goal = pose_to_goal(self.subject_position,
                                   self.robot_pose,
                                   self.blackboard.robot_approach_distance)

        self.publisher.publish(goal)

        self.logger.debug(
            f"Published goal pose: "
            f"x={goal.pose.position.x:.2f}, "
            f"y={goal.pose.position.y:.2f}"
        )

        return py_trees.common.Status.SUCCESS
    
    def subject_callback(self, msg: PoseStamped):
        """
        Callback to store the latest subject position.
        We only need the position Point.
        """
        self.subject_position = msg.pose.position # geometry_msgs/Point

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        """
        Callback to store the latest robot pose.
        We only need the Pose component (which contains the position Point).
        """
        self.robot_pose = msg.pose.pose# geometry_msgs/Pose
class TargetToGoalPose(py_trees.behaviour.Behaviour): #TODO
    """
    Reads a PoseStamped 'target' from the blackboard and publishes it
    to /goal_pose as a Nav2 goal.
    
    Adds a timeout: if no new target is published for N seconds,
    a STOP goal is sent and the behaviour returns FAILURE.
    """

    def __init__(self, name="TargetToGoalPose", timeout_sec=5.0):
        super().__init__(name)
        
        # ROS publisher
        self.publisher = None
        self.node = None

        # timeout
        self.timeout_sec = timeout_sec
        self.last_update_time = None
        self.robot_pose = None

        # Blackboard key
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key( 
            key="robot_approach_distance",
            access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="target_position",
            access=py_trees.common.Access.READ
        )

    def setup(self, **kwargs):
        """
        Called once by py_trees_ros after ROS init.
        """
        self.node = kwargs["node"]
        self.publisher = self.node.create_publisher(
            PoseStamped,
            "/goal_pose",
            10
        )
        # Define QoS profile to match the amcl publisher (TRANSIENT_LOCAL)
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )
        self.robot_subscriber = self.node.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.amcl_callback,
            qos_profile
        )

        self.logger.info("TargetToGoalPose: Goal pose publisher ready")

    def initialise(self):
        """Called on first tick or when behaviour is re-entered."""
        self.last_update_time = self.node.get_clock().now()

    def update(self):
        """Read target pose and publish a goal, or stop robot if timeout."""
        now = self.node.get_clock().now()
        target_posi = self.blackboard.target_position

        # Check timeout
        if (now - self.last_update_time) > rclpy.duration.Duration(seconds=self.timeout_sec):
            self.logger.warn("Timeout reached — stopping robot")
            self._publish_stop_goal()
            return py_trees.common.Status.FAILURE

        # No target yet
        if self.robot_pose is None:
            self.logger.warn("No robot pose available yet")
            return py_trees.common.Status.RUNNING
        if target_posi is None:
            self.logger.warn("No target posi available yet")
            return py_trees.common.Status.RUNNING

        # Got a target — publish goal
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = now.to_msg()
        goal.pose = pose_to_goal(target_posi, self.robot_pose, 0.5)

        self.publisher.publish(goal)

        self.logger.info(
            f"Published goal pose → x={goal.pose.position.x:.2f}, "
            f"y={goal.pose.position.y:.2f}"
        )

        # Refresh timestamp
        self.last_update_time = now

        return py_trees.common.Status.SUCCESS
    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        """
        Callback to store the latest robot pose.
        We only need the Pose component (which contains the position Point).
        """
        self.robot_pose = msg.pose.pose # geometry_msgs/Pose
    
    def _publish_stop_goal(self):
        """Sends a stop command by publishing the robot’s current position."""
        stop = PoseStamped()
        stop.header.frame_id = "map"
        stop.header.stamp = self.node.get_clock().now().to_msg()
        stop.pose.position = self.robot_pose.position
        stop.pose.orientation = self.robot_pose.orientation
        # STOP = current pose, meaning: "don't move / stay"
        stop.pose.orientation.w = 1.0
        
        self.publisher.publish(stop)
        self.logger.info("Robot STOP goal published")

class TurnToward(py_trees.behaviour.Behaviour):
    """
    Reads a target position, turns to face it, and retries if the robot stops early.
    """

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
        
        self.angular_velocity = 0.0
        self.turning = False
        self.target_type = target_type
        
        # Helper to prevent checking "stopped" immediately after sending a goal
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
        # 1. Safety Checks
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
                    return self.send_turn_command(self.compare_position)
                else:
                    self.node.get_logger().info(f"{self.name}: Waiting for previous goal to finish")
                    return py_trees.common.Status.RUNNING
            else:
                return self.send_turn_command(self.compare_position)
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
    
    def send_turn_command(self, target_position):
        """Helper to create and publish the command with fresh timestamp"""
        desired_orientation = calculate_facing_orientation(self.robot_pose, target_position)
        
        self.turn_cmd = PoseStamped()
        self.turn_cmd.header.frame_id = "map"
        self.cmd_send_time = self.node.get_clock().now()
        self.turn_cmd.header.stamp = self.cmd_send_time.to_msg() 
        self.turn_cmd.pose.position = self.robot_pose.position
        self.turn_cmd.pose.orientation = desired_orientation
        
        self.publisher.publish(self.turn_cmd)
        
        self.goal_sent = True
        
        self.node.get_logger().info(f"{self.name}: Published turn command \n Directions: X: {desired_orientation.x} Y: {desired_orientation.y} Z: {desired_orientation.z} W: {desired_orientation.w}")

        return py_trees.common.Status.RUNNING

    def goal_status_callback(self, msg):
        """Callback to monitor nav2goal status updates."""
        self.logger.info(f"{self.name}: Received goal status update with {len(msg.status_list)} entries")
        self.goals_in_sys = msg.status_list
        
    def amcl_callback(self, msg):
        self.robot_pose = msg.pose.pose
        self.robot_orientation = msg.pose.pose.orientation

    def subject_callback(self, msg):
        self.subject_pose = msg



class CheckApproachSuccess(py_trees.behaviour.Behaviour): #TODO
    """
    Checks if the robot has successfully approached the subject
    based on distance threshold from blackboard entries.
    """

    def __init__(self, name="CheckApproachSuccess", threshold=0.5):
        super().__init__(name)

        # Blackboard keys
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            key="subject_within_robot_threshold",
            access=py_trees.common.Access.READ
        )

    def update(self):
        if self.blackboard.subject_within_robot_threshold is None:
            self.logger.info("No subject-robot distance info yet")
            return py_trees.common.Status.FAILURE
        if self.blackboard.subject_within_robot_threshold:
            self.logger.info("Approach successful: within threshold")
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.info("Approach  not successful, restarting approach")
            return py_trees.common.Status.FAILURE

class CheckSubjectTargetSuccess(py_trees.behaviour.Behaviour): #TODO
    """
    Checks if the subject has successfully reached the target
    based on distance threshold from blackboard entries.
    """

    def __init__(self, name="CheckSubjectTargetSuccess"):
        super().__init__(name)

        # Blackboard keys
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            key="target_within_subject_threshold",
            access=py_trees.common.Access.READ
        )

    def update(self):

        if self.blackboard.target_within_subject_threshold is None:
            self.logger.info("No distance info yet")
            return py_trees.common.Status.FAILURE
        if self.blackboard.target_within_subject_threshold:
            self.logger.info("Subject reached target: within threshold")
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.info("Subject has not reached target yet")
            return py_trees.common.Status.FAILURE

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

def pose_to_goal(object_position,robot_pose,approach_distance):
            Ox = object_position.x
            Oy = object_position.y
            Rx = robot_pose.position.x
            Ry = robot_pose.position.y
            
            dx = Rx - Ox
            dy = Ry - Oy
            dist = math.sqrt(dx*dx + dy*dy)

            # If already too close, do nothing
            if dist < approach_distance:
                return robot_pose

            # Compute approach position
            X = Rx + dx(1-approach_distance/dist) 
            Y = Ry + dy(1-approach_distance/dist) 
            # Orientation: face the person
            yaw = math.atan2(Oy - Y, Ox - X)
            q = quaternion_from_euler(0, 0, yaw)

            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.pose.position.x = X
            goal.pose.position.y = Y
            goal.pose.orientation.x = q[0]
            goal.pose.orientation.y = q[1]
            goal.pose.orientation.z = q[2]
            goal.pose.orientation.w = q[3]
            return goal

def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    return math.atan2(siny_cosp, cosy_cosp)