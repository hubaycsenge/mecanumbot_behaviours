import math
import rclpy
import py_trees
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler, euler_from_quaternion


class SubjectToGoalPose(py_trees.behaviour.Behaviour): #TODO
    """
    Reads a PoseStamped 'subject' entry from the blackboard,
    and publishes it as a Nav2 goal pose to /goal_pose.
    """

    def __init__(self, name="SubjectToGoalPose"):
        super().__init__(name)

        # Create the ROS publisher
        self.publisher = None

        # Blackboard keys
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key="subject_pose",access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="robot_pose",access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="robot_approach_subject_distance",access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        """
        Called once by py_trees_ros after ROS init.
        Create publisher here.
        """
        node = kwargs["node"]
        self.publisher = node.create_publisher(
            PoseStamped,
            "goal_pose",
            10
        )

        self.logger.info("Goal pose publisher ready")

    def update(self):
        """
        Read subject pose from blackboard and publish as goal.
        """
        def subject_pose_to_goal(subject_pose,robot_pose,approach_distance):
            Sx = subject_pose.pose.position.x
            Sy = subject_pose.pose.position.y
            Rx = robot_pose.x
            Ry = robot_pose.y
            
            dx = Rx - Sx
            dy = Ry - Sy
            dist = math.sqrt(dx*dx + dy*dy)

            # If already too close, do nothing
            if dist < approach_distance:
                return robot_pose

            # Compute approach position
            X = Rx + dx(1-approach_distance/dist) 
            Y = Ry + dy(1-approach_distance/dist) 
            # Orientation: face the person
            yaw = math.atan2(Sy - Y, Sx - X)
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
        
        subject_pose = self.blackboard.subject_pose
        robot_pose = self.blackboard.robot_pose

        if subject_pose is None:
            self.logger.warn("No subject pose on blackboard yet")
            return py_trees.common.Status.RUNNING
        
        X,Y = 
        # Construct goal message
        goal = subject_pose_to_goal(subject_pose,
                                   robot_pose,
                                   self.blackboard.robot_approach_subject_distance)

        self.publisher.publish(goal)

        self.logger.debug(
            f"Published goal pose: "
            f"x={goal.pose.position.x:.2f}, "
            f"y={goal.pose.position.y:.2f}"
        )

        return py_trees.common.Status.SUCCESS

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

        # Blackboard key
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            key="target",
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
        self.logger.info("TargetToGoalPose: Goal pose publisher ready")

    def initialise(self):
        """Called on first tick or when behaviour is re-entered."""
        self.last_update_time = self.node.get_clock().now()

    def update(self):
        """Read target pose and publish a goal, or stop robot if timeout."""
        now = self.node.get_clock().now()
        target_pose = self.blackboard.target

        # Check timeout
        if (now - self.last_update_time) > rclpy.duration.Duration(seconds=self.timeout_sec):
            self.logger.warn("Timeout reached — stopping robot")
            self._publish_stop_goal()
            return py_trees.common.Status.FAILURE

        # No target yet
        if target_pose is None:
            self.logger.warn("No target pose available yet")
            return py_trees.common.Status.RUNNING

        # Got a target — publish goal
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = now.to_msg()
        goal.pose = target_pose.pose   # use the pose directly

        self.publisher.publish(goal)

        self.logger.info(
            f"Published goal pose → x={goal.pose.position.x:.2f}, "
            f"y={goal.pose.position.y:.2f}"
        )

        # Refresh timestamp
        self.last_update_time = now

        return py_trees.common.Status.SUCCESS

    def _publish_stop_goal(self):
        """Sends a stop command by publishing the robot’s current position."""
        stop = PoseStamped()
        stop.header.frame_id = "map"
        stop.header.stamp = self.node.get_clock().now().to_msg()

        # STOP = current pose, meaning: "don't move / stay"
        stop.pose.orientation.w = 1.0
        
        self.publisher.publish(stop)
        self.logger.info("Robot STOP goal published")

class TurnTowardsSubject(py_trees.behaviour.Behaviour): #TODO
    """
    Reads a PoseStamped 'subject' entry from the blackboard,
    and publishes a turn goal position command to face the target.
    """

    def __init__(self, name="TurnTowardsSubject"):
        super().__init__(name)

        # Create the ROS publisher
        self.publisher = None

        # Blackboard keys
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            key="subject_pose",
            access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="robot_pose",
            access=py_trees.common.Access.READ
        )

    def setup(self, **kwargs):
        """
        Called once by py_trees_ros after ROS init.
        Create publisher here.
        """
        node = kwargs["node"]
        self.publisher = node.create_publisher(
            PoseStamped,
            "goal_pose",
            10
        )
        self.logger.info("Turn command publisher ready")

    def update(self):
        """
        Read subject and robot pose from blackboard and publish turn command.
        """

        subject_pose = self.blackboard.subject_pose
        robot_pose = self.blackboard.robot_pose

        if subject_pose is None or robot_pose is None:
            self.logger.warn("No target pose on blackboard yet")
            return py_trees.common.Status.RUNNING

        # Construct turn command message
        turn_cmd = PoseStamped()
        turn_cmd.header.frame_id = "map"
        turn_cmd.header.stamp = rclpy.clock.Clock().now().to_msg()
        turn_cmd.pose = robot_pose.pose  # Keep current position
        # Set orientation to face target (this is a placeholder, actual calculation needed)
        turn_cmd.pose.orientation = calculate_facing_orientation(robot_pose, subject_pose.pose.position)
        

        self.publisher.publish(turn_cmd)

        self.logger.debug(
            f"Published turn command: direction={turn_cmd.pose.orientation}"
        )

class TurnTowardTarget(py_trees.behaviour.Behaviour): #TODO
    """
    Reads a Pose 'target' entry from the blackboard,
    and publishes a turn goal position command to face the target.
    """

    def __init__(self, name="TurnTowardsTarget"):
        super().__init__(name)

        # Create the ROS publisher
        self.publisher = None

        # Blackboard keys
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            key="target",
            access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="robot",
            access=py_trees.common.Access.READ
        )

    def setup(self, **kwargs):
        """
        Called once by py_trees_ros after ROS init.
        Create publisher here.
        """
        node = kwargs["node"]
        self.publisher = node.create_publisher(
            PoseStamped,
            "goal_pose",
            10
        )
        self.logger.info("Turn command publisher ready")

    def update(self):
        """
        Read subject and robot pose from blackboard and publish turn command.
        """

        target_position = self.blackboard.target_position # geometry_msgs/Point
        robot_pose = self.blackboard.robot_pos #geometry_msgs/PoseStamped

        if target_position is None or robot_pose is None:
            self.logger.warn("No target pose on blackboard yet")
            return py_trees.common.Status.RUNNING

        # Construct turn command message
        turn_cmd = PoseStamped()
        turn_cmd.header.frame_id = "map"
        turn_cmd.header.stamp = rclpy.clock.Clock().now().to_msg()
        turn_cmd.pose = robot_pose.pose  # Keep current position
        # Set orientation to face target (this is a placeholder, actual calculation needed)
        turn_cmd.pose.orientation = self._calculate_facing_orientation(robot_pose, target_position)
        

        self.publisher.publish(turn_cmd)

        self.logger.debug(
            f"Published turn command: direction={turn_cmd.pose.orientation}"
        )
        return py_trees.common.Status.SUCCESS

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

    def __init__(self, name="CheckSubjectTargetSuccess", threshold=0.5):
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

def calculate_facing_orientation(robot_pose,target_position):

    X = robot_pose.pose.position.x - target_position.x
    Y = robot_pose.pose.position.y - target_position.y
    yaw = np.arctan2(Y, X)
    robot_yaw = euler_from_quaternion(robot_pose.pose.orientation)[2]
    total_yaw = yaw - robot_yaw
    q = quaternion_from_euler(0, 0, total_yaw)
    orientation = PoseStamped().pose.orientation
    orientation.x = q[0]
    orientation.y = q[1]
    orientation.z = q[2]
    orientation.w = q[3]
    return orientation

    # This should compute the quaternion that makes the robot face the subject
