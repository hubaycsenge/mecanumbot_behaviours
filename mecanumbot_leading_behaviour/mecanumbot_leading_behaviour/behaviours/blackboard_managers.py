import rclpy
import py_trees
from  py_trees_ros import subscribers
from geometry_msgs.msg import PoseStamped

class SubjectToBlackboard(subscribers.ToBlackboard):
    def __init__(self, name, topic_name):
        super(SubjectToBlackboard, self).__init__(name=name,
                                           topic_name=topic_name,
                                           topic_type= PoseStamped,
                                           blackboard_variables={"subject": None},
                                           clearing_policy=py_trees.common.ClearingPolicy.NEVER
                                           )
        self.blackboard.subject = PoseStamped()
        self.blackboard.subject.header.frame_id = "map"
        self.blackboard.subject.pose.position.x = 0.0
        self.blackboard.subject.pose.position.y = 0.0
        self.blackboard.subject.pose.position.z = 0.0
        self.blackboard.subject.pose.orientation.x = 0.0
        self.blackboard.subject.pose.orientation.y = 0.0
        self.blackboard.subject.pose.orientation.z = 0.0
        self.blackboard.subject.pose.orientation.w = 1.0

    def update(self):
        """
        Call the parent to write the raw data to the blackboard
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)
        status = super(ToBlackboard, self).update()
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
                                           blackboard_variables={"robot": None},
                                           clearing_policy=py_trees.common.ClearingPolicy.NEVER
                                           )
        self.blackboard.robot = PoseStamped()
        self.blackboard.robot.header.frame_id = "map"
        self.blackboard.robot.pose.position.x = 0.0
        self.blackboard.robot.pose.position.y = 0.0
        self.blackboard.robot.pose.position.z = 0.0
        self.blackboard.robot.pose.orientation.x = 0.0
        self.blackboard.robot.pose.orientation.y = 0.0
        self.blackboard.robot.pose.orientation.z = 0.0
        self.blackboard.robot.pose.orientation.w = 1.0

    def update(self):
        """
        Call the parent to write the raw data to the blackboard
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)
        status = super(ToBlackboard, self).update()
        if status != py_trees.common.Status.RUNNING:
            rospy.loginfo("RobotToBlackboard is finished")
            # else don't do anything in between - i.e. avoid the ping pong problems
        else: 
            rospy.loginfo("RobotToBlackboard is running")
        return status

class DistanceToBlackboard(py_trees.behaviour.Behaviour):
    """
    Reads blackboard.robot.pose.position, blackboard.target and blackboard.subject.pose.position (Points),
    computes Euclidean distance, writes:
       - blackboard.distance
       - blackboard.within_threshold (True/False)
    """
    def __init__(self, name="ComputeDistance", threshold= 0.5, target_reached_threshold = 0.20):
        super().__init__(name)

        # create a blackboard client
        self.blackboard = py_trees.blackboard.BlackboardClient(name=name)

        # register the keys we READ
        self.blackboard.register_key("robot", py_trees.common.Access.READ)
        self.blackboard.register_key("subject", py_trees.common.Access.READ)
        self.blackboard.register_key("target", py_trees.common.Access.READ)

        # register the keys we WRITE
        self.blackboard.register_key("distance_from_subject", py_trees.common.Access.WRITE)
        self.blackboard.register_key("subject_within_threshold", py_trees.common.Access.WRITE)

        self.blackboard.register_key("distance_from_target", py_trees.common.Access.WRITE)
        self.blackboard.register_key("target_within_threshold", py_trees.common.Access.WRITE)

        self.blackboard.register_key("target_subject_distance", py_trees.common.Access.WRITE)
        self.blackboard.register_key("target_reached_within_threshold", py_trees.common.Access.WRITE)

        self.threshold = threshold
        self.target_reached_threshold = target_reach_threshold
    
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
        robot_position = self.blackboard.robot.pose.position #geometry_msgs/PoseStamped
        subject_position = self.blackboard.subject.pose.position #geometry_msgs/PoseStamped
        target_pose = self.blackboard.target # geometry_msgs/Point


        if robot_pose is None or subject_pose is None:
            self.feedback_message = "waiting for poses"
            return py_trees.common.Status.RUNNING

        # compute Euclidean distance
        d_subject = self.calculate_distance(robot_position, subject_position)
        d_target = self.calculate_distance(robot_position, target_pose)
        d_subject_target = self.calculate_distance(subject_position, target_pose)

        # write to blackboard
        self.blackboard.distance_from_subject = d_subject
        self.blackboard.subject_within_threshold = (d_subject < self.threshold)

        # write to blackboard
        self.blackboard.distance_from_target = d_target
        self.blackboard.target_within_threshold = (d_target < self.threshold)

        self.blackboard.target_subject_distance = d_subject_target
        self.blackboard.target_reached_within_threshold = (d_subject_target < self.target_reached_threshold)

        self.feedback_message = f"distance from subject = {d_subject:.2f}, within = {self.blackboard.subject_within_threshold}/n
                                  distance from target = {d_target:.2f}, within = {self.blackboard.target_within_threshold}"

        return py_trees.common.Status.SUCCESS
