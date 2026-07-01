import random

import py_trees
import numpy as np
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseArray, PoseStamped, PoseWithCovarianceStamped, Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.time import Time

from mecanumbot_leading_behaviour.behaviours.movement_managers import (
    Approach,
    CheckRobotAtLastCheckpoint,
    CheckRobotHasBall,
    CheckSubjectTargetSuccess,
    RelativeTurnPattern,
    TurnToward,
    calculate_facing_orientation,
    normalize_angle,
    pose_to_goal,
    yaw_from_quaternion,
    STATUS_ABORTED,
    STATUS_ACCEPTED,
    STATUS_CANCELING,
    STATUS_CANCELED,
    STATUS_EXECUTING,
    STATUS_SUCCEEDED,
    STATUS_UNKNOWN,
)


class FindPeople(py_trees.behaviour.Behaviour):
    """Spin in place until a fresh people_fusion message is received."""

    def __init__(self, name="FindPeople", spin_speed=0.2, sight_timeout=1.0):
        super().__init__(name)
        self.spin_speed = float(spin_speed)
        self.sight_timeout = float(sight_timeout)
        self.publisher = None
        self.people_poses = []
        self.last_people_seen_time = None

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.publisher = self.node.create_publisher(Twist, "/cmd_vel", 10)
        self.people_subscriber = self.node.create_subscription(
            PoseArray,
            "/mecanumbot/people_fusion",
            self.people_callback,
            10,
        )
        self.logger.info(f"{self.name}: Setup complete")

    def has_fresh_people(self):
        if self.last_people_seen_time is None or len(self.people_poses) == 0:
            return False

        now = self.node.get_clock().now()
        age = (now - self.last_people_seen_time).nanoseconds / 1e9
        return age <= self.sight_timeout

    def publish_spin(self):
        cmd = Twist()
        cmd.angular.z = self.spin_speed
        self.publisher.publish(cmd)

    def stop_robot(self):
        self.publisher.publish(Twist())

    def update(self):
        if self.has_fresh_people():
            self.stop_robot()
            self.node.get_logger().info(f"{self.name}: People detected, stopping spin")
            return py_trees.common.Status.SUCCESS

        self.publish_spin()
        return py_trees.common.Status.RUNNING

    def people_callback(self, msg):
        self.people_poses = list(msg.poses)
        if msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0:
            self.last_people_seen_time = Time.from_msg(msg.header.stamp)
        else:
            self.last_people_seen_time = self.node.get_clock().now()


class GoToRandomPerson(py_trees.behaviour.Behaviour):
    """Select a fresh person at random from people_fusion and navigate toward them."""

    def __init__(self, name="GoToRandomPerson", sight_timeout=3.0):
        super().__init__(name)
        self.publisher = None
        self.people_poses = []
        self.robot_pose = None
        self.selected_person = None
        self.goal_sent = False
        self.goal_uuid = None
        self.goals_in_sys = None
        self.compare_position = None
        self.cmd_send_time = None
        self.goal_status = STATUS_UNKNOWN
        self.sight_timeout = float(sight_timeout)
        self.last_people_seen_time = None

    def setup(self, **kwargs):
        self.node = kwargs["node"]

        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.publisher = self.node.create_publisher(PoseStamped, "/goal_pose", 10)
        self.people_subscriber = self.node.create_subscription(
            PoseArray,
            "/mecanumbot/people_fusion",
            self.people_callback,
            10,
        )
        self.robot_subscriber = self.node.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.amcl_callback,
            qos_profile,
        )
        self.status_sub = self.node.create_subscription(
            GoalStatusArray,
            "/navigate_to_pose/_action/status",
            self.goal_status_callback,
            10,
        )

        self.logger.info(f"{self.name}: Setup complete")

    def initialise(self):
        self.goal_sent = False
        self.cmd_send_time = None
        self.goal_uuid = None
        self.goals_in_sys = None
        self.compare_position = None
        self.selected_person = None
        self.goal_status = STATUS_UNKNOWN
        return super().initialise()

    def has_fresh_people(self):
        if self.last_people_seen_time is None or len(self.people_poses) == 0:
            return False

        now = self.node.get_clock().now()
        age = (now - self.last_people_seen_time).nanoseconds / 1e9
        return age <= self.sight_timeout

    def select_random_person(self):
        if not self.people_poses:
            return None

        self.selected_person = random.choice(self.people_poses)
        self.compare_position = self.selected_person.position
        return self.compare_position

    def update(self):
        if self.robot_pose is None:
            return py_trees.common.Status.RUNNING


        if self.compare_position is None:
            self.node.get_logger().info(f"{self.name}: Selecting a random person to approach")
            if not self.has_fresh_people():
                self.node.get_logger().info(f"{self.name}: No fresh people detected, cannot select a target")
                self.goal_sent = False
                self.goal_uuid = None
                self.compare_position = None
                self.selected_person = None
                return py_trees.common.Status.FAILURE
            
            self.select_random_person()

        if self.compare_position is None:
            return py_trees.common.Status.FAILURE

        if not self.goal_sent:
            if self.goals_in_sys is not None and len(self.goals_in_sys) >= 0:
                if not self.check_if_running():
                    self.send_goal_command()
                    return py_trees.common.Status.RUNNING
                self.node.get_logger().info(f"{self.name}: Waiting for previous goal to finish")
                return py_trees.common.Status.RUNNING

            self.send_goal_command()
            return py_trees.common.Status.RUNNING

        if self.goal_uuid is None:
            self.assign_goal_uuid()
        if self.goal_uuid is None:
            self.node.get_logger().info(f"{self.name}: No goal UUID assigned yet")
            return py_trees.common.Status.RUNNING

        self.goal_status = STATUS_UNKNOWN
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
        
        if self.goal_status == STATUS_SUCCEEDED:
            self.node.get_logger().info(f"{self.name}: Random-person navigation completed successfully")
            return py_trees.common.Status.SUCCESS
        if self.goal_status in [STATUS_EXECUTING, STATUS_ACCEPTED]:
            return py_trees.common.Status.RUNNING
        if self.goal_status in [STATUS_ABORTED, STATUS_CANCELED, STATUS_CANCELING, STATUS_UNKNOWN]:
            self.node.get_logger().info(f"{self.name}: Navigation failed, selecting another person")
            self.goal_sent = False
            self.goal_uuid = None
            self.compare_position = None
            self.selected_person = None
            return py_trees.common.Status.RUNNING

        return py_trees.common.Status.RUNNING

    def assign_goal_uuid(self):
        if self.goals_in_sys is None or self.cmd_send_time is None:
            self.node.get_logger().info(f"{self.name}: Cannot assign goal UUID, goals_in_sys or cmd_send_time is None") 
            return

        cmd_time = Time.from_msg(self.cmd_send_time.to_msg())
        for goal_status in reversed(self.goals_in_sys):
            goal_time = Time.from_msg(goal_status.goal_info.stamp)
            if goal_time >= cmd_time:
                self.goal_uuid = goal_status.goal_info.goal_id.uuid
                self.node.get_logger().info(f"{self.name}: Locked onto Goal UUID {self.goal_uuid}")
                return

    def check_if_running(self):
        if self.goals_in_sys is None:
            return False

        for goal_status in self.goals_in_sys:
            if goal_status.status == STATUS_EXECUTING:
                return True
        return False

    def send_goal_command(self):
        desired_pose = pose_to_goal(self.compare_position, self.robot_pose,stop_threshold=0.5,go_threshold=10)

        self.goal_cmd = PoseStamped()
        self.goal_cmd.header.frame_id = "map"
        self.cmd_send_time = self.node.get_clock().now()
        self.goal_cmd.header.stamp = self.cmd_send_time.to_msg()
        self.goal_cmd.pose = desired_pose

        self.publisher.publish(self.goal_cmd)
        self.goal_sent = True
        self.node.get_logger().info(
            f"{self.name}: Published goal command for selected person \n"
            f"Directions: X: {desired_pose.position.x} Y: {desired_pose.position.y} "
            f"Z: {desired_pose.orientation.z} W: {desired_pose.orientation.w}"
        )

    def goal_status_callback(self, msg):
        if len(msg.status_list) > 5:
            self.goals_in_sys = msg.status_list[-5:]
        else:
            self.goals_in_sys = msg.status_list

    def amcl_callback(self, msg):
        self.robot_pose = msg.pose.pose

    def people_callback(self, msg):
        self.people_poses = list(msg.poses)
        if msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0:
            self.last_people_seen_time = Time.from_msg(msg.header.stamp)
        else:
            self.last_people_seen_time = self.node.get_clock().now()
