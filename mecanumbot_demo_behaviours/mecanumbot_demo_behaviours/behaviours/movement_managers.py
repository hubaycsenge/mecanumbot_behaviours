import random

import py_trees
import numpy as np
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseArray, PoseStamped, PoseWithCovarianceStamped, Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.time import Time

from mecanumbot_leading_behaviour.behaviours.constants import (
    TUNABLE_DEFAULTS as SHARED_DEFAULTS,
    register_param_keys,
    resolve,
)
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


# The demo trees' own tunables, on top of the ones the leading behaviour library
# declares. They are read off the blackboard through the same helper, so a value
# is taken from whichever constants YAML the tree loaded and falls back to the
# number here when that file does not mention it. The demo package's own
# `config/*.yaml` files declare all of them.
#
# They are demo-specific on purpose: wandering up to whoever happens to be in the
# room is a different job from leading somebody along a route, so it spins more
# gently, gives a detection longer to count and stops further away.
DEMO_DEFAULTS = {
    "demo_spin_speed": 0.2,
    "demo_sight_timeout": 3.0,
    "demo_person_stop_distance": 0.5,
    "demo_tick_period_ms": 10.0,
    "demo_map_name": "AI_dept",
}

DEMO_KEYS = tuple(DEMO_DEFAULTS)


# ---------------------------------------------------------
# 1. Custom Node: Check Detections & Update Target
# ---------------------------------------------------------
class ProcessDetections(py_trees.behaviour.Behaviour):
    def __init__(self, name="Process Detections"):
        super().__init__(name)
        # Create a blackboard client to read detections and write targets
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("pose_array", access=py_trees.common.Access.READ)
        self.blackboard.register_key(
            "intercept_goal", access=py_trees.common.Access.WRITE
        )

    def update(self):
        # Read the latest PoseArray from the blackboard
        pose_array_msg = self.blackboard.pose_array

        if pose_array_msg is None or len(pose_array_msg.poses) == 0:
            return (
                py_trees.common.Status.FAILURE
            )  # No detections, tree will fall back to patrolling

        # We have a detection! Extract the first one and format it for Nav2
        target_pose = PoseStamped()
        target_pose.header = pose_array_msg.header
        target_pose.pose = pose_array_msg.poses[0]

        # Write to blackboard for the Action Client to use
        self.blackboard.intercept_goal = target_pose

        return py_trees.common.Status.SUCCESS


# ---------------------------------------------------------
# 2. Custom Node: Cycle Waypoints
# ---------------------------------------------------------
class GetNextWaypoint(py_trees.behaviour.Behaviour):
    def __init__(self, name="Get Waypoint"):
        super().__init__(name)
        self.current_idx = 0
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("waypoints", access=py_trees.common.Access.READ)
        self.blackboard.register_key("patrol_goal", access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        try:
            self.node = kwargs["node"]
        except KeyError:
            self.node = None
        return True

    def _waypoint_to_pose(self, waypoint):
        pose = waypoint
        if hasattr(waypoint, "position"):
            pose = waypoint.position

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.pose.position.x = float(pose.x)
        goal_pose.pose.position.y = float(pose.y)
        goal_pose.pose.position.z = float(getattr(pose, "z", 0.0))
        goal_pose.pose.orientation.w = 1.0
        return goal_pose

    def update(self):
        waypoints = self.blackboard.waypoints
        if not waypoints:
            if self.node is not None:
                self.node.get_logger().info(
                    f"{self.name}: waiting for waypoints on blackboard"
                )
            return py_trees.common.Status.RUNNING

        if self.current_idx >= len(waypoints):
            self.current_idx = 0

        goal_msg = self._waypoint_to_pose(waypoints[self.current_idx])

        self.blackboard.patrol_goal = goal_msg

        # Increment for the next time this succeeds
        self.current_idx = (self.current_idx + 1) % len(waypoints)
        return py_trees.common.Status.SUCCESS


class FindPeople(py_trees.behaviour.Behaviour):
    """Spin in place until a fresh people_fusion message is received.

    The spin speed is `demo_spin_speed` and the detection age `sight_timeout`,
    both taken from the constants YAML the tree loaded.
    """

    def __init__(self, name="FindPeople", spin_speed=None, sight_timeout=None):
        super().__init__(name)
        self.spin_speed = spin_speed
        self.sight_timeout = sight_timeout
        self.publisher = None
        self.people_poses = []
        self.last_people_seen_time = None

        self.blackboard = self.attach_blackboard_client(name=name)
        register_param_keys(self.blackboard, "sight_timeout", *DEMO_KEYS)

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.spin_speed = float(
            resolve(self.spin_speed, self.blackboard, "demo_spin_speed", DEMO_DEFAULTS)
        )
        self.sight_timeout = float(
            resolve(
                self.sight_timeout, self.blackboard, "sight_timeout", SHARED_DEFAULTS
            )
        )
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
    """Select a fresh person at random from people_fusion and navigate toward them.

    `demo_sight_timeout` is how old a detection may be and still be walked to --
    longer than the leading trees' `sight_timeout`, because here a person who was
    seen a moment ago is still somewhere worth going.
    """

    def __init__(self, name="GoToRandomPerson", sight_timeout=None, stop_distance=None):
        super().__init__(name)
        self.stop_distance = stop_distance
        self.blackboard = self.attach_blackboard_client(name=name)
        register_param_keys(self.blackboard, *DEMO_KEYS)
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
        self.sight_timeout = sight_timeout
        self.last_people_seen_time = None

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.sight_timeout = float(
            resolve(
                self.sight_timeout, self.blackboard, "demo_sight_timeout", DEMO_DEFAULTS
            )
        )
        self.stop_distance = float(
            resolve(
                self.stop_distance,
                self.blackboard,
                "demo_person_stop_distance",
                DEMO_DEFAULTS,
            )
        )

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
            self.node.get_logger().info(
                f"{self.name}: Selecting a random person to approach"
            )
            if not self.has_fresh_people():
                self.node.get_logger().info(
                    f"{self.name}: No fresh people detected, cannot select a target"
                )
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
                self.node.get_logger().info(
                    f"{self.name}: Waiting for previous goal to finish"
                )
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
            # self.node.get_logger().info(f"{self.name}: Checking goal UUID {goal.goal_info.goal_id.uuid} against {self.goal_uuid}")
            if np.array_equal(goal.goal_info.goal_id.uuid, self.goal_uuid):
                self.goal_status = goal.status
        if self.goal_status == STATUS_SUCCEEDED:
            self.node.get_logger().info(f"{self.name}: Turn completed successfully")
            return py_trees.common.Status.SUCCESS
        elif self.goal_status in [STATUS_EXECUTING, STATUS_ACCEPTED]:
            # self.node.get_logger().info(f"{self.name}: Turn in progress")
            return py_trees.common.Status.RUNNING
        elif self.goal_status in [
            STATUS_ABORTED,
            STATUS_CANCELED,
            STATUS_CANCELING,
            STATUS_UNKNOWN,
        ]:
            self.node.get_logger().info(f"{self.name}: Turn failed, retrying")
            self.goal_sent = False
            self.goal_uuid = None
            return py_trees.common.Status.RUNNING

        if self.goal_status == STATUS_SUCCEEDED:
            self.node.get_logger().info(
                f"{self.name}: Random-person navigation completed successfully"
            )
            return py_trees.common.Status.SUCCESS
        if self.goal_status in [STATUS_EXECUTING, STATUS_ACCEPTED]:
            return py_trees.common.Status.RUNNING
        if self.goal_status in [
            STATUS_ABORTED,
            STATUS_CANCELED,
            STATUS_CANCELING,
            STATUS_UNKNOWN,
        ]:
            self.node.get_logger().info(
                f"{self.name}: Navigation failed, selecting another person"
            )
            self.goal_sent = False
            self.goal_uuid = None
            self.compare_position = None
            self.selected_person = None
            return py_trees.common.Status.RUNNING

        return py_trees.common.Status.RUNNING

    def assign_goal_uuid(self):
        if self.goals_in_sys is None or self.cmd_send_time is None:
            self.node.get_logger().info(
                f"{self.name}: Cannot assign goal UUID, goals_in_sys or cmd_send_time is None"
            )
            return

        cmd_time = Time.from_msg(self.cmd_send_time.to_msg())
        for goal_status in reversed(self.goals_in_sys):
            goal_time = Time.from_msg(goal_status.goal_info.stamp)
            if goal_time >= cmd_time:
                self.goal_uuid = goal_status.goal_info.goal_id.uuid
                self.node.get_logger().info(
                    f"{self.name}: Locked onto Goal UUID {self.goal_uuid}"
                )
                return

    def check_if_running(self):
        if self.goals_in_sys is None:
            return False

        for goal_status in self.goals_in_sys:
            if goal_status.status == STATUS_EXECUTING:
                return True
        return False

    def send_goal_command(self):
        # "exact" mode: drive all the way to the person bar `stop_distance`, so
        # no per-goal step length is involved.
        desired_pose = pose_to_goal(
            self.compare_position,
            self.robot_pose,
            stop_threshold=self.stop_distance,
        )

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
