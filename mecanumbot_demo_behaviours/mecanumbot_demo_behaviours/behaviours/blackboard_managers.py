import math
import os
import py_trees
import yaml
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from ament_index_python.packages import get_package_share_directory
from mecanumbot_demo_behaviours.utils.map_generate import generate_waypoints


# The constants loader is the leading package's, not a copy of it. There used to
# be a fork here that duplicated every key, defaulted `last_distance` to a
# different number and never learned about the tunables added to the YAML since;
# nothing imported it, and two loaders disagreeing about one file is worse than
# an import. `wander_between_people.py` already imported the real one.
from mecanumbot_leading_behaviour.behaviours.blackboard_managers import (  # noqa: F401
    ConstantParamsToBlackboard,
)


class DistanceToBlackboard(py_trees.behaviour.Behaviour):  # Checks done - works
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
        # self.blackboard.register_key("subject_position", py_trees.common.Access.READ)
        self.blackboard.register_key("target_position", py_trees.common.Access.READ)
        self.blackboard.register_key(
            "robot_closeness_threshold", py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            "target_reached_threshold", py_trees.common.Access.READ
        )

        # register the keys we WRITE
        self.blackboard.register_key(
            "robot_distance_from_subject", py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            "subject_within_robot_threshold", py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            "distance_from_target", py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            "target_within_robot_threshold", py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            "target_distance_from_subject", py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            "target_within_subject_threshold", py_trees.common.Access.WRITE
        )

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
            node = kwargs["node"]
            self.node = node
        except KeyError:
            raise KeyError("The 'node' argument was not passed to setup()")

        # Define QoS profile to match the amcl publisher (TRANSIENT_LOCAL)
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        # Create the subscriber using the provided node context
        self.robot_subscriber = node.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.amcl_callback,
            qos_profile=qos_profile,
        )
        self.subject_subscriber = node.create_subscription(
            PoseStamped,
            "/mecanumbot/subject_pose",
            self.subject_callback,
            qos_profile=10,
        )
        self.feedback_message = "DistanceToBlackboard setup complete"
        self.logger.info(self.feedback_message)
        return True

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        """
        Callback to store the latest robot pose.
        We only need the Pose component (which contains the position Point).
        """
        self.robot_pose = msg.pose.pose  # geometry_msgs/Pose

    def subject_callback(self, msg: PoseStamped):
        """
        Callback to store the latest subject position.
        We only need the position Point.
        """
        self.subject_position = msg.pose.position  # geometry_msgs/Point

    def calculate_distance(self, position1: Point, position2: Point):
        """
        Compute Euclidean distance between two Points
        """
        x1, y1 = position1.x, position1.y
        x2, y2 = position2.x, position2.y

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
            target_position = self.blackboard.target_position  # geometry_msgs/Point
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
        self.blackboard.subject_within_robot_threshold = (
            d_subject < self.blackboard.robot_closeness_threshold
        )

        self.blackboard.distance_from_target = d_target
        self.blackboard.target_within_robot_threshold = (
            d_target < self.blackboard.robot_closeness_threshold
        )

        self.blackboard.target_distance_from_subject = d_subject_target
        self.blackboard.target_within_subject_threshold = (
            d_subject_target < self.blackboard.target_reached_threshold
        )

        self.feedback_message = f"""d(R, S) = {d_subject:.2f} | d(R, T) = {d_target:.2f} | d(S, T) = {d_subject_target:.2f}"""
        self.node.get_logger().info(self.feedback_message)
        return py_trees.common.Status.SUCCESS


class MapWaypointsToBlackboard(py_trees.behaviour.Behaviour):
    """
    Loads a map-specific waypoint YAML into the blackboard.

    If the waypoint file does not exist yet, it generates the waypoints from the
    corresponding map using generate_waypoints and then loads the generated file.
    """

    def __init__(self, name="MapWaypointsToBlackboard", base_name=None):
        super().__init__(name)

        self.base_name = base_name
        self.map_name = base_name + ".pgm"
        self.map_yaml_path = base_name + ".yaml"
        self.waypoints_yaml_path = base_name + "_waypoints.yaml"

        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            "patrol_start_waypoint", access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            "patrol_end_waypoint", access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key("waypoints", access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        try:
            self.node = kwargs["node"]
        except KeyError as exc:
            raise KeyError("The 'node' argument was not passed to setup()") from exc

        self.feedback_message = "MapWaypointsToBlackboard setup complete"
        self.logger.info(self.feedback_message)
        return True

    def _resolve_waypoints_yaml_path(self, map_name):
        if self.waypoints_yaml_path:
            return self.waypoints_yaml_path

        if self.map_yaml_path:
            map_dir = os.path.dirname(self.map_yaml_path)
            return os.path.join(map_dir, f"{map_name}_waypoints.yaml")

        description_base_path = get_package_share_directory("mecanumbot_description")
        return os.path.join(
            description_base_path, "maps", map_name, f"{map_name}_waypoints.yaml"
        )

    def _load_waypoints(self, waypoints_yaml_path):
        with open(waypoints_yaml_path, "r", encoding="utf-8") as stream:
            payload = yaml.safe_load(stream) or {}

        waypoints = payload.get("waypoints", payload)
        if not isinstance(waypoints, list):
            raise ValueError(
                f"Waypoint file {waypoints_yaml_path} does not contain a waypoint list"
            )

        parsed_waypoints = []
        for waypoint in waypoints:
            point = Point()
            if isinstance(waypoint, dict):
                point.x = float(waypoint["x"])
                point.y = float(waypoint["y"])
                point.z = float(waypoint.get("z", 0.0))
            else:
                point.x = float(waypoint[0])
                point.y = float(waypoint[1])
                point.z = float(waypoint[2]) if len(waypoint) > 2 else 0.0
            parsed_waypoints.append(point)

        if not parsed_waypoints:
            raise ValueError(
                f"Waypoint file {waypoints_yaml_path} did not contain any waypoints"
            )

        return parsed_waypoints

    def initialise(self):
        waypoints_yaml_path = os.path.join(
            get_package_share_directory("mecanumbot_description"),
            "maps",
            self.base_name,
            f"{self.base_name}_waypoints.yaml",
        )

        if not os.path.exists(waypoints_yaml_path):
            self.node.get_logger().info(
                f"Waypoint file {waypoints_yaml_path} not found, generating waypoints for map {self.base_name}"
            )
            generate_waypoints(self.base_name)

        if not os.path.exists(waypoints_yaml_path):
            raise FileNotFoundError(
                f"Unable to load or generate waypoint file for map {self.base_name}: {waypoints_yaml_path}"
            )

        patrol_waypoints = self._load_waypoints(waypoints_yaml_path)

        self.blackboard.patrol_start_waypoint = patrol_waypoints[0]
        self.blackboard.target_end_waypoint = patrol_waypoints[-1]
        self.blackboard.waypoints = patrol_waypoints

        self.node.get_logger().info(
            f"Loaded {len(patrol_waypoints)} waypoints for map {self.base_name} from {waypoints_yaml_path}"
        )
        return super().initialise()

    def update(self):
        return py_trees.common.Status.SUCCESS
