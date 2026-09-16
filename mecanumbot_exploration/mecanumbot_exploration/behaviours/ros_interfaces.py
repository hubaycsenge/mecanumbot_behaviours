"""ROS 2 interfaces for the exploration pass.

Three thin wrappers (PoseTracker, ExploreClient, NavClient) and one publisher
bundle (ExplorationSignals).  None of them decide anything; all decisions are
in monitoring.py and exploration_node.py.
"""

import math

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool


class PoseTracker:
    """Subscribe to slam_toolbox's pose and track covariance + displacement."""

    def __init__(self, node, pose_topic):
        self.xy = None
        self.covariance_trace = 0.0
        self._distance = 0.0
        self._last_xy = None
        node.create_subscription(
            PoseWithCovarianceStamped, pose_topic, self._cb, 10)

    def _cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        c = msg.pose.covariance
        # 2-D trace: x variance + y variance + yaw variance (6x6 row-major).
        self.covariance_trace = c[0] + c[7] + c[35]
        if self._last_xy is not None:
            dx = x - self._last_xy[0]
            dy = y - self._last_xy[1]
            self._distance += math.hypot(dx, dy)
        self._last_xy = (x, y)
        self.xy = (x, y)

    @property
    def distance(self):
        return self._distance


class ExploreClient:
    """Wrap the /explore/resume SetBool service."""

    def __init__(self, node, service_name):
        self._node = node
        self._client = node.create_client(SetBool, service_name)

    def pause(self, callback=None):
        self._call(False, callback)

    def resume(self, callback=None):
        self._call(True, callback)

    def _call(self, value, callback):
        if not self._client.wait_for_service(timeout_sec=3.0):
            self._node.get_logger().warn(
                "{} not available; skipping".format(
                    self._client.srv_name))
            if callback:
                callback()
            return
        req = SetBool.Request()
        req.data = value
        future = self._client.call_async(req)
        if callback:
            future.add_done_callback(lambda _f: callback())


class NavClient:
    """Wrap the NavigateToPose action."""

    def __init__(self, node, action_name):
        self._node = node
        self._client = ActionClient(node, NavigateToPose, action_name)

    def navigate_to(self, x, y, on_done):
        """Send a goal to (x, y); call on_done(success=bool) when it finishes."""
        if not self._client.wait_for_server(timeout_sec=5.0):
            self._node.get_logger().error(
                "navigate_to_pose action server not available")
            on_done(success=False)
            return
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self._node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.w = 1.0
        future = self._client.send_goal_async(goal)
        future.add_done_callback(
            lambda f: self._on_goal_response(f, on_done))

    def _on_goal_response(self, future, on_done):
        handle = future.result()
        if not handle.accepted:
            self._node.get_logger().warn("revisit goal rejected by nav2")
            on_done(success=False)
            return
        handle.get_result_async().add_done_callback(
            lambda f: on_done(
                success=f.result().status == GoalStatus.STATUS_SUCCEEDED))


class ExplorationSignals:
    """Publish exploration/finished (latched) and exploration/state."""

    _LATCHING = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

    def __init__(self, node, finished_topic, state_topic):
        self._finished_pub = node.create_publisher(
            Bool, finished_topic, self._LATCHING)
        self._state_pub = node.create_publisher(String, state_topic, 10)
        msg = Bool()
        msg.data = False
        self._finished_pub.publish(msg)

    def finish(self):
        msg = Bool()
        msg.data = True
        self._finished_pub.publish(msg)

    def state(self, text):
        msg = String()
        msg.data = text
        self._state_pub.publish(msg)
