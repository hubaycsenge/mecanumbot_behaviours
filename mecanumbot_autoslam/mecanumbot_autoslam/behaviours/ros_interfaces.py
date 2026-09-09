"""
The ROS plumbing the autoslam behaviours own instead of re-implementing.

Same idea as `mecanumbot_movement_behaviours/ros_interfaces.py`, and the nav2
half is literally that module: `ExplorationNavigator` is its `Nav2PoseNavigator`
with the action name made settable, so a goal from this package is sent,
followed and cancelled exactly the way a leading tree's goal is.

What is here rather than there is the input side of an exploration pass, which
no other behaviour in the repository has: the occupancy grid, the robot's pose
read from the transform tree rather than from AMCL, and the two things the
Deep3R server publishes.

Everything is a subscription plus the last value. Nothing here decides.
"""

import rclpy
from geometry_msgs.msg import Point, Pose, PoseArray, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from mecanumbot_msgs.msg import MapCloudAgreement

from mecanumbot_custom_nav2.exit_criteria import CloudProgress, travelled
from mecanumbot_custom_nav2.occupancy import Grid

from mecanumbot_movement_behaviours.ros_interfaces import (
    NAV2_TO_POSE_ACTION,
    STATUS_ABORTED,
    STATUS_CANCELED,
    STATUS_SUCCEEDED,
    Nav2PoseNavigator,
)

# Both the map and AMCL publish transient-local, so a subscriber that joins late
# still gets the current value instead of waiting for the next update -- and the
# map updates every 5 s by default, which is a long time to sit still.
LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)

#: Below this fraction of the previous known area, a new grid is a
#: re-rasterisation rather than an update. slam_toolbox redraws the whole map
#: after a loop closure, and that is the only thing that makes the known area
#: go *down*.
LOOP_CLOSURE_SHRINK = 0.98


class MapTracker:
    """The occupancy grid, and whether slam_toolbox just re-rasterised it."""

    def __init__(self, node, topic, free_threshold, occupied_threshold):
        """Subscribe to the grid and hold the latest one, decoded."""
        self.grid = None
        self._free = int(free_threshold)
        self._occupied = int(occupied_threshold)
        self._known_cells = None
        self._loop_closed = False
        self._subscription = node.create_subscription(
            OccupancyGrid, topic, self._callback, LATCHED
        )

    def _callback(self, msg):
        self.grid = Grid.from_message(
            msg, free_threshold=self._free, occupied_threshold=self._occupied
        )
        known = self.grid.known_cells()
        if self._known_cells is not None and \
                known < self._known_cells * LOOP_CLOSURE_SHRINK:
            self._loop_closed = True
        self._known_cells = known

    def known_cells(self):
        """Return the number of observed cells in the latest grid."""
        return 0 if self.grid is None else self.grid.known_cells()

    def take_loop_closure(self):
        """
        Report a loop closure once, and forget it.

        Read rather than subscribed to, because both things that have to react
        -- the exit criteria's settle timer and the RRT, whose trees were grown
        against free space that no longer exists -- do so on the next tick and
        not from inside a callback.
        """
        closed, self._loop_closed = self._loop_closed, False
        return closed


class PoseSource:
    """
    Where the robot is, and how far it has driven to get there.

    T1 runs under slam_toolbox, which publishes `map -> odom` but no
    `/amcl_pose`, so the default reads the transform tree. `amcl` runs the same
    behaviours against a saved map, which is what makes them usable outside an
    exploration pass.
    """

    def __init__(self, node, source, pose_topic, map_frame, base_frame):
        """Start whichever pose source `source` names."""
        self.node = node
        self.xy = None
        self.distance = 0.0
        self._previous = None
        self._map_frame = map_frame
        self._base_frame = base_frame
        self._buffer = None
        self._listener = None
        if source == "tf":
            self._buffer = Buffer()
            self._listener = TransformListener(self._buffer, node)
        else:
            node.create_subscription(
                PoseWithCovarianceStamped, pose_topic, self._on_pose, LATCHED
            )

    def poll(self):
        """Read the transform tree, if that is where the pose comes from."""
        if self._buffer is None:
            return
        try:
            transform = self._buffer.lookup_transform(
                self._map_frame, self._base_frame, rclpy.time.Time()
            )
        except Exception as error:  # the chain is not up yet, or has gone stale
            self.node.get_logger().warn(
                "no {} -> {} transform ({})".format(
                    self._map_frame, self._base_frame, error),
                throttle_duration_sec=5.0,
            )
            return
        self._set(
            (transform.transform.translation.x, transform.transform.translation.y)
        )

    def _on_pose(self, msg):
        position = msg.pose.pose.position
        self._set((position.x, position.y))

    def _set(self, xy):
        self.xy = xy
        self.distance += travelled(self._previous, self.xy)
        self._previous = self.xy


class AgreementTracker:
    """
    The server's verdict on its own reconstruction, kept as a rate.

    `CloudProgress` is `mecanumbot_custom_nav2`'s, and so is every judgement
    made from it: this only feeds it.
    """

    def __init__(self, node, topic, window):
        """Subscribe to the agreement topic and accumulate its history."""
        self.node = node
        self.progress = CloudProgress(window=window)
        self.verdicts = 0
        node.create_subscription(MapCloudAgreement, topic, self._callback, 10)

    def _callback(self, msg):
        self.verdicts += 1
        self.progress.add(
            self.node.get_clock().now().nanoseconds / 1e9,
            msg.grid_coverage,
            msg.cloud_points,
            msg.uncertain_scores,
            uncertain_points=[(p.x, p.y) for p in msg.uncertain_points],
            cloud_map_id=msg.cloud_map_id,
            agreement=msg.agreement,
        )
        self.node.get_logger().info(
            "server: cloud covers {:.0f}% of the map, agreement {:.0f}%, "
            "{} uncertain region(s)".format(
                msg.grid_coverage * 100, msg.agreement * 100,
                len(msg.uncertain_points))
        )


class RevisitTracker:
    """
    The regions the comparison handler says are still worth another look.

    Already filtered and ordered by `mecanumbot_map_agreement`, which also drops
    a region once the robot has been near it -- so nothing is marked serviced
    here. A list that empties is the loop having closed.
    """

    def __init__(self, node, topic):
        """Subscribe to the revisit list."""
        self.points = []
        node.create_subscription(PoseArray, topic, self._callback, 10)

    def _callback(self, msg):
        self.points = [(p.position.x, p.position.y) for p in msg.poses]


class ExplorationNavigator(Nav2PoseNavigator):
    """`Nav2PoseNavigator` with the action name settable."""

    def __init__(self, node, action_name=NAV2_TO_POSE_ACTION):
        """Bind to `action_name` instead of the workspace default."""
        # Set before super().__init__, which reads it to build the client.
        self.ACTION_NAME = action_name
        super().__init__(node)

    def go_to_point(self, point):
        """
        Drive to an (x, y), facing whichever way the robot already faces.

        A frontier has no natural orientation, and asking nav2 for a specific
        yaw at one costs a turn at the end of every leg for nothing.
        """
        pose = Pose()
        pose.position.x = float(point[0])
        pose.position.y = float(point[1])
        pose.orientation.w = 1.0
        self.go_to(pose)

    def settled(self):
        """Report whether nav2 has finished with our goal, however it ended."""
        return self.status() in (STATUS_SUCCEEDED, STATUS_ABORTED, STATUS_CANCELED)

    def succeeded(self):
        """Report whether nav2 drove the goal to completion."""
        return self.status() == STATUS_SUCCEEDED


class ExplorationSignals:
    """What the pass tells the rest of the robot: finished, state, markers."""

    def __init__(self, node, finished_topic, state_topic, marker_topic):
        """Create the three publishers and latch `finished` as false."""
        self.node = node
        self._finished = node.create_publisher(Bool, finished_topic, LATCHED)
        self._state = node.create_publisher(String, state_topic, 10)
        self._markers = node.create_publisher(MarkerArray, marker_topic, 10)
        self._finished.publish(Bool(data=False))

    def finish(self):
        """Latch `finished`. This is how T1 hands over to T2."""
        self._finished.publish(Bool(data=True))

    def state(self, text):
        """Publish the one-line state string."""
        self._state.publish(String(data=text))

    def frontiers(self, scored, goal):
        """Publish the frontier candidates and the goal being driven to."""
        markers = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.ns = "frontiers"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = marker.scale.y = marker.scale.z = 0.2
        marker.color.g = 1.0
        marker.color.a = 0.8
        marker.pose.orientation.w = 1.0
        for frontier in scored:
            marker.points.append(Point(x=frontier.x, y=frontier.y, z=0.1))
        markers.markers.append(marker)

        if goal is not None:
            chosen = Marker()
            chosen.header = marker.header
            chosen.ns = "frontier_goal"
            chosen.id = 1
            chosen.type = Marker.SPHERE
            chosen.action = Marker.ADD
            chosen.scale.x = chosen.scale.y = chosen.scale.z = 0.35
            chosen.color.r = 1.0
            chosen.color.a = 0.9
            chosen.pose.position.x = goal[0]
            chosen.pose.position.y = goal[1]
            chosen.pose.position.z = 0.1
            chosen.pose.orientation.w = 1.0
            markers.markers.append(chosen)
        self._markers.publish(markers)
