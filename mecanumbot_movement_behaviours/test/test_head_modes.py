"""
Who has the neck: a named pose, a gesture, the look-down, or the body gaze.

The gaze runs on a timer beside the tree, so what keeps it from fighting a
gesture script or a route turn for the head is not the order behaviours tick in
but one piece of state -- what the head was last *asked for*. These tests hold
that hand-over. No ROS graph: the node is a fake.
"""

import pytest

ros_interfaces = pytest.importorskip("mecanumbot_movement_behaviours.ros_interfaces")

from geometry_msgs.msg import Pose, PoseArray, PoseWithCovarianceStamped  # noqa: E402
from rclpy.time import Time  # noqa: E402

from mecanumbot_movement_behaviours.body_gaze import (  # noqa: E402
    BEAN_BAG,
    BodyGazeController,
    Seat,
)

AccessoryCommander = ros_interfaces.AccessoryCommander
BodyGaze = ros_interfaces.BodyGaze
HeadLookDown = ros_interfaces.HeadLookDown
HEAD_SEEK = ros_interfaces.HEAD_SEEK
HEAD_LEVEL = ros_interfaces.HEAD_LEVEL


class _Publisher:
    def __init__(self):
        self.sent = []

    def publish(self, message):
        self.sent.append(message)


class _Logger:
    def info(self, *args, **kwargs):
        pass

    warn = info


class _Clock:
    def __init__(self):
        self.seconds = 100.0

    def now(self):
        return Time(nanoseconds=int(self.seconds * 1e9))


class _Node:
    def __init__(self):
        self.clock = _Clock()
        self.publishers = {}
        self.callbacks = {}
        self.timers = []

    def get_clock(self):
        return self.clock

    def get_logger(self):
        return _Logger()

    def create_publisher(self, message_type, topic, qos):
        return self.publishers.setdefault(topic, _Publisher())

    def create_subscription(self, message_type, topic, callback, qos):
        self.callbacks.setdefault(topic, []).append(callback)
        return object()

    def create_timer(self, period, callback):
        self.timers.append(callback)
        return object()

    def deliver(self, topic, message):
        for callback in self.callbacks.get(topic, []):
            callback(message)

    def neck_commands(self):
        return [cmd.n_pos for cmd in self.publishers[ros_interfaces.ACCESSORY_TOPIC].sent]


@pytest.fixture
def node():
    """Give each test a fresh robot: no head position, no gaze, no look-down."""
    saved = (AccessoryCommander.seek_pos, AccessoryCommander.level_pos)
    AccessoryCommander._last_neck_pos = None
    AccessoryCommander._head_mode = None
    AccessoryCommander.configure(seek_pos=6.5, level_pos=6.0)
    BodyGaze._instance = None
    HeadLookDown._state = None
    yield _Node()
    AccessoryCommander._last_neck_pos = None
    AccessoryCommander._head_mode = None
    AccessoryCommander.configure(seek_pos=saved[0], level_pos=saved[1])
    BodyGaze._instance = None
    HeadLookDown._state = None


def robot_at(node, x=0.0, y=0.0):
    """Tell every pose tracker the robot is at `(x, y)`, looking down +x."""
    message = PoseWithCovarianceStamped()
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    message.pose.pose.orientation.w = 1.0
    node.deliver(ros_interfaces.AMCL_TOPIC, message)


def lidar_sees(node, *points):
    message = PoseArray()
    for x, y in points:
        pose = Pose()
        pose.position.x, pose.position.y = float(x), float(y)
        message.poses.append(pose)
    node.deliver(ros_interfaces.CANDIDATES_TOPIC, message)


def start_gaze(node, seats=()):
    gaze = BodyGaze.start(node, BodyGazeController(seats=seats))
    robot_at(node)
    return gaze


def tick(node, seconds=0.5):
    node.clock.seconds += seconds
    for timer in node.timers:
        timer()


class TestWithoutAGaze:
    def test_seek_and_level_are_the_two_fixed_poses(self, node):
        head = AccessoryCommander(node)
        head.look(HEAD_SEEK)
        head.look(HEAD_LEVEL)
        assert node.neck_commands() == [6.5, 6.0]

    def test_a_redundant_look_sends_nothing(self, node):
        head = AccessoryCommander(node)
        head.look(HEAD_SEEK)
        head.look(HEAD_SEEK)
        assert node.neck_commands() == [6.5]

    def test_look_none_leaves_the_head_and_its_mode_alone(self, node):
        head = AccessoryCommander(node)
        head.look(HEAD_SEEK)
        head.look(None)
        assert AccessoryCommander._head_mode == HEAD_SEEK
        assert node.neck_commands() == [6.5]


class TestTheGazeSteersASeekingHead:
    def test_a_near_candidate_lifts_the_head(self, node):
        head = AccessoryCommander(node)
        start_gaze(node)
        head.look(HEAD_SEEK)
        lidar_sees(node, (0.8, 0.0))
        tick(node)
        assert node.neck_commands()[-1] > 7.0

    def test_a_seat_lowers_it(self, node):
        head = AccessoryCommander(node)
        start_gaze(node, seats=[Seat(BEAN_BAG, 1.5, 0.0, 0.7)])
        head.look(HEAD_SEEK)
        lidar_sees(node, (1.43, 0.0))
        tick(node)
        assert node.neck_commands()[-1] < 6.5

    def test_a_stale_candidate_list_is_no_list(self, node):
        head = AccessoryCommander(node)
        start_gaze(node)
        head.look(HEAD_SEEK)
        lidar_sees(node, (0.8, 0.0))
        tick(node, seconds=2.0)  # perception stopped two seconds ago
        assert node.neck_commands() == [6.5]

    def test_asking_for_the_seeking_head_again_does_not_drop_it(self, node):
        # Every search behaviour opens with look(HEAD_SEEK); with a gaze
        # running that must not snap the head back to the resting pose.
        head = AccessoryCommander(node)
        start_gaze(node)
        head.look(HEAD_SEEK)
        lidar_sees(node, (0.8, 0.0))
        tick(node)
        lifted = node.neck_commands()[-1]
        head.look(HEAD_SEEK)
        assert node.neck_commands()[-1] == lifted

    def test_only_one_gaze_is_ever_started(self, node):
        assert start_gaze(node) is start_gaze(node)
        assert len(node.timers) == 1


class TestWhatTakesTheHeadAway:
    def test_a_level_head_is_left_alone(self, node):
        head = AccessoryCommander(node)
        start_gaze(node)
        head.look(HEAD_LEVEL)
        lidar_sees(node, (0.8, 0.0))
        tick(node)
        assert node.neck_commands() == [6.0]

    def test_a_gesture_script_takes_the_neck(self, node):
        head = AccessoryCommander(node)
        start_gaze(node)
        head.look(HEAD_SEEK)
        head.send(7.0, 6.83, 3.36)  # one step of the attention nod
        lidar_sees(node, (3.5, 0.0))
        tick(node)
        assert node.neck_commands()[-1] == 7.0
        assert AccessoryCommander._head_mode is None

    def test_and_the_next_seeking_behaviour_gets_it_back(self, node):
        head = AccessoryCommander(node)
        start_gaze(node)
        head.look(HEAD_SEEK)
        head.send(7.0, 6.83, 3.36)
        head.look(HEAD_SEEK)
        lidar_sees(node, (3.5, 0.0))
        tick(node)
        assert node.neck_commands()[-1] < 7.0

    def test_the_look_down_holds_the_gaze_off(self, node):
        head = AccessoryCommander(node)
        start_gaze(node)
        head.look(HEAD_SEEK)
        look_down = HeadLookDown(node, head, 0.5, 3.0, 4.0, 0.5)
        node.deliver(ros_interfaces.LOW_HEAD_TOPIC, None)
        look_down.update()
        assert HeadLookDown.active()
        lowered = node.neck_commands()[-1]
        lidar_sees(node, (0.8, 0.0))
        tick(node)
        assert node.neck_commands()[-1] == lowered
        # ... and the look-down moving the neck did not end the seeking head.
        assert AccessoryCommander._head_mode == HEAD_SEEK

    def test_without_a_robot_pose_the_gaze_waits(self, node):
        head = AccessoryCommander(node)
        BodyGaze.start(node, BodyGazeController())
        head.look(HEAD_SEEK)
        lidar_sees(node, (0.8, 0.0))
        tick(node)
        assert node.neck_commands() == [6.5]
