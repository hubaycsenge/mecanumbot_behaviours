"""
Holding a leading tree still until somebody has come in.

`WaitForTrialStart` sits straight after the constants loader in every leading
tree. What counts as somebody coming in is `trial_start.py`'s business; this is
the ROS end of it -- the camera's detections, the scan, the head -- and the
latch.
"""

import py_trees
from mecanumbot_msgs.msg import CamPersonDetectionArray
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from mecanumbot_movement_behaviours.ros_interfaces import AccessoryCommander, now_seconds

from mecanumbot_leading_behaviour.behaviours.defaults import constant, register_param_keys
from mecanumbot_leading_behaviour.behaviours.trial_start import (
    CameraSighting,
    ScanChangeDetector,
    ScanChangeSettings,
    waiting_neck_pos,
)

SCAN_TOPIC = "/mecanumbot/scan"
# The detector's own output rather than `people_fusion`, which also carries
# bodies only the LiDAR has seen.
CAM_DETECTIONS_TOPIC = "/mecanumbot/cam_people_detections"

START_KEYS = (
    "trial_start_enabled",
    "trial_start_camera",
    "trial_start_camera_hold",
    "trial_start_camera_gap",
    "trial_start_lidar",
    "trial_start_head",
    "trial_start_look_distance",
    "body_gaze_min_pos",
    "body_gaze_max_pos",
    "trial_start_baseline_scans",
    "trial_start_min_change",
    "trial_start_angle_tolerance",
    "trial_start_max_range",
    "trial_start_min_beams",
    "trial_start_persist_scans",
    "trial_start_persist_gate",
)


class WaitForTrialStart(py_trees.behaviour.Behaviour):
    """
    RUNNING until a person is first on camera (or, if enabled, the LiDAR pattern changes).

    While it waits it holds the head where a standing person at
    `trial_start_look_distance` is in frame (`trial_start_head`). It leaves the
    head there when the trial starts: the tree's own first head command takes
    it from there.

    Latched: once it has succeeded it succeeds at once on every later tick, so
    a root that fails and is ticked again from the top does not wait for a
    second participant. Switched off (`trial_start_enabled: false`) it succeeds
    straight away, which is the old behaviour.
    """

    def __init__(self, name="WaitForTrialStart"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        register_param_keys(self.blackboard, *START_KEYS)
        self.started = False
        self.scan_change = None
        self.sighting = None

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self._scan_sub = self.node.create_subscription(
            LaserScan, SCAN_TOPIC, self._scan_callback, qos_profile_sensor_data
        )
        # The camera's detections, not `people_fusion`: the fusion also holds
        # bodies the LiDAR alone has followed, which started trials in an
        # empty room (2026-10-28). A person here has passed the detector's gate.
        self._camera_sub = self.node.create_subscription(
            CamPersonDetectionArray, CAM_DETECTIONS_TOPIC, self._camera_callback, 10
        )
        self.accessories = AccessoryCommander(self.node)

    def initialise(self):
        if self.started or self.scan_change is not None:
            return

        def value(key):
            return constant(self.blackboard, key)

        self.enabled = bool(value("trial_start_enabled"))
        self.use_camera = bool(value("trial_start_camera"))
        self.use_lidar = bool(value("trial_start_lidar"))
        self.sighting = CameraSighting(
            hold=float(value("trial_start_camera_hold")),
            gap=float(value("trial_start_camera_gap")),
        )
        # Built on the first tick rather than in setup(), because the constants
        # are only on the blackboard once the loader has run.
        self.scan_change = ScanChangeDetector(
            ScanChangeSettings(
                baseline_scans=int(value("trial_start_baseline_scans")),
                min_change=float(value("trial_start_min_change")),
                angle_tolerance=float(value("trial_start_angle_tolerance")),
                max_range=float(value("trial_start_max_range")),
                min_beams=int(value("trial_start_min_beams")),
                persist_scans=int(value("trial_start_persist_scans")),
                persist_gate=float(value("trial_start_persist_gate")),
            )
        )
        sources = [
            name
            for name, used in (
                ("a person is on camera", self.use_camera),
                ("something new stands in the LiDAR's room", self.use_lidar),
            )
            if used
        ]
        if self.enabled and sources:
            self.node.get_logger().info(
                f"{self.name}: holding still until {' or '.join(sources)}"
            )
            if bool(value("trial_start_head")):
                self._raise_head(float(value("trial_start_look_distance")),
                                 value("body_gaze_min_pos"), value("body_gaze_max_pos"))

    def _raise_head(self, distance, min_pos, max_pos):
        # Not a named pose, so the body gaze (which moves only a seeking head)
        # leaves it alone until the tree takes the head over.
        n_pos = waiting_neck_pos(distance, min_pos, max_pos)
        self.accessories.send(n_pos)
        self.node.get_logger().info(
            f"{self.name}: head -> n_pos={n_pos:.2f}, a standing person at {distance:.1f} m"
        )

    def _scan_callback(self, msg):
        if self.started or self.scan_change is None or not self.use_lidar:
            return
        self.scan_change.update(
            msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max
        )

    def _camera_callback(self, msg):
        if self.started or self.sighting is None or not self.use_camera:
            return
        self.sighting.update(now_seconds(self.node), bool(msg.people))

    def update(self):
        if self.started:
            return py_trees.common.Status.SUCCESS
        if not self.enabled or not (self.use_camera or self.use_lidar):
            self.started = True
            return py_trees.common.Status.SUCCESS

        reason = None
        if self.sighting.seen:
            reason = self.sighting.reason
        elif self.scan_change.changed:
            reason = self.scan_change.reason
        if reason is not None:
            self.started = True
            self.node.get_logger().info(f"{self.name}: trial started -- {reason}")
            return py_trees.common.Status.SUCCESS

        if self.use_lidar and not self.scan_change.ready:
            self.feedback_message = "learning the room from the LiDAR"
        else:
            self.feedback_message = "waiting for somebody to come in"
        return py_trees.common.Status.RUNNING
