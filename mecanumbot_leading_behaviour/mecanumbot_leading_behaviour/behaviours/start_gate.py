"""
Holding a leading tree still until somebody has come in.

`WaitForTrialStart` sits straight after the constants loader in every leading
tree. What counts as somebody coming in is `trial_start.py`'s business; this is
the ROS end of it -- the scan and the fused people -- and the latch.
"""

import py_trees
from geometry_msgs.msg import PoseArray
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from mecanumbot_movement_behaviours.ros_interfaces import PEOPLE_TOPIC

from mecanumbot_leading_behaviour.behaviours.defaults import constant, register_param_keys
from mecanumbot_leading_behaviour.behaviours.trial_start import (
    ScanChangeDetector,
    ScanChangeSettings,
)

SCAN_TOPIC = "/mecanumbot/scan"

START_KEYS = (
    "trial_start_enabled",
    "trial_start_camera",
    "trial_start_lidar",
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
    RUNNING until a person is first seen or the LiDAR pattern first changes.

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
        self._person_seen = False

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self._scan_sub = self.node.create_subscription(
            LaserScan, SCAN_TOPIC, self._scan_callback, qos_profile_sensor_data
        )
        # `people_fusion` rather than the camera's own detections: a person
        # there has passed the detector's gate *and* been placed on the map,
        # near ones only with the LiDAR's backing, so the trial never starts on
        # somebody the tree then has no position to walk to.
        self._people_sub = self.node.create_subscription(
            PoseArray, PEOPLE_TOPIC, self._people_callback, 10
        )

    def initialise(self):
        if self.started or self.scan_change is not None:
            return

        def value(key):
            return constant(self.blackboard, key)

        self.enabled = bool(value("trial_start_enabled"))
        self.use_camera = bool(value("trial_start_camera"))
        self.use_lidar = bool(value("trial_start_lidar"))
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
                ("a person is tracked", self.use_camera),
                ("something new stands in the LiDAR's room", self.use_lidar),
            )
            if used
        ]
        if self.enabled and sources:
            self.node.get_logger().info(
                f"{self.name}: holding still until {' or '.join(sources)}"
            )

    def _scan_callback(self, msg):
        if self.started or self.scan_change is None or not self.use_lidar:
            return
        self.scan_change.update(
            msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max
        )

    def _people_callback(self, msg):
        if self.started or self.scan_change is None or not self.use_camera:
            return
        if msg.poses:
            self._person_seen = True

    def update(self):
        if self.started:
            return py_trees.common.Status.SUCCESS
        if not self.enabled or not (self.use_camera or self.use_lidar):
            self.started = True
            return py_trees.common.Status.SUCCESS

        reason = None
        if self._person_seen:
            reason = "a person is being tracked"
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
