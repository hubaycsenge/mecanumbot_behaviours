"""Default parameter values for the exploration pass.

Every key declared here is also declared as a ROS parameter in ExplorationNode
so that the constants YAML overrides it without the node needing to enumerate
them twice.  `/**` in the YAML matches any namespace, so the file works whether
the node is launched at the root or in `mecanumbot`.
"""


def all_defaults():
    return {
        # Topics ---------------------------------------------------------------
        # slam_toolbox publishes pose with covariance here in mapping mode.
        "pose_topic": "/pose",
        "map_frame": "map",
        "base_frame": "mecanumbot/base_link",

        # The two output topics.  `finished_topic` is latched.
        "finished_topic": "exploration/finished",
        "state_topic": "exploration/state",

        # Uncertainty monitoring -----------------------------------------------
        # Covariance trace (x + y + yaw variance) above which the robot returns
        # to the revisit point to force a loop closure before continuing.
        "uncertainty_threshold": 0.05,
        # How often to check the covariance.  A shorter period costs nothing:
        # the check is a comparison, and the subscription callback fires on
        # every slam_toolbox pose update anyway.
        "check_period": 5.0,
        # Where to return for a loop closure.  Map origin is the right value
        # for most rooms: the robot starts there, so it always has a clear path
        # back, and the overlap with the start of the map is where a closure
        # pays the most.
        "revisit_x": 0.0,
        "revisit_y": 0.0,

        # explore_lite interface -----------------------------------------------
        "explore_resume_service": "/explore/resume",
        "nav2_action": "/navigate_to_pose",

        # Exit criteria --------------------------------------------------------
        # How long the robot must stand still (not revisiting) before the pass
        # is considered complete.  explore_lite has exhausted all frontiers when
        # the robot stops moving; this window filters out momentary pauses while
        # a new goal is being planned.
        "frontier_quiet_time": 60.0,
        # Less than this many metres of movement in one tick counts as standing
        # still for the quiet-time counter.
        "movement_threshold": 0.05,
        # Hard time budget in seconds; 0 disables it.  Not a quality criterion:
        # it is what stops an unattended run if the robot loops forever.
        "max_duration": 0.0,
        # Ticks per second.  One tick per second is enough -- covariance updates
        # at the slam_toolbox pose rate, and the quiet-time check is seconds.
        "rate": 1.0,

        # Preflight ------------------------------------------------------------
        # Whether to stop AMCL and map_server before starting the pass.  The
        # localization stack publishes map -> odom; with slam_toolbox also
        # publishing it the transforms conflict and the robot's odometry jumps.
        "preflight": True,
        "preflight_discovery": 2.0,
        # Per-service budget for a single lifecycle transition.
        "preflight_timeout": 15.0,
        # Budget for the localization manager's manage_nodes call.  It answers
        # only once every node under it has finished its transitions.
        "preflight_manager_timeout": 60.0,
    }
