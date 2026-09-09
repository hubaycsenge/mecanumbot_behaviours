"""
Every constant the autoslam behaviours take, and what it does when unset.

The constants themselves are a YAML file in `config/`, loaded as ROS parameters
by the node in `tree_nodes/`. This module is the *declaration* of that file: the
name of each parameter, its packaged default and its type. Nothing here reads a
file and nothing here talks to ROS, so the defaults can be tested and the node
can declare its parameters from one place rather than from a literal buried in
its constructor.

Two groups, and the difference matters while tuning. `EXPLORATION` is where the
robot goes and how it decides -- change it and the robot explores differently.
`EXIT` is when it stops, and every one of those is a claim about what "this
place has been scanned" means; `mecanumbot_custom_nav2/exit_criteria.py` is
where the shape of that rule is argued.

`PREFLIGHT` is neither: it is what the launch shuts down before any of this
starts. See `preflight.py`.
"""

#: Topics, frames and the tick rate.
INTERFACES = {
    "map_topic": "/map",
    # tf | amcl. T1 runs under slam_toolbox, which publishes map -> odom but no
    # /amcl_pose, so the robot's position has to be read out of the transform
    # tree; `amcl` runs the same behaviours against a saved map instead.
    "pose_source": "tf",
    "pose_topic": "/amcl_pose",
    "map_frame": "map",
    "base_frame": "mecanumbot/base_link",
    "agreement_topic": "/mecanumbot/deep3r/map_agreement",
    "revisit_topic": "/mecanumbot/deep3r/revisit_regions",
    "finished_topic": "exploration/finished",
    "state_topic": "exploration/state",
    "marker_topic": "exploration/frontiers",
    "nav2_action": "/navigate_to_pose",
    # Feeds the BUDGET criterion's battery test, which was declared and never
    # supplied before. Empty disables the subscription; `min_battery_voltage`
    # is 0.0 by default, so the test is off either way until it is set.
    "battery_topic": "/mecanumbot/cr_battery_state",
    "rate": 1.0,
}

#: How the occupancy grid is read, how frontiers are found and scored, and how
#: the server's uncertain regions are interleaved with them.
EXPLORATION = {
    "free_threshold": 25,
    "occupied_threshold": 65,
    "rrt_step_size": 1.0,
    "rrt_snap_radius": 0.5,
    "rrt_global_iterations": 60,
    "rrt_local_iterations": 30,
    "rrt_local_radius": 5.0,
    "rrt_seed": 0,
    "cluster_radius": 0.6,
    "revalidate_radius": 0.4,
    "gain_radius": 1.5,
    "gain_weight": 1.0,
    "cost_weight": 0.4,
    "min_gain": 8,
    "hysteresis": 0.25,
    "hysteresis_radius": 1.0,
    "goal_timeout": 60.0,
    "goal_reached_distance": 0.6,
    "revisit_uncertain": True,
    "uncertain_every": 3,
}

#: When the pass is over. Every one of these is a research decision.
EXIT = {
    "progress_window": 45.0,
    "cloud_window": 60.0,
    "frontier_quiet_time": 20.0,
    "min_cells_per_metre": 25.0,
    "max_map_growth": 0.01,
    "loop_closure_settle": 15.0,
    "min_agreement": 0.15,
    "cloud_reset_settle": 30.0,
    "min_runtime": 60.0,
    "min_grid_coverage": 0.75,
    "max_uncertain_regions": 3,
    "uncertain_score_threshold": 0.5,
    "max_cloud_growth": 0.02,
    "cloud_verdict_timeout": 30.0,
    "require_cloud": True,
    # 0 means no limit here, and the packaged constants file overrides it with
    # 900 s -- see the file. This is the declaration, not the setting.
    "max_duration": 0.0,
    "max_distance": 0.0,
    "min_battery_voltage": 0.0,
}

#: What the preflight shuts down, and how hard it is allowed to try.
PREFLIGHT = {
    "preflight": True,
    "preflight_discovery": 2.0,
    "preflight_timeout": 15.0,
    "preflight_kill_processes": True,
}


def all_defaults():
    """Return every parameter this package declares, as one flat dict."""
    values = {}
    for group in (INTERFACES, EXPLORATION, EXIT, PREFLIGHT):
        values.update(group)
    return values
