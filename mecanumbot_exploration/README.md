# mecanumbot_exploration

**m-explore-ros2 based exploration pass: map an unknown room using explore_lite
with slam_toolbox, guided by an uncertainty monitor that keeps the pose
estimate consistent.**

Unlike `mecanumbot_autoslam`, which implements its own RRT frontier detector
and integrates with the Deep3R reconstruction server, this pass delegates
frontier detection and nav2 goal dispatch entirely to explore_lite
(m-explore-ros2).  The orchestrator's job is narrower:

```text
    /pose ──────────────────► UncertaintyMonitor ──► /explore/resume (pause/resume)
    (PoseWithCovarianceStamped)                   ──► /navigate_to_pose (loop closure)

    /pose ──────────────────► FinishDetector ──► exploration/finished (latched)

    exploration/state ◄── one-line summary every tick
```

## Why it is not a `py_trees` tree

Same reason as autoslam: the pass has no branch to select.  It checks
covariance, acts if high, checks whether it is done, acts if so.  Four steps
in a fixed order with no condition that changes what happens next.

The behaviours are plain objects with `setup()`, `update()` and `terminate()`,
ticked in order by `tree_nodes/exploration_node.py`.

## Why this instead of `mecanumbot_autoslam`

| | `mecanumbot_exploration` | `mecanumbot_autoslam` |
|---|---|---|
| Frontier detection | explore_lite (wavefront) | Custom RRT |
| Exit criterion | Robot stops moving | 2D + 3D agreement |
| Deep3R integration | No | Yes |
| Preflight | Stops AMCL only | Stops AMCL + full nav2 restart |
| Use case | 2D mapping, dry runs | Full T1 for a seeking trial |

Choose this pass when the goal is a 2D occupancy map without a 3D
reconstruction, or when testing the navigation stack without the Deep3R
server.

## The preflight

slam_toolbox and AMCL both publish the `map -> odom` transform.  With both
running the transforms conflict and the robot's position jumps.

Unlike the autoslam preflight, this pass does **not** restart the nav2
navigation stack.  It only stops the *localization* lifecycle manager
(which manages AMCL and map_server), then exits.  Nav2's controller,
planner and bt_navigator stay up and continue to accept explore_lite's goals;
slam_toolbox provides the live map and the transform.

If the preflight cannot confirm AMCL is gone (`preflight_strict:=true`, the
default), the launch stops rather than starting slam_toolbox into a conflict.

## Running

```bash
# Three terminals, in this order:
ros2 launch mecanumbot_bringup launch_mecanumbot_base.launch.py
ros2 launch mecanumbot_exploration launch_exploration.launch.py

# or, skipping the preflight if you've already stopped AMCL by hand:
ros2 launch mecanumbot_exploration launch_exploration.launch.py use_preflight:=false

# Watch
ros2 topic echo /mecanumbot/exploration/state
ros2 topic echo /mecanumbot/exploration/finished

# Save the map when finished
ros2 run nav2_map_server map_saver_cli -f <maps>/AI_dept/AI_dept
```

**explore_lite must be built from source** -- there is no Humble apt binary:

```bash
git clone https://github.com/robo-friends/m-explore-ros2.git \
  src/m-explore-ros2
colcon build --packages-select explore_lite
source install/setup.bash
```

### Launch arguments

| Argument | Default | Function |
|---|---|---|
| `params` | `config/exploration_constants.yaml` | The orchestrator constants |
| `slam_params` | `mecanumbot_description/param/mecanumbot_slam_mapping.yaml` | slam_toolbox parameters |
| `namespace` | `mecanumbot` | Namespace for `exploration_node` |
| `use_preflight` | `true` | Stop AMCL before starting |
| `preflight_strict` | `true` | Stop if AMCL could not be confirmed gone |
| `use_sim_time` | `false` | Set by `sim.launch.py` |

## Node: `exploration_node`

### Subscribers

| Topic | Type | Function |
|---|---|---|
| `/pose` | `geometry_msgs/PoseWithCovarianceStamped` | slam_toolbox pose + covariance; drives both the uncertainty monitor and the finish detector |

### Publishers

| Topic | Type | Function |
|---|---|---|
| `exploration/finished` | `std_msgs/Bool` | Latched. `false` at start-up, `true` once done |
| `exploration/state` | `std_msgs/String` | One line per tick: distance, covariance, quiet time, revisiting flag, elapsed |

### Service clients

| Service | Type | Function |
|---|---|---|
| `/explore/resume` | `std_srvs/SetBool` | Pause (`false`) / resume (`true`) explore_lite during a loop-closure revisit |

### Action clients

| Action | Type | Function |
|---|---|---|
| `/navigate_to_pose` | `nav2_msgs/NavigateToPose` | Drive to the revisit point |

## The constants

`config/exploration_constants.yaml` documents every constant where it is set.
`defaults.py` declares the same names with their default values.

The root key is `/**` and not the node name, for the same reason as autoslam:
a bare node name only reaches a node in the root namespace, and this node runs
in `mecanumbot`.

## Node: `exploration_preflight`

Stops `lifecycle_manager_localization` via its `manage_nodes` SHUTDOWN call,
verifies AMCL is gone from the graph, and exits.  Exit 0 means the pass may
start; exit 1 means AMCL is still running and the launch stops.
