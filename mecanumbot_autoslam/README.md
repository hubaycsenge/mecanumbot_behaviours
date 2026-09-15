# mecanumbot_autoslam

**T1 of the Deep3R seeking system: drive the robot around a place it has never
seen until the place has been scanned.**

The robot builds a 2D map with slam_toolbox while the cluster builds a 3D point
cloud from the same drive, and this package decides where to go next and when
the pass is over. When it is, it latches `exploration/finished`, which is how T2
starts.

```text
    /map ─────────────────► DetectFrontiers ──┐
    map -> base_link (tf) ─►                  │
                                              ▼
    deep3r/map_agreement ─► FinishExploration ─► exploration/finished (latched)
    deep3r/revisit_regions ──────────────┐    │
                                         ▼    ▼
                                     DriveToGoal ──► nav2 NavigateToPose
                                         │
              exploration/state, exploration/frontiers (markers)
```

## Why this is a behaviour package

It used to be `mecanumbot_frontier_explorer_node` in `mecanumbot_custom_nav2`,
alongside the RRT, the frontier scoring, the occupancy model and the exit
criteria. Those are judgements about a map and they stayed there; this is a
decision to send the robot somewhere, and in this workspace that is a behaviour
whatever it is implemented with. `mecanumbot_custom_nav2` now commands no motion
at all — it is a library plus one node that turns the server's verdict into a
nav2 keepout mask.

Everything this package reasons *with* is imported from there and unchanged, so
the 173 tests behind the detector and the exit criteria still cover the pass.

## Why it is not a `py_trees` tree

Every other experiment in this repository is, because every other experiment has
branches: a person to commit to or not, an object reachable or not, a condition
that changes what signalling happens. An exploration pass has none. It looks, it
decides whether it is done, and if it is not it drives somewhere — four steps in
a fixed order, one goal in flight at a time. A tree would add a runner, a
blackboard and a tick policy without making a single decision clearer.

What is worth borrowing from `py_trees` is the shape, so the behaviours are
named objects with `setup()`, `update()` and `terminate()`, ticked in order by
`tree_nodes/autoslam_node.py`. The nav2 plumbing is not borrowed but *reused*:
`ExplorationNavigator` is `mecanumbot_movement_behaviours`'
`Nav2PoseNavigator`, so a goal from this package is sent, followed and cancelled
exactly the way a leading tree's goal is.

| Module | What it decides |
| --- | --- |
| `behaviours/detecting.py` | Where the known map stops. Grows both RRTs one budget at a time and scores what they find. Moves nothing. |
| `behaviours/choosing.py` | The next goal: the best frontier, or — every `uncertain_every` goals — a region the server is unsure about. A pure function, and the one research decision here that is testable as a function. |
| `behaviours/driving.py` | **The only thing in this package that moves the robot.** One nav2 `NavigateToPose` goal at a time. |
| `behaviours/finishing.py` | Whether the pass is over, and latching it when it is. |
| `behaviours/ros_interfaces.py` | The map, the pose, the server's two topics, the three publishers, and the navigator. Decides nothing. |
| `preflight.py` | What has to be shut down before any of this starts, and how. |

## The preflight

T1 is the one pass in this workspace that **replaces** the navigation stack
rather than using it: slam_toolbox owns `map -> odom`, and nav2 comes up from
`navigation_launch.py` with no AMCL and no map server. So anything left running
from an ordinary session does not merely waste CPU — it contradicts the pass.
AMCL publishes `map -> odom` as well; `map_server` serves the saved study map on
the topic slam_toolbox is building the new one on; the study nav2 servers
register the same node names and answer the same `navigate_to_pose`; a behaviour
tree sends its own goals, and the two sets cancel each other in turn.

Until now the answer was an instruction — start the base launch with
`use_nav2:=false`, do not have a tree running — and an instruction is something
to get wrong at the start of a trial. `launch_autoslam.launch.py` runs
`autoslam_preflight` first and holds everything else behind its **exit**, not
behind a timer, so the stack comes up into a graph that has already been
cleared.

Three ways to stop something, in order of how gentle they are:

| Method | Used for | How |
| --- | --- | --- |
| Lifecycle manager | AMCL + `map_server`, the keepout servers | `manage_nodes` with `SHUTDOWN` — one call retires everything under a manager, in dependency order. The only method here nav2 itself considers supported. |
| `change_state` | A managed node whose manager is already gone | Its current state is read first, because the shutdown transition is a different id from each of unconfigured, inactive and active. |
| `SIGTERM`, then `SIGKILL` | **The whole study nav2 navigation stack**, `slam_toolbox`, an old pass, and the behaviour trees | Matched on the executable name **or the `__node:=` remap** in a process's command line, never a free-text search; never this process or its own process group. Behind `preflight_kill_processes`. |

### Stopping something, versus removing it

The distinction the first version got wrong, and it is the whole difference
between a pass that starts and one that cannot.

`navigation_launch.py` registers `controller_server`, `planner_server`,
`bt_navigator` and five more under **exactly the names the study stack already
holds** — and a lifecycle shutdown does not free a name. A finalized node is
still on the graph and still answers `<name>/change_state`. So a surviving old
server answers our own manager's `configure`; an *active* one rejects it
outright, nav2 logs `Failed to bring up all requested nodes. Aborting bringup`,
and then tears down the servers it had just started. The pass gets no navigation
at all and sits there logging `nav2 action server not ready`.

So the rule is: what merely **contradicts** is stopped, and what **collides** is
removed. AMCL and `map_server` contradict — autoslam registers nothing under
those names, and a finalized AMCL publishes no transform, which is all that is
needed. The navigation stack collides, so it is signalled. That is the same
signal `ros2 launch` sends it on Ctrl-C, and it leaves the rest of the base
launch — drivers, joystick, web GUI — running.

The navigation *manager* is matched by its `__node:=lifecycle_manager_navigation`
remap and never by its executable, which is a bare `lifecycle_manager` shared
with the localization and keepout managers that must be shut down cleanly
instead.

**And "removed" is usually one process, not eight.** `bringup_launch.py`
defaults `use_composition` to True, so the study stack is not eight processes
named `controller_server`, `planner_server` and so on — it is eight *nodes*
composed into a single `component_container_isolated` called `nav2_container`.
Looking for a process per node finds nothing there, and "nothing" read as "not
running on this machine, must be someone else's" — so the preflight declared the
graph clear and the pass started straight into the collision it exists to
prevent. The container is what gets signalled, matched by its
`__node:=nav2_container` remap alone so that no other container in the workspace
can be caught by it. Autoslam's own nav2 comes from `navigation_launch.py`,
whose `use_composition` defaults to False, so it is eight ordinary processes and
there is no container of ours to confuse with it.

### It verifies, and it refuses to start over a collision

Two outcomes worth telling apart. A tree on the operator PC that cannot be
signalled from here **degrades** a pass — a warning. A study `controller_server`
still holding a name we need **prevents** one, so the preflight exits non-zero
and `launch_autoslam.launch.py` stops rather than spending a trial discovering
it. `preflight_strict:=false` starts anyway.

Verification asks the **node graph**, because that is what a name collision is
about: is the name still taken. A composed node has no process to look for and a
finalized lifecycle node still holds its name, so the process table answers a
different question — and answering that one instead is exactly how eight live
nav2 nodes were reported as cleared. The graph also catches a node that came
back, which a launch file with `respawn:=true` will do a couple of seconds after
a SIGTERM.

**What is deliberately not stopped.** The **joystick** — it publishes
`/cmd_vel`, so by the letter of the rule it contradicts an autonomous pass, and
it is also the only way a person in the room can take the robot off a wall. A
run that has disarmed the human override is a worse failure than one that gets a
joystick nudge in the log. And the **drivers, camera, perception stack and
Deep3R client**: none of them decide where the robot goes, and T1 needs all of
them.

**It reports what it could not do and starts anyway.** A node on the operator PC
cannot be signalled from the robot. Saying so in the log is the difference
between a pass that fails mysteriously and one that fails with its cause on the
screen; refusing to explore over it would not be.

## Running

```bash
# one terminal: drivers, then camera + Deep3R client + the pass
ros2 launch mecanumbot_autoslam launch_t1.launch.py

# or two terminals: the drivers, then everything else
ros2 launch mecanumbot_bringup launch_mecanumbot_base.launch.py use_nav2:=false
ros2 launch mecanumbot_autoslam launch_autoslam.launch.py   # needs the tunnel up

# mapping only, with no server at all
ros2 launch mecanumbot_autoslam launch_autoslam.launch.py require_cloud:=false use_deep3r:=false

# watch
ros2 topic echo /mecanumbot/exploration/state       # every criterion, with its reason
ros2 topic echo /mecanumbot/exploration/finished    # latches true when T1 is over
ros2 topic echo /mecanumbot/deep3r/map_agreement    # the server's verdict
```

**`launch_autoslam.launch.py` starts the camera and the Deep3R client itself**,
straight away and in parallel with the preflight, which leaves both alone. The
web GUI's **Autoslam (T1)** runs the same file, so a pass started from the
browser brings them up too. They are part of the pass because T1 cannot finish
without them: no frames means no cloud, no `map_agreement`, and a `CLOUD`
criterion that is never met, with no error anywhere. The camera is the robot's
USB webcam, published on `/camera/image_raw/compressed`, which is where the
client reads it. If either is already running, pass `use_camera:=false` /
`use_deep3r:=false`: the camera can be opened once, and a second client is a
second run, for which the server wipes its reconstruction. The drivers are
still not part of this file, so that it never becomes a second owner of the
OpenCR link.

**slam_toolbox reads `/mecanumbot/scan_grid`, not the driver's scan.** The launch file also starts
`mecanumbot_core`'s `mecanumbot_scan_grid_node`, which republishes every LD08
revolution on 200 fixed sectors. Without it slam_toolbox drops every scan whose
reading count differs from the first one it saw, and the LD08's count changes
every revolution. On an unlucky launch the map then stays 0 × 0, and nav2 reports
`Robot is out of bounds of the costmap!` against a 5 × 5 m default square. See
that package's README.

The cluster server is **not** started by any of this and cannot be: it is a
Slurm job behind an SSH tunnel. The canonical two-machine sequence is
`mecanumbot_custom_nav2/README.md`, under "Starting T1".

Saving the map is deliberately manual — the map T2 localizes against is worth
looking at before it is written:

```bash
ros2 run nav2_map_server map_saver_cli -f <maps>/AI_dept/AI_dept
```

### Launch arguments

| Argument | Default | Function |
| --- | --- | --- |
| `params` | `config/autoslam_setting_constants.yaml` | The constants, as a ROS parameter file. |
| `preflight_strict` | `true` | Stop if the preflight could not clear a **name collision**. Those make nav2 bringup abort, so starting anyway wastes the run rather than degrading it. |
| `require_cloud` | `true` | Whether the pass may only end once the server says the reconstruction is good enough. `false` is right for a dry run and wrong during a trial. |
| `use_preflight` | `true` | Shut the contradicting nodes down first. |
| `use_camera` | `true` | Start the compressed camera publisher (USB backend) on `/camera/image_raw/compressed`. `false` when something already publishes it. |
| `camera_width` / `camera_height` / `camera_fps` | `1280` / `720` / `15.0` | The camera's frame; matches `deep3r.yaml`'s advertised size. |
| `use_deep3r` | `true` | Start the Deep3R client (`mecanumbot_deep3r/deep3r.launch.py`). `false` when one is already running, or for a mapping-only run with `require_cloud:=false`. |
| `server` / `client_path` | `tcp://127.0.0.1:5555` / `~/robocam_client.py` | Passed to the client: the local end of the tunnel, and the deployed `robocam_client.py`. |
| `run_id` | *(empty)* | Passed to the client. Empty starts a fresh reconstruction on the server; a previous run's id (the client logs it at startup) resumes that run across a restart. |
| `use_agreement` | `true` | Start the 2D/3D comparison handler. `false` for a session that already has one from the T2 launch. |
| `agreement_params` | `mecanumbot_custom_nav2/config/map_agreement.yaml` | The comparison handler's constants. |
| `slam_params` / `nav2_params` | `mecanumbot_description/param/mecanumbot_slam_mapping.yaml` / `mecanumbot_exploration_nav2.yaml` | slam_toolbox and the exploration nav2 file, which has no AMCL block. It does have a static layer: that is where the global costmap gets its size from slam_toolbox's map. |
| `namespace` | `mecanumbot` | Namespace for the pass's node. |
| `use_sim_time` | `false` | Set by `sim.launch.py`. |

`launch_t1.launch.py` starts the base launch and, after `explorer_delay`
(15 s), this file. It passes `require_cloud`, `use_camera`, the camera size,
`use_deep3r`, `server`, `client_path` and `run_id` through, and hard-codes
`use_nav2:=false` for the base launch, because there is no T1 in which the study
nav2 stack is what you want.

## Node: `autoslam_node`

### Subscribers

| Topic | Type | Function |
| --- | --- | --- |
| `/map` | `nav_msgs/OccupancyGrid` | The map being built. A *shrinking* known area is read as a loop closure — slam_toolbox re-rasterises the whole grid — so both RRTs are thrown away and regrown and the settle timer restarts. |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | Robot pose, only with `pose_source: amcl`. |
| `/mecanumbot/deep3r/map_agreement` | `mecanumbot_msgs/MapCloudAgreement` | The server's verdict. Feeds the `CLOUD` exit criterion. |
| `/mecanumbot/deep3r/revisit_regions` | `geometry_msgs/PoseArray` | Regions worth another look, already filtered and ordered by `mecanumbot_map_agreement`. |
| `/mecanumbot/cr_battery_state` | `sensor_msgs/BatteryState` | The `BUDGET` criterion's battery test. See below. |

With `pose_source: tf` (the default) the pose comes from the `map ->
mecanumbot/base_link` transform. **T1 runs under slam_toolbox, which publishes no
`/amcl_pose`**, so `tf` is the setting this package exists for; `amcl` is for
running the same behaviours against a saved map.

### Publishers

| Topic | Type | Function |
| --- | --- | --- |
| `exploration/finished` | `std_msgs/Bool` | Latched. `false` at start-up, `true` once T1 is over. This is the handover: `mecanumbot_deep3r` watches the latch and tells the server to change phase. |
| `exploration/state` | `std_msgs/String` | One line per tick: frontier count, distance driven, goal source, and every exit criterion with its reason. The first thing to look at when the robot is not moving. |
| `exploration/frontiers` | `visualization_msgs/MarkerArray` | Every scored frontier (green) and the chosen goal (red). Seeing the frontiers the robot *rejected* is most of the debugging. |

### Actions

`/navigate_to_pose` — one goal in flight at a time, cancelled and re-decided on
timeout. A goal nav2 gives up on is **retired rather than re-proposed**: a
frontier the planner cannot reach is behind something, and the detector will
happily suggest it again for ever.

`/cmd_vel` is never published. The explorer has no reason to turn in place, and
turning in place is the one thing in this workspace that bypasses nav2 on
purpose.

**Nav2 is not optional, and it has to be *activated*.** Every metre this pass
drives is a nav2 goal; with no nav2 there is no exploration at all. The action
server exists from the moment `bt_navigator` is constructed, but it rejects
every goal until the lifecycle manager activates it — which never happens if the
bringup aborted. A *rejection* is therefore treated as "nav2 is not ready" and
not as "that frontier was no good": the pass waits `nav2_retry_delay`, does not
count the goal, and says in the log that the stack is up but not activated.
Without that it sent a goal per tick and had every one rejected, which reads
like a robot that will not move for a hundred lines before it reads like a nav2
that never came up.

## Keeping the map stable

The failure this section exists for looks like this: the pass runs, the map
grows, and then the whole thing rotates under the robot — walls duplicated at
several angles, dashed free-space rays fanning out through them, and a robot
confidently somewhere it is not. It is not one bug. T1 is the only thing in this
workspace that *builds* the map it drives on, so it is the only thing that pays
for a lidar and an odometry that the study behaviours can get away with, and
four separate things were making it pay.

**The lidar is a rotating 10 Hz sensor and nobody deskews it.** One LDS-02 scan
takes 100 ms to sweep. `ld08_driver` stamps it and hands it over as if it were
instantaneous, so a scan taken while turning is smeared by a full 100 ms of
rotation — 6° at the old 1.0 rad/s. Scan matching against a smeared scan is
scan matching against geometry that was never there.

**Mecanum odometry lies hardest exactly then.** The rollers slip most under
yaw, and `mecanumbot_sensorproc_node` integrates wheel ticks: it reports the
rotation the wheels turned through, not the one the robot did.

**And slam_toolbox is not watching while the robot turns.** This is the one
worth knowing, because it is structural rather than a setting.
`shouldProcessScan` in `slam_toolbox_common.cpp` gates an incoming scan on
**translation only** — `dist2 < 0.8 * minimum_travel_distance^2` and the scan
is dropped. There is no heading term in it. Karto's own `HasMovedEnough` does
check heading, but it never receives the scan. So during an in-place turn the
pose graph gets *nothing*: no nodes, no matches, no correction. The only
account of the turn is the wheel odometry — the estimate that is least
trustworthy in exactly that motion — and the first scan after the turn is
matched against a prior carrying the whole spin's accumulated error.
`coarse_search_angle_offset` (20°) is how much of that the matcher can absorb
before it locks onto a wrong alignment, and locking onto a wrong alignment is
the map jumping.

No slam_toolbox parameter fixes that, which is why the two changes below are
about turning *less* and turning *slower* rather than about the mapper.
`minimum_travel_distance` stays at 0.1 for the same reason: it is the distance
the robot has to translate before the map may look again, so it is the recovery
latency after every turn.

**So the pass now turns about half as fast** —
`mecanumbot_description/param/mecanumbot_exploration_nav2.yaml`, difference 6 in
its header. The controller, the spin recovery and the velocity smoother are
capped at 0.6 rad/s (from the study's 1.0, 1.0 and 0.7), the rotation shim at
0.4 rad/s (from 0.7), and yaw acceleration is halved in the controller and the
spin recovery (the smoother's goes from 1.0 to 0.8).
Translation limits are untouched, so a leg is driven the way a trial's leg is
driven; only the turns are slower.

**And it turns much less often.** `ExplorationNavigator.go_to_point` used to
send an identity quaternion, described in its own docstring as "facing whichever
way the robot already faces". It is not: the goal is stamped in the **map**
frame, so an identity quaternion is map yaw 0, and nav2 finished every leg by
turning in place to face map-east. nav2 has no way to be told "any orientation
will do", so the fix is to send a heading worth arriving at — the bearing from
the robot to the frontier. The rotation shim has already put the robot roughly
on that bearing at the start of the leg, so the terminal turn is small, and it
leaves the robot looking into the unknown region the frontier borders, which is
where the next cycle's scans have to come from.

**Long readings were painting free space through the walls.** In
`mecanumbot_slam_mapping.yaml`, `max_laser_range` was 10.0 — but Karto raytraces
any beam longer than that as *free* out to the threshold without marking an
endpoint, and `ld08_driver` advertises 12 m, which is the protocol's range and
not the sensor's. Every no-return beam drew a 10 m free ray. That is the
starburst, and it is not only cosmetic: the same clipped readings build the
correlation grid the matcher runs on. Now capped at the 8.0 m
`mecanumbot_lds.lua` has always used for this lidar in this arena, with a 0.16 m
floor below which the returns are the robot's own body.

**And a false loop closure rewrites the map rather than degrading it.** The four
loop-closure gates were at the slam_toolbox defaults, and one of them is wrong
for a corridor: `loop_match_minimum_chain_size: 10` at 0.1 m node spacing made a
candidate chain about a metre of travel, and one metre of corridor wall matches
any other metre of corridor wall. The gates are now deliberately asymmetric — a
missed closure only lets drift accumulate, and over a 900 s pass in a 10 × 9 m
arena that is bounded, while a false one re-optimises the entire pose graph and
re-rasterises the entire grid. Chain size is 25 nodes at 0.1 m spacing,
about 2.5 m of geometry.

Two knobs deliberately **not** turned:

- **`odom_params.from_imu`.** The OpenCR publishes a fused quaternion, and yaw
  from it drifts rather than slipping, which is the slower and more forgiving
  of the two errors. It is still `false`, because it has never run: the
  `odom -> base_footprint` broadcast used to sit inside the wheel branch, so
  setting it true published an `/odom` topic and no transform at all — a TF tree
  with a hole in it, and therefore no SLAM, no costmaps and no nav2. That is
  fixed, so the switch is now testable; it wants a bench pass (spin the robot
  360° by joystick, compare `/odom` yaw against the wall) before a trial.
- **`min_pass_through` and `occupancy_threshold`.** They are what leaves the
  black speckle scattered through open floor, and raising them would clean it
  up. They also decide which cells the map calls occupied, and *that* is the
  denominator of `MapCloudAgreement.agreement` — which `min_agreement: 0.15`
  is a threshold on and which T1 will not finish without. Changing them moves
  an exit criterion, so it is a research decision, not a tuning one.

What to watch during a pass:

```bash
ros2 topic echo /mecanumbot/exploration/state
ros2 run tf2_ros tf2_echo map mecanumbot/odom   # map -> odom should creep, not step
```

A `map -> odom` transform that *jumps* is the pose graph being re-optimised. One
after a genuine loop closure is the system working; a run of them, or one that
moves the whole map by more than the robot could have drifted, is the failure
above coming back — and the next thing to try is `from_imu`.

## The constants

`config/autoslam_setting_constants.yaml`, which documents every constant where
it is set. `defaults.py` declares the same names with the value each takes when
unset, so the node has one place to declare its parameters from.

**The root key is `/**` and not the node name, and that is not cosmetic.** A ROS
2 parameter file keyed by a bare node name only reaches a node in the **root**
namespace. This node runs in `mecanumbot`, so `autoslam_node:` would load
nothing and every constant would silently be the packaged default — which is
exactly what happened to this file in its previous life as
`mecanumbot_custom_nav2/config/frontier_explorer.yaml`, keyed
`mecanumbot_frontier_explorer:` against a node launched into the `mecanumbot`
namespace. It never read a line of it.

Fixing that key changes exactly one value in practice, because the file and the
node's declared defaults otherwise agree: **`max_duration`, from no limit to
900 s.** That is the file's stated intent — the BUDGET criterion is what stops
an unattended run — but it is a real change in when a pass ends, so it is worth
knowing before the first trial.

There is no `Eto_` variant, unlike the other behaviour packages. Everything here
describes the lidar, the robot and the reconstruction rather than the building;
the only room-dependent constant is `max_duration`, and a smaller room wants a
smaller one.

### The battery criterion now has a reading

`min_battery_voltage` was a declared exit criterion that nothing ever supplied,
so the one budget meant to protect a whole session could not fire — and a flat
battery mid-reconstruction loses the cloud, which lives on the server against a
map id that will not survive the robot coming back. `FinishExploration`
subscribes to `battery_topic` and passes the voltage through. The default is
still `0.0`, which means no limit, so nothing changes until it is set.

## Tests

```bash
cd src/mecanumbot_behaviours/mecanumbot_autoslam
PYTHONPATH=.:$PYTHONPATH /usr/bin/python3 -m pytest test/ -q \
  -p no:launch_testing -p no:launch_testing_ros \
  --ignore=test/test_flake8.py --ignore=test/test_copyright.py --ignore=test/test_pep257.py
```

39 tests (26 preflight, 10 choosing, 3 goal heading), no ROS graph. Use
`/usr/bin/python3`, not the conda one. `test_preflight.py` and `test_choosing.py`
are pure Python; `test_goal_heading.py` imports `behaviours/driving.py`, so it
needs a sourced workspace (`rclpy`, `mecanumbot_movement_behaviours`) and skips
without one.

| File | Covers |
| --- | --- |
| `test_preflight.py` | What contradicts an exploration pass and what does not — including that the joystick is never stopped, that a namespaced node contradicts exactly as much as a bare one, that a manager's shutdown does not cover another manager's nodes, and that a launch file merely *naming* a tree is not matched as one. Plus the collision rules: that the whole nav2 navigation stack is removed rather than shut down, that a tree is not a collision, that a surviving collision blocks the pass and a surviving tree does not, and that the navigation manager is matched by its remap so the localization manager is spared |
| `test_choosing.py` | The interleaving: a frontier is the ordinary goal, every Nth is the server's, the server is still visited when the frontiers run out, switching revisiting off leaves the pass on frontiers alone, and a rejected goal does not advance the ratio |
| `test_goal_heading.py` | Which way a frontier goal asks the robot to end up facing: the bearing to the frontier, and — the bug it guards — never map yaw 0 by accident, neither with no pose to measure from nor when the robot is standing on the frontier already |

The detector, the scoring, the occupancy model and the exit criteria are tested
in `mecanumbot_custom_nav2` — 173 tests — because that is where they live.

`test_flake8` fails here as it does everywhere in this workspace: the installed
flake8 cannot load its own `pycodestyle` plugin. `ament_pep257` passes.
