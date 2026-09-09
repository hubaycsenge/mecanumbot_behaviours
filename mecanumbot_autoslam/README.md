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
# one terminal: drivers, the Deep3R client, then the pass, in that order
ros2 launch mecanumbot_autoslam launch_t1.launch.py

# or by hand, three terminals
ros2 launch mecanumbot_bringup launch_mecanumbot_base.launch.py use_nav2:=false
ros2 launch mecanumbot_deep3r deep3r.launch.py          # needs the tunnel up
ros2 launch mecanumbot_autoslam launch_autoslam.launch.py

# mapping only, with no server at all
ros2 launch mecanumbot_autoslam launch_autoslam.launch.py require_cloud:=false

# watch
ros2 topic echo /mecanumbot/exploration/state       # every criterion, with its reason
ros2 topic echo /mecanumbot/exploration/finished    # latches true when T1 is over
ros2 topic echo /mecanumbot/deep3r/map_agreement    # the server's verdict
```

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
| `use_agreement` | `true` | Start the 2D/3D comparison handler. `false` for a session that already has one from the T2 launch. |
| `slam_params` / `nav2_params` | `mecanumbot_description/param/` | slam_toolbox and the exploration nav2 file, which has no AMCL block and no static layer. |
| `namespace` | `mecanumbot` | Namespace for the pass's node. |
| `use_sim_time` | `false` | Set by `sim.launch.py`. |

`launch_t1.launch.py` adds `use_deep3r`, `deep3r_delay`, `explorer_delay`,
`server` and `client_path`, and hard-codes `use_nav2:=false` for the base
launch, because there is no T1 in which the study nav2 stack is what you want.

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
PYTHONPATH=. python3 -m pytest test/ -q -p no:launch_testing \
  --ignore=test/test_flake8.py --ignore=test/test_copyright.py --ignore=test/test_pep257.py
```

36 tests, pure Python, no ROS graph. Use `/usr/bin/python3`, not the conda one.

| File | Covers |
| --- | --- |
| `test_preflight.py` | What contradicts an exploration pass and what does not — including that the joystick is never stopped, that a namespaced node contradicts exactly as much as a bare one, that a manager's shutdown does not cover another manager's nodes, and that a launch file merely *naming* a tree is not matched as one. Plus the collision rules: that the whole nav2 navigation stack is removed rather than shut down, that a tree is not a collision, that a surviving collision blocks the pass and a surviving tree does not, and that the navigation manager is matched by its remap so the localization manager is spared |
| `test_choosing.py` | The interleaving: a frontier is the ordinary goal, every Nth is the server's, the server is still visited when the frontiers run out, switching revisiting off leaves the pass on frontiers alone, and a rejected goal does not advance the ratio |

The detector, the scoring, the occupancy model and the exit criteria are tested
in `mecanumbot_custom_nav2` — 173 tests — because that is where they live.

`test_flake8` fails here as it does everywhere in this workspace: the installed
flake8 cannot load its own `pycodestyle` plugin. `ament_pep257` passes.
