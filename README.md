# mecanumbot_behaviours

ROS 2 repository for high-level behaviour orchestration on Mecanumbot, built on
`py_trees` / `py_trees_ros`. The trees do not drive the wheels directly for
navigation — the leading trees send Nav2 goals through its actions and the
ostensive tree publishes them on `/goal_pose`; `/cmd_vel` is used only for
in-place spins.

The repository is split into **library** packages and **experiment** packages. A
library package holds behaviour that any experiment could want; an experiment
package holds the trees, the constants and the one condition it is about.

## Packages in this repository

| Package | Kind | Purpose |
| --- | --- | --- |
| `mecanumbot_behaviours` | Meta package (`ament_cmake`) | Groups the behaviour packages for build/release. No executable nodes. |
| `mecanumbot_bt_config` | Library | Constants YAML → blackboard, and the tree runner. Knows no key names: it takes them from the file. |
| `mecanumbot_movement_behaviours` | Library | Turning, approaching, driving a route and searching it, plus the Nav2 navigators, trackers and geometry every tree needs. |
| `mecanumbot_leading_behaviour` | Experiment | Behaviour trees for the leading / attention-guidance conditions, their LED and gesture signalling, and their constants. |
| `mecanumbot_demo_behaviours` | Experiment | Demo trees (wander between people, hide and seek). |
| `mecanumbot_ostensive_behaviour` | Experiment | The ostensive condition: a person bids for attention by gesture, the robot commits to them and follows their pointing cue. |

Dependencies run one way: experiments depend on libraries, and
`mecanumbot_movement_behaviours` on `mecanumbot_bt_config`. One experiment
depends on another, deliberately: the demo trees run against the leading
experiment's constants files, so they reuse its loader and its LED sequences.

## Node overview

### `mecanumbot_behaviours` (meta package)

| Item         | Value                       |
| ------------ | --------------------------- |
| Nodes        | None                        |
| Launch files | None                        |
| Role         | Dependency aggregation only |

### `mecanumbot_bt_config` and `mecanumbot_movement_behaviours` (libraries)

No nodes and no launch files. See their READMEs — `mecanumbot_bt_config` for the
configuration model, `mecanumbot_movement_behaviours` for every movement
behaviour and the ROS interfaces they create.

### `mecanumbot_leading_behaviour`

Provided executables:

- `control_leading_bt_node` — `tree_nodes/ctrl_tree.py`
- `doglike_leading_bt_node` — `tree_nodes/dog_tree.py`
- `LED_leading_bt_node` — `tree_nodes/LED_tree.py`
- `bottom_up_tree_node` — `tree_nodes/bottom_up_tree.py`

See the package README for full node interface tables and per-tree logic.

### `mecanumbot_demo_behaviours`

Provided executables:

- `wander_between_people_node` — `tree_nodes/wander_between_people.py`

`tree_nodes/hide_and_seek.py` is present and complete but has no entry point in
`setup.py`, so it is not installed as an executable.

### `mecanumbot_ostensive_behaviour`

Provided executables:

- `ostensive_bt_node` — `tree_nodes/ostensive_tree.py`

The only tree in this repository that registers under its own node name rather
than `bottom_up_tree_node`, so it can run alongside a leading tree.

It runs no detector: gestures are decoded from the COCO-17 keypoints on
`/mecanumbot/cam_people_detections` and metric positions come from
`/mecanumbot/people_fusion`, both from `mecanumbot_sensorprocess_smart`. The
perception logic lives in three ROS-free modules (`keypoints.py`, `gestures.py`,
`cue_geometry.py`) with 50 unit tests that run without a ROS graph — the fastest
feedback loop in this repository. See the package README for the sign conventions
and the gesture thresholds.

## Which experiment condition maps to which tree

| Condition | Node                      | Signalling channel                             |
| --------- | ------------------------- | ---------------------------------------------- |
| `Control` | `control_leading_bt_node` | None — navigation only.                        |
| `Doglike` | `doglike_leading_bt_node` | Neck/gripper gestures on `/cmd_accessory_pos`. |
| `LED`     | `LED_leading_bt_node`     | LED patterns via the `set_led_status` service. |

`bottom_up_tree_node` is a simpler baseline that combines approach with LED
indication and is run directly with `ros2 run`.

The ostensive condition is not part of that launcher's `condition` argument — it
is a different experiment, with its own launch file and its own constants
schema, and the robot is led by the human rather than leading them.

## Where the numbers live

Every number a tree acts on comes from a constants YAML and reaches the
behaviours through the py_trees blackboard. There is nothing to edit in Python
to retune a run.

`mecanumbot_bt_config` reads the file and writes every key it declares onto the
blackboard, taking the names from the file rather than from a schema — so adding
a constant is one line in one place. What a file *cannot* say about itself is
declared by the tree that loads it, and there are two kinds of key:

* **experiment parameters** — the distances, the route, the LED and gesture
  scripts. Required: a file that omits one fails to load rather than having a
  value invented for it;
* **tunables** — turn speeds and profiles, timeouts, scan lengths, retry counts,
  pacing delays, head poses. Optional: omitting one keeps the packaged default,
  which is what lets a constants file written before a key existed still load.

The defaults live with the behaviours that read them —
`mecanumbot_movement_behaviours/defaults.py` for the movement ones, each
experiment package's `behaviours/defaults.py` for its own — because a default is
a statement about a behaviour, not about a file format.

Angles are declared in degrees with a `_deg` suffix and reach the blackboard in
radians under the name without it; the loader converts any key spelled that way,
so no list of which keys are angles exists anywhere. The same goes for the
structured strings (`{X,Y,Z}`, `{fl,fr,bl,br}`, `{n_pos,gl_pos,gr_pos}`), which
are decoded into messages by the keys inside them.

`mecanumbot_ostensive_behaviour` and `mecanumbot_demo_behaviours` build on the
same mechanism — the ostensive files set the subset of the movement tunables
their tree exercises, and the demo trees add a handful of `demo_*` keys of their
own. Each package's README lists its keys; the mechanism itself is documented in
`mecanumbot_bt_config/README.md`.

## Repository structure

| Path                            | Function                                                                               |
| ------------------------------- | -------------------------------------------------------------------------------------- |
| `mecanumbot_behaviours/`          | Meta package (`CMakeLists.txt`, `package.xml`) with no runtime node logic.             |
| `mecanumbot_bt_config/`           | The constants loader and the tree runner. No nodes, no config of its own.              |
| `mecanumbot_movement_behaviours/` | The movement behaviour library and the pacing tests. No nodes.                         |
| `mecanumbot_leading_behaviour/`   | Leading tree nodes, their signalling behaviours, launch, config and the route checker. |
| `mecanumbot_demo_behaviours/`     | Demo trees and their own movement/blackboard behaviours plus a map waypoint generator. |
| `mecanumbot_ostensive_behaviour/` | Ostensive tree, its gesture-decoding library and the unit tests for it.                |

Each Python package additionally carries `resource/` (ROS 2 resource index marker)
and `test/` (`flake8`, `pep257`, copyright lint scaffolding).

## Launch and runtime logic

Primary launcher:

- `mecanumbot_leading_behaviour/launch/launch_wifi_condition_sequence.launch.py`

What it does:

1. Detects the current Wi-Fi SSID (`nmcli`, falling back to `iwgetid`).
2. Selects the default parameter YAML from the SSID — `MecanumNet` →
   `behaviour_setting_constants.yaml`, `MecanumetoNet` →
   `Eto_behaviour_setting_constants.yaml`, anything else → the former.
3. Declares the `params`, `yaml_path`, `namespace` (default `mecanumbot`) and
   `condition` (default `Doglike`) launch arguments.
4. Exports `YAML_PATH` and `BEHAVIOUR_YAML_PATH` so the tree scripts can find the YAML.
5. Starts exactly one BT node based on `condition` (`Doglike`, `Control`, or `LED`),
   remapping `/mecanumbot/cmd_vel` and `/mecanumbot/cmd_accessory_pos` out of the
   namespace.

`mecanumbot_demo_behaviours` ships a copy of this launch file, but it still points at
`doglike_demo_bt_node` / `control_demo_bt_node` / `LED_demo_bt_node`, none of which
are registered in that package's `setup.py`. Start the demo tree with `ros2 run`
instead.

## Dependencies

All five Python packages need `py_trees`, `py_trees_ros`, `rclpy`,
`geometry_msgs`, `action_msgs`, `nav2_msgs` and `mecanumbot_msgs` at runtime,
except `mecanumbot_bt_config`, which needs only `python3-yaml`, `py_trees` and
`ament_index_python` (its message decoders are optional and skip themselves
without ROS). `utils/map_generate.py` in the demo package additionally needs
`opencv-python` and `mecanumbot_description` (for the map files).

`py_trees` and `py_trees_ros` are pip/apt installs that `rosdep` will not fetch
for the packages that do not name them; see the workspace `CLAUDE.md`.

## Build

```bash
colcon build --symlink-install --packages-select \
  mecanumbot_bt_config mecanumbot_movement_behaviours \
  mecanumbot_leading_behaviour mecanumbot_demo_behaviours \
  mecanumbot_ostensive_behaviour mecanumbot_behaviours
source install/setup.bash
```
