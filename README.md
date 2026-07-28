# mecanumbot_behaviours

ROS 2 repository for high-level behaviour orchestration on Mecanumbot, built on
`py_trees` / `py_trees_ros`. The trees do not drive the wheels directly for
navigation — they publish Nav2 goals on `/goal_pose` and watch
`/navigate_to_pose/_action/status`, and use `/cmd_vel` only for in-place spins.

## Packages in this repository

| Package                        | Type                         | Purpose                                                                                                 |
| ------------------------------ | ---------------------------- | ------------------------------------------------------------------------------------------------------- |
| `mecanumbot_behaviours`        | Meta package (`ament_cmake`) | Groups the behaviour packages for build/release. No executable nodes.                                   |
| `mecanumbot_leading_behaviour` | Python package               | Behaviour trees for the leading / attention-guidance experiments. Holds the reusable behaviour library. |
| `mecanumbot_demo_behaviours`   | Python package               | Demo trees (wander between people, hide and seek) that reuse the leading package's behaviours.          |

## Node overview

### `mecanumbot_behaviours` (meta package)

| Item         | Value                       |
| ------------ | --------------------------- |
| Nodes        | None                        |
| Launch files | None                        |
| Role         | Dependency aggregation only |

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

## Which experiment condition maps to which tree

| Condition | Node                      | Signalling channel                             |
| --------- | ------------------------- | ---------------------------------------------- |
| `Control` | `control_leading_bt_node` | None — navigation only.                        |
| `Doglike` | `doglike_leading_bt_node` | Neck/gripper gestures on `/cmd_accessory_pos`. |
| `LED`     | `LED_leading_bt_node`     | LED patterns via the `set_led_status` service. |

`bottom_up_tree_node` is a simpler baseline that combines approach with LED
indication and is run directly with `ros2 run`.

## Repository structure

| Path                            | Function                                                                               |
| ------------------------------- | -------------------------------------------------------------------------------------- |
| `mecanumbot_behaviours/`        | Meta package (`CMakeLists.txt`, `package.xml`) with no runtime node logic.             |
| `mecanumbot_leading_behaviour/` | Behaviour library, tree nodes, launch and config.                                      |
| `mecanumbot_demo_behaviours/`   | Demo trees and their own movement/blackboard behaviours plus a map waypoint generator. |

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

Both Python packages need `py_trees`, `py_trees_ros`, `rclpy`, `geometry_msgs`,
`action_msgs`, `nav2_msgs` and `mecanumbot_msgs` at runtime, and
`mecanumbot_demo_behaviours` imports directly from `mecanumbot_leading_behaviour`.
Its `utils/map_generate.py` additionally needs `opencv-python` and
`mecanumbot_description` (for the map files). None of these are declared in the
`package.xml` files, which only list the lint test dependencies — the packages build
regardless, but `rosdep` will not install the runtime requirements for you.

## Build

```bash
colcon build --symlink-install --packages-select mecanumbot_behaviours mecanumbot_leading_behaviour mecanumbot_demo_behaviours
source install/setup.bash
```
