# mecanumbot_behaviours

ROS 2 repository for high-level behaviour orchestration on Mecanumbot.

## Packages in this repository

| Package | Type | Purpose |
| --- | --- | --- |
| `mecanumbot_behaviours` | Meta package | Groups behaviour packages for build/release. No executable nodes. |
| `mecanumbot_leading_behaviour` | Python package | `py_trees_ros` behaviour trees for leading/attention-guidance experiments. |
| `mecanumbot_demo_behaviours` | Python package | Demo clone of the leading-behaviour tree package with the same ROS 2 structure. |

## Node overview

### `mecanumbot_behaviours` (meta package)

| Item | Value |
| --- | --- |
| Nodes | None |
| Launch files | None |
| Role | Dependency aggregation only |

### `mecanumbot_leading_behaviour`

Provided executables:

- `control_leading_bt_node`
- `doglike_leading_bt_node`
- `LED_leading_bt_node`
- `bottom_up_tree_node`

See the package README for full node interface tables and BT logic details.

### `mecanumbot_demo_behaviours`

Provided executables:

- `control_demo_bt_node`
- `doglike_demo_bt_node`
- `LED_demo_bt_node`
- `bottom_up_demo_tree_node`

See the package README for the mirrored tree structure and launch file.

## Repository structure

| Path | Function |
| --- | --- |
| `mecanumbot_behaviours/` | Meta package (`CMakeLists.txt`, `package.xml`) with no runtime node logic. |
| `mecanumbot_leading_behaviour/` | Full behaviour-tree implementation: nodes, behaviours, launch, and config. |
| `mecanumbot_demo_behaviours/` | Mirrored behaviour-tree package with the same folder layout and launch/config scaffolding. |
| `resource/` | ROS 2 resource index entries for Python package registration. |
| `test/` | Lint/test scaffolding (`flake8`, `pep257`, copyright). |

## Launch and runtime logic

Primary launcher:

- `mecanumbot_leading_behaviour/launch/launch_wifi_condition_sequence.launch.py`

What it does:

1. Detects current Wi-Fi SSID.
2. Selects default parameter YAML (`behaviour_setting_constants.yaml` or `Eto_behaviour_setting_constants.yaml`).
3. Sets `YAML_PATH` and `BEHAVIOUR_YAML_PATH` environment variables.
4. Starts one BT node based on `condition` argument (`Doglike`, `Control`, or `LED`).

## Build

```bash
colcon build --symlink-install --packages-select mecanumbot_behaviours mecanumbot_leading_behaviour mecanumbot_demo_behaviours
source install/setup.bash
```
