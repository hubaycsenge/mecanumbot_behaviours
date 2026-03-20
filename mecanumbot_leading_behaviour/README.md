# mecanumbot_leading_behaviour

`py_trees_ros` behaviour-tree package for Mecanumbot leading experiments.

It coordinates navigation goals, accessory gestures, and LED service calls to guide a human subject toward checkpoints.

## Executable nodes

| Executable | Source file | Main behavior |
| --- | --- | --- |
| `control_leading_bt_node` | `tree_nodes/ctrl_tree.py` | Start/target turn and approach sequence. |
| `doglike_leading_bt_node` | `tree_nodes/dog_tree.py` | Dog-style lead loop (catch attention, point, move checkpoint-by-checkpoint). |
| `LED_leading_bt_node` | `tree_nodes/LED_tree.py` | LED-driven guidance sequence with approach + near-target signaling. |
| `bottom_up_tree_node` | `tree_nodes/bottom_up_tree.py` | Baseline sequence combining approach and LED indication. |

## Node interfaces

The ROS interfaces are created inside BT behaviour classes and used by the executable tree nodes.

### Publishers

| Topic | Data type | Function |
| --- | --- | --- |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | Sends Nav2 navigation goals for turn/approach actions. |
| `cmd_accessory_pos` | `mecanumbot_msgs/msg/AccessMotorCmd` | Sends neck/gripper command sequences (gestures for dog inspired behaviour). |

### Subscribers

| Topic | Data type | Processing |
| --- | --- | --- |
| `/amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Tracks robot pose for goal generation and checkpoint selection. |
| `/mecanumbot/subject_pose` | `geometry_msgs/msg/PoseStamped` | Tracks detected subject pose for following and success checks. |
| `/navigate_to_pose/_action/status` | `action_msgs/msg/GoalStatusArray` | Monitors Nav2 goal execution states and retries/failure handling. |

### Services handled

| Service | Type | Behavior |
| --- | --- | --- |
| None | - | Nodes do not provide ROS services. |

### Service clients used

| Service | Type | Function |
| --- | --- | --- |
| `/set_led_status` | `mecanumbot_msgs/srv/SetLedStatus` | Sends LED patterns during init, attention cues, and target indication. |

## Launch files

### `launch/launch_wifi_condition_sequence.launch.py`

Functions:

1. Detects active Wi-Fi SSID (`nmcli` fallback `iwgetid`).
2. Chooses default constants YAML based on SSID.
3. Declares launch args: `params`, `yaml_path`, `namespace`, `condition`.
4. Exports `YAML_PATH` and `BEHAVIOUR_YAML_PATH` env vars for BT scripts.
5. Starts exactly one node by `condition`: `Doglike` -> `doglike_leading_bt_node`, `Control` -> `control_leading_bt_node`, `LED` -> `LED_leading_bt_node`.

## Folder structure

| Path | Role |
| --- | --- |
| `mecanumbot_leading_behaviour/tree_nodes/` | Top-level BT compositions and executable entry points. |
| `mecanumbot_leading_behaviour/behaviours/` | Reusable BT leaf behaviours (movement, LED, dog gestures, blackboard loaders). |
| `mecanumbot_leading_behaviour/utils/` | Subtree construction helpers (legacy/experimental composition utilities). |
| `launch/` | Runtime launcher with condition-based node selection. |
| `config/` | Behaviour constants and sequences in YAML (`behaviour_setting_constants.yaml`, `Eto_behaviour_setting_constants.yaml`). |
| `resource/` | ROS package resource marker. |
| `test/` | Package lint tests. |

## BT logic

### Common pattern

1. Load YAML constants into blackboard (`ConstantParamsToBlackboard`).
2. Build a sequence/selector/retry structure from movement and signalling behaviours.
3. Tick tree periodically (`tick_tock`) and keep node alive with `rclpy.spin`.

### `ctrl_tree.py` logic

1. Load constants.
2. Turn toward start.
3. Approach start.
4. Turn toward target.
5. Approach target.

### `dog_tree.py` logic

1. Load constants.
2. Initial seek-attention stage (turn to subject + catch attention gesture).
3. Repeat loop with selector.
   - Branch A: if subject near target, run show/point sequence;
   - Branch B: lead one step to checkpoint and verify following behavior.
4. Continue until externally stopped.

### `LED_tree.py` logic

1. Load constants and wait delay.
2. Retry subject-approach until successful.
3. Catch attention with LED.
4. Approach target and indicate target.
5. Loop near-target indication while success condition remains active.

### `bottom_up_tree.py` logic

1. Load constants and wait delay.
2. Approach subject.
3. Approach target.
4. Indicate target with LED.
5. Selector repeatedly alternates check/show behavior until close condition is satisfied.

## Configuration model

`config/*.yaml` keys define:

- Thresholds: `robot_closeness_threshold`, `target_reached_threshold`, `visibility_time_threshold`.
- Navigation guidance: `robot_approach_distance`, `Dog_checkpoints`.
- LED scripts: `LED_*_seq`, `LED_*_times`.
- Accessory scripts: `Dog_*_seq`, `Dog_*_times`.

## Run examples

```bash
ros2 launch mecanumbot_leading_behaviour launch_wifi_condition_sequence.launch.py condition:=Doglike
ros2 launch mecanumbot_leading_behaviour launch_wifi_condition_sequence.launch.py condition:=Control
ros2 launch mecanumbot_leading_behaviour launch_wifi_condition_sequence.launch.py condition:=LED
ros2 run mecanumbot_leading_behaviour bottom_up_tree_node
```
