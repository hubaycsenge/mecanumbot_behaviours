# mecanumbot_leading_behaviour

`py_trees_ros` behaviour-tree package for Mecanumbot leading experiments.

It coordinates Nav2 navigation goals, accessory gestures, and LED service calls to
guide a human subject toward checkpoints. This package also holds the reusable
behaviour library that `mecanumbot_demo_behaviours` imports.

## Executable nodes

| Executable                | Source file                    | Main behavior                                                                  |
| ------------------------- | ------------------------------ | ------------------------------------------------------------------------------ |
| `control_leading_bt_node` | `tree_nodes/ctrl_tree.py`      | Start/target turn and approach sequence — the no-signalling control condition. |
| `doglike_leading_bt_node` | `tree_nodes/dog_tree.py`       | Dog-style lead loop (catch attention, point, move checkpoint-by-checkpoint).   |
| `LED_leading_bt_node`     | `tree_nodes/LED_tree.py`       | LED-driven guidance sequence with approach + near-target signalling.           |
| `bottom_up_tree_node`     | `tree_nodes/bottom_up_tree.py` | Baseline sequence combining approach and LED indication.                       |

All four call `tree_node.setup(node_name="bottom_up_tree_node")`, so every tree
registers the same ROS node name regardless of which executable was started.

## Node interfaces

The ROS interfaces are created inside the BT behaviour classes and used by the
executable tree nodes.

### Publishers

| Topic               | Data type                            | Function                                                                                   |
| ------------------- | ------------------------------------ | ------------------------------------------------------------------------------------------ |
| `/goal_pose`        | `geometry_msgs/msg/PoseStamped`      | Sends Nav2 navigation goals for turn/approach actions.                                     |
| `/cmd_vel`          | `geometry_msgs/msg/Twist`            | Direct velocity only for in-place search spins (`FindPeople`, `Spin360`, `WaitForPerson`). |
| `cmd_accessory_pos` | `mecanumbot_msgs/msg/AccessMotorCmd` | Sends neck/gripper command sequences (gestures for the dog-inspired behaviour).            |

### Subscribers

| Topic                              | Data type                                     | Processing                                                                                                                        |
| ---------------------------------- | --------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------- |
| `/amcl_pose`                       | `geometry_msgs/msg/PoseWithCovarianceStamped` | Tracks robot pose for goal generation and checkpoint selection. Subscribed with `RELIABLE` + `TRANSIENT_LOCAL` QoS to match AMCL. |
| `/mecanumbot/people_fusion`        | `geometry_msgs/msg/PoseArray`                 | Fused people detections — used to find, select and approach the subject and to test target success.                               |
| `/mecanumbot/subject_pose`         | `geometry_msgs/msg/PoseStamped`               | Tracked subject pose, used by `DogCheckFollowing` and `DistanceToBlackboard`.                                                     |
| `/mecanumbot/has_object`           | `std_msgs/msg/Bool`                           | Ball-handover trigger read by `CheckRobotHasBall`.                                                                                |
| `/navigate_to_pose/_action/status` | `action_msgs/msg/GoalStatusArray`             | Monitors Nav2 goal execution states; goals are matched by UUID and retried on abort/cancel.                                       |

### Services handled

| Service | Type | Behavior                           |
| ------- | ---- | ---------------------------------- |
| None    | -    | Nodes do not provide ROS services. |

### Service clients used

| Service                      | Type                               | Function                                                               |
| ---------------------------- | ---------------------------------- | ---------------------------------------------------------------------- |
| `/mecanumbot/set_led_status` | `mecanumbot_msgs/srv/SetLedStatus` | Sends LED patterns during init, attention cues, and target indication. |

## Behaviour library

### `behaviours/movement_managers.py`

| Behaviour                    | Role                                                                                                                   |
| ---------------------------- | ---------------------------------------------------------------------------------------------------------------------- |
| `TurnToward`                 | Publishes a goal that only rotates the robot to face a `subject` / `target` / `start` / `checkpoint`.                  |
| `Approach`                   | Navigates to a target; `mode="exact"` drives to the point, `mode="fixed_distance"` stops at `robot_approach_distance`. |
| `RelativeTurnPattern`        | Small alternating turns in place, used as an attention-getting wiggle.                                                 |
| `FindPeople`                 | Spins on `/cmd_vel` until a fresh `people_fusion` message arrives.                                                     |
| `WaitForPerson`              | Blocks until a person is seen; used as the interrupt half of the lost-recovery parallel.                               |
| `Spin360`                    | Full in-place rotation scan.                                                                                           |
| `ManageSearchCheckpoint`     | Advances the search checkpoint index during recovery patrol.                                                           |
| `CheckSubjectTargetSuccess`  | SUCCESS when the subject is within `target_reached_threshold` of the target.                                           |
| `CheckRobotHasBall`          | SUCCESS while `/mecanumbot/has_object` is true.                                                                        |
| `CheckRobotAtLastCheckpoint` | SUCCESS when `Dog_current_checkpoint >= Dog_max_checkpoint`.                                                           |

Module-level helpers: `pose_to_goal`, `calculate_facing_orientation`,
`yaw_from_quaternion`, `normalize_angle`, and the `STATUS_*` Nav2 goal-status
constants (also imported by the demo package).

### `behaviours/blackboard_managers.py`

| Behaviour                    | Role                                                                                                                     |
| ---------------------------- | ------------------------------------------------------------------------------------------------------------------------ |
| `ConstantParamsToBlackboard` | Loads the YAML constants, parses LED/gesture/checkpoint entries into message objects, and applies the start LED setting. |
| `DistanceToBlackboard`       | Computes robot–subject, robot–target and subject–target distances plus their threshold flags.                            |

`ConstantParamsToBlackboard` reads the YAML under the fixed
`bottom_up_tree_node: ros__parameters:` key, splits `Dog_checkpoints` so the first
entry becomes `start_position` and the last becomes `target_position`, and leaves the
remainder as the checkpoint list.

### `behaviours/LED_behaviours.py` and `behaviours/dog_behaviours.py`

`LEDBehaviourSequence(name, mode)` and `DogBehaviourSequence(name, mode)` step through
a timed sequence taken from the blackboard. Supported modes:

| Mode                    | Blackboard keys used                                                       |
| ----------------------- | -------------------------------------------------------------------------- |
| `catch_attention`       | `LED_catch_attention_seq` / `_times`, `Dog_catch_attention_seq` / `_times` |
| `indicate_target`       | `LED_indicate_target_seq` / `_times`, `Dog_indicate_target_seq` / `_times` |
| `indicate_close_target` | `LED_indicate_close_target_seq` / `_times` (LED only)                      |
| `thank`                 | `LED_thank_seq` / `_times`, `Dog_thank_seq` / `_times`                     |

`dog_behaviours.py` also provides `DogCheckFollowing` (verifies the subject stayed
within `Dog_following_max_threshold` while leading) and `DogSelectTarget`.

## Launch files

### `launch/launch_wifi_condition_sequence.launch.py`

Functions:

1. Detects active Wi-Fi SSID (`nmcli`, fallback `iwgetid`).
2. Chooses default constants YAML based on SSID (`MecanumNet` → `behaviour_setting_constants.yaml`, `MecanumetoNet` → `Eto_behaviour_setting_constants.yaml`).
3. Declares launch args: `params`, `yaml_path`, `namespace` (default `mecanumbot`), `condition` (default `Doglike`).
4. Exports `YAML_PATH` and `BEHAVIOUR_YAML_PATH` env vars for BT scripts.
5. Starts exactly one node by `condition`: `Doglike` -> `doglike_leading_bt_node`, `Control` -> `control_leading_bt_node`, `LED` -> `LED_leading_bt_node`.

Each tree resolves its YAML in this order: `--yaml_path` argument, then `YAML_PATH`,
then `BEHAVIOUR_YAML_PATH`, then a packaged fallback. `bottom_up_tree_node` is not
started by the launch file — run it with `ros2 run`.

## Folder structure

| Path                                       | Role                                                                                                                    |
| ------------------------------------------ | ----------------------------------------------------------------------------------------------------------------------- |
| `mecanumbot_leading_behaviour/tree_nodes/` | Top-level BT compositions and executable entry points.                                                                  |
| `mecanumbot_leading_behaviour/behaviours/` | Reusable BT leaf behaviours (movement, LED, dog gestures, blackboard loaders).                                          |
| `mecanumbot_leading_behaviour/utils/`      | Subtree construction helpers (legacy/experimental composition utilities).                                               |
| `launch/`                                  | Runtime launcher with condition-based node selection.                                                                   |
| `config/`                                  | Behaviour constants and sequences in YAML (`behaviour_setting_constants.yaml`, `Eto_behaviour_setting_constants.yaml`). |
| `resource/`                                | ROS package resource marker.                                                                                            |
| `test/`                                    | Package lint tests.                                                                                                     |

## BT logic

### Common pattern

1. Load YAML constants into blackboard (`ConstantParamsToBlackboard`).
2. Build a sequence/selector/retry structure from movement and signalling behaviours.
3. Tick tree periodically (`tick_tock`) and keep node alive with `rclpy.spin`.

### Lost-subject recovery

`dog_tree.py` and `LED_tree.py` share a `create_recover_lost_sequence()` helper: a
parallel with `SuccessOnOne` policy running `WaitForPerson` against an endless patrol
loop of `Spin360` → `ManageSearchCheckpoint` → `Approach(checkpoint)`. As soon as a
person is seen the patrol is cancelled.

### `ctrl_tree.py` logic

1. Load constants.
2. Turn toward start.
3. Approach start.
4. Turn toward target.
5. Approach target.

### `dog_tree.py` logic

1. Load constants.
2. `SeekOrFind` selector — normal seek-attention stage (approach subject, turn to them,
   attention wiggle, catch-attention gesture) or, on failure, lost recovery followed by
   the same approach and attention sequence.
3. Endless `ShowOrLeadSelector` loop:
   - Ball reaction: if the robot has been handed the ball, recover the subject, approach
     them and play the thank gesture;
   - else if the subject is near the target, run the show/point sequence;
   - else lead one step to the next checkpoint and verify the subject is following.
4. Continue until externally stopped.

### `LED_tree.py` logic

1. Load constants.
2. `SeekOrFind` selector — approach the subject, or run lost recovery then approach.
3. Turn toward the subject and catch attention with the LEDs.
4. Approach the target and indicate it.
5. Loop near-target indication while the subject stays close.
6. Endless ball-reaction loop (has-ball → find person → thank pattern).

### `bottom_up_tree.py` logic

1. Load constants and wait delay.
2. Approach subject.
3. Approach target.
4. Indicate target with LED.
5. Selector repeatedly alternates check/show behavior until close condition is satisfied.

## Configuration model

Both `config/*.yaml` files use the same schema, nested under
`bottom_up_tree_node: ros__parameters:`. Sequence entries are strings holding Python
dict literals, parsed with `ast.literal_eval`.

| Group               | Keys                                                                                                                                                            |
| ------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Timing              | `init_delay`, `visibility_time_threshold`                                                                                                                       |
| Distance thresholds | `robot_closeness_threshold`, `target_reached_threshold`, `robot_approach_distance`                                                                              |
| Dog condition       | `Dog_following_max_threshold`, `Dog_max_wander_allowed`, `Dog_checkpoints`                                                                                      |
| LED scripts         | `LED_start_setting`, `LED_catch_attention_seq`/`_times`, `LED_indicate_target_seq`/`_times`, `LED_indicate_close_target_seq`/`_times`, `LED_thank_seq`/`_times` |
| Gesture scripts     | `Dog_catch_attention_seq`/`_times`, `Dog_indicate_target_seq`/`_times`, `Dog_thank_seq`/`_times`                                                                |

Each `*_seq` list pairs with a `*_times` list of equal length giving the duration of
each step in seconds. LED entries are `{fl,fr,bl,br}` dicts of `color`/`mode` values;
gesture entries are `{n_pos, gl_pos, gr_pos}` dicts; checkpoints are `{X, Y, Z}` dicts.

## Run examples

```bash
ros2 launch mecanumbot_leading_behaviour launch_wifi_condition_sequence.launch.py condition:=Doglike
ros2 launch mecanumbot_leading_behaviour launch_wifi_condition_sequence.launch.py condition:=Control
ros2 launch mecanumbot_leading_behaviour launch_wifi_condition_sequence.launch.py condition:=LED
ros2 run mecanumbot_leading_behaviour bottom_up_tree_node
```

Override the parameter file explicitly:

```bash
ros2 launch mecanumbot_leading_behaviour launch_wifi_condition_sequence.launch.py \
  condition:=LED yaml_path:=/absolute/path/to/behaviour_setting_constants.yaml
```

The trees expect Nav2 (with AMCL localized on a map), the people-detection pipeline
publishing `/mecanumbot/people_fusion`, and — for the LED condition — the
`mecanumbot_led` service node.
