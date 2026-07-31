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

| Topic                | Data type                            | Function                                                                                                    |
| -------------------- | ------------------------------------ | ----------------------------------------------------------------------------------------------------------- |
| `/goal_pose`         | `geometry_msgs/msg/PoseStamped`      | Sends Nav2 navigation goals — only for `Approach`, i.e. whenever the robot actually drives somewhere.       |
| `/cmd_vel`           | `geometry_msgs/msg/Twist`            | Every in-place rotation (`TurnToward`, `RelativeTurnPattern`, `ScanSpin`/`FindPeople`/`Spin360`), profiled. |
| `/cmd_accessory_pos` | `mecanumbot_msgs/msg/AccessMotorCmd` | Neck (camera tilt) and gripper commands: gesture sequences plus the lifted/level head poses.                |

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

The library is split by concern; `behaviours/movement_managers.py` re-exports
everything, so `from ...behaviours.movement_managers import X` keeps working
(that is what `mecanumbot_demo_behaviours` imports).

| Module                 | Contents                                                                                          |
| ---------------------- | ------------------------------------------------------------------------------------------------- |
| `geometry.py`          | Pure geometry: angles, bearings, `signed_rotation`, `pose_to_goal`, checkpoint lookups, `route_progress`. |
| `ros_interfaces.py`    | Topic names, QoS, pose/people/ball trackers, the Nav2 goal monitor, velocity and neck commanders. |
| `targets.py`           | What a `target_type` points at, and the head pose / turn direction that goes with it.             |
| `turning.py`           | `SmoothTurner` plus the in-place turning behaviours.                                              |
| `searching.py`         | `WaitForPerson`, `ManageSearchCheckpoint` — the lost-human patrol.                                |
| `movement_managers.py` | `Approach`, the condition checks, and the re-exports above.                                       |

| Behaviour                    | Role                                                                                                                                                |
| ---------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------- |
| `Approach`                   | Navigates to a target through Nav2; `mode="exact"` drives to the point, `mode="fixed_distance"` steps `robot_approach_distance` closer.             |
| `TurnToward`                 | Rotates in place to face a `subject` / `target` / `start` / `checkpoint` / `patrol` / `last_checkpoint`, in a chosen direction (see below).         |
| `RelativeTurnPattern`        | Attention-getting wiggle: alternating turns that end on the starting heading, beginning in the direction of the last search turn.                   |
| `ScanSpin`                   | Spins in place looking for people, head lifted; `FindPeople` (spin until somebody is seen) and `Spin360` (one full scan) are configured subclasses. |
| `WaitForPerson`              | Interrupt half of the lost-recovery parallel: waits with a lifted head, and records whether the person turned up ahead of or behind the robot.      |
| `ManageSearchCheckpoint`     | Walks the patrol index along the route, reversing at either end.                                                                                    |
| `DogResumeLeading`           | After a search: leads on from the checkpoint nearest the robot, or the one after it when the human has already walked past it.                      |
| `CheckSubjectTargetSuccess`  | SUCCESS when the subject is within `target_reached_threshold` of the target.                                                                        |
| `CheckRobotHasBall`          | SUCCESS while `/mecanumbot/has_object` is true.                                                                                                     |
| `CheckRobotAtLastCheckpoint` | SUCCESS when `Dog_current_checkpoint >= Dog_max_checkpoint`.                                                                                        |

### Turning: smoothness and direction

In-place rotations do **not** go through Nav2. `SmoothTurner` drives `/cmd_vel`
with a profile that ramps up under an acceleration limit and eases out
proportionally to the angle left, and it tracks progress on AMCL yaw,
dead-reckoning from the commanded speed between pose updates. Tunables per
behaviour: `max_speed`, `accel`, `decel_gain`, `min_speed`, `tolerance` (defaults
stay inside the Nav2 controller's `max_vel_theta` / `max_angular_accel`).

Owning the rotation is what makes the direction selectable. `TurnToward` takes
`direction`:

| `direction`  | Meaning                                                                                                                                                 |
| ------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `"shortest"` | Short way round. Default for human targets, and asked for explicitly on the dog tree's turn to the next checkpoint.                                     |
| `"unwind"`   | Opposite to the last search turn — back the way the robot came. Default for route targets.                                                              |
| `"repeat"`   | Same handedness as the last search turn.                                                                                                                |

Any turn or scan that looks for a human stores its handedness in the
`search_spin_sign` blackboard key (`+1` counterclockwise, `-1` clockwise), so a
glance back over the right shoulder can be answered over the left — retracing
the rotation instead of carrying on around, taking the long way when it has to.

That is a gesture, so it is applied where the robot is dealing with the human:
`RelativeTurnPattern` starts its wiggle in the direction of the last glance, and
`TurnToward(TARGET)` unwinds it when pointing the target out. The dog tree's turn
to the next checkpoint overrides the route-target default with
`direction="shortest"` — a checkpoint is somewhere to drive to, and unwinding
there only made the robot swing the long way before setting off.

Because rotation bypasses Nav2, the local costmap does not supervise it; that was
already true of the old search spins, and only in-place rotation is affected.

### Head (neck) poses

`n_pos` is the neck-mounted camera tilt (`2.0 … 8.6`, larger looks further up).
Two named poses in `ros_interfaces.py`:

| Constant         | Value | Used when                                                                                                                                                    |
| ---------------- | ----- | ------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `NECK_SEEK_POS`  | `7.0` | The robot is looking for or at a human: reads as seeking contact, and gives YOLO26n-pose a full-body view instead of a pair of knees, which it often misses. |
| `NECK_LEVEL_POS` | `6.0` | The robot is driving its route or pointing at the target.                                                                                                    |

Behaviours pick the pose from their target type (`head="seek"` / `"level"`), so
the head stays lifted for the whole seeking phase rather than flicking up for a
moment. `head=None` leaves the neck untouched — the LED and control trees pass
that so their comparison conditions carry no head gestures at all.

### `behaviours/blackboard_managers.py`

| Behaviour                    | Role                                                                                                                     |
| ---------------------------- | ------------------------------------------------------------------------------------------------------------------------ |
| `ConstantParamsToBlackboard` | Loads the YAML constants, parses LED/gesture/checkpoint entries into message objects, and applies the start LED setting. |
| `DistanceToBlackboard`       | Computes robot–subject, robot–target and subject–target distances plus their threshold flags.                            |

`ConstantParamsToBlackboard` reads the YAML under the fixed
`bottom_up_tree_node: ros__parameters:` key, splits `Dog_checkpoints` so the first
entry becomes `start_position` and the last becomes `target_position`, and leaves the
remainder as the checkpoint list. It also seeds the runtime state the other
behaviours expect, so nothing has to guess whether a key exists yet:

| Key                                              | Meaning                                                                       |
| ------------------------------------------------ | ----------------------------------------------------------------------------- |
| `Dog_current_checkpoint` / `Dog_max_checkpoint`   | Progress along the route the robot leads.                                     |
| `patrol_checkpoints` / `patrol_current_checkpoint` | Separate index used while searching for a lost human.                        |
| `patrol_direction`                               | `+1` search forwards along the route, `-1` backwards (set by `WaitForPerson`). |
| `patrol_initialized`                             | `False` makes the next patrol snap to the nearest checkpoint.                  |
| `search_spin_sign`                               | Handedness of the last turn made to look at a human; route turns unwind it.    |
| `last_distance`                                  | Most recent robot–human distance from `DogCheckFollowing`.                     |

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
within `Dog_following_max_threshold` while leading) and `DogResumeLeading` (picks the
checkpoint leading carries on from after a search).

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
| `mecanumbot_leading_behaviour/tree_nodes/tree_common.py` | YAML resolution, the shared recovery branch, and the tree runner used by all four executables. |
| `launch/`                                  | Runtime launcher with condition-based node selection.                                                                   |
| `config/`                                  | Behaviour constants and sequences in YAML (`behaviour_setting_constants.yaml`, `Eto_behaviour_setting_constants.yaml`). |
| `resource/`                                | ROS package resource marker.                                                                                            |
| `test/`                                    | Package lint tests.                                                                                                     |

## BT logic

### Common pattern

1. Load YAML constants into blackboard (`ConstantParamsToBlackboard`).
2. Build a sequence/selector/retry structure from movement and signalling behaviours.
3. `tree_common.run_tree()` sets the tree up, ticks it every 100 ms and keeps the
   node alive with `rclpy.spin`.

### Lost-subject recovery

`tree_common.create_recover_lost_sequence()` builds it for every tree: a parallel
with `SuccessOnOne` policy running `WaitForPerson` against an endless patrol loop of
`Spin360` → `ManageSearchCheckpoint` → `Approach(target_type="patrol")`. The robot
therefore scans a full circle, walks to the next checkpoint, scans again, and so on
along the route, reversing at either end. As soon as a person is seen the patrol is
cancelled, and `WaitForPerson` sets `patrol_direction` from which side of the route
the person appeared on.

Because `WaitForPerson` succeeds on the very first tick when somebody is already
visible, the same branch covers both cases: a human who merely lagged behind is
fetched immediately, and only a real disappearance turns into a patrol.

Which checkpoint the robot resumes from is a separate decision, made by
`DogResumeLeading` and based on `geometry.route_progress()` — the human's position
projected onto the checkpoint polyline as a float index, so `1.4` means "four tenths
of the way from checkpoint 1 to checkpoint 2". The robot takes the checkpoint nearest
itself and leads on to the one after it when the human is already past it (by more
than `PASSED_MARGIN`, 0.15 of a stretch), otherwise to that checkpoint itself. The
same helper backs `path_progress_sign()`, which decides the patrol direction.

### `ctrl_tree.py` logic

1. Load constants.
2. Turn toward start.
3. Approach start.
4. Turn toward target.
5. Approach the last checkpoint (`target_type="last_checkpoint"`), i.e. stop short of
   the target itself — the target is where the human should end up. `LED_tree.py` and
   `bottom_up_tree.py` do the same; only `TurnToward` faces `target_position` proper.

### `dog_tree.py` logic

1. Load constants.
2. `SeekOrFind` selector — the seek-attention sequence (approach the human, face them,
   attention wiggle, catch-attention gesture) or, on failure, lost recovery followed by
   the same sequence. Both branches come from `create_seek_attention()`.
3. Endless `ShowOrLeadSelector` loop:
   - Ball reaction: if the robot has been handed the ball, recover the human, approach
     them and play the thank gesture. It needs no glance of its own, so it stays ahead
     of the cycle below;
   - else `LeadOrRecoverSelector`:
     - `GlanceThenActSequence` — one glance back for the human (`FindPeople`, head
       lifted) per cycle, then `ShowOrLeadStepSelector` acts on it:
       - if the human is near the target (`CheckSubjectTargetSuccess`), run the
         show/point sequence;
       - else lead one step: check they kept up (`DogCheckFollowing`), turn to face the
         next checkpoint by the smaller angle (`direction="shortest"`, head levelled)
         and drive there.

       Both used to open with their own `FindPeople`. That only scans when nobody is in
       view, but on a cycle where the closeness check failed the robot could still sweep
       twice over the same glance, which reads as dithering;
     - if the glance finds nobody, or the lead step fails — the human is gone — recover
       and resume: patrol the route until somebody is seen, walk up to them and ask for
       their attention again (`create_seek_attention()`), then `DogResumeLeading` picks
       the checkpoint to carry on from. Wrapped in `Retry(num_failures=3)`, because the
       human is most likely to slip out of view again during that walk up to them.
4. The root only fails when even the patrol turns up nobody; it is then ticked again
   from the top, which runs `SeekOrFind` before leading resumes.

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
