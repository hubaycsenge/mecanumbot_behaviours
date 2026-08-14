# mecanumbot_leading_behaviour

`py_trees_ros` behaviour-tree package for Mecanumbot leading experiments.

It coordinates Nav2 navigation goals, accessory gestures, and LED service calls to
guide a human subject toward checkpoints. The movement behaviours it is built
out of live in `mecanumbot_movement_behaviours` and the constants loader in
`mecanumbot_bt_config`; what is here is the leading experiment itself.

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
executable tree nodes. Most of them belong to `mecanumbot_movement_behaviours`,
whose README documents them too; they are repeated here because these are the
interfaces a running leading tree has.

### Publishers

| Topic                | Data type                            | Function                                                                                                                    |
| -------------------- | ------------------------------------ | ----------------------------------------------------------------------------------------------------------------------------- |
| `/cmd_vel`           | `geometry_msgs/msg/Twist`            | Every in-place rotation (`TurnToward`, `RelativeTurnPattern`, `GlanceBack`, `ScanSpin`/`FindPeople`/`Spin360`), profiled.   |
| `/cmd_accessory_pos` | `mecanumbot_msgs/msg/AccessMotorCmd` | Neck (camera tilt) and gripper commands: gesture sequences plus the lifted/level head poses.                                |
| `/goal_pose`         | `geometry_msgs/msg/PoseStamped`      | Created by `Nav2GoalMonitor`, which the leading behaviours no longer use — it stays for the ostensive package's pointing goals. |

### Action clients used

Driving goes through Nav2's actions rather than the `/goal_pose` topic, so a
goal has an identity, a result and a cancel (see *Navigating by action* below).

| Action                    | Type                             | Used by                                                          |
| ------------------------- | -------------------------------- | ------------------------------------------------------------------ |
| `/navigate_to_pose`       | `nav2_msgs/action/NavigateToPose` | `Approach` — one place to drive to.                              |
| `/navigate_through_poses` | `nav2_msgs/action/NavigateThroughPoses` | `FollowRoute` — a leg of route checkpoints in one goal. |

### Subscribers

| Topic                              | Data type                                     | Processing                                                                                                                        |
| ---------------------------------- | --------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------- |
| `/amcl_pose`                       | `geometry_msgs/msg/PoseWithCovarianceStamped` | Tracks robot pose for goal generation and checkpoint selection. Subscribed with `RELIABLE` + `TRANSIENT_LOCAL` QoS to match AMCL. |
| `/mecanumbot/people_fusion`        | `geometry_msgs/msg/PoseArray`                 | Fused people detections — used to find, select and approach the subject and to test target success.                               |
| `/mecanumbot/subject_pose`         | `geometry_msgs/msg/PoseStamped`               | Tracked subject pose. Read together with the fused detections by `FollowedSubjectTracker`, whichever of the two is fresher.       |
| `/mecanumbot/has_object`           | `std_msgs/msg/Bool`                           | Ball-handover trigger read by `CheckRobotHasBall`.                                                                                |
| `/navigate_to_pose/_action/status` | `action_msgs/msg/GoalStatusArray`             | Only for `Nav2GoalMonitor.busy()` — "is Nav2 driving right now", which is what a turn waits for before it takes `/cmd_vel`.       |
| `/navigate_through_poses/_action/status` | `action_msgs/msg/GoalStatusArray`       | The same question for a waypoint run, so a turn does not take `/cmd_vel` in the middle of one.                                     |

### Services handled

| Service | Type | Behavior                           |
| ------- | ---- | ---------------------------------- |
| None    | -    | Nodes do not provide ROS services. |

### Service clients used

| Service                      | Type                               | Function                                                               |
| ---------------------------- | ---------------------------------- | ---------------------------------------------------------------------- |
| `/mecanumbot/set_led_status` | `mecanumbot_msgs/srv/SetLedStatus` | Sends LED patterns during init, attention cues, and target indication. |

## Behaviour library

The reusable half of what used to live here has moved out, so this package is
the leading *experiment* and nothing else:

| Package | Holds |
| --- | --- |
| `mecanumbot_movement_behaviours` | `Approach`, `FollowRoute`, every turn and scan, the patrol, the Nav2 navigators, the trackers, the geometry and the pacing rules. Its README documents them, including the ROS interfaces listed above. |
| `mecanumbot_bt_config` | Reading a constants YAML onto the blackboard, and spinning a tree on it. |

What is left in `behaviours/` is this experiment:

| Module | Contents |
| --- | --- |
| `keys.py` | `LEADING_KEYS` — that these files call the route `Dog_checkpoints`. |
| `defaults.py` | The four tunables that are about this experiment rather than about moving, the merged `Tunables`, and `REQUIRED`: what a constants file may not leave out. |
| `route_behaviours.py` | The movement behaviours bound to `LEADING_KEYS`, plus the ones that need no binding re-exported. **The trees import their movement from here.** |
| `blackboard_managers.py` | `ConstantParamsToBlackboard`, `ConfiguredTimer`, `DistanceToBlackboard`. |
| `dog_behaviours.py` | The gesture sequence player and the following/check-in/resume decisions. |
| `LED_behaviours.py` | The LED pattern sequence player. |

| Behaviour | Role |
| --- | --- |
| `DogBehaviourSequence` | Plays one timed accessory-gesture sequence from the blackboard. |
| `DogCheckFollowing` | Verifies the subject is within `Dog_following_max_threshold` — asked both after a check-in glance and after a recovery patrol, to decide whether the robot has to go back to them at all. |
| `DogCheckInDue` | SUCCESS when a look back is due — the pacing measured against `pacing.check_in_due()`. |
| `DogWaitForCatchUp` | Stands still, head up, while a trailing human catches up; FAILURE after `check_in_catch_up_timeout` sends the robot back to fetch them. |
| `DogResumeLeading` | After a search: leads on from the checkpoint nearest the robot, or the one after it when the human has already walked past it. |
| `LEDBehaviourSequence` | Plays one timed LED pattern sequence from the blackboard. |

`LEDBehaviourSequence(name, mode)` and `DogBehaviourSequence(name, mode)` step
through a sequence taken from the blackboard. Supported modes:

| Mode | Blackboard keys used |
| --- | --- |
| `catch_attention` | `LED_catch_attention_seq` / `_times`, `Dog_catch_attention_seq` / `_times` |
| `indicate_target` | `LED_indicate_target_seq` / `_times`, `Dog_indicate_target_seq` / `_times` |
| `indicate_close_target` | `LED_indicate_close_target_seq` / `_times` (LED only) |
| `thank` | `LED_thank_seq` / `_times`, `Dog_thank_seq` / `_times` |

### `behaviours/blackboard_managers.py`

| Behaviour | Role |
| --- | --- |
| `ConstantParamsToBlackboard` | Loads the YAML constants, reads the checkpoint list as a route, and applies the start LED setting. |
| `ConfiguredTimer` | A `py_trees` timer whose duration is a blackboard key, looked up when the timer starts rather than when the tree is built. |
| `DistanceToBlackboard` | Computes robot-subject, robot-target and subject-target distances plus their threshold flags. |

The loading itself is `mecanumbot_bt_config`'s `ParamsToBlackboard`, which knows
no key names at all — it writes whatever the file declares. What this subclass
adds is the three things a constants file cannot say about itself:

* **what is required.** `defaults.REQUIRED` plus the signalling scripts of the
  condition being run: each tree passes `scripts=` naming the sequences it
  actually plays, so the control condition's file needs no LED sequences while
  the LED condition's would fail without them;
* **what the checkpoint list means.** The first entry becomes `start_position`,
  the last becomes `target_position`, and the rest — the first included — is the
  route in `Dog_checkpoints`, copied to `patrol_checkpoints` for the search;
* **what is state rather than configuration.** Seeded on every entry, so nothing
  has to guess whether a key exists yet:

| Key | Meaning |
| --- | --- |
| `Dog_current_checkpoint` / `Dog_max_checkpoint` | Progress along the route the robot leads. |
| `patrol_checkpoints` / `patrol_current_checkpoint` | Separate index used while searching for a lost human. |
| `patrol_direction` | `+1` search forwards along the route, `-1` backwards (set by `WaitForPerson`). |
| `patrol_initialized` | `False` makes the next patrol snap to the nearest checkpoint. |
| `search_spin_sign` | Handedness of the last turn made to look at a human; route turns unwind it. |
| `last_distance` | Most recent robot-human distance from `DogCheckFollowing`. |

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
| `mecanumbot_leading_behaviour/behaviours/` | This experiment's leaf behaviours (LED, dog gestures, its constants loader) and its bindings of the shared movement library. |
| `mecanumbot_leading_behaviour/tree_nodes/tree_common.py` | The package and node names, and the shared recovery branch, on top of `mecanumbot_bt_config.tree_runner`. |
| `mecanumbot_leading_behaviour/tools/route_check.py` | `check_route`: plans every hop of the configured route against the live costmap, without running a tree. |
| `launch/`                                  | Runtime launcher with condition-based node selection.                                                                   |
| `config/`                                  | Behaviour constants and sequences in YAML (`behaviour_setting_constants.yaml`, `Eto_behaviour_setting_constants.yaml`). |
| `resource/`                                | ROS package resource marker.                                                                                            |
| `test/`                                    | The package lint tests. |

## Tests

This package has no unit tests of its own any more: `test_pacing.py` moved to
`mecanumbot_movement_behaviours` with the rules it covers.

```bash
colcon test --packages-select mecanumbot_leading_behaviour   # lint only
```

## BT logic

### Common pattern

1. Load YAML constants into blackboard (`ConstantParamsToBlackboard`) — always the
   first leaf, because every other behaviour reads its constants during `setup()`.
2. Build a sequence/selector/retry structure from movement and signalling behaviours.
3. `tree_common.run_tree()` sets the tree up, ticks it every `tick_period_ms`
   (100 ms by default) and keeps the node alive with `rclpy.spin`.

### Lost-subject recovery

`tree_common.create_recover_lost_sequence()` builds it for every tree: a parallel
with `SuccessOnOne` policy running `WaitForPerson` against an endless patrol loop of
`Spin360` → `ManageSearchCheckpoint` → `Approach(target_type="patrol")`. The robot
therefore scans a full circle, walks to the next checkpoint, scans again, and so on
along the route, reversing at either end. As soon as a person is seen the patrol is
cancelled, and `WaitForPerson` sets `patrol_direction` from which side of the route
the person appeared on.

The cancellation is immediate and mid-movement: `WaitForPerson` succeeding makes the
parallel succeed, which invalidates the running search branch, so whichever leaf was
running is stopped on the same tick — a `Spin360` a third of the way round is not
played out to the full revolution (`InPlaceTurn.terminate()` stops the turner and
zeroes `/cmd_vel`), and a drive to the next search checkpoint is not driven to the
end (`Approach.terminate()` cancels the nav2 goal). `Spin360` itself is configured
`stop_on_person=False` precisely because it is not the branch that watches for
people; `WaitForPerson` is, and it sees the same `people_fusion` detections.

Because `WaitForPerson` succeeds on the very first tick when somebody is already
visible, the same branch covers both cases: a human who merely lagged behind is
fetched immediately, and only a real disappearance turns into a patrol.

Which checkpoint the robot resumes from is a separate decision, made by
`DogResumeLeading` and based on `geometry.route_progress()` — the human's position
projected onto the checkpoint polyline as a float index, so `1.4` means "four tenths
of the way from checkpoint 1 to checkpoint 2". The robot takes the checkpoint nearest
itself and leads on to the one after it when the human is already past it (by more
than `resume_passed_margin`, 0.15 of a stretch), otherwise to that checkpoint itself.
The same helper backs `path_progress_sign()`, which decides the patrol direction.

### `ctrl_tree.py` logic

1. Load constants.
2. Turn toward start.
3. Approach start.
4. Turn toward target.
5. Approach the last checkpoint (`target_type="last_checkpoint"`), i.e. stop short of
   the target itself — the target is where the human should end up. `LED_tree.py` and
   `bottom_up_tree.py` do the same; only `TurnToward` faces `target_position` proper.

### `dog_tree.py` logic

The loop is built around walking and checking being different things, and the
walking being most of it. A dog does not stop at every step to look round, and
it does not ignore its person for the whole walk either.

1. Load constants.
2. `SeekOrFind` selector — the seek-attention sequence (approach the human, face them,
   attention wiggle, catch-attention gesture) or, on failure, lost recovery followed by
   the same sequence. Both branches come from `create_seek_attention()`.
3. Endless `ShowOrLeadSelector` loop:
   - Ball reaction: if the robot has been handed the ball, recover the human, approach
     them and play the thank gesture. It needs no look back of its own, so it stays
     ahead of the cycle below;
   - else `LeadOrRecoverSelector`:
     - `LeadCycleSequence` — check in if it is time, then lead on:
       - `LeadCheckInIfDueSelector` (`create_check_in()`): `DogCheckInDue` under an
         inverter, so "not due" is the branch that lets the robot carry straight on
         driving. When one *is* due:
         - `GlanceBack` turns slowly the whole way round, setting off towards the
           human's last known place and stopping on the first person it sees.
           FAILURE here — and only here — means the human is lost;
         - `DogCheckFollowing` then says whether they are close enough. If they are
           not: `DogWaitForCatchUp` stands still and gives them
           `check_in_catch_up_timeout` seconds, and only when that runs out does the
           robot walk back (`create_seek_attention()`) and `DogResumeLeading` pick up
           the route again from wherever the pair now are;
       - `ShowOrLeadStepSelector`: if the human is near the target
         (`CheckSubjectTargetSuccess`), run the show/point sequence; otherwise lead a
         leg — turn to face the next checkpoint by the smaller angle
         (`direction="shortest"`, head levelled, and only if the route really bends by
         more than `route_turn_min`), then `FollowRoute`;
     - if the check-in found nobody, the human is gone: recover and resume — patrol the
       route until somebody is seen, then `RegainOrCarryOnSelector` decides what that
       sighting was worth. `DogCheckFollowing` first: somebody within
       `Dog_following_max_threshold` of where the patrol stopped is close enough to be
       led on from there, and the robot skips straight to `DogResumeLeading`. Otherwise
       `create_seek_attention()` walks up to them and asks for their attention again
       before resuming. Wrapped in `Retry(num_failures=3)`, because the human is most
       likely to slip out of view again during that walk up to them.
4. The root only fails when even the patrol turns up nobody; it is then ticked again
   from the top, which runs `SeekOrFind` before leading resumes.

#### Leading a leg

`FollowRoute` hands Nav2 several checkpoints in one `NavigateThroughPoses` goal
and lets it drive through them without stopping. `geometry.route_poses()`
orients every waypoint along the route, so the robot passes each one already
pointing where it is about to go, and the last one faces the target beyond the
route. How long a leg is comes from `pacing.route_leg()` — as far as the
look-back pacing allows, so the robot never plans past a check-in it is about to
owe, and never further than `route_lookahead` checkpoints.

Checkpoints count as driven past by distance (`checkpoint_reached_distance`)
rather than by Nav2's own arrival, so `Dog_current_checkpoint` moves along while
the robot drives through and does not depend on which Nav2 behaviour tree is
loaded. A leg is cut short — with SUCCESS, not FAILURE — when the human has been
out of sight or trailing for `check_in_grace` seconds together: stopping the
drive is not a failure of it, and the check-in that comes next is due by exactly
the same measurements.

#### Pacing the look backs

`behaviours/pacing.py` holds the rule and imports nothing, so it can be read and
tested on its own. A look back is due when

* the human has not been seen at all, or not for `visibility_time_threshold`
  seconds;
* they are further away than `Dog_following_max_threshold`;
* `check_in_every_checkpoints` checkpoints have been driven since the last one;
* `check_in_interval` seconds have passed since the last one.

The first three are reasons to look back *now*; the last two are the habit.
Either counter set to `0` switches that half of the habit off. The clock starts
the first time `DogCheckInDue` is asked rather than when the constants load,
because leading begins with the robot face to face with its human.

#### Looking back, and being lost

`GlanceBack` is the check-in, and it is a different movement from a search. It
runs at `glance_spin_speed`, slower than either scan, for two reasons at once: a
slow turn reads as looking rather than casting about, and it gives the camera
detector time to find a person the robot sweeps past — turning fast past
somebody and declaring them missing is exactly how a following human gets
treated as a lost one. It has three phases: `glance_revolutions` full turns in
place, set off towards the bearing of the place the human was last seen (or back
down the route towards the checkpoint the pair came from, if they have never
been seen) so that the shoulder the robot looks over first is the one they ought
to be behind; a ramp-down the moment somebody is detected; and a short
correction onto their bearing, so the check-in ends face to face however far
round the spin had got — the LiDAR detector reports people well off to the side
of where the camera points, and a check-in that ends looking past its human is
not one.

Turning the whole way round rather than only onto the last known bearing is what
lets a human who has stepped off the route be found at all: the check-in no
longer assumes they are still where they were. Only detections that arrive
*after* the spin starts count, or somebody already visible when the check-in
falls due — the human walking behind the robot, held by the LiDAR for the whole
leg — would end the glance on its first tick and the robot would never look
round at all. Running out of `glance_timeout` is a failure only while the robot
is still spinning: once the human has been seen the check-in has its answer,
whether or not there was time left to turn onto them.

Only its FAILURE starts the recovery patrol. The robot does not go hunting
through the building for somebody it has not properly looked for yet, and a
human who is merely two metres behind is fetched rather than searched for.

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

Both `config/*.yaml` files use the same schema. They are nested under
`bottom_up_tree_node: ros__parameters:`, though nothing depends on that name any
more — `mecanumbot_bt_config` reads the root key off the file. Sequence entries
are strings holding Python dict literals, decoded into messages by the keys
inside them.

There are two kinds of key, and they behave differently when one is missing.

### Experiment parameters — required

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

These describe the run, so a file that omits one fails to load rather than
having a distance invented for it — they are `defaults.REQUIRED` plus the
signalling scripts of the condition being run. Which scripts those are is the
tree's `scripts=` argument, so the control condition needs none of them.

### Tunables — optional, defaulted in Python

Everything the behaviour library used to carry as a constructor default or a
module constant. Declaring one in the YAML changes it everywhere; omitting it
keeps the packaged default, which is what lets a constants file written before a
key existed still load. The shipped files declare all of them, so a run is
described by one file.

The defaults live with the behaviours that read them —
`mecanumbot_movement_behaviours.defaults.MOVEMENT_DEFAULTS` for the movement
ones, `behaviours/defaults.py` for the four that are about this experiment, and
`mecanumbot_bt_config.tree_runner.RUNTIME_DEFAULTS` for the two the tree runner
settles before the first tick.

| Group             | Keys                                                                                                              | Read by                                          |
| ----------------- | ------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------- |
| Tree runtime      | `tick_period_ms`, `setup_timeout`                                                                                 | `bt_config.tree_runner.run_tree`                 |
| Seeing people     | `sight_timeout`                                                                                                   | every behaviour that watches `people_fusion`     |
| Rotation profile  | `turn_max_speed`, `turn_accel`, `turn_decel_gain`, `turn_min_speed`, `turn_tolerance_deg`, `facing_epsilon_deg`   | `SmoothTurner`, `signed_rotation`                |
| Turning           | `turn_timeout`, `turn_nav2_wait`, `turn_target_timeout`, `turn_corrections`, `attention_turn_step_deg`            | `InPlaceTurn`, `TurnToward`, `RelativeTurnPattern` |
| Scanning          | `scan_spin_speed`, `scan_timeout`; `full_scan_spin_speed`, `full_scan_timeout`, `full_scan_revolutions`           | `FindPeople`; `Spin360`                          |
| Looking back      | `glance_spin_speed`, `glance_timeout`, `glance_revolutions`                                                      | `GlanceBack`                                     |
| Check-in pacing   | `check_in_every_checkpoints`, `check_in_interval`, `check_in_grace`, `check_in_catch_up_timeout`                 | `DogCheckInDue`, `FollowRoute`, `DogWaitForCatchUp` |
| Approaching       | `approach_target_timeout`, `approach_goal_timeout`, `route_step_distance`, `route_stop_distance`, `nav_goal_retries` | `Approach`                                  |
| Leading a leg     | `route_lookahead`, `checkpoint_reached_distance`, `route_turn_min_deg`                                           | `FollowRoute`, the dog tree's checkpoint turn    |
| Getting them back | `resume_passed_margin`, `recover_retries`                                                                         | `DogResumeLeading`, the dog tree's `Retry`       |
| Pacing            | `thank_delay`, `show_turn_delay`                                                                                  | the dog tree's `ConfiguredTimer`s                |
| Accessory poses   | `neck_seek_pos`, `neck_level_pos`, `gripper_left_neutral`, `gripper_right_neutral`                                | `AccessoryCommander`                             |

Angles are declared in **degrees** with a `_deg` suffix and reach the blackboard
in **radians** under the name without it — the convention
`mecanumbot_ostensive_behaviour` established. Nothing converts an angle twice.

How a tunable reaches the code:

* behaviours register the keys in `__init__` and read them in `setup()`, through
  their package's `constant()` / `resolve()`. `setup()` runs depth-first in tree
  order and `ConstantParamsToBlackboard` is the first leaf, so the values are on
  the blackboard before anything else asks for them;
* a constructor argument left at `None` means "take the configured value", and
  passing a number still wins — the YAML is the default for every turn in the
  tree, not a ceiling on what one turn may be;
* the handful settled while the tree is still being *built* — `tick_period_ms`,
  `setup_timeout`, `recover_retries` — come from the file through
  `tree_common.build_params()` / `defaults.file_constant()`, because no
  blackboard exists yet. Timer durations avoid this with `ConfiguredTimer`,
  which looks its duration up when it starts.

Angles are the loader's business, not this package's: any key whose name ends in
`_deg` reaches the blackboard in radians under the name without it, so no list
of which keys are angles exists anywhere.

`mecanumbot_ostensive_behaviour` borrows the same scanning and turning behaviours
and declares the subset of the movement keys it uses in its own constants file,
under the same names.

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
