# mecanumbot_movement_behaviours

The movement half of the behaviour library: turning, approaching, driving a
route and searching it. Every tree in this repository is built out of these,
plus whatever its own condition adds.

No nodes and no launch files — it is a library. The ROS interfaces below are
created inside the behaviour classes, so a tree that uses one gets them.

| Item | Value |
| --- | --- |
| Nodes | None |
| Launch files | None |
| Depends on | `mecanumbot_bt_config`, `mecanumbot_msgs`, `rclpy`, `py_trees`, `geometry_msgs`, `std_msgs`, `action_msgs`, `nav2_msgs`, `numpy` |

## Modules

| Module | Contents |
| --- | --- |
| `geometry.py` | Pure geometry: angles, bearings, `signed_rotation`, `pose_to_goal`, `route_poses`, checkpoint lookups, `route_progress`. |
| `body_gaze.py` | Which way the head has to tilt to show the body the LiDAR expects: the neck model, the body profiles, the seats, and the choice among what is in view. Pure Python. |
| `look_down.py` | The look-down state: head lowered onto a face at the bottom of the frame, and always raised again. Pure Python. |
| `pacing.py` | When a look back falls due, how far a leg may be, and the `+1, -2, +2, -1` step pattern of the attention wiggle. Imports nothing — the decision logic on its own. |
| `ros_interfaces.py` | Topic names, QoS, pose/people/ball trackers, the Nav2 action navigators, velocity and neck commanders. |
| `keys.py` | `KeyMap`: what an experiment calls the things these behaviours read off the blackboard. |
| `defaults.py` | `MOVEMENT_DEFAULTS` — every tunable these behaviours used to hard-code — and the `Tunables` bound to them. |
| `targets.py` | What a `target_type` points at, and the head pose that goes with it. |
| `turning.py` | `SmoothTurner` plus the in-place turning behaviours. |
| `approach.py` | `Approach`, and the checks that go with having arrived. |
| `routes.py` | `FollowRoute` and the lost-human patrol over the same checkpoint list. |

## Behaviours

| Behaviour | Role |
| --- | --- |
| `Approach` | Navigates to a target through Nav2; `mode="exact"` drives to the point, `mode="fixed_distance"` steps the approach distance closer once, `mode="stepped"` keeps stepping (re-aimed at the human each time) until within `closeness_threshold` + `approach_arrive_margin`, at most `approach_max_steps` steps. |
| `FollowRoute` | Leads one leg of the route — several checkpoints in a single `NavigateThroughPoses` goal, cut short when the human stops following: out of sight for `sight_timeout`, or further than the following threshold, for `check_in_grace` seconds. A checkpoint nav2 has dropped from the goal counts as passed. |
| `TurnToward` | Rotates in place to face a `subject` / `target` / `start` / `checkpoint` / `patrol` / `last_checkpoint`, in a chosen direction (see below). |
| `GlanceBack` | The look over the shoulder: a slow full turn, set off towards the human's last known place and stopped by the first detection made during it. FAILURE is what starts the patrol. Looks down onto a face at the bottom of the frame, holding the turn meanwhile (`HeadLookDown`). |
| `RelativeTurnPattern` | Attention-getting wiggle: alternating turns that end on the starting heading, beginning in the direction of the last search turn. |
| `ScanSpin` | Spins in place looking for people, head lifted; `FindPeople` (spin until somebody is seen) and `Spin360` (one full scan) are configured subclasses. |
| `WaitForPerson` | Interrupt half of the lost-recovery parallel: waits with a lifted head, and records whether the person turned up ahead of or behind the robot. Looks down onto a face at the bottom of the frame (`HeadLookDown`), which also holds a scan running beside it; `look_down=False` leaves that out (the LED leading tree). |
| `ManageSearchCheckpoint` | Walks the patrol index along the route, reversing at either end; sets off towards where the human was last seen. |
| `CheckSubjectTargetSuccess` | SUCCESS when the subject is within the reached threshold of the target. |
| `CheckRobotHasBall` | SUCCESS while `/mecanumbot/has_object` is true. |
| `CheckRobotAtLastCheckpoint` | SUCCESS when the robot stands at the last checkpoint: within the larger of `checkpoint_reached_distance` and `route_stop_distance` of it, plus `route_end_margin`. The route index must also read "last" unless `by_index=False`; on its own the index only says where the robot is heading. |

**How close a goal gets.** `Approach` never aims at its target itself, it aims
short of it — at the closeness threshold for a human and `route_stop_distance`
for a place on the route. The two are separate numbers because they answer
different questions. The human one is measured from `base_link` to the *detected
person centre*, and the footprint reaches 0.2875 m ahead of `base_link`, so the
bumper stops that much nearer than the number says: at `0.75` it ends 0.46 m
from the point the detector reported, which is about 0.25 m of air in front of a
real person. Set it under about 0.5 m and the robot touches them, however
correctly the threshold is applied.

## Naming: the `KeyMap`

These behaviours are shared, but the blackboard keys they read are spelled by
whichever experiment loaded the constants file — the leading conditions call
their route `Dog_checkpoints`, because that is the name in their YAML and the
loader writes what the file says. Hard-coding that spelling in code every
experiment uses would put one experiment's vocabulary in the shared library.

So the spelling is a `KeyMap` of fixed *fields* (`checkpoints`,
`current_checkpoint`, `max_checkpoint`, `patrol_*`, `start_position`,
`target_position`, `closeness_threshold`, `approach_distance`,
`reached_threshold`, `following_threshold`, `visibility_timeout`,
`checkpoints_since`, `last_check_in`, `spin_sign`) to blackboard names. Every
behaviour that reads one takes `keys=` at construction, and a package binds its
own once on a subclass rather than passing it at every call site:

```python
LEADING_KEYS = DEFAULT_KEYS.derive(
    checkpoints="Dog_checkpoints",
    current_checkpoint="Dog_current_checkpoint",
    max_checkpoint="Dog_max_checkpoint",
    following_threshold="Dog_following_max_threshold",
)

class FollowRoute(routes.FollowRoute):
    KEYS = LEADING_KEYS
```

`mecanumbot_leading_behaviour/behaviours/keys.py` spells `LEADING_KEYS`, and
`mecanumbot_leading_behaviour/behaviours/route_behaviours.py` binds it onto
subclasses for the leading experiments; that is what its trees import. Deriving with a name
that is not a field is an error rather than a key nothing ever reads.

## Navigating by action

Every drive is a Nav2 action goal. A pose published on `/goal_pose` is a message
shouted into the dark: Nav2 never says which goal id it became, so the outcome
had to be guessed from the newest entry of the status array, and there was no
way to take the goal back. `Nav2PoseNavigator` and `Nav2RouteNavigator` wrap
`NavigateToPose` and `NavigateThroughPoses`, which give all three — the goal
handle identifies the goal, the result says how it ended, and `cancel()` stops
it. Everything is asynchronous: callbacks land between ticks on the same
executor, a behaviour only reads `status()` during its tick, and nothing waits.

Cancelling is what makes the leading seamless. A drive is cancelled the moment
the behaviour that owns it stops, so the turn that comes next has `/cmd_vel` to
itself instead of waiting `turn_nav2_wait` seconds for Nav2 to notice it is
finished, and `FollowRoute` can stop a leg mid-way when the human drops behind.

`Nav2GoalMonitor` stays for `mecanumbot_ostensive_behaviour`, which still
publishes goal poses, and for `busy()` — "is Nav2 driving right now", which a
turn asks before it takes `/cmd_vel`, and which is a question about the status
topics rather than about a goal of our own.

## Turning: smoothness and direction

In-place rotations do **not** go through Nav2. `SmoothTurner` drives `/cmd_vel`
with a profile that ramps up under an acceleration limit and eases out
proportionally to the angle left, and it tracks progress on AMCL yaw,
dead-reckoning from the commanded speed between pose updates. The profile comes
from the constants YAML — `turn_max_speed`, `turn_accel`, `turn_decel_gain`,
`turn_min_speed`, `turn_tolerance_deg` (keep them inside the Nav2 controller's
`max_vel_theta` / `max_angular_accel`) — and a call site may still override any
of them for one behaviour by passing the `SmoothTurner` keyword.

Owning the rotation is what makes the direction selectable. `TurnToward` takes
`direction`:

| `direction` | Meaning |
| --- | --- |
| `"shortest"` | Short way round. Default for human targets, and asked for explicitly on the dog tree's turn to the next checkpoint. |
| `"unwind"` | Opposite to the last search turn — back the way the robot came. Default for route targets. |
| `"repeat"` | Same handedness as the last search turn. |

Any turn or scan that looks for a human stores its handedness in the `spin_sign`
key (`search_spin_sign` by default; `+1` counterclockwise, `-1` clockwise), so a
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

## Head (neck) poses

`n_pos` is the neck-mounted camera tilt (`2.0 … 8.6`, larger looks further up).
Two named poses, set from the YAML and held on `AccessoryCommander` — they are
class state, because there is only one head, and the parameter loader hands them
over once through the `defaults.configure_accessories` load hook before any tree
is ticked:

| YAML key | Value | Used when |
| --- | --- | --- |
| `neck_seek_pos` | `7.0` | The robot is looking for or at a human: reads as seeking contact, and gives YOLO26n-pose a full-body view instead of a pair of knees, which it often misses. |
| `neck_level_pos` | `6.0` | The robot is driving its route or pointing at the target. |

**Looking down onto a low face** (`look_down.py`, `ros_interfaces.HeadLookDown`).
A lifted head puts somebody sitting low -- on a bean bag -- at the bottom edge of
the frame with only their face in shot, and the detection gate wants a torso. The
camera detector publishes `cam_people_detections/low_head` when it sees a face
there; `WaitForPerson` and `GlanceBack` then drop the neck by `look_down_step`
(`0.5`, ~15 deg) for `look_down_hold` (`3.0` s) and put it back exactly where it
was, and ignore the cue for `look_down_cooldown` (`4.0` s) so a face the gate
never accepts cannot keep the head low. A scan (`ScanSpin`, `GlanceBack`) holds
its turn while the head is down. Ending the search always raises the head.

**Following the body the LiDAR expects** (`body_gaze.py`, `ros_interfaces.BodyGaze`).
No single tilt shows this camera a person. It sits 0.23 m off the floor behind a
lens that sees 30° top to bottom, so at `6.5` it sees the jeans of somebody
standing 0.7 m away and only the face of somebody on a bean bag, and at `6.0` it
loses every standing head inside four metres. What decides the right tilt is how
far away the body is and how tall it should be, and the robot knows both before
the camera has seen anything:

- **the LiDAR says how far.** The fusion node publishes `lidar_candidates` —
  anything the scan sees that is the size of a person and not part of the mapped
  room — and `people_fusion` is a better answer still once somebody is tracked.
  Nearer means further up: `7.9` for a standing person at 0.7 m, `6.8` at 2 m,
  `6.45` at 3.5 m.
- **a seat says how tall.** A body is expected to be a standing one unless it
  is on one of the `seats` the constants file lists for the room, where it is
  looked for at the height of its `body_profiles` entry instead: `6.4` for the
  bean bag at 1.1 m, where a standing body would have had `7.4`. A seat is looked
  at even with no LiDAR return on it, because that is the case the LiDAR is bad
  at — a person in a chair against the wall is one return with the wall.

Each thing in view asks for a band of heights to be in frame, hips to the top of
the head. The head takes the tilt that centres the first one's band — a tracked
person before a candidate, nearer before further — moved as little as it must be
to keep the others in frame too. A body taller than the frame at its range keeps
its top. `body_gaze_deadband`, `_min_interval` and `_release_delay` keep it from
following range noise or nodding when a scan drops out.

It is **opt-in per tree**, by listing `defaults.start_body_gaze` among the
constants loader's hooks; only the dog-inspired leading tree does. It runs from
a timer on the tree's node rather than from any one behaviour, and it moves the
head **only while the head is in `HEAD_SEEK`** — which is what `look("seek")`
now means there: not one pose but "the gaze has the head", with `neck_seek_pos`
where it rests when nothing is in view. Three things take the head away from it,
and none of them needs to know it exists: `look("level")`; any direct
`send()`, which is how the gesture scripts move the neck; and the look-down
while it is down. The next `look("seek")` hands it back.

| YAML key | Default | |
| --- | --- | --- |
| `body_gaze_enabled` | `true` | Only read by a tree that installs the hook; `false` goes back to one fixed seeking pose. |
| `body_gaze_period` | `0.2` | Seconds between two looks at what is in view. |
| `body_gaze_max_range` | `4.5` | Metres beyond which a body is not tilted for. |
| `body_gaze_view_margin_deg` | `12` | How far beyond the edge of the picture a target is still attended to, so the head is ready as a turn brings it into shot. |
| `body_gaze_frame_margin_deg` | `3` | Kept clear above and below a body in the frame. |
| `body_gaze_min_pos` / `_max_pos` | `5.5` / `8.2` | The neck positions the gaze may command. |
| `body_gaze_deadband` / `_min_interval` | `0.1` / `0.4` | Smallest change worth a command (`0.1` is ~3°) and the shortest time between two [s]. |
| `body_gaze_release_delay` | `1.0` | Seconds with nothing in view before the head returns to `neck_seek_pos`. |
| `body_gaze_candidate_timeout` | `0.5` | How old the `lidar_candidates` list may be [s]. |
| `body_profiles` | standing `0.85–1.80`, chair `0.40–1.30`, bean_bag `0.10–0.75` | `{'body': kind, 'low': .., 'high': ..}` — the heights the detector needs of each kind of body. |
| `seats` | none | `{'seat': kind, 'x': .., 'y': .., 'radius': ..}` in map coordinates, optionally `'low'` / `'high'`. Lower-case `x`/`y`: `X`/`Y` is a route checkpoint. They belong to a room, so a constants file lists its own. |

The neck model itself (`body_gaze.NeckModel`: 4.7° up at `6.0`, 29° per unit of
`n_pos`, the 51° × 30° lens) is not in the YAML. It is the calibration
`mecanumbot_sensorprocess_smart` places a ball with and `mecanumbot_deep3r`
reconstructs with — the same servo, so change all three or none.

`gripper_left_neutral` / `gripper_right_neutral` (`6.83` / `3.36`) are the
gripper positions any command that does not name its own uses; the gesture
sequences move between the same values.

Behaviours pick the pose from their target type (`head="seek"` / `"level"`), so
the head stays lifted for the whole seeking phase rather than flicking up for a
moment. `head=None` leaves the neck untouched — the control tree passes that so its
comparison condition carries no head movement at all. The LED tree carries no head
gestures either, but since 2026-10-05 it runs the body gaze while it looks for the
person.

## Tunables

`defaults.MOVEMENT_DEFAULTS` is every number these behaviours used to carry as a
constructor default or a module constant, and its value is that number. They are
optional in a constants file: a YAML written before a key existed keeps what the
code used to hard-code. A behaviour reads one through `constant()`, or through
`resolve()` where a call site may still override it; `mecanumbot_bt_config`'s
README explains the mechanism, and each experiment package's README lists the
values its own files set.

Angles are stored here in radians and declared in a YAML in degrees with a
`_deg` suffix — the loader converts any key spelled that way, so no list of
which tunables are angles exists anywhere.

## ROS interfaces

Created inside the behaviour classes, so a tree gets them by using the
behaviours.

### Publishers

| Topic | Type | Function |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Every in-place rotation, profiled. |
| `/cmd_accessory_pos` | `mecanumbot_msgs/msg/AccessMotorCmd` | Neck (camera tilt) and gripper commands. |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | Created by `Nav2GoalMonitor` (every in-place turn builds one), unused here — it stays for the ostensive package's pointing goals. |

### Action clients

| Action | Type | Used by |
| --- | --- | --- |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | `Approach` — one place to drive to; also `FollowRoute`'s fallback when nav2 gives up on a leg, one checkpoint at a time. |
| `/navigate_through_poses` | `nav2_msgs/action/NavigateThroughPoses` | `FollowRoute` — a leg of route checkpoints in one goal. |

### Subscribers

| Topic | Type | Processing |
| --- | --- | --- |
| `/amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Robot pose for goal generation and checkpoint selection. `RELIABLE` + `TRANSIENT_LOCAL` QoS, to match AMCL. |
| `/mecanumbot/people_fusion` | `geometry_msgs/msg/PoseArray` | Fused people detections — finding, selecting and approaching the subject. |
| `/mecanumbot/subject_pose` | `geometry_msgs/msg/PoseStamped` | Tracked subject pose. Read together with the fused detections by `FollowedSubjectTracker`, whichever is fresher. |
| `/mecanumbot/has_object` | `std_msgs/msg/Bool` | Ball-handover trigger read by `CheckRobotHasBall`. |
| `/mecanumbot/lidar_candidates` | `geometry_msgs/msg/PoseArray` | Where the LiDAR sees something person-sized the map does not explain. Read by `BodyGaze`, in a tree that starts one. |
| `/mecanumbot/cam_people_detections/low_head` | `std_msgs/msg/Float32` | A face at the bottom of the frame; the look-down's cue. |
| `/navigate_to_pose/_action/status` | `action_msgs/msg/GoalStatusArray` | `Nav2GoalMonitor`: `busy()` — what a turn waits for before it takes `/cmd_vel` — and the outcome of a published `/goal_pose` for the ostensive package. |
| `/navigate_through_poses/_action/status` | `action_msgs/msg/GoalStatusArray` | The same question for a waypoint run. |

## Tests

`test/test_pacing.py` — 22 tests over the look-back pacing rules: when a look
back falls due, how long a leg may be, the waypoint poses a leg is sent as, and
the wiggle's sweep pattern. `pacing.py`
imports nothing, so most of it runs against a bare interpreter; the three
`route_poses` tests need `geometry_msgs` and skip without it.

`test/test_body_gaze.py` — 40 tests over the gaze: nearer means further up, a
seat means lower, a seat is looked at with no LiDAR return on it, a tracked
person comes before a nearer candidate, and the head does not follow range noise.
The scenes are the ones in the bags of 2026-09-30. Pure Python; the two cases
that read the shipped leading constants skip without `mecanumbot_bt_config`.

`test/test_head_modes.py` — 13 tests over who has the neck: the gaze only
steers a seeking head, a gesture script or a route turn takes it away, the next
seeking behaviour gets it back, and the look-down holds the gaze off. The node
is a fake; needs `rclpy` for the message types only.

```bash
cd src/mecanumbot_behaviours/mecanumbot_movement_behaviours
PYTHONPATH=.:$PYTHONPATH python3 -m pytest test/test_pacing.py -v
colcon test --packages-select mecanumbot_movement_behaviours   # from the workspace root
```
