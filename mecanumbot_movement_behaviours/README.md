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
| Depends on | `mecanumbot_bt_config`, `mecanumbot_msgs`, `rclpy`, `py_trees`, `geometry_msgs`, `action_msgs`, `nav2_msgs` |

## Modules

| Module | Contents |
| --- | --- |
| `geometry.py` | Pure geometry: angles, bearings, `signed_rotation`, `pose_to_goal`, `route_poses`, checkpoint lookups, `route_progress`. |
| `pacing.py` | When a look back falls due and how far a leg may be. Imports nothing — the decision logic on its own. |
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
| `Approach` | Navigates to a target through Nav2; `mode="exact"` drives to the point, `mode="fixed_distance"` steps the approach distance closer. |
| `FollowRoute` | Leads one leg of the route — several checkpoints in a single `NavigateThroughPoses` goal, cut short when the human stops following. |
| `TurnToward` | Rotates in place to face a `subject` / `target` / `start` / `checkpoint` / `patrol` / `last_checkpoint`, in a chosen direction (see below). |
| `GlanceBack` | The look over the shoulder: a slow turn onto the human's last known place, then a slow sweep around it. FAILURE is what starts the patrol. |
| `RelativeTurnPattern` | Attention-getting wiggle: alternating turns that end on the starting heading, beginning in the direction of the last search turn. |
| `ScanSpin` | Spins in place looking for people, head lifted; `FindPeople` (spin until somebody is seen) and `Spin360` (one full scan) are configured subclasses. |
| `WaitForPerson` | Interrupt half of the lost-recovery parallel: waits with a lifted head, and records whether the person turned up ahead of or behind the robot. |
| `ManageSearchCheckpoint` | Walks the patrol index along the route, reversing at either end. |
| `CheckSubjectTargetSuccess` | SUCCESS when the subject is within the reached threshold of the target. |
| `CheckRobotHasBall` | SUCCESS while `/mecanumbot/has_object` is true. |
| `CheckRobotAtLastCheckpoint` | SUCCESS when the current checkpoint index has reached the last one. |

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

`mecanumbot_leading_behaviour/behaviours/route_behaviours.py` is that binding
for the leading experiments, and is what its trees import. Deriving with a name
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

`gripper_left_neutral` / `gripper_right_neutral` (`6.83` / `3.36`) are the
gripper positions any command that does not name its own uses; the gesture
sequences move between the same values.

Behaviours pick the pose from their target type (`head="seek"` / `"level"`), so
the head stays lifted for the whole seeking phase rather than flicking up for a
moment. `head=None` leaves the neck untouched — the LED and control trees pass
that so their comparison conditions carry no head gestures at all.

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
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | Created by `Nav2GoalMonitor`, unused here — it stays for the ostensive package's pointing goals. |

### Action clients

| Action | Type | Used by |
| --- | --- | --- |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | `Approach` — one place to drive to. |
| `/navigate_through_poses` | `nav2_msgs/action/NavigateThroughPoses` | `FollowRoute` — a leg of route checkpoints in one goal. |

### Subscribers

| Topic | Type | Processing |
| --- | --- | --- |
| `/amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Robot pose for goal generation and checkpoint selection. `RELIABLE` + `TRANSIENT_LOCAL` QoS, to match AMCL. |
| `/mecanumbot/people_fusion` | `geometry_msgs/msg/PoseArray` | Fused people detections — finding, selecting and approaching the subject. |
| `/mecanumbot/subject_pose` | `geometry_msgs/msg/PoseStamped` | Tracked subject pose. Read together with the fused detections by `FollowedSubjectTracker`, whichever is fresher. |
| `/mecanumbot/has_object` | `std_msgs/msg/Bool` | Ball-handover trigger read by `CheckRobotHasBall`. |
| `/navigate_to_pose/_action/status` | `action_msgs/msg/GoalStatusArray` | Only for `Nav2GoalMonitor.busy()` — what a turn waits for before it takes `/cmd_vel`. |
| `/navigate_through_poses/_action/status` | `action_msgs/msg/GoalStatusArray` | The same question for a waypoint run. |

## Tests

`test/test_pacing.py` covers the look-back pacing rules — when a look back falls
due, how long a leg may be, and the waypoint poses a leg is sent as. `pacing.py`
imports nothing, so most of it runs against a bare interpreter; the three
`route_poses` tests need `geometry_msgs` and skip without it.

```bash
cd src/mecanumbot_behaviours/mecanumbot_movement_behaviours
PYTHONPATH=.:$PYTHONPATH python3 -m pytest test/test_pacing.py -v
colcon test --packages-select mecanumbot_movement_behaviours   # from the workspace root
```
