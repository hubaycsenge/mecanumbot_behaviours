# mecanumbot_ostensive_behaviour

`py_trees_ros` behaviour-tree package for the Mecanumbot **ostensive** condition.

A person signals for the robot's attention, the robot commits to them, shows that
it has, reads a pointing gesture off their body, and drives where they indicated.
One pass of the tree is one complete ostensive exchange; the tree is that exchange
repeated for ever.

Perception is **not** done here. Every keypoint and every metric position comes
from `mecanumbot_sensorprocess_smart`, which already runs a pose detector on the
camera and fuses it with the LiDAR. This package subscribes to the results and
decides what they mean.

## The exchange

```text
   BeAddressed ──► FaceAddressee ──► Acknowledge ──► ReadCueWhileFocused ──► GoWherePointed
  (wave/hand up)     (turn on the      (nod)         (point held 1 s,          (nav2 goal
   becomes the        image offset)                   while re-centring)        2 m along
   addressee)                                                                   the cue)
        ▲                                                                            │
        └──────────────────── release the addressee, start again ◄───────────────────┘
```

Every step after the first fails if the addressee walks off, and all failures land
in the same place: `AbandonExchange` drops the lock and the loop begins again. An
exchange resumed halfway through, against somebody who has already left, is worse
than one begun from scratch.

## Relationship to the original requirements

The brief was written for a standalone node that owned its own perception. It is
satisfied here, but two of the six points are met by the pipeline rather than by
this package, which is the deliberate change:

| Requirement | Where it is met |
| --- | --- |
| Take camera input from `/camera/image_raw` | Upstream. `mecanumbot_cam_detect_people` consumes the camera (by default `camera/image_raw/compressed`) and publishes `cam_people_detections`. This package never touches an image. |
| Detect people with pose estimates | Upstream. YOLO26n-pose (or the DeepStream equivalent) produces the COCO-17 keypoints, already gated for keypoint evidence, hysteresis and temporal confirmation. |
| Detect an attention signal | `behaviours/gestures.py` — a raised hand, or a wave (a raised hand that keeps reversing direction). |
| Choose a target person after the signal | `behaviours/attention.py` + `behaviours/target_lock.py` — every person in frame is watched, the first to hold a signal becomes the addressee, and the lock is what every later behaviour is about. |
| Decipher direction cues from movement | `behaviours/gestures.py` (`pointing_cue`) and `behaviours/cue_geometry.py` (image azimuth → map bearing). |
| Keep them in focus using image position + `/amcl_pose`, publish to `/goal_pose` | `behaviours/focus.py` turns on the image offset; `behaviours/pointing.py` builds the goal from the AMCL pose and publishes it through nav2. |

Running the detector once, in the package built for it, is what makes the two
camera back ends interchangeable, keeps a second copy of a pose model off the
Orin Nano's GPU, and lets the behaviour layer use the fused metric positions that
the camera alone cannot provide.

### Differences from `src/mecanumbot_ostensive`

That package — the earlier standalone prototype — is a single node with its own
MediaPipe pipeline and a three-state FSM. It is still in the workspace and still
builds; nothing here touches it. This package replaces it, and differs in the
ways that matter:

| | `mecanumbot_ostensive` (prototype) | this package |
| --- | --- | --- |
| Control structure | Hand-rolled FSM inside one node | `py_trees` tree, one behaviour per decision |
| Perception | Own MediaPipe Pose, own `cv_bridge`, own camera subscription | `cam_people_detections` + `people_fusion` |
| People in frame | One (MediaPipe Pose is single-person) | All of them, until one signals |
| Attention signal | Static raised hand | Raised hand or wave, with a dwell |
| Gesture thresholds | Fractions of the image and of shoulder width | All ratios of body scale, with a side-on fallback |
| Where the pointing ray starts | The robot | The person who pointed |
| Person's metric position | Never known; goals were placed relative to the robot | From `people_fusion`, matched by bearing |
| Goal handling | Published and forgotten, on a cooldown | Monitored through nav2, resent when nav2 drops it |
| Turning | A goal pose per frame, fighting the pointed goal for the publish slot | Profiled `/cmd_vel` rotation with a deadband, off nav2 |
| Acknowledgement | None | A nod, so the person can see their bid landed |

## Executable

| Executable | Source file | ROS node name |
| --- | --- | --- |
| `ostensive_bt_node` | `tree_nodes/ostensive_tree.py` | `ostensive_bt_node` |

Unlike the leading package, the node name is the package's own — the two trees can
run side by side and `ros2 node list` says which is which.

## Node interfaces

The ROS interfaces are created inside the behaviour classes; most of them are the
leading package's helper objects, reused rather than reimplemented.

### Publishers

| Topic | Data type | Function |
| --- | --- | --- |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | The cued goal — the only thing that drives the robot anywhere. |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | In-place rotation only, to keep the addressee centred. Profiled by `SmoothTurner`. |
| `/cmd_accessory_pos` | `mecanumbot_msgs/msg/AccessMotorCmd` | The camera neck: the lifted seeking pose, and the acknowledgement nod. |

### Subscribers

| Topic | Data type | Processing |
| --- | --- | --- |
| `/mecanumbot/cam_people_detections` | `mecanumbot_msgs/msg/CamPersonDetectionArray` | COCO-17 keypoints per person. Everything about gestures is read from here. |
| `/mecanumbot/people_fusion` | `geometry_msgs/msg/PoseArray` | Fused map-frame positions; matched to the addressee by bearing, and used by `LookAround` to know when somebody is in view. |
| `/amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Robot pose, for turning and for anchoring goals. `RELIABLE` + `TRANSIENT_LOCAL` to match AMCL. |
| `/navigate_to_pose/_action/status` | `action_msgs/msg/GoalStatusArray` | Goal outcome, matched by UUID; a dropped goal is resent. |

### Services

None, in either direction. The ostensive condition signals with movement, so it
never calls `set_led_status`.

## Behaviour library

Split so that everything decidable without ROS is decidable without ROS — the
three pure modules hold all the perception logic and are unit-tested with plain
tuples, no graph and no camera.

| Module | Pure? | Contents |
| --- | --- | --- |
| `keypoints.py` | yes | COCO-17 access, the two "joint not found" conventions, torso, body scale, the image mirror. |
| `gestures.py` | yes | `raised_hand`, `WaveTracker`, `pointing_cue`, `azimuth_from_lateral_ratio`. |
| `cue_geometry.py` | yes | Image column → bearing, cue azimuth → map bearing, goal placement, both association functions. |
| `ros_interfaces.py` | no | `CamDetectionTracker`, `PersonObservation`, and the re-exports of the leading package's helpers. |
| `target_lock.py` | no | `TargetLock` (the addressee) and `TargetFollower` (the per-frame refresh). |
| `blackboard_managers.py` | no | `OstensiveParamsToBlackboard`, `ClearTargetLock`. |
| `attention.py` | no | `WaitForAttentionSignal`. |
| `focus.py` | no | `KeepTargetInFocus`, `FaceTarget`. |
| `pointing.py` | no | `DirectionCue`, `DecodeDirectionCue`, `FollowDirectionCue`. |
| `acknowledge.py` | no | `NodAcknowledge`. |

| Behaviour | Role |
| --- | --- |
| `OstensiveParamsToBlackboard` | Loads the YAML constants and clears the interaction state. **Must be the first leaf in the tree** — `setup()` runs depth-first in tree order and every other behaviour reads its constants during its own `setup()`. |
| `WaitForAttentionSignal` | Watches everybody in frame; SUCCESS when one holds a signal for `attention_dwell`, FAILURE after `attention_timeout`. |
| `FaceTarget` | Turns until the addressee is inside the focus deadband, then SUCCESS. |
| `NodAcknowledge` | Steps the neck through `ack_neck_seq` / `ack_neck_times`. |
| `KeepTargetInFocus` | Re-centres the addressee, for ever; FAILURE when they are lost. Runs in a parallel next to the cue decoder. |
| `DecodeDirectionCue` | SUCCESS once a pointing gesture settles; writes the map-frame direction. |
| `FollowDirectionCue` | Sends the cued goal through nav2 and follows it to completion. |
| `ClearTargetLock` | Releases the addressee. Used both to end an exchange and to absorb a failed one. |

Reused unchanged from `mecanumbot_movement_behaviours`: `InPlaceTurn` and
`SmoothTurner` (which `KeepTargetInFocus` inherits from), `FindPeople`,
`RobotPoseTracker`, `PeopleTracker`, `Nav2GoalMonitor`, `AccessoryCommander` and
`quaternion_from_yaw`. The constants loader is `mecanumbot_bt_config`'s.

## Sign conventions

These are the part that produces a robot which confidently drives the wrong way
rather than an error anybody notices, so they are pinned by tests in
`test/test_cue_geometry.py`.

**Is the image mirrored?** This is the first question, because it is upstream of
every other convention here. A mirrored image reflects *everything* read off
image x at once — which way the robot turns to keep somebody centred, which arm
the pose model calls their left, which way that arm points, and so where the
goal is placed — and the result is a robot that confidently drives to the
reflection of the place it was sent to, with no other symptom.

`mirror_image_x` (default **true**) is that switch. Set, `keypoints.mirror_joints`
reflects each pose once, in the subscription callback, before anything has been
measured off it: image columns become `1 - x`, and the `left_*`/`right_*` labels
are swapped with them, because a pose model naming joints by how the body looks
calls a mirrored person's left arm their right one. Image y is untouched — a
horizontal mirror moves nothing up or down, so every gesture that reads a wrist
against a shoulder or a hip is unaffected.

Doing it once, at the edge, is deliberate: the mirror is a property of the
camera, not a sign for each behaviour to remember, and the conventions below are
then true of every pose the package handles.

To check which way it should be set, stand clearly off to the robot's left and
watch `KeepTargetInFocus`: with `mirror_image_x` right, it turns *towards* you.
`OstensiveParamsToBlackboard` logs the setting on startup for the same reason.

**Image x to bearing.** Normalized image x runs 0 at the left edge to 1 at the
right; robot yaw is counter-clockwise-positive. The left of the image is
therefore the *positive* side:

```text
bearing_offset = (0.5 - x) * hfov
```

This matches `cam_to_angle` in both camera detectors, which invert x for the same
reason — and note that they do *not* apply `mirror_image_x`, so a mirrored camera
also mirrors `bound_angle_min`/`bound_angle_max` and, through
`mecanumbot_locate_detections`, the camera's contribution to `people_fusion`.
This package does not use those fields (see below), but anything else that does
is mirrored with them.

> The detectors' own `bound_angle_min` / `bound_angle_max` fields are **not** used.
> They disagree between back ends: the Ultralytics node adds the robot's AMCL yaw
> and so reports absolute map angles, while the DeepStream node reports angles
> relative to the robot. Computing the bearing here from image x and `/amcl_pose`
> is the same arithmetic and behaves identically whichever detector is running.

**Where a pointing arm points.** `pointing_cue` returns an azimuth measured from
the direction the person faces, positive towards *their* left. Assuming they are
facing the robot, their facing direction is the bearing from them to the robot,
so:

```text
cue_bearing = bearing(person → robot) + azimuth
goal        = person + cue_distance * (cos, sin)(cue_bearing)
```

Worked example, and the test that holds it: the robot sits at the origin looking
along +x, the person stands two metres ahead facing back at it, and points to
their own left. Facing each other, their left is the robot's right, so the goal
lands at `(2, -2)` and the robot arrives facing −90°.

The ray is anchored at the **person**, not the robot. "Over there" is said from
where the speaker stands, and with the robot typically a metre or two off to one
side, a ray drawn from the robot lands somewhere nobody indicated.

## Reading the gestures

Every threshold is a ratio against the person's own **body scale** — shoulder
width, or shoulder-to-hip height for somebody turned far enough side-on that
their shoulders foreshorten — never a number of pixels. The same gesture
therefore reads the same at two metres and at five.

**Attention.** A raised hand is a wrist held `wrist_above_shoulder_margin` body
scales above its shoulder. A wave is a raised hand whose sideways position keeps
reversing: turning points are counted over `wave_window` seconds with an
amplitude hysteresis, so a held-up arm swaying slightly counts as zero reversals
rather than one per frame. `attention_signal_mode` picks which of the two is
accepted; `any` takes either and reports a wave as a wave.

**Pointing.** Three gates, all of which have to pass:

1. `cue_min_extension` — the wrist is far enough from the shoulder that the arm is
   held out rather than bent;
2. the wrist is above the hip line — this is what separates a pointing arm from
   one hanging down, which clears the extension gate on its own, being just as
   long;
3. `cue_min_lateral` — the arm reaches *sideways*.

The third gate is the interesting one. Monocular keypoints cannot recover the
depth component of a pointing arm: an arm aimed straight at the camera and one
aimed straight away from it project to the same short vector. Rather than mapping
that to "straight ahead" and driving somewhere nobody indicated, the cue is
refused. A cue has to commit to a side to count.

The azimuth comes from how far the wrist reaches sideways against
`cue_full_extension`, treated as a **sine** rather than as a linear fraction, so a
half-extended arm reads as 30° rather than 45° — which is what an arm swinging on
its shoulder actually does.

## Holding on to one person

The camera detector publishes no track IDs, so the addressee's identity is
maintained two ways at once:

* **in the image**, by how far their torso moved between frames
  (`target_max_image_jump`). The torso is used rather than the mean of all joints
  because an outstretched arm drags the mean sideways, and a person who points
  must not read as having stepped aside;
* **in the map**, by matching the bearing read off the image against the
  `people_fusion` poses (`target_bearing_tolerance`). A pose further off than the
  tolerance is not matched at all, which keeps a second person standing elsewhere
  in the room from being adopted when the addressee's own fused pose drops out.

When no fused pose matches, the previous map position is kept rather than
cleared. The fused array drops stationary tracks regularly — the LiDAR detector
suppresses them deliberately — and a person standing still to point at something
is exactly the case that produces the dropout.

The refresh is idempotent per camera frame. Several behaviours tick against the
same lock in one pass of the tree, and the frame stamp is what stops them
re-running the association for each of them, or disagreeing about the result.

## Keeping the addressee in frame

`KeepTargetInFocus` turns on the *image* offset, not on the map bearing: that
works without a fused pose, and what has to stay true is that the person remains
inside the frame the gestures are read from.

Two things keep it from hunting: a deadband (`focus_tolerance`), so the robot
ignores somebody shifting their weight; and acting at most once per camera frame,
never on a frame captured while the robot was still turning — otherwise it would
command a second turn from a pre-turn image and swing past centre.

As with every in-place rotation on this robot, the turn bypasses nav2 and so is
not supervised by the local costmap.

## Configuration

`config/ostensive_setting_constants.yaml` (AI_dept) and
`config/Eto_ostensive_setting_constants.yaml` (LED_exp), both nested under
`ostensive_bt_node: ros__parameters:`. The two differ only in the three values
that depend on room size; the gestures are identical, being ratios of the person
rather than of the space.

Angles are written in degrees and land on the blackboard in radians, under the
name without the `_deg` suffix. No behaviour converts an angle twice.

| Group | Parameter | Default | Meaning |
| --- | --- | --- | --- |
| Camera | `camera_hfov_deg` | `60.0` | Must match `camera_params.camera_fov` of the running detector. Getting it wrong scales every turn towards a person. |
| | `mirror_image_x` | `true` | The camera shows a mirrored view of the room, so every pose is reflected on arrival. Set the wrong way, the robot turns away from the addressee and drives to the mirror image of where they pointed. Optional; omitting it keeps the default. |
| | `detection_timeout` | `1.0` | How stale a camera frame may be and still be acted on. |
| Attention | `attention_signal_mode` | `any` | `raised_hand`, `wave` or `any`. |
| | `attention_dwell` | `1.0` | How long the signal is held before the robot commits. |
| | `attention_timeout` | `30.0` | Give up watching and look elsewhere. |
| | `wrist_above_shoulder_margin` | `0.15` | Raise height, in body scales. |
| | `wave_window` | `1.5` | Seconds of history the reversals are counted over. |
| | `wave_min_amplitude` | `0.25` | Excursion below this is noise, in body scales. |
| | `wave_min_reversals` | `2` | Turning points needed inside the window. |
| Lock | `target_max_image_jump` | `0.25` | Frame-to-frame torso step still counted as the same person. |
| | `target_bearing_tolerance_deg` | `20.0` | How closely a fused pose must agree with the image bearing. |
| | `track_loss_timeout` | `2.0` | How long the addressee may be missing before the exchange is abandoned. |
| Focus | `focus_tolerance_deg` | `8.0` | Deadband; smaller drift is ignored. |
| | `focus_turn_speed` | `0.4` | Rotation speed [rad/s] for re-centring. Overrides `turn_max_speed` for these turns. |
| | `focus_timeout` | `20.0` | How long `FaceTarget` gets to centre the addressee. Only the one-shot turn is bounded; the version beside the cue decoder ends with its parallel. |
| Cue | `cue_min_extension` | `1.1` | Wrist-to-shoulder distance, in body scales, for the arm to count as held out. |
| | `cue_full_extension` | `1.5` | Sideways reach of a fully outstretched arm; sets the angle scale. |
| | `cue_min_lateral` | `0.45` | Minimum sideways reach for the cue to be believed. |
| | `cue_dwell` | `1.0` | How long the arm must hold still. |
| | `cue_stability_deg` | `20.0` | How much the azimuth may wander inside the dwell. |
| | `cue_timeout` | `20.0` | Give up on the exchange if no cue settles. |
| Goal | `cue_distance` | `2.0` (`1.5` Eto) | How far along the cue the goal is placed, from the person. |
| | `cue_goal_timeout` | `60.0` (`45.0` Eto) | How long nav2 gets to deliver the robot. |
| Nod | `ack_neck_seq` | `[7.6, 6.2, 7.6, 7.0]` | Neck tilt positions (`2.0 … 8.6`, larger looks up). The last entry is where the head is left, and the loader warns when it is not `neck_seek_pos`. |
| | `ack_neck_times` | `[0.35, 0.35, 0.35, 0.3]` | How long each is held. |

### Borrowed tunables

This tree uses `mecanumbot_movement_behaviours`' `FindPeople` to look around and
its `InPlaceTurn`/`SmoothTurner` to re-centre, and those read their constants off
the blackboard under fixed names. The constants files declare that subset under
exactly those names, so the ostensive condition's turns are described in the
ostensive file rather than inherited silently. Every one is optional — omit it
and that package's own default applies.

| Group | Parameter | Default | Meaning |
| --- | --- | --- | --- |
| Tree runtime | `tick_period_ms`, `setup_timeout` | `100.0`, `15.0` | Read from the file before the first tick, so they cannot come off the blackboard. |
| Look around | `sight_timeout` | `1.0` | How fresh a `people_fusion` detection has to be. (`detection_timeout` is the camera's equivalent.) |
| | `scan_spin_speed`, `scan_timeout` | `0.5`, `10.0` | The `FindPeople` spin when nobody is there to be addressed. |
| Rotation profile | `turn_max_speed`, `turn_accel`, `turn_decel_gain`, `turn_min_speed`, `turn_tolerance_deg`, `facing_epsilon_deg` | `0.6`, `0.8`, `1.6`, `0.12`, `3.0`, `1.5` | The `SmoothTurner` profile. `focus_turn_speed` replaces `turn_max_speed` for the re-centring turns. |
| | `turn_timeout`, `turn_nav2_wait` | `20.0`, `2.0` | Bound the look-around and how long a turn waits for nav2 to release `/cmd_vel`. |
| Accessories | `neck_seek_pos`, `neck_level_pos`, `gripper_left_neutral`, `gripper_right_neutral` | `7.0`, `6.0`, `6.83`, `3.36` | Handed to `AccessoryCommander` by the loader; the nod moves between them. |

The full list is in `mecanumbot_movement_behaviours/README.md`; how a constants
file is read at all is in `mecanumbot_bt_config/README.md`.

Everything else in `config/*.yaml` is **required**: the ostensive schema is flat,
and every scalar in it is the experiment rather than a tuning of it, so a file
that omits one fails to load rather than having a field of view invented for it.
`behaviours/blackboard_managers.OSTENSIVE_PARAMS` is that list, and is also what
a behaviour registers to read.

## Build and run

```bash
colcon build --symlink-install --packages-select \
  mecanumbot_bt_config mecanumbot_movement_behaviours mecanumbot_ostensive_behaviour
source install/setup.bash
```

The tree needs, already running:

* the people-detection pipeline, for `cam_people_detections` and `people_fusion`;
* nav2 with AMCL localized on a map.

```bash
# perception
ros2 launch mecanumbot_sensorprocess_smart mecanumbot_peopledetect.launch.py

# the tree, with the YAML chosen from the Wi-Fi SSID
ros2 launch mecanumbot_ostensive_behaviour launch_ostensive.launch.py

# or with an explicit YAML
ros2 launch mecanumbot_ostensive_behaviour launch_ostensive.launch.py \
  yaml_path:=/absolute/path/to/ostensive_setting_constants.yaml

# or bare
ros2 run mecanumbot_ostensive_behaviour ostensive_bt_node
```

Watching what it thinks:

```bash
ros2 topic echo /goal_pose
ros2 run py_trees_ros_viewer py-trees-ros-viewer   # feedback messages per behaviour
```

## Tests

`test/test_gestures.py` and `test/test_cue_geometry.py` are real unit tests — 57
of them, covering the gesture decoding, the image mirror and every sign
convention. They import
only the three pure modules, so they need neither a ROS graph nor a sourced
workspace:

```bash
cd src/mecanumbot_behaviours/mecanumbot_ostensive_behaviour
PYTHONPATH=. python3 -m pytest test/test_gestures.py test/test_cue_geometry.py -v
```

The rest of `test/` is the usual ament lint scaffolding. `test_flake8` fails on
this machine for the reason it fails in every package here — the installed
`flake8` cannot load its own `pycodestyle` plugin — and that is the environment,
not the code. `test_pep257` passes and is worth keeping green.

## Known limits

* **Whether the image is mirrored is declared, not detected.** `mirror_image_x`
  is a constant in the YAML; nothing checks it against the camera, and nothing
  else in the workspace reads it. Change the camera, its `csi_flip_method`, or
  which topic the detector consumes, and this is the value to revisit.
* **A cue's azimuth is recovered from one image**, so its depth component is not.
  Pointing towards or away from the camera is refused rather than guessed at, and
  a cue given at a steep angle up or down is read only by its horizontal part.
* **The person is assumed to be facing the robot.** That is what makes their
  facing direction knowable, and it holds for somebody who has just waved at it.
  Somebody who points while turned away will have their cue rotated by however far
  they turned.
* **The goal is a point in open space**, not a place on a route. Nothing
  guarantees it is reachable, or inside the map. Nav2 rejecting it is a normal
  outcome and ends the exchange.
* **`people_fusion` is required to act on a cue.** Without a fused pose there is
  no metric position to anchor the ray at, and `DecodeDirectionCue` warns and
  keeps waiting rather than inventing a range.
* **Re-centring bypasses the local costmap**, as every in-place rotation on this
  robot does.
