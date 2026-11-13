# mecanumbot_object_choice_behaviour

`py_trees_ros` behaviour tree for the **object-choice task**: the dog
object-choice paradigm, run on the robot. Coloured bricks (≈6.5 cm cubes, a
tennis ball's size, so the grabbers close on them) lie on the floor; a person
stands among them facing the robot, bids for its attention and points at one.
The robot's choice is the brick it goes to — or none, which is recorded too.

The tree is the **ostensive** exchange and the **fetch** approach joined by a
choice, and it reuses both packages' behaviours unchanged. Perception is the
pose detector plus `mecanumbot_detect_bricks` (in `mecanumbot_sensorprocess_smart`).

## One trial

```text
ROOT
├── LoadObjectChoiceParams      ostensive + fetch + oc_* constants together
├── RememberStartPose           every trial starts and ends here
└── TrialLoop (repeat for ever)
    └── TrialOrAbandon (selector)
        ├── ObjectChoiceTrial
        │   ├── WaitForGo                /mecanumbot/object_choice/next (std_msgs/Empty)
        │   ├── StartTrial               numbered record opened
        │   ├── SurveyBricks             HEAD DOWN: which bricks, where (map frame)
        │   ├── ReadThePerson            HEAD UP -- the ostensive tree's behaviours:
        │   │   ├── WatchForBid              attention bid (no look-around fallback)
        │   │   ├── FaceAddressee
        │   │   ├── AcknowledgeAddressee     nod
        │   │   └── ReadCueWhileFocused      settled pointing ray
        │   ├── ChooseBrick              ray x survey -> one brick, or abstain (FAILURE)
        │   ├── response                 oc_response_mode:
        │   │     approach: LookAtBrick, ApproachBrick + FaceBrick (+ head tracking), hold
        │   │     grab:     LookAtBrick, fetch's SecureBall, fetch's HandOver
        │   ├── ReturnToStart            nav2
        │   ├── RecordTrial              outcome "completed"
        │   └── ForgetTheBrick
        └── AbandonTrial
            ├── RecordUnfinishedTrial    outcome read off how far the trial got
            ├── ForgetTheBrickAnyway
            └── ReturnIfWeCan
```

**Why the head goes down, then up.** The camera is ~0.22 m off the floor with a
~29° vertical view. One tilt cannot see bricks on the floor *and* a standing
person's shoulders and wrists (the wiki's `mecanumbot-behaviours` page records
the same tension for the ostensive tree). So the bricks are surveyed first at
`oc_survey_head`, remembered in the map, and the head then lifts to
`neck_seek_pos` to read the person.

## Running

```bash
# robot: drivers + nav2 (AMCL localized), robot at the trial's start pose,
# facing the layout
ros2 launch mecanumbot_bringup launch_mecanumbot_base.launch.py
ros2 launch mecanumbot_object_choice_behaviour launch_object_choice.launch.py

# each trial: set the bricks out, then
ros2 topic pub --once /mecanumbot/object_choice/next std_msgs/msg/Empty

# watch / record
ros2 topic echo /mecanumbot/object_choice/state
ros2 topic echo /mecanumbot/object_choice/trial          # one JSON line per trial
# record these too: both detectors' debug images, the trial and state topics
#   /mecanumbot/cam_people_detections/debug_image/compressed
#   /mecanumbot/cam_brick_detections/debug_image/compressed
```

The launch file starts perception itself (`detector:=pose camera_source:=topic
bricks:=true`): the brick detector **owns the webcam** and republishes it for
the pose detector, so nothing else may hold `/dev/video0` — not a Deep3R client,
not a second perception launch. `use_perception:=false` if it is already up the
same way.

## The trial record

`/mecanumbot/object_choice/trial`, `std_msgs/String`, one JSON object per trial,
published whatever happened:

| Field | Meaning |
| --- | --- |
| `trial`, `started`, `ended` | number since the tree started; ROS time [s] |
| `response_mode` | `approach` or `grab` |
| `bricks` | `{colour: [x, y, z]}` surveyed, map frame |
| `bricks_seen` | colours seen when the survey timed out |
| `cue` | `side` (arm), `azimuth_deg` (from where the person faces), `bearing_deg` (map), `person` [x, y] |
| `choice` | `choice` (colour or null), `reason`, `margin_deg`, `candidates` [{colour, deviation_deg, distance_from_person, excluded}] |
| `gripped`, `responded`, `grasped`, `delivered` | how far the response got |
| `final_distance_to_choice`, `final_robot_position` | where the robot ended up |
| `outcome` | `completed`, `no_bricks`, `not_addressed`, `no_cue`, `no_choice`, `response_failed`, `return_failed` |

`no_choice` with `reason: ambiguous` or `none in range` is the robot abstaining,
which is a result, not an error.

## How the choice is made

`choice_geometry.py` (pure Python, tested): for each surveyed brick, the angle at
the **person** between their pointing ray and the direction to the brick. The
smallest wins if it is within `oc_max_deviation_deg` (35°) **and** beats the
second by `oc_min_margin_deg` (10°); otherwise the trial abstains. Bricks nearer
the person than `oc_min_brick_distance` are left out. The ray comes from the
ostensive package unchanged, with its limits: one image gives no depth, so a
point towards or away from the camera is refused, and the person is assumed to
face the robot. The classic layout — bricks to either side of the person, both
lateral to the camera — is the case it handles well.

**These thresholds decide what counts as a choice and are research decisions**,
not tuning: changing them changes which trials are scored as abstentions.

## Why it imports two experiments

The repository's rule is that experiments depend on libraries, not on each other
(`mecanumbot_fetch_behaviour/behaviours/ros_interfaces.py` says so of its own
`GripperCommander`). This package breaks it on purpose: the cue reader and the
approach/grab are the two most hardware-tuned pieces of code here, and copying
them would let the object-choice task drift from the experiments it is compared
with. The cost is that **a change to the ostensive or fetch behaviours changes
this experiment too**. If that becomes a problem, the shared behaviours belong
in `mecanumbot_movement_behaviours`.

### The relay

The fetch behaviours are single-target: each acts on *the* ball, the nearest in
the map or the biggest in the image. With several bricks out they would drift to
the nearest one. So the launch file remaps their inputs
(`/mecanumbot/ball_detections`, `/mecanumbot/cam_ball_boxes`) onto
`/mecanumbot/object_choice/target_*`, and `TargetRelay` forwards only the chosen
colour there. To the fetch behaviours the chosen brick is the only ball. The tree
logs an **error** at setup if the remapping is missing (e.g. started with
`ros2 run`). `/mecanumbot/fetch/state` is remapped onto the object-choice state
topic, so fetch's phase labels appear there too.

## Configuration

`config/object_choice_setting_constants.yaml`, root key `object_choice_bt_node`.
It carries three trees' keys: every ostensive key (required, as in that
package), the fetch keys this tree uses, and the `oc_*` keys
(`defaults.py`). Fetch defaults that are wrong for a brick are overridden in
`defaults.FETCH_OVERRIDES` (no ball-label filter, **LEDs off** so nothing but the
movement can cue the person, brick score threshold, head range).

| Key | Default | Meaning |
| --- | --- | --- |
| `oc_response_mode` | `approach` | `approach` or `grab`; read at build time |
| `oc_choice_hold` | 3.0 | s held at the chosen brick |
| `oc_brick_colours` | `[]` (file: `[red, blue]`) | colours the survey must find; empty = any `oc_min_bricks` |
| `oc_min_bricks` | 2 | |
| `oc_survey_head` | 4.8 | neck tilt for the survey |
| `oc_survey_settle` / `_window` / `_min_samples` / `_timeout` | 1.0 / 2.0 / 3 / 20.0 | |
| `oc_max_deviation_deg` | 35 | see above |
| `oc_min_margin_deg` | 10 | see above |
| `oc_min_brick_distance` | 0.3 | m |
| `oc_wait_for_go` | true | wait for `/mecanumbot/object_choice/next` |
| `oc_rest` | 5.0 | s between trials when not waiting |
| `oc_return_timeout` | 60.0 | s |

### Before the first trial — not yet checked on the robot

1. **Neck tilts.** `oc_survey_head: 4.8` and `neck_seek_pos: 5.7` (with the nod
   around it) are **computed** from the 2026-10-21 neck calibration, not
   measured. Check the bricks in the brick debug image at the survey tilt, and a
   standing participant's shoulders *and* wrists in the pose debug image at the
   seek tilt. (The ostensive and fetch files' tilts predate that calibration.)
2. **Brick colours** under the room's light: `calibrate: true` on
   `mecanumbot_detect_bricks` logs the median HSV in a centre box.
3. **Brick side**: a brick on the robot's left must come out at positive `y` in
   the base frame (`mirror_image_x` on the brick node, default false).
4. **Return pose**: the start pose is the first AMCL pose after launch, so place
   the robot before starting the tree.

## Tests

```bash
cd src/mecanumbot_behaviours/mecanumbot_object_choice_behaviour
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=.:$PYTHONPATH /usr/bin/python3 -m pytest test/ -q
```

`test_choice_geometry.py` (9 tests, no ROS) holds the decision. The tree's wiring
was exercised against fake AMCL, go and brick topics on an isolated
`ROS_DOMAIN_ID`, but **not with a real person, camera or nav2**.
