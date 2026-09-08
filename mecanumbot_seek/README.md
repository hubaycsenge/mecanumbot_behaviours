# mecanumbot_seek

The seeking behaviour tree — **T2** of the Deep3R seeking system. The robot is
told what to find and where the server last saw it, then goes and finds it: it
watches for the object the whole time while driving to where it was and
searching outwards from there, and a sighting anywhere in that process
interrupts everything and turns into an approach.

The tree is modelled on **Panksepp's SEEKING circuit** — the appetitive foraging
system — and the model is not decoration. It decides how wide the robot
searches, how weak a detection it will act on, and when it gives up.

An episode has **two endings**. Either the robot picks the object up, or it
cannot — the thing is on a table, in a recess, or behind something nav2 could
not route around — and it goes and **tells a person**, by driving up to them and
alternating its orientation between them and the object.

```text
  seek/request     ──┐                        ┌──► grip it
  seek/target      ──┼──►  SeekEpisode ──────►┤
  seek/detections  ──┘          │             └──► tell somebody ──► seek/alert
   (server + detector)          │
                                └──►  seek/state   (the modelled circuit)
```

## The tree

```text
ROOT (Sequence)
├── LoadSeekParams
└── SeekLoop (Repeat, for ever)
    └── EpisodeOrRelease (Selector)
        ├── SeekEpisode (Sequence)
        │   ├── AcquireTarget                 what to find, and where it was
        │   ├── SeekUntilSighted (Parallel, SuccessOnSelected[WatchForObject])
        │   │   ├── SeekingCircuit            ticks the drive; always RUNNING
        │   │   ├── WatchForObject            ◄── succeeds on a sighting
        │   │   └── GoalDirectedSearch (Sequence)
        │   │       ├── GoToWhereItWas        nav2, to the server's answer
        │   │       ├── LookAroundWhereItWas  one revolution on the spot
        │   │       └── SearchOutwards        widening rings, until extinction
        │   ├── ApproachOrRecord (Selector)
        │   │   ├── ApproachObject            re-aims as the estimate refines
        │   │   └── CouldNotGetThere          a failed drive becomes a *reason*
        │   ├── SecureOrAlert (Selector)
        │   │   ├── SecureObject (Sequence)
        │   │   │   ├── CheckObjectGraspable  ◄── the branch point: height
        │   │   │   └── GraspObject
        │   │   └── AlertToUnreachable (Sequence)
        │   │       ├── TellIfAnyoneIsThere (FailureIsSuccess)
        │   │       │   └── TellSomebody (Sequence)
        │   │       │       ├── FindSomeoneToTell   watch, else scan
        │   │       │       ├── GoToThePerson       nav2
        │   │       │       └── ShowingGesture      ◄── object ⇄ person, ×N
        │   │       └── AnnounceUnreachable   publishes SeekAlert either way
        │   └── EndEpisode
        └── AbandonEpisode                    absorbs any failure
```

### Why the two branches are a parallel

The requirement is that the robot *stops along the way if it finds the item*. In
a sequence that would mean a check for the object between every pair of steps,
and the check would only fire at the seams — the robot would drive straight past
the thing it is looking for and notice on arrival.

As a parallel, `WatchForObject` does nothing else for the whole episode.
Wherever the robot happens to be in the goal-directed branch — halfway to the
remembered place, spinning at it, three rings out into the search — a sighting
ends the parallel, `terminate()` cancels the nav2 goal in flight, and the
approach starts from wherever the robot is standing.

### Why `SuccessOnSelected` and not `SuccessOnOne`

With `SuccessOnOne` the parallel would also succeed if the *search* branch
succeeded, and the tree would fall into `ApproachObject` with nothing sighted.
`SuccessOnSelected([WatchForObject])` makes the sighting the only thing that can
end the parallel successfully, so the claim "after this parallel, the object has
been seen" actually holds. The search branch can only stay RUNNING or fail, and
its failure — extinction — fails the parallel and ends the episode.

`SeekingCircuit` is the third child and always returns RUNNING, so the circuit
advances for exactly as long as the episode lasts and neither ends it nor fails
it.

## The SEEKING model

`seeking.py`. Two coupled layers, after the discrete state-space engine of
Szabó et al. (2011) — whose parallel-circuit structure came from Panksepp in the
first place:

| Layer | Meaning | Time constant |
| --- | --- | --- |
| `arousal` | how hard the robot is seeking *right now* | 8 s |
| `expectancy` | how much it still believes *this object is findable* — Szabó's "mood" | 90 s |

Four properties of Panksepp's system are implemented, and each one changes what
the robot does:

**It is expectancy, not receipt.** SEEKING is appetitive: engaged by
anticipation, switched off by finding. `GraspObject` calls `consummate()`, which
collapses both layers. A robot still seeking with the thing in its grabbers
would be the model being wrong, not the robot being keen.

**It outlasts the stimulus.** *"Positive feedback that sustains arousal after
the precipitating event has passed"* is one of Panksepp's defining attributes of
an emotional system. Here it is the long leak on `arousal`: losing sight of the
object is explicitly **not** an event with a cost, so the robot keeps working
near where it glimpsed the thing instead of losing interest the instant the
detector drops a frame.

**It modulates sensory input** — incentive salience. `detection_threshold()`
falls as arousal rises, and both `WatchForObject` and `SeekingCircuit` take
their gate from it, so a strongly seeking robot acts on a weaker detection. It
is a feedback loop: a first glimpse raises arousal, which lowers the threshold,
which makes the next marginal frame count too. It is the reason a hungry animal
sees food in a shadow, and it is bounded at both ends because a robot that
accepts anything is not motivated but broken.

**It wants nothing in particular before it is directed.** *"This system does not
'want' anything specific before learning; it just wants opportunities to explore
the world."* That is what the `undirected` phase is for: with no object named the
circuit sits at baseline, and T2 is the same circuit with an incentive attached.

> **The circuit does not currently drive T1, and this is an open decision.**
> `mecanumbot_custom_nav2`'s explorer scores frontiers on its own terms and
> contains no reference to `arousal`, `expectancy` or `SeekingState`; the
> `undirected` phase is implemented here and read by nothing. So the Panksepp
> grounding is real for T2 and, for now, a claim about T1 rather than a
> mechanism in it.
>
> Making it true means deciding what the circuit is entitled to change during
> exploration — whether falling expectancy widens the frontier search the way it
> widens the T2 rings, whether extinction is allowed to end T1 alongside the
> exit criteria, whether an `undirected` circuit modulates anything at all when
> there is no incentive object to be salient about. Those are thesis questions
> about how far the model reaches, not plumbing, and answering them by wiring
> would be answering them by accident. Until then the honest reading is: T1 is
> exploration with a modelled circuit running beside it, and T2 is exploration
> the circuit steers.

### What the drive is *for*

| Read by | From | Effect |
| --- | --- | --- |
| `WatchForObject`, `SeekingCircuit` | `detection_threshold()` | how weak a detection is acted on |
| `SearchAround` | `search_radius()` | how wide the rings go — falling expectancy widens the search |
| *(nothing yet)* | `undirected` phase | what T1 would read, if the circuit drove exploration — see above |
| `SearchAround` | `extinguished` | **when the robot gives up** |

That last one matters most. The search does not end when the waypoints run out —
they never do, because a completed sweep is a non-reward and a non-reward
rebuilds the rings wider. It ends when expectancy falls below
`seek_extinction_threshold`. **The robot gives up because it has stopped
believing the thing is findable**, which is the one place in this system where a
behaviour ends for a modelled reason rather than a numeric one.
`seek_search_timeout` exists as a backstop, and a run that ends *there* instead
is worth looking at.

### This is deliberately not a reward-prediction error

Panksepp rejects that reduction explicitly and at length — §7 of the 2005 paper
is titled *"The SEEKING/expectancy/wanting system of the brain: It's not just
'reward prediction error'"*, and the 2011 paper repeats it. The thesis wiki
records the SEEKING–dopamine–TD bridge as a contradiction and notes that it
**must not appear in the thesis in its original form**.

So nothing here compares a predicted value against a received one. `expectancy`
falls on **frustrative non-reward** — the *event* of a sweep completing with
nothing found — and the methods are named as events (`sight`, `cue`,
`arrive_at_expected`, `non_reward`, `consummate`) precisely so that the
distinction survives contact with the code.

## Alerting a person to something it cannot have

An object located and not obtained is **not a failed search**. The robot found
the thing; what it could not do is pick it up, and it now knows something a
person would want to know. Ending that run as a failure throws away the most
useful result it produced.

### The branch point is height

`CheckObjectGraspable` decides, and it decides on the object's height — the one
number the 2D map cannot supply.

| | |
| --- | --- |
| `too_high` | above 0.15 m — on a table, a shelf |
| `too_low` | below 0.03 m — in a recess, under something |
| `no_route` | nav2 could not get the robot to it |
| `grip_failed` | it reached it and the grabbers caught nothing |
| `lost` | it was seen and could not be found again |

The band is hardware, not taste: the grabber shafts sit at z ≈ 0.034 m with a
0.116 m clear gap and there is no lift DOF, so an object outside it cannot be had
however close the robot gets.

**This is the clearest payoff of having the reconstruction at all.** From the
lidar alone, a mug on a table and a mug on the floor behind a chair are the same
fact — "something is at (x, y) and the robot cannot get to it". The Deep3R
hypothesis carries a `z`, so they become different things to tell a person, and
`mecanumbot_seek` needs the server rather than merely tolerating it.

Two deliberate asymmetries in `reachability.assess`:

- **height is judged before distance.** A robot parked against a table is close
  enough and still cannot have the mug on it; reporting that as a distance
  problem sends a person looking for an obstacle that is not there;
- **an unknown height does not refuse the grasp.** The cloud may be thin, and
  refusing on that basis would make the robot give up on everything on the floor.
  The grasp itself is the check that always runs.

### The gesture is orienting alternation

The robot faces the object, holds, faces the person, holds — `seek_show_alternations`
times. In dogs this is *showing*, and gaze alternation between a human and a
referent is its core.

It is built out of the movement library's `TurnToward`, and that turns out to be
better than writing it here: `TurnToward` picks the lifted "seeking" head pose
for a human target and the level pose for a place, so **the neck rises to address
the person and drops to indicate the object** without a line in this package
saying so. The robot has no eyes; the wiki's `ostensive-signalling` page argues
that orienting with the neck is the functional analogue of eye contact, and that
the *outbound* direction — the robot producing ostensive signals — is the one
almost nothing in the corpus tests.

> **This is apparatus for a question, not an answer to it.** The one piece of
> evidence the thesis corpus holds on robot-produced ostension is **negative**:
> infants followed a robot's gaze but did not learn from it, while following
> *and* learning from an equivalent human (Okumura et al. 2013; reported in
> `okumura-2020-ostension-attention`). Whether a person reads this alternation as
> the robot showing them something is the thing to measure. `SeekAlert` carries
> `gaze_alternations` for exactly that: a person who walks off halfway through
> has been shown fewer times than one who stayed, and the trial should be able
> to say so.

### Nobody to tell is an outcome, not a failure

`AnnounceUnreachable` is the last leaf and runs whatever happened before it —
`TellIfAnyoneIsThere` is a `FailureIsSuccess` decorator over the whole
find-approach-show sequence. An alert is published even when the room was empty,
with `person_informed: false` and `gaze_alternations: 0`. Losing those runs
because nobody was around would be losing some of the more interesting ones.

The LED flash (`FAST_BLINK` / `YELLOW` by default) is fire-and-forget and skipped
entirely when `set_led_status` is not up, so the tree runs on a bench with no
Arduino attached. `seek_alert_led_mode: 0` disables it outright.

## Interfaces

| Topic | Type | Direction | Function |
| --- | --- | --- | --- |
| `/mecanumbot/seek/request` | `std_msgs/String` | in | what to look for — **free text**, handed to the server's open-vocabulary detector verbatim |
| `/mecanumbot/seek/target` | `vision_msgs/Detection3DArray` | in | where the **server** found it in the T1 cloud, in the robot's `map` frame |
| `/mecanumbot/seek/detections` | `vision_msgs/Detection3DArray` | in | live **onboard** detections of the same object, in `map` |
| `/mecanumbot/seek/state` | `mecanumbot_msgs/SeekingState` | out | the modelled circuit, every tick |
| `/mecanumbot/seek/alert` | `mecanumbot_msgs/SeekAlert` | out | found it, cannot have it: what, where, how high, why, and whether anybody was told |
| `/mecanumbot/people_fusion` | `geometry_msgs/PoseArray` | in | who there is to tell (via the movement library's tracker) |
| `/mecanumbot/set_led_status` | `mecanumbot_msgs/SetLedStatus` | srv | the alert flash; skipped when the service is absent |
| `/cmd_accessory_pos` | `mecanumbot_msgs/AccessMotorCmd` | out | the grabbers (and the neck, held still) |
| `/mecanumbot/has_object` | `std_msgs/Bool` | in | whether a grasp actually caught anything |
| `/amcl_pose` | `PoseWithCovarianceStamped` | in | via the movement library's tracker |
| `/navigate_to_pose` | `nav2_msgs/NavigateToPose` | action | every drive, including the search waypoints |

**Why two detection topics rather than one.** The server's answer is a *memory*:
one hypothesis, computed once from the T1 cloud, that stays put until the server
sends another. The onboard detections are *perception*: many per second, each
only briefly true. Merging them would leave the tree unable to tell "where it
was" from "where it is", which is the distinction the whole two-branch search is
built on. It shows up on the blackboard as two keys — `seek_last_known` (the
memory, what the rings are centred on) and `seek_target_position` (the current
best guess, what the approach drives to).

`Detection3DArray` is a standard type on purpose: a labelled pose with a
confidence is exactly what it carries, and `mecanumbot_msgs` is kept for data no
standard type covers. `SeekingState` is the one new message, and it is robot
data — the circuit as computed, so a trial can be replayed against what the
robot was doing and why.

## What this package reuses

Almost all the navigation. `keys.py` derives a `KeyMap` that respells one field
— `target_position` → `seek_target_position` — and binds it onto
`SeekApproach(Approach)`. That single line gives the tree the movement library's
whole single-goal drive: nav2 action goals, resends, stop thresholds, the cancel
on terminate. `SeekScan` is the library's `Spin360` with the same binding.

What is genuinely new here is the circuit, the ring search, the re-aiming
approach, the grasp, and the reachability judgement behind the alert.

The showing gesture is reuse too: `SeekTurnToward` and `SeekFindPeople` are the
library's `TurnToward` and `FindPeople` with the seek key spelling bound on, and
the alternation is a `Repeat` over a four-leaf sequence in the tree rather than
a behaviour of its own — it is a shape, not a mechanism.

## Running

**T2 follows T1 and does not stand alone**: the target it drives to is a
coordinate the server located in the cloud T1 built. Run T1 first — the full
sequence is in `mecanumbot_custom_nav2/README.md` under "Starting T1" — and let
it latch `/mecanumbot/exploration/finished`.

The handover itself is automatic. `mecanumbot_deep3r` sees the latch and sends
the server a `phase` message, so by the time this tree starts the decision stage
is already looking for the target. Nothing here has to be told that T1 ended.

```bash
# T2 needs nav2 localized against the map T1 saved -- the robot's own 2D map is
# the more stable frame, which is why this phase uses it rather than the cloud
ros2 launch mecanumbot_bringup launch_mecanumbot_base.launch.py
ros2 launch mecanumbot_deep3r deep3r.launch.py     # the second scan updates the first

# ask for something. FREE TEXT, not a class label: it is the query the server's
# open-vocabulary detector is given verbatim, so the description does work.
ros2 topic pub --once /mecanumbot/seek/request std_msgs/String \
  "{data: 'the red mug on the desk'}"

ros2 launch mecanumbot_seek launch_seek.launch.py

# watch the circuit, and the two inputs it runs on
ros2 topic echo /mecanumbot/seek/state
ros2 topic echo /mecanumbot/seek/target        # the memory: where it was in T1
ros2 topic echo /mecanumbot/seek/detections    # perception: where it is now
ros2 topic echo /mecanumbot/seek/alert         # found it, cannot have it
```

For the point cloud's keepouts in this phase, start the base launch against the
T2 parameters:

```bash
NAV2_PARAMS_FILE=$(ros2 pkg prefix mecanumbot_description)/share/mecanumbot_description/param/mecanumbot_seek_nav2.yaml \
  ros2 launch mecanumbot_bringup launch_mecanumbot_base.launch.py
```

Both detection topics are published by `mecanumbot_deep3r` from the server's
`found` announcements, split on its `basis` field — `memory` to `seek/target`,
`live` to `seek/detections`. So the tree does nothing until the link is up and
the server has a target; if `AcquireSeekTarget` times out, check the tunnel and
the server before looking at the tree.

The launch file picks its constants from the Wi-Fi SSID, like every other
behaviour launcher here: `MecanumetoNet` → `Eto_seek_setting_constants.yaml`,
anything else → `seek_setting_constants.yaml`. The two files carry the same 56
keys and differ only in the six that depend on how big the room is.

The tree registers as `seek_bt_node`, not the `bottom_up_tree_node` the four
leading executables share, so it can run alongside them and `ros2 node list`
says which is which.

## Constants

`config/seek_setting_constants.yaml` documents every constant where it is set.
Two blocks, and the difference is worth keeping in mind while tuning:

- the **SEEKING block** is a *model*. Changing it gives the robot a differently
  shaped motivation, which is a research decision;
- everything below it is *engineering*: distances, timeouts and gripper
  positions describing this robot's body and this room.

Eight keys are `required` — the ones with no sensible stand-in, because a run
with an invented grasp distance is not a run. Two of them,
`robot_approach_distance` and `robot_closeness_threshold`, are about how close
the robot may drive to a *person* when it goes to tell them something; every
other experiment in this repository requires them for the same reason. Everything else has a packaged default
in `defaults.py`, so a constants file written before a key existed keeps
behaving as it did.

The loader warns (rather than failing) about four configurations that are
individually legal and jointly nonsense: a search that cannot widen, an
incentive salience that runs backwards, a grasp distance further out than the
approach parks, and a grasp band with its floor above its ceiling — which would
make every episode end in an alert.

## Tests

```bash
cd src/mecanumbot_behaviours/mecanumbot_seek
PYTHONPATH=. python3 -m pytest test/test_seeking.py test/test_search_patterns.py \
  test/test_reachability.py -q -p no:launch_testing
```

82 tests, pure Python — no ROS graph, no nav2, no detector. Use
`/usr/bin/python3`, not the conda one.

`test_seeking.py` is written as claims about Panksepp's system rather than as
coverage of the arithmetic — that it is engaged by anticipation, that it
outlasts the stimulus, that it modulates what the robot perceives, that it falls
under non-reward, and that it switches off on consummation. If one of those
fails, the model has stopped being the thing it says it is. It also pins that
the decay is independent of the tick period, because a behaviour tree's tick
period is nominal and a slow tick under load must not silently change every rate
in the model.

`test_search_patterns.py` pins the property the ring pattern exists for: every
point at one radius is visited before any point further out, so the likely
places are searched first — including that being *closer* to a later waypoint
does not promote it.

`test_reachability.py` pins the two judgements that route an episode into the
alert: that height is decided before distance, and that an unknown height does
not by itself stop the robot from trying.

`test_flake8` fails here as it does everywhere in this workspace — the installed
flake8 cannot load its own `pycodestyle` plugin. `ament_pep257` reports clean.
