# mecanumbot_leading_behaviour

`mecanumbot_leading_behaviour` provides a set of **behavior-tree driven control sequences** for the Mecanumbot platform.

It is built on **py_trees** + **py_trees_ros** and implements a set of reusable behaviour building blocks (movement managers, LED sequences, target selection, etc.) that can be configured via YAML.

---

## 🔧 Key concepts

### Behavior trees
The package defines multiple behaviour trees (one per experiment mode), each built from:
- **movement managers** (`movement_managers.py`) for turning, approaching, etc.
- **LED behaviours** (`LED_behaviours.py`) for visual feedback
- **Dog behaviours** (`dog_behaviours.py`) for sequence-driven “dog-like” interactions
- **Blackboard managers** (`blackboard_managers.py`) for loading constants and publishing state

### Runtime entrypoints
This package exposes the following console scripts (via `setup.py`):

| Executable | Tree file | Typical use |
|-----------|-----------|-------------|
| `control_leading_bt_node` | `tree_nodes/ctrl_tree.py` | Generic control + approach sequence |
| `doglike_leading_bt_node` | `tree_nodes/dog_tree.py` | “Dog-like” follow-and-point behaviour |
| `LED_leading_bt_node` | `tree_nodes/LED_tree.py` | LED-only signalling behaviour |

---

## ▶️ Running the behaviour trees

### Recommended: use provided launch files

The package ships with launch files for each behaviour mode.

```bash
ros2 launch mecanumbot_leading_behaviour launch_control_sequence.launch.py
```

Other launch options:
- `launch_doglike_sequence.launch.py` – dog-like sequence
- `launch_LED_sequence.launch.py` – LED signalling only
- `launch_bottomup.launch.py` – bottom-up mode

There are also `Eto_*` variants (experimental/experiment configs) whose launch names match the non-`Eto` equivalents.

### Running directly (without launch)

If you prefer, you can run the node directly:

```bash
ros2 run mecanumbot_leading_behaviour control_leading_bt_node
```

---

## ⚙️ Configuration

All behaviour constants are loaded from YAML at startup.

Default config file:
- `config/behaviour_setting_constants.yaml`

You can override the YAML path at launch using the `params` argument (see the launch files):

```bash
ros2 launch mecanumbot_leading_behaviour launch_control_sequence.launch.py params:=/path/to/my_params.yaml
```

### What’s in the config file
The YAML contains:
- distances/thresholds (e.g., `robot_closeness_threshold`, `target_reached_threshold`)
- LED sequences & timing (`LED_indicate_target_seq`, `LED_catch_attention_times`, etc.)
- gripper/camera setpoints (e.g., `Dog_indicate_target_seq`)
- waypoint/checkpoint lists (e.g., `Dog_checkpoints`)

---

## 🧩 Topics (expected)

The behaviour node does not directly control the motors. It typically drives other nodes by publishing/reading from standard topics such as:

- `/cmd_vel` (geometry_msgs/Twist) – velocity commands
- `/cmd_accessory_pos` (mecanumbot_msgs/AccessMotorCmd) – accessory position commands
- `/subject` (custom topic containing LiDAR-based info about the subject's whereabouts)

> ⚠️ Topic names may vary depending on how you remap them in your system; the included launch files apply a remap for `/mecanumbot/cmd_vel` → `/cmd_vel` and `/mecanumbot/cmd_accessory_pos` → `/cmd_accessory_pos`.

---

## ✅ Build & test

From your workspace root:

```bash
colcon build --symlink-install --packages-select mecanumbot_leading_behaviour
source install/setup.bash
```

Run tests:

```bash
colcon test --packages-select mecanumbot_leading_behaviour
```

---

## 📌 Notes

- This package is intended for higher-level behaviour experimentation, not for low-level motor control (that is handled by other `mecanumbot_*` packages).
- If you update the YAML config, restart the behaviour node to reload new parameters.
