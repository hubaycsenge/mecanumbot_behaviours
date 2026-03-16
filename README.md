# mecanumbot_behaviours

This repository contains ROS 2 packages that implement higher-level behaviour trees and control sequences for the **Mecanumbot** platform.

The primary package in this repo so far is:

- `mecanumbot_leading_behaviour` - a `py_trees_ros` based behaviour tree implementation used to drive the robot's leading behaviour using predefined sequences and sensor-based decisions.

---

## 🧩 Repository structure

- `mecanumbot_leading_behaviour/` – the main behaviour tree implementation plus launch/config files.

---

## 🚀 Build & install

From your ROS 2 workspace root (e.g., `~/dev_ws`):

```bash
colcon build --symlink-install
source install/setup.bash
```

> ⚠️ The behaviour nodes depend on other `mecanumbot_*` packages (e.g., the LED control and navigation nodes and the low-level drive/actuator drivers). Make sure those packages are also built and sourced.

---


## 📄 Notes

- This repository is mainly intended for research/behaviour experimentation (experiment sequences, visual cues, and controlled movement patterns).
- For low-level motion control, see the `mecanumbot_core` / `mecanumbot_msgs` packages in the main workspace.
