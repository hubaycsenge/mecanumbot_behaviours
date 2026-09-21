# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repository is

`mecanumbot_behaviours` (`hubaycsenge/mecanumbot_behaviours`) is one of the separate git repos under `~/Documents/mecanumbot_ws/src/`: every behaviour that decides where the robot goes: the `bt_config` and `movement_behaviours` libraries, and the leading, demo, ostensive, seek, fetch and autoslam experiments. **Changing a tree changes an experiment's meaning.**

Build, test, environment and cross-repo conventions are in the workspace file `~/Documents/mecanumbot_ws/CLAUDE.md`, which is authoritative. Read it before building or testing. Each package's README has its full node interface.

## Knowledge sources: look in the wikis first

When a task needs information the code does not carry, look in these LLM-maintained wikis **before** inferring it from code, guessing, or answering from general knowledge.

**1. `~/Documents/PhD_research`**, the thesis research wiki (`github.com/hubaycsenge/PhD_research`, with its own `CLAUDE.md`). It is the authority on why the robot and its experiments are designed the way they are. Start at `index.md`, then grep the whole vault, because the index lags the content. The source papers behind it are in its `raw/` (immutable). Likely starting points for this repo:

- `wiki/systems/mecanumbot-behaviours.md`: the trees as the thesis describes them
- `wiki/models/panksepp-affective-systems.md`: the SEEKING model behind `mecanumbot_seek` and `mecanumbot_autoslam`, and why fetch is PLAY
- `wiki/models/dog-human-interaction-model.md` and `wiki/concepts/ostensive-signalling.md`: the leading and ostensive conditions
- `wiki/experiments/seek-deep3r.md` and `wiki/experiments/validation-study.md`: how the studies are meant to run

**2. `~/Documents/deep3r/deep3r/wiki/`**, the Deep3R wiki (Bitbucket `nipg/deep3r`, with its schema in `~/Documents/deep3r/deep3r/CLAUDE.md`). It covers 3D reconstruction, CUT3R, registration, scene change detection and the seeking server. Start at `wiki/index.md`, then grep. For this repo:

- `wiki/concepts/autonomous-exploration-coverage.md` and `wiki/synthesis/task3-mobile-robot-perception.md`: the T1 exploration pass and T2 seeking

**Only go when you need to.** Building, debugging, node interfaces, ROS plumbing and refactoring do not need the wikis, and both are too large to read as background. Never write to either wiki, or to any `raw/`, from here: record findings in this repo.
