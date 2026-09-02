"""Fear HRI behaviour tree entry point."""

from __future__ import annotations

import py_trees

from mecanumbot_fear_hri_behaviour.behaviours.blackboard_managers import FearParamsToBlackboard
from mecanumbot_fear_hri_behaviour.behaviours.fear_actions import ExecuteSelectedAction, LogFearState
from mecanumbot_fear_hri_behaviour.behaviours.fear_update import SelectFearAction, UpdateFearFromScan
from mecanumbot_fear_hri_behaviour.tree_nodes.tree_common import run_tree

TREE_NAME = "fear_hri_tree"
DEFAULT_YAML_FILENAME = "fear_hri_constants.yaml"


def create_fear_tick() -> py_trees.composites.Sequence:
    return py_trees.composites.Sequence(
        name="FearTick",
        memory=True,
        children=[
            UpdateFearFromScan(),
            SelectFearAction(),
            ExecuteSelectedAction(),
            LogFearState(),
        ],
    )


def create_root(yaml_path=None):
    root = py_trees.composites.Sequence(name="ROOT", memory=True)
    root.add_children([
        FearParamsToBlackboard(name="LoadFearParams", yaml_path=yaml_path),
        py_trees.decorators.Repeat(name="FearLoop", child=create_fear_tick(), num_success=-1),
    ])
    return root


def main(args=None):
    run_tree(create_root, TREE_NAME, DEFAULT_YAML_FILENAME, args=args)
