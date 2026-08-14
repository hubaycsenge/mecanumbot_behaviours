"""
Tests for the schema-free constants loader.

They import `params` and `decoders` only, which pull in nothing from ROS, so
this file runs with nothing sourced::

    PYTHONPATH=. python3 -m pytest test/test_params.py
"""

import math
import textwrap

import pytest

from mecanumbot_bt_config import decoders
from mecanumbot_bt_config.params import (
    blackboard_key,
    blackboard_values,
    load_params,
    merge_defaults,
    missing_keys,
    undeclared_keys,
)


def write(tmp_path, text):
    """Write a YAML document to a temporary file and return its path."""
    path = tmp_path / "constants.yaml"
    path.write_text(textwrap.dedent(text))
    return str(path)


# --- finding the parameter block -------------------------------------------


def test_root_key_is_read_from_the_file(tmp_path):
    path = write(
        tmp_path,
        """
        some_tree_node:
          ros__parameters:
            robot_closeness_threshold: 0.75
        """,
    )
    assert load_params(path) == {"robot_closeness_threshold": 0.75}


def test_any_node_name_works_without_being_declared(tmp_path):
    """The fork of this loader existed only because the name was hard-coded."""
    first = write(
        tmp_path,
        """
        bottom_up_tree_node:
          ros__parameters:
            turn_max_speed: 0.6
        """,
    )
    second = str(tmp_path / "other.yaml")
    with open(second, "w") as handle:
        handle.write("ostensive_bt_node:\n  ros__parameters:\n    turn_max_speed: 0.4\n")
    assert load_params(first)["turn_max_speed"] == 0.6
    assert load_params(second)["turn_max_speed"] == 0.4


def test_explicit_root_keys_still_work(tmp_path):
    path = write(
        tmp_path,
        """
        named_node:
          ros__parameters:
            a: 1
        """,
    )
    assert load_params(path, ("named_node", "ros__parameters")) == {"a": 1}


def test_two_node_blocks_are_an_error_rather_than_a_guess(tmp_path):
    path = write(
        tmp_path,
        """
        one_node:
          ros__parameters:
            a: 1
        other_node:
          ros__parameters:
            a: 2
        """,
    )
    with pytest.raises(KeyError):
        load_params(path)


def test_a_file_without_a_parameter_block_is_an_error(tmp_path):
    path = write(tmp_path, "just_a_value: 3\n")
    with pytest.raises(KeyError):
        load_params(path)


def test_an_empty_file_is_an_error(tmp_path):
    path = write(tmp_path, "\n")
    with pytest.raises(ValueError):
        load_params(path)


# --- the degree convention --------------------------------------------------


def test_degrees_reach_the_blackboard_in_radians():
    values = blackboard_values({"turn_tolerance_deg": 3.0})
    assert values["turn_tolerance"] == pytest.approx(math.radians(3.0))
    assert "turn_tolerance_deg" not in values


def test_a_declared_angle_beats_its_radian_default():
    values = blackboard_values(
        {"turn_tolerance_deg": 6.0}, {"turn_tolerance": math.radians(3.0)}
    )
    assert values["turn_tolerance"] == pytest.approx(math.radians(6.0))


def test_an_undeclared_angle_keeps_its_default():
    defaults = {"turn_tolerance": math.radians(3.0)}
    values = blackboard_values({}, defaults)
    assert values["turn_tolerance"] == pytest.approx(math.radians(3.0))


def test_blackboard_key_strips_only_the_suffix():
    assert blackboard_key("route_turn_min_deg") == "route_turn_min"
    assert blackboard_key("route_turn_min") == "route_turn_min"


# --- defaults and types -----------------------------------------------------


def test_a_missing_key_keeps_its_packaged_default():
    values = blackboard_values({"turn_max_speed": 0.4}, {"turn_max_speed": 0.6, "turn_accel": 0.8})
    assert values == {"turn_max_speed": 0.4, "turn_accel": 0.8}


def test_a_count_written_as_a_float_arrives_as_an_int():
    values = blackboard_values({"route_lookahead": 3.0}, {"route_lookahead": 3})
    assert values["route_lookahead"] == 3
    assert isinstance(values["route_lookahead"], int)


def test_a_threshold_written_as_an_int_arrives_as_a_float():
    values = blackboard_values({"turn_max_speed": 1}, {"turn_max_speed": 0.6})
    assert isinstance(values["turn_max_speed"], float)


def test_a_key_with_no_default_keeps_the_type_the_file_gave_it():
    values = blackboard_values({"Dog_max_wander_allowed": 1})
    assert values["Dog_max_wander_allowed"] == 1


def test_defaults_merge_in_order():
    assert merge_defaults(({"a": 1, "b": 2}, {"b": 3})) == {"a": 1, "b": 3}


# --- what the file has to declare -------------------------------------------


def test_missing_keys_names_what_is_absent():
    assert missing_keys({"a": 1}, ("a", "b")) == ("b",)


def test_a_required_angle_counts_as_declared_in_degrees():
    assert missing_keys({"turn_tolerance_deg": 3.0}, ("turn_tolerance",)) == ()


def test_undeclared_keys_lists_the_defaulted_ones():
    assert undeclared_keys({"a": 1}, {"a": 0, "b": 0, "c": 0}) == ("b", "c")


# --- decoding by shape ------------------------------------------------------


def test_a_literal_no_decoder_claims_becomes_a_dictionary():
    values = blackboard_values({"thing": "{'unknown': 1}"})
    assert values["thing"] == {"unknown": 1}


def test_a_plain_string_is_left_alone():
    values = blackboard_values({"attention_signal_mode": "any"})
    assert values["attention_signal_mode"] == "any"


def test_a_registered_decoder_claims_a_list_entry_by_entry():
    decoders.register(("mode", "colour"), lambda literal: ("led", literal["colour"]))
    try:
        values = blackboard_values(
            {"seq": ["{'mode': 4, 'colour': 'red'}", "{'mode': 4, 'colour': 'blue'}"]}
        )
        assert values["seq"] == [("led", "red"), ("led", "blue")]
    finally:
        decoders._DECODERS.pop()


def test_a_malformed_literal_is_left_as_the_string_it_is():
    values = blackboard_values({"broken": "{not: valid, python"})
    assert values["broken"] == "{not: valid, python"
