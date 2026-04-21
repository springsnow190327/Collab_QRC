import pytest

from door_task.llm.backend import parse_action_json


def test_parse_clean_json():
    raw = '{"reason": "go", "robot_a": {"mode": "stop"}}'
    out = parse_action_json(raw)
    assert out["reason"] == "go"
    assert out["robot_a"]["mode"] == "stop"


def test_parse_extracts_from_prose():
    raw = 'Sure, here is the plan:\n{"reason": "x", "robot_a": {"mode": "stop"}}\nDone.'
    out = parse_action_json(raw)
    assert out["robot_a"]["mode"] == "stop"


def test_parse_no_json_raises():
    with pytest.raises(ValueError):
        parse_action_json("no json here")
