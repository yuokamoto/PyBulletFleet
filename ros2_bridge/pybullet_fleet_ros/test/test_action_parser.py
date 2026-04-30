"""Tests for action_parser — JSON→Action conversion.

These tests do NOT require ROS 2.  They verify the pure-Python parsing
logic used by both the /{robot}/execute_action topic and action server.
"""

import json
import sys
from pathlib import Path

import pytest

# action_parser lives inside the ros2_bridge package tree which is not
# pip-installed.  Add its parent to sys.path so we can import it directly.
_bridge_pkg = str(Path(__file__).resolve().parent.parent / "pybullet_fleet_ros")
if _bridge_pkg not in sys.path:
    sys.path.insert(0, _bridge_pkg)

from pybullet_fleet.action import (
    DropAction,
    JointAction,
    MoveAction,
    PickAction,
    PoseAction,
    WaitAction,
)
from action_parser import parse_action_goal, _ACTIONS


class TestMoveAction:
    def test_basic_path(self):
        params = json.dumps({"path": [[1.0, 2.0, 0.0], [3.0, 4.0, 0.0]]})
        action = parse_action_goal("move", params)
        assert isinstance(action, MoveAction)
        assert len(action.path.waypoints) == 2

    def test_missing_path_returns_none(self):
        action = parse_action_goal("move", "{}")
        assert action is None

    def test_empty_path_returns_none(self):
        action = parse_action_goal("move", json.dumps({"path": []}))
        assert action is None

    def test_optional_auto_approach(self):
        params = json.dumps({"path": [[1.0, 0.0, 0.0]], "auto_approach": False})
        action = parse_action_goal("move", params)
        assert isinstance(action, MoveAction)
        assert action.auto_approach is False

    def test_optional_direction(self):
        params = json.dumps({"path": [[1.0, 0.0, 0.0]], "direction": "backward"})
        action = parse_action_goal("move", params)
        assert isinstance(action, MoveAction)
        assert action.direction == "backward"


class TestPickAction:
    def test_empty_params_creates_default_pick(self):
        action = parse_action_goal("pick", "{}")
        assert isinstance(action, PickAction)

    def test_with_search_radius(self):
        params = json.dumps({"search_radius": 2.0})
        action = parse_action_goal("pick", params)
        assert isinstance(action, PickAction)
        assert action.search_radius == 2.0

    def test_with_target_position(self):
        params = json.dumps({"target_position": [5.0, 3.0, 0.1]})
        action = parse_action_goal("pick", params)
        assert isinstance(action, PickAction)
        assert action.target_position == [5.0, 3.0, 0.1]

    def test_with_attach_link_string(self):
        params = json.dumps({"attach_link": "end_effector"})
        action = parse_action_goal("pick", params)
        assert isinstance(action, PickAction)
        assert action.attach_link == "end_effector"

    def test_use_approach_false(self):
        params = json.dumps({"use_approach": False})
        action = parse_action_goal("pick", params)
        assert isinstance(action, PickAction)
        assert action.use_approach is False


class TestDropAction:
    def test_drop_pose_as_list(self):
        params = json.dumps({"drop_pose": [5.0, 3.0, 0.1]})
        action = parse_action_goal("drop", params)
        assert isinstance(action, DropAction)
        assert action.drop_pose.x == pytest.approx(5.0)
        assert action.drop_pose.y == pytest.approx(3.0)

    def test_drop_pose_as_dict(self):
        params = json.dumps(
            {
                "drop_pose": {
                    "position": [5.0, 3.0, 0.1],
                    "orientation": [0.0, 0.0, 0.7071, 0.7071],
                }
            }
        )
        action = parse_action_goal("drop", params)
        assert isinstance(action, DropAction)
        assert action.drop_pose.x == pytest.approx(5.0)

    def test_missing_drop_pose_returns_none(self):
        action = parse_action_goal("drop", "{}")
        assert action is None

    def test_optional_place_gently(self):
        params = json.dumps({"drop_pose": [0, 0, 0], "place_gently": False})
        action = parse_action_goal("drop", params)
        assert isinstance(action, DropAction)
        assert action.place_gently is False


class TestWaitAction:
    def test_basic_duration(self):
        params = json.dumps({"duration": 5.0})
        action = parse_action_goal("wait", params)
        assert isinstance(action, WaitAction)
        assert action.duration == 5.0

    def test_missing_duration_returns_none(self):
        action = parse_action_goal("wait", "{}")
        assert action is None

    def test_optional_action_type(self):
        params = json.dumps({"duration": 3.0, "action_type": "charge"})
        action = parse_action_goal("wait", params)
        assert isinstance(action, WaitAction)
        assert action.action_type == "charge"


class TestJointAction:
    def test_positions_list(self):
        params = json.dumps({"target_joint_positions": [0.0, 1.57, 0.0, -1.57]})
        action = parse_action_goal("joint", params)
        assert isinstance(action, JointAction)
        assert action.target_joint_positions == [0.0, 1.57, 0.0, -1.57]

    def test_positions_dict(self):
        params = json.dumps({"target_joint_positions": {"joint_1": 0.5, "joint_2": -0.3}})
        action = parse_action_goal("joint", params)
        assert isinstance(action, JointAction)
        assert action.target_joint_positions == {"joint_1": 0.5, "joint_2": -0.3}

    def test_missing_positions_returns_none(self):
        action = parse_action_goal("joint", "{}")
        assert action is None

    def test_optional_max_force(self):
        params = json.dumps({"target_joint_positions": [0.0], "max_force": 100.0})
        action = parse_action_goal("joint", params)
        assert isinstance(action, JointAction)
        assert action.max_force == 100.0


class TestEdgeCases:
    def test_unknown_action_type_returns_none(self):
        action = parse_action_goal("unknown", "{}")
        assert action is None

    def test_invalid_json_returns_none(self):
        action = parse_action_goal("move", "not valid json")
        assert action is None

    def test_empty_json_string(self):
        action = parse_action_goal("pick", "")
        assert isinstance(action, PickAction)

    def test_whitespace_json_string(self):
        action = parse_action_goal("pick", "   ")
        assert isinstance(action, PickAction)


class TestAutoDiscovery:
    """Verify _ACTIONS is built automatically from action.py subclasses."""

    def test_all_core_actions_discovered(self):
        expected = {"move", "pick", "drop", "wait", "joint", "pose"}
        assert expected.issubset(_ACTIONS.keys())

    def test_class_mapping(self):
        assert _ACTIONS["move"] is MoveAction
        assert _ACTIONS["pick"] is PickAction
        assert _ACTIONS["drop"] is DropAction
        assert _ACTIONS["wait"] is WaitAction
        assert _ACTIONS["joint"] is JointAction
        assert _ACTIONS["pose"] is PoseAction

    def test_pose_action_via_parser(self):
        params = json.dumps({"target_position": [0.3, 0.0, 0.5]})
        action = parse_action_goal("pose", params)
        assert isinstance(action, PoseAction)
        assert action.target_position == [0.3, 0.0, 0.5]
