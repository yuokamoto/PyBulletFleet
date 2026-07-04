"""Tests for ROS bridge interface config that run in default pytest."""

import sys
from pathlib import Path

BRIDGE_PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "ros2_bridge" / "pybullet_fleet_ros"
sys.path.insert(0, str(BRIDGE_PACKAGE_ROOT))

from pybullet_fleet_ros.interface_config import resolve_bridge_api_config


def test_default_bridge_interface_config_preserves_per_robot_interfaces():
    cfg = resolve_bridge_api_config({})

    assert cfg.fleet_api.enabled is False
    assert cfg.fleet_api.states is False
    assert cfg.fleet_api.navigate is False
    assert cfg.per_robot_api.enabled is True
    assert cfg.per_robot_api.state_publishers is True
    assert cfg.per_robot_api.tf is True
    assert cfg.per_robot_api.command_topics is True
    assert cfg.per_robot_api.services is True
    assert cfg.per_robot_api.actions is True


def test_bridge_interface_config_rejects_invalid_sections():
    for key, value in (
        ("fleet_api", True),
        ("fleet_api", None),
        ("per_robot_api", ["robot0"]),
        ("per_robot_api", None),
    ):
        try:
            resolve_bridge_api_config({key: value})
        except ValueError as exc:
            assert f"Expected '{key}' to be a mapping" in str(exc)
        else:
            raise AssertionError(f"invalid {key} section was accepted")


def test_bridge_interface_config_robot_filters_are_immutable():
    cfg = resolve_bridge_api_config(
        {
            "per_robot_api": {
                "include_robots": ["robot0", "robot1"],
                "exclude_robots": ["robot1"],
            }
        }
    )

    assert cfg.per_robot_api.include_robots == ("robot0", "robot1")
    assert cfg.per_robot_api.exclude_robots == ("robot1",)
    assert cfg.per_robot_api.robot_enabled("robot0") is True
    assert cfg.per_robot_api.robot_enabled("robot1") is False
    assert cfg.per_robot_api.robot_enabled("robot2") is False
