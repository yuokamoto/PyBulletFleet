"""Tests for bridge API interface configuration helpers.

These tests intentionally avoid ROS imports.
"""

from pybullet_fleet_ros.interface_config import resolve_bridge_api_config


def test_default_config_preserves_per_robot_interfaces():
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


def test_explicit_fleet_api_does_not_disable_per_robot_interfaces():
    cfg = resolve_bridge_api_config(
        {
            "fleet_api": {
                "enabled": True,
                "states": True,
                "navigate": True,
            }
        }
    )

    assert cfg.fleet_api.enabled is True
    assert cfg.fleet_api.states is True
    assert cfg.fleet_api.navigate is True
    assert cfg.per_robot_api.enabled is True
    assert cfg.per_robot_api.robot_enabled("robot0") is True


def test_fleet_api_and_per_robot_api_are_independent():
    cfg = resolve_bridge_api_config(
        {
            "fleet_api": {
                "enabled": True,
                "states": True,
                "navigate": True,
            },
            "per_robot_api": {
                "enabled": True,
                "state_publishers": False,
                "tf": True,
                "command_topics": True,
                "services": False,
                "actions": False,
            },
        }
    )

    assert cfg.fleet_api.enabled is True
    assert cfg.fleet_api.states is True
    assert cfg.fleet_api.navigate is True
    assert cfg.per_robot_api.enabled is True
    assert cfg.per_robot_api.state_publishers is False
    assert cfg.per_robot_api.tf is True
    assert cfg.per_robot_api.command_topics is True
    assert cfg.per_robot_api.services is False
    assert cfg.per_robot_api.actions is False


def test_invalid_api_sections_raise_clear_errors():
    try:
        resolve_bridge_api_config({"fleet_api": True})
    except ValueError as exc:
        assert "Expected 'fleet_api' to be a mapping" in str(exc)
    else:
        raise AssertionError("invalid fleet_api section was accepted")

    try:
        resolve_bridge_api_config({"fleet_api": None})
    except ValueError as exc:
        assert "Expected 'fleet_api' to be a mapping" in str(exc)
    else:
        raise AssertionError("null fleet_api section was accepted")

    try:
        resolve_bridge_api_config({"per_robot_api": ["robot0"]})
    except ValueError as exc:
        assert "Expected 'per_robot_api' to be a mapping" in str(exc)
    else:
        raise AssertionError("invalid per_robot_api section was accepted")

    try:
        resolve_bridge_api_config({"per_robot_api": None})
    except ValueError as exc:
        assert "Expected 'per_robot_api' to be a mapping" in str(exc)
    else:
        raise AssertionError("null per_robot_api section was accepted")


def test_robot_include_exclude_filters():
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


def test_no_per_robot_groups_means_no_handler_needed():
    cfg = resolve_bridge_api_config(
        {
            "per_robot_api": {
                "enabled": True,
                "state_publishers": False,
                "tf": False,
                "command_topics": False,
                "services": False,
                "actions": False,
            }
        }
    )

    assert cfg.per_robot_api.any_group_enabled is False
    assert cfg.per_robot_api.robot_enabled("robot0") is False
