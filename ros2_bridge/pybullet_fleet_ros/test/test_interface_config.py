"""Tests for bridge API interface configuration and ROS QoS resolution."""

import pytest
from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy

from pybullet_fleet_ros.interface_config import StateScope, resolve_bridge_api_config


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


def test_fleet_api_without_manager_interfaces_defaults_to_an_empty_list():
    cfg = resolve_bridge_api_config({"fleet_api": {"enabled": True, "states": True}})

    assert cfg.manager_interfaces == ()
    assert cfg.fleet_api.state_scope is StateScope.ALL_AGENTS


def test_legacy_state_include_managers_implies_manager_state_scope():
    cfg = resolve_bridge_api_config({"fleet_api": {"state_include_managers": ["delivery"]}})

    assert cfg.fleet_api.state_scope is StateScope.MANAGERS
    assert cfg.fleet_api.state_include_managers == ("delivery",)


def test_fleet_state_qos_resolves_preset_and_overrides():
    cfg = resolve_bridge_api_config(
        {
            "fleet_api": {
                "state_qos": {
                    "preset": "fleet_state_best_effort",
                    "history": "keep_all",
                    "depth": 1,
                    "durability": "transient_local",
                }
            }
        }
    )

    assert cfg.fleet_api.state_qos.reliability == ReliabilityPolicy.BEST_EFFORT
    assert cfg.fleet_api.state_qos.history == HistoryPolicy.KEEP_ALL
    assert cfg.fleet_api.state_qos.depth == 1
    assert cfg.fleet_api.state_qos.durability == DurabilityPolicy.TRANSIENT_LOCAL


@pytest.mark.parametrize(
    ("state_qos", "field"),
    [
        ({"preset": "unknown"}, "preset"),
        ({"reliability": "invalid"}, "reliability"),
        ({"history": "invalid"}, "history"),
        ({"depth": 0}, "depth"),
        ({"durability": "invalid"}, "durability"),
    ],
)
def test_fleet_state_qos_rejects_invalid_profile(state_qos, field):
    with pytest.raises(ValueError, match=rf"state_qos\.{field}"):
        resolve_bridge_api_config({"fleet_api": {"state_qos": state_qos}})


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


def test_manager_interfaces_resolve_scoped_endpoint_configuration():
    cfg = resolve_bridge_api_config(
        {
            "fleet_api": {
                "state_scope": "managers",
                "state_include_managers": ["delivery", "inspection"],
                "manager_interfaces": [
                    {
                        "manager": "delivery",
                        "states": True,
                        "state_publish_rate": 2.5,
                        "state_qos": {"reliability": "best_effort", "history": "keep_last", "depth": 1},
                        "navigate": True,
                        "stop": True,
                    }
                ],
            }
        }
    )

    assert cfg.fleet_api.state_include_managers == ("delivery", "inspection")
    assert cfg.fleet_api.state_scope is StateScope.MANAGERS
    assert len(cfg.manager_interfaces) == 1
    manager = cfg.manager_interfaces[0]
    assert manager.manager == "delivery"
    assert manager.state_publish_rate == 2.5
    assert manager.state_qos.reliability == ReliabilityPolicy.BEST_EFFORT
    assert manager.navigate is True
    assert manager.stop is True


def test_manager_interface_allows_a_numeric_namespace_segment():
    cfg = resolve_bridge_api_config({"fleet_api": {"manager_interfaces": [{"manager": "1"}]}})

    assert cfg.manager_interfaces[0].manager == "1"


@pytest.mark.parametrize(
    ("manager_interfaces", "match"),
    [
        ({}, "must be a list"),
        ([True], "must be a mapping"),
        ([{}], "must be a non-empty string"),
        ([{"manager": "one/two"}], "one ROS namespace segment"),
        ([{"manager": "one"}, {"manager": "one"}], "duplicate manager"),
        ([{"manager": "one", "state_publish_rate": 0}], "state_publish_rate must be positive"),
    ],
)
def test_manager_interfaces_reject_invalid_entries(manager_interfaces, match):
    with pytest.raises(ValueError, match=match):
        resolve_bridge_api_config({"fleet_api": {"manager_interfaces": manager_interfaces}})


@pytest.mark.parametrize(
    ("fleet_api", "match"),
    [
        ({"state_scope": "invalid"}, "state_scope"),
        ({"state_scope": "managers"}, "requires state_include_managers"),
        ({"state_scope": "all_agents", "state_include_managers": ["delivery"]}, "cannot use"),
    ],
)
def test_state_scope_rejects_ambiguous_or_invalid_filters(fleet_api, match):
    with pytest.raises(ValueError, match=match):
        resolve_bridge_api_config({"fleet_api": fleet_api})


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
