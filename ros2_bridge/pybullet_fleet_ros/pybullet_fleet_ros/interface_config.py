"""Configuration helpers for fleet and per-robot ROS interfaces.

This module intentionally has no ROS imports so it can be tested outside a
ROS workspace. BridgeNode can use it to translate explicit ``fleet_api`` /
``per_robot_api`` sections into one normalized configuration object.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import Any

from pybullet_fleet.config_utils import config_get_bool, config_get_str_list


@dataclass(frozen=True)
class FleetApiConfig:
    """Fleet-level API group switches."""

    enabled: bool = False
    states: bool = False
    navigate: bool = False
    joint_command: bool = False
    stop: bool = False
    execute_action: bool = False
    attach: bool = False
    charging: bool = False


@dataclass(frozen=True)
class PerRobotApiConfig:
    """Per-robot API group switches."""

    enabled: bool = True
    state_publishers: bool = True
    tf: bool = True
    command_topics: bool = True
    services: bool = True
    actions: bool = True
    include_robots: tuple[str, ...] = field(default_factory=tuple)
    exclude_robots: tuple[str, ...] = field(default_factory=tuple)

    def robot_enabled(self, name: str) -> bool:
        """Return whether any per-robot API may be created for *name*."""
        if not self.enabled:
            return False
        if self.include_robots and name not in self.include_robots:
            return False
        if name in self.exclude_robots:
            return False
        return self.any_group_enabled

    @property
    def any_group_enabled(self) -> bool:
        """Whether at least one per-robot interface group is enabled."""
        return any(
            (
                self.state_publishers,
                self.tf,
                self.command_topics,
                self.services,
                self.actions,
            )
        )


@dataclass(frozen=True)
class BridgeApiConfig:
    """Normalized bridge API configuration."""

    fleet_api: FleetApiConfig = field(default_factory=FleetApiConfig)
    per_robot_api: PerRobotApiConfig = field(default_factory=PerRobotApiConfig)


def _fleet_api_from_dict(config: Mapping[str, Any], base: FleetApiConfig) -> FleetApiConfig:
    return FleetApiConfig(
        enabled=config_get_bool(config, "enabled", base.enabled),
        states=config_get_bool(config, "states", base.states),
        navigate=config_get_bool(config, "navigate", base.navigate),
        joint_command=config_get_bool(config, "joint_command", base.joint_command),
        stop=config_get_bool(config, "stop", base.stop),
        execute_action=config_get_bool(config, "execute_action", base.execute_action),
        attach=config_get_bool(config, "attach", base.attach),
        charging=config_get_bool(config, "charging", base.charging),
    )


def _per_robot_api_from_dict(config: Mapping[str, Any], base: PerRobotApiConfig) -> PerRobotApiConfig:
    return PerRobotApiConfig(
        enabled=config_get_bool(config, "enabled", base.enabled),
        state_publishers=config_get_bool(config, "state_publishers", base.state_publishers),
        tf=config_get_bool(config, "tf", base.tf),
        command_topics=config_get_bool(config, "command_topics", base.command_topics),
        services=config_get_bool(config, "services", base.services),
        actions=config_get_bool(config, "actions", base.actions),
        include_robots=tuple(config_get_str_list(config, "include_robots", list(base.include_robots))),
        exclude_robots=tuple(config_get_str_list(config, "exclude_robots", list(base.exclude_robots))),
    )


def _optional_mapping_section(config: Mapping[str, Any], key: str) -> Mapping[str, Any] | None:
    value = config.get(key)
    if value is None:
        return None
    if not isinstance(value, Mapping):
        raise ValueError(f"Expected '{key}' to be a mapping, got {type(value).__name__}")
    return value


def resolve_bridge_api_config(bridge_config: Mapping[str, Any]) -> BridgeApiConfig:
    """Resolve explicit bridge API config into one object."""
    fleet_api = FleetApiConfig()
    per_robot_api = PerRobotApiConfig()

    explicit_fleet = _optional_mapping_section(bridge_config, "fleet_api")
    if explicit_fleet is not None:
        fleet_api = _fleet_api_from_dict(explicit_fleet, fleet_api)

    explicit_per_robot = _optional_mapping_section(bridge_config, "per_robot_api")
    if explicit_per_robot is not None:
        per_robot_api = _per_robot_api_from_dict(explicit_per_robot, per_robot_api)

    return BridgeApiConfig(fleet_api=fleet_api, per_robot_api=per_robot_api)
