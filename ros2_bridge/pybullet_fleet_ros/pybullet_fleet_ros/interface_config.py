"""Configuration helpers for fleet and per-robot ROS interfaces.

BridgeNode uses this module to translate explicit ``fleet_api`` /
``per_robot_api`` sections into one normalized configuration object.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Any

from pybullet_fleet.config_utils import config_get_bool, config_get_str_list

try:
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
except ImportError:  # pragma: no cover - exercised by core-only configuration tests
    # Keep configuration parsing importable in the normal, non-ROS test
    # environment. A running bridge always imports the rclpy policy enums.
    class ReliabilityPolicy(Enum):
        RELIABLE = auto()
        BEST_EFFORT = auto()

    class HistoryPolicy(Enum):
        KEEP_LAST = auto()
        KEEP_ALL = auto()

    class DurabilityPolicy(Enum):
        VOLATILE = auto()
        TRANSIENT_LOCAL = auto()


@dataclass(frozen=True)
class FleetStateQosConfig:
    """Internal QoS settings used by the FleetState scale-check profiles."""

    reliability: ReliabilityPolicy = ReliabilityPolicy.RELIABLE
    history: HistoryPolicy = HistoryPolicy.KEEP_LAST
    depth: int = 10
    durability: DurabilityPolicy = DurabilityPolicy.VOLATILE


FLEET_STATE_QOS_PRESETS = {
    "fleet_state_reliable": FleetStateQosConfig(),
    "fleet_state_best_effort": FleetStateQosConfig(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1),
}


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
    transport_probe: bool = False
    # The scale checker writes this internal setting into its temporary bridge
    # config. It is not yet a documented general bridge configuration API.
    state_qos: FleetStateQosConfig = field(default_factory=FleetStateQosConfig)


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
    state_qos = _fleet_state_qos_from_dict(config.get("state_qos"), base.state_qos)
    return FleetApiConfig(
        enabled=config_get_bool(config, "enabled", base.enabled),
        states=config_get_bool(config, "states", base.states),
        navigate=config_get_bool(config, "navigate", base.navigate),
        joint_command=config_get_bool(config, "joint_command", base.joint_command),
        stop=config_get_bool(config, "stop", base.stop),
        execute_action=config_get_bool(config, "execute_action", base.execute_action),
        attach=config_get_bool(config, "attach", base.attach),
        charging=config_get_bool(config, "charging", base.charging),
        transport_probe=config_get_bool(config, "transport_probe", base.transport_probe),
        state_qos=state_qos,
    )


def _fleet_state_qos_from_dict(config: Any, base: FleetStateQosConfig) -> FleetStateQosConfig:
    if config is None:
        return base
    if not isinstance(config, Mapping):
        raise ValueError("fleet_api.state_qos must be a mapping")
    preset = str(config.get("preset", "fleet_state_reliable")).strip().lower()
    if preset not in FLEET_STATE_QOS_PRESETS:
        choices = ", ".join(sorted(FLEET_STATE_QOS_PRESETS))
        raise ValueError(f"fleet_api.state_qos.preset must be one of: {choices}")
    selected = FLEET_STATE_QOS_PRESETS[preset]
    reliability = _qos_policy_from_dict(
        config,
        "reliability",
        selected.reliability,
        ReliabilityPolicy,
        (ReliabilityPolicy.RELIABLE, ReliabilityPolicy.BEST_EFFORT),
    )
    history = _qos_policy_from_dict(
        config,
        "history",
        selected.history,
        HistoryPolicy,
        (HistoryPolicy.KEEP_LAST, HistoryPolicy.KEEP_ALL),
    )
    depth = config.get("depth", selected.depth)
    if not isinstance(depth, int) or isinstance(depth, bool) or depth < 1:
        raise ValueError("fleet_api.state_qos.depth must be a positive integer")
    durability = _qos_policy_from_dict(
        config,
        "durability",
        selected.durability,
        DurabilityPolicy,
        (DurabilityPolicy.VOLATILE, DurabilityPolicy.TRANSIENT_LOCAL),
    )
    return FleetStateQosConfig(reliability=reliability, history=history, depth=depth, durability=durability)


def _qos_policy_from_dict(config: Mapping[str, Any], key: str, default, policy_type, supported):
    raw_value = config.get(key, default)
    if isinstance(raw_value, policy_type):
        value = raw_value
    else:
        try:
            value = policy_type[str(raw_value).strip().upper()]
        except KeyError as exc:
            choices = " or ".join(repr(item.name.lower()) for item in supported)
            raise ValueError(f"fleet_api.state_qos.{key} must be {choices}") from exc
    if value not in supported:
        choices = " or ".join(repr(item.name.lower()) for item in supported)
        raise ValueError(f"fleet_api.state_qos.{key} must be {choices}")
    return value


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
    if key not in config:
        return None
    value = config[key]
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
