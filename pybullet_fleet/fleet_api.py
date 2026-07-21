"""Transport-neutral fleet state and command API.

The classes in this module intentionally avoid ROS imports. ROS bridge wrappers,
RMF plugin paths, and future replay tools can share these Python data structures
and call sites without depending on a specific transport.
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass, field
from numbers import Real
from types import MappingProxyType
from typing import Any, Iterable, Mapping, Sequence
from uuid import uuid4

from pybullet_fleet.geometry import Pose


FLEET_COMMAND_EVENT = "fleet_command"


Vec2 = tuple[float, float]
Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]


@dataclass(frozen=True)
class RobotState2D:
    """Minimal planar state for one robot."""

    name: str
    object_id: int
    position: Vec2
    yaw: float
    linear_velocity: Vec2 = (0.0, 0.0)
    angular_velocity: float = 0.0
    is_moving: bool = False
    battery_soc: float | None = None
    is_charging: bool | None = None


@dataclass(frozen=True)
class RobotState3D:
    """Minimal 3D state for one robot."""

    name: str
    object_id: int
    position: Vec3
    orientation: Quat
    linear_velocity: Vec3 = (0.0, 0.0, 0.0)
    angular_velocity: Vec3 = (0.0, 0.0, 0.0)
    is_moving: bool = False
    battery_soc: float | None = None
    is_charging: bool | None = None


@dataclass(frozen=True)
class RobotGoalCommand2D:
    """Planar navigation command for one robot."""

    name: str
    position: Vec2
    yaw: float = 0.0
    z: float = 0.0
    command_id: str | None = None

    def to_pose(self) -> Pose:
        """Convert this command to a simulation ``Pose``."""
        return Pose.from_yaw(self.position[0], self.position[1], self.z, self.yaw)


@dataclass(frozen=True)
class RobotGoalCommand3D:
    """3D navigation command for one robot."""

    name: str
    position: Vec3
    orientation: Quat = (0.0, 0.0, 0.0, 1.0)
    command_id: str | None = None

    def to_pose(self) -> Pose:
        """Convert this command to a simulation ``Pose``."""
        return Pose(position=list(self.position), orientation=list(self.orientation))


@dataclass(frozen=True)
class RobotJointPositionsCommand:
    """Ordered joint target command for one robot."""

    name: str
    positions: tuple[float, ...]


@dataclass(frozen=True)
class RobotNamedJointPositionsCommand:
    """Named joint target command for one robot."""

    name: str
    positions: Mapping[str, float]

    def __post_init__(self) -> None:
        object.__setattr__(self, "positions", MappingProxyType(dict(self.positions)))


@dataclass(frozen=True)
class RobotAttachCommand:
    """Attach or detach a simulation object for one robot."""

    name: str
    attach: bool
    object_name: str = ""
    parent_link: str = "base_link"
    offset: Pose = field(default_factory=lambda: Pose.from_xyz(0.0, 0.0, 0.0))
    search_radius: float = 0.5


@dataclass(frozen=True)
class RobotActionCommand:
    """Generic PyBulletFleet action command for one robot."""

    name: str
    action_type: str
    action_params_json: str = ""
    command_id: str | None = None


@dataclass(frozen=True)
class CommandAck:
    """Result of accepting or rejecting a fleet command."""

    command_id: str
    source: str
    sim_time: float
    accepted_names: tuple[str, ...] = field(default_factory=tuple)
    rejected: Mapping[str, str] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "rejected", MappingProxyType(dict(self.rejected)))

    @property
    def ok(self) -> bool:
        """Whether at least one target was accepted and none were rejected."""
        return bool(self.accepted_names) and not self.rejected


@dataclass(frozen=True)
class CommandEvent:
    """Command trace event emitted before mutating simulation state."""

    command_id: str
    source: str
    sim_time: float
    command_type: str
    target_names: tuple[str, ...]
    accepted_names: tuple[str, ...]
    rejected: Mapping[str, str] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "rejected", MappingProxyType(dict(self.rejected)))


class FleetStateProvider:
    """Generate deterministic fleet state values from a simulation core."""

    def __init__(self, sim_core: Any) -> None:
        self.sim_core = sim_core

    def get_states(self) -> list[RobotState3D]:
        """Return 3D states for all agents.

        ``get_states()`` uses the more general 3D representation by default.
        Call ``get_states_2d()`` when a planar transport wants a smaller model.
        """
        return self.get_states_3d()

    def get_states_2d(self, names: Iterable[str] | None = None) -> list[RobotState2D]:
        """Return planar states for all or selected agents."""
        selected = _normalize_names(names)
        states = []
        for agent in _iter_agents(self.sim_core):
            name = _agent_name(agent)
            if selected is not None and name not in selected:
                continue
            pose = agent.get_pose()
            velocity = _vec3(getattr(agent, "velocity", (0.0, 0.0, 0.0)))
            states.append(
                RobotState2D(
                    name=name,
                    object_id=_agent_object_id(agent),
                    position=(float(pose.x), float(pose.y)),
                    yaw=float(pose.yaw),
                    linear_velocity=(velocity[0], velocity[1]),
                    angular_velocity=_yaw_rate(getattr(agent, "angular_velocity", 0.0)),
                    is_moving=bool(getattr(agent, "is_moving", False)),
                    battery_soc=_optional_float(getattr(agent, "battery_soc", None)),
                    is_charging=_optional_bool(getattr(agent, "is_charging", None)),
                )
            )
        return states

    def get_states_3d(self, names: Iterable[str] | None = None) -> list[RobotState3D]:
        """Return 3D states for all or selected agents."""
        selected = _normalize_names(names)
        states = []
        for agent in _iter_agents(self.sim_core):
            name = _agent_name(agent)
            if selected is not None and name not in selected:
                continue
            pose = agent.get_pose()
            states.append(
                RobotState3D(
                    name=name,
                    object_id=_agent_object_id(agent),
                    position=_vec3(pose.position),
                    orientation=_quat(pose.orientation),
                    linear_velocity=_vec3(getattr(agent, "velocity", (0.0, 0.0, 0.0))),
                    angular_velocity=_angular_velocity_vec3(getattr(agent, "angular_velocity", (0.0, 0.0, 0.0))),
                    is_moving=bool(getattr(agent, "is_moving", False)),
                    battery_soc=_optional_float(getattr(agent, "battery_soc", None)),
                    is_charging=_optional_bool(getattr(agent, "is_charging", None)),
                )
            )
        return states


class FleetCommandDispatcher:
    """Validate and apply fleet commands against a simulation core."""

    def __init__(self, sim_core: Any) -> None:
        self.sim_core = sim_core
        self.command_events: list[CommandEvent] = []
        self._name_index: dict[str, Any | None] = {}
        self.refresh_name_index()

    def refresh_name_index(self) -> None:
        """Rebuild the robot-name index used for command dispatch."""
        index: dict[str, Any | None] = {}
        for agent in _iter_agents(self.sim_core):
            explicit_name = _explicit_agent_name(agent)
            if explicit_name is None:
                continue
            index[explicit_name] = None if explicit_name in index else agent
        self._name_index = index

    def navigate(
        self,
        goals: Iterable[RobotGoalCommand2D | RobotGoalCommand3D],
        *,
        source: str = "python",
        command_id: str | None = None,
    ) -> CommandAck:
        """Apply one or more navigation goals by robot name."""
        commands = tuple(goals)
        resolved_id = _resolve_command_id(command_id, commands)
        accepted, rejected = self._resolve_targets(command.name for command in commands)
        ack = self._ack("navigate", resolved_id, source, tuple(command.name for command in commands), accepted, rejected)
        by_name = _first_command_by_name(commands)
        for name in ack.accepted_names:
            accepted[name].set_goal_pose(by_name[name].to_pose())
        return ack

    def joint_command(
        self,
        commands: Iterable[RobotJointPositionsCommand | RobotNamedJointPositionsCommand],
        *,
        source: str = "python",
        command_id: str | None = None,
    ) -> CommandAck:
        """Apply one or more joint target commands by robot name."""
        command_tuple = tuple(commands)
        resolved_id = _resolve_command_id(command_id, command_tuple)
        accepted, rejected = self._resolve_targets(command.name for command in command_tuple)
        ack = self._ack(
            "joint_command",
            resolved_id,
            source,
            tuple(command.name for command in command_tuple),
            accepted,
            rejected,
        )
        by_name = _first_command_by_name(command_tuple)
        for name in ack.accepted_names:
            command = by_name[name]
            agent = accepted[name]
            if isinstance(command, RobotJointPositionsCommand):
                agent.set_all_joints_targets(list(command.positions))
            else:
                agent.set_joints_targets_by_name(dict(command.positions))
        return ack

    def stop(
        self,
        names: Iterable[str],
        *,
        source: str = "python",
        command_id: str | None = None,
    ) -> CommandAck:
        """Stop one or more robots by name."""
        target_names = tuple(names)
        resolved_id = command_id or _new_command_id()
        accepted, rejected = self._resolve_targets(target_names)
        ack = self._ack("stop", resolved_id, source, target_names, accepted, rejected)
        for name in ack.accepted_names:
            accepted[name].stop()
        return ack

    def attach(
        self,
        commands: Iterable[RobotAttachCommand],
        *,
        source: str = "python",
        command_id: str | None = None,
    ) -> CommandAck:
        """Attach or detach objects for one or more robots by name."""
        command_tuple = tuple(commands)
        resolved_id = _resolve_command_id(command_id, command_tuple)
        accepted, rejected = self._resolve_targets(command.name for command in command_tuple)
        by_name = _first_command_by_name(command_tuple)
        targets: dict[str, Any] = {}
        for name, agent in tuple(accepted.items()):
            target, reason = _resolve_attach_target(agent, by_name[name])
            if reason is not None:
                rejected[name] = reason
                del accepted[name]
                continue
            targets[name] = target

        event = self._emit_command_event(
            "attach",
            resolved_id,
            source,
            tuple(command.name for command in command_tuple),
            accepted,
            rejected,
        )
        for name, agent in tuple(accepted.items()):
            command = by_name[name]
            target = targets[name]
            if command.attach:
                ok = agent.attach_object(
                    target,
                    parent_link_index=command.parent_link or "base_link",
                    relative_pose=command.offset,
                )
            else:
                ok = agent.detach_object(target)
            if not ok:
                rejected[name] = "attach mutation failed" if command.attach else "detach mutation failed"
                del accepted[name]

        return self._command_ack(event, accepted, rejected)

    def execute_action(
        self,
        commands: Iterable[RobotActionCommand],
        *,
        source: str = "python",
        command_id: str | None = None,
    ) -> CommandAck:
        """Queue generic PyBulletFleet actions for one or more robots."""
        command_tuple = tuple(commands)
        resolved_id = _resolve_command_id(command_id, command_tuple)
        accepted, rejected = self._resolve_targets(command.name for command in command_tuple)
        by_name = _first_command_by_name(command_tuple)
        actions: dict[str, Any] = {}
        for name in tuple(accepted.keys()):
            action = _build_action_command(by_name[name])
            if action is None:
                rejected[name] = f"invalid action '{by_name[name].action_type}'"
                del accepted[name]
                continue
            actions[name] = action

        ack = self._ack(
            "execute_action",
            resolved_id,
            source,
            tuple(command.name for command in command_tuple),
            accepted,
            rejected,
        )
        for name in ack.accepted_names:
            accepted[name].add_action(actions[name])
        return ack

    def _resolve_targets(self, names: Iterable[str]) -> tuple[dict[str, Any], dict[str, str]]:
        self.refresh_name_index()
        accepted: dict[str, Any] = {}
        rejected: dict[str, str] = {}
        target_names = tuple(names)
        counts = Counter(target_names)
        for name in target_names:
            if name in accepted or name in rejected:
                continue
            if counts[name] > 1:
                rejected[name] = "duplicate target"
                continue
            if name not in self._name_index:
                rejected[name] = "unknown robot"
                continue
            agent = self._name_index[name]
            if agent is None:
                rejected[name] = "ambiguous robot name"
                continue
            accepted[name] = agent
        return accepted, rejected

    def _ack(
        self,
        command_type: str,
        command_id: str,
        source: str,
        target_names: tuple[str, ...],
        accepted: Mapping[str, Any],
        rejected: Mapping[str, str],
    ) -> CommandAck:
        event = self._emit_command_event(command_type, command_id, source, target_names, accepted, rejected)
        return self._command_ack(event, accepted, rejected)

    def _emit_command_event(
        self,
        command_type: str,
        command_id: str,
        source: str,
        target_names: tuple[str, ...],
        accepted: Mapping[str, Any],
        rejected: Mapping[str, str],
    ) -> CommandEvent:
        sim_time = float(getattr(self.sim_core, "sim_time", 0.0))
        accepted_names = tuple(accepted.keys())
        event = CommandEvent(
            command_id=command_id,
            source=source,
            sim_time=sim_time,
            command_type=command_type,
            target_names=target_names,
            accepted_names=accepted_names,
            rejected=dict(rejected),
        )
        self.command_events.append(event)
        events = getattr(self.sim_core, "events", None)
        if events is not None and hasattr(events, "emit"):
            events.emit(FLEET_COMMAND_EVENT, command_event=event)
        return event

    def _command_ack(
        self,
        event: CommandEvent,
        accepted: Mapping[str, Any],
        rejected: Mapping[str, str],
    ) -> CommandAck:
        return CommandAck(
            command_id=event.command_id,
            source=event.source,
            sim_time=event.sim_time,
            accepted_names=tuple(accepted.keys()),
            rejected=dict(rejected),
        )


def _iter_agents(sim_core: Any) -> Sequence[Any]:
    return tuple(getattr(sim_core, "agents", ()))


def _agent_name(agent: Any) -> str:
    name = _explicit_agent_name(agent)
    if name is not None:
        return name
    return f"agent_{getattr(agent, 'object_id', 'unknown')}"


def _explicit_agent_name(agent: Any) -> str | None:
    name = getattr(agent, "name", None)
    if name:
        return str(name)
    return None


def _agent_object_id(agent: Any) -> int:
    return int(getattr(agent, "object_id", -1))


def _normalize_names(names: Iterable[str] | None) -> set[str] | None:
    if names is None:
        return None
    return {str(name) for name in names}


def _vec3(value: Any) -> Vec3:
    if isinstance(value, Real):
        return (float(value), 0.0, 0.0)
    seq = tuple(value)
    if len(seq) == 0:
        return (0.0, 0.0, 0.0)
    if len(seq) == 1:
        return (float(seq[0]), 0.0, 0.0)
    if len(seq) == 2:
        return (float(seq[0]), float(seq[1]), 0.0)
    return (float(seq[0]), float(seq[1]), float(seq[2]))


def _quat(value: Any) -> Quat:
    seq = tuple(value)
    if len(seq) != 4:
        raise ValueError(f"Expected quaternion with 4 values, got {len(seq)}")
    return (float(seq[0]), float(seq[1]), float(seq[2]), float(seq[3]))


def _yaw_rate(value: Any) -> float:
    if isinstance(value, Real):
        return float(value)
    seq = tuple(value)
    if len(seq) == 0:
        return 0.0
    return float(seq[-1])


def _angular_velocity_vec3(value: Any) -> Vec3:
    if isinstance(value, Real):
        return (0.0, 0.0, float(value))
    return _vec3(value)


def _optional_float(value: Any) -> float | None:
    return None if value is None else float(value)


def _optional_bool(value: Any) -> bool | None:
    return None if value is None else bool(value)


def _resolve_command_id(command_id: str | None, commands: Iterable[Any]) -> str:
    if command_id:
        return command_id
    for command in commands:
        item_id = getattr(command, "command_id", None)
        if item_id:
            return str(item_id)
    return _new_command_id()


def _new_command_id() -> str:
    return uuid4().hex


def _first_command_by_name(commands: Iterable[Any]) -> dict[str, Any]:
    by_name: dict[str, Any] = {}
    for command in commands:
        by_name.setdefault(command.name, command)
    return by_name


def _resolve_attach_target(agent: Any, command: RobotAttachCommand) -> tuple[Any | None, str | None]:
    if command.attach:
        if command.object_name:
            obj = _find_sim_object(agent, command.object_name)
            if obj is None:
                return None, f"object '{command.object_name}' not found"
            return obj, None
        finder = getattr(agent, "find_nearest_pickable", None)
        if finder is None:
            return None, "robot does not support nearest pickable search"
        search_radius = float(command.search_radius)
        if search_radius <= 0.0:
            search_radius = 0.5
        obj = finder(search_radius=search_radius)
        if obj is None:
            return None, f"no pickable object within {search_radius:.3g}m"
        return obj, None

    attached_getter = getattr(agent, "get_attached_objects", None)
    if attached_getter is None:
        return None, "robot does not expose attached objects"
    attached = tuple(attached_getter())
    if command.object_name:
        for obj in attached:
            if getattr(obj, "name", None) == command.object_name:
                return obj, None
        return None, f"object '{command.object_name}' not attached"
    if not attached:
        return None, "no attached object to detach"
    return attached[0], None


def _find_sim_object(agent: Any, object_name: str) -> Any | None:
    sim_core = getattr(agent, "sim_core", None)
    if sim_core is None:
        return None
    for obj in getattr(sim_core, "sim_objects", ()):
        if getattr(obj, "name", None) == object_name:
            return obj
    return None


def _build_action_command(command: RobotActionCommand) -> Any | None:
    from pybullet_fleet.action_parser import parse_action_goal

    return parse_action_goal(command.action_type, command.action_params_json)
