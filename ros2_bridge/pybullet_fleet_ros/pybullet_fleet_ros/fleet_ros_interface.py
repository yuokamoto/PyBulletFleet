"""ROS wrappers for the transport-neutral fleet API."""

from __future__ import annotations

from typing import Iterable

from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import Header

from pybullet_fleet.commands import (
    CommandAck as PbfCommandAck,
    DEFAULT_ATTACH_SEARCH_RADIUS,
    RobotActionCommand,
    RobotAttachCommand,
    RobotGoalCommand2D,
    RobotGoalCommand3D,
    RobotJointPositionsCommand,
    RobotNamedJointPositionsCommand,
)
from pybullet_fleet.fleet_api import (
    FleetCommandDispatcher,
    FleetStateProvider,
    RobotState3D,
)
from pybullet_fleet.geometry import Pose as PbfPose

try:
    from pybullet_fleet_msgs.msg import (
        CommandAck,
        FleetAttach,
        FleetExecuteAction,
        FleetJointCommand,
        FleetNavigate,
        FleetState,
        FleetStop,
        RobotGoal2D,
        RobotGoal3D,
        RobotJointPositionsCommand as RobotJointPositionsCommandMsg,
        RobotNamedJointPositionsCommand as RobotNamedJointPositionsCommandMsg,
        RobotState3D as RobotState3DMsg,
    )
    from pybullet_fleet_msgs.srv import FleetAttach as FleetAttachSrv
    from pybullet_fleet_msgs.srv import FleetExecuteAction as FleetExecuteActionSrv
    from pybullet_fleet_msgs.srv import FleetJointCommand as FleetJointCommandSrv
    from pybullet_fleet_msgs.srv import FleetNavigate as FleetNavigateSrv
    from pybullet_fleet_msgs.srv import FleetStop as FleetStopSrv

    _FLEET_MSGS_IMPORT_ERROR: ImportError | None = None
except ImportError as exc:
    CommandAck = None
    FleetAttach = None
    FleetExecuteAction = None
    FleetJointCommand = None
    FleetNavigate = None
    FleetState = None
    FleetStop = None
    RobotGoal2D = None
    RobotGoal3D = None
    RobotJointPositionsCommandMsg = None
    RobotNamedJointPositionsCommandMsg = None
    RobotState3DMsg = None
    FleetAttachSrv = None
    FleetExecuteActionSrv = None
    FleetJointCommandSrv = None
    FleetNavigateSrv = None
    FleetStopSrv = None
    _FLEET_MSGS_IMPORT_ERROR = exc

from .interface_config import FleetApiConfig


def _require_fleet_msgs() -> None:
    if _FLEET_MSGS_IMPORT_ERROR is None:
        return
    raise RuntimeError(
        "fleet_api is enabled but pybullet_fleet_msgs Python bindings are unavailable. "
        "Build/source the ROS workspace so pybullet_fleet_msgs is generated."
    ) from _FLEET_MSGS_IMPORT_ERROR


class FleetRosInterface:
    """Expose ``FleetStateProvider`` and ``FleetCommandDispatcher`` over ROS."""

    def __init__(self, node, sim_core, config: FleetApiConfig) -> None:
        self.node = node
        self.config = config

        self._state_pub = None
        self._navigate_sub = None
        self._navigate_srv = None
        self._stop_sub = None
        self._stop_srv = None
        self._attach_sub = None
        self._attach_srv = None
        self._execute_action_sub = None
        self._execute_action_srv = None
        self._joint_sub = None
        self._joint_srv = None

        if not config.enabled:
            return
        _require_fleet_msgs()

        self.state_provider = FleetStateProvider(sim_core)
        self.command_dispatcher = FleetCommandDispatcher(sim_core)
        self._rmf_frame_offset = _node_frame_offset(node)

        if config.states:
            self._state_pub = node.create_publisher(FleetState, "/fleet/states", 10)
        if config.navigate:
            self._navigate_sub = node.create_subscription(FleetNavigate, "/fleet/navigate", self._on_navigate, 10)
            self._navigate_srv = node.create_service(FleetNavigateSrv, "/fleet/navigate", self._on_navigate_service)
        if config.stop:
            self._stop_sub = node.create_subscription(FleetStop, "/fleet/stop", self._on_stop, 10)
            self._stop_srv = node.create_service(FleetStopSrv, "/fleet/stop", self._on_stop_service)
        if config.attach:
            self._attach_sub = node.create_subscription(FleetAttach, "/fleet/attach", self._on_attach, 10)
            self._attach_srv = node.create_service(FleetAttachSrv, "/fleet/attach", self._on_attach_service)
        if config.execute_action:
            self._execute_action_sub = node.create_subscription(
                FleetExecuteAction,
                "/fleet/execute_action",
                self._on_execute_action,
                10,
            )
            self._execute_action_srv = node.create_service(
                FleetExecuteActionSrv,
                "/fleet/execute_action",
                self._on_execute_action_service,
            )
        if config.joint_command:
            self._joint_sub = node.create_subscription(
                FleetJointCommand,
                "/fleet/joint_command",
                self._on_joint_command,
                10,
            )
            self._joint_srv = node.create_service(
                FleetJointCommandSrv,
                "/fleet/joint_command",
                self._on_joint_command_service,
            )

    def post_step(self, stamp=None) -> None:
        """Publish fleet state after a simulation step."""
        if self._state_pub is None:
            return
        self._state_pub.publish(
            fleet_state_to_msg(
                self.state_provider.get_states(),
                stamp=stamp,
                xy_offset=self._rmf_frame_offset,
            )
        )

    def destroy(self) -> None:
        """Destroy ROS entities created by this wrapper."""
        for attr, destroy in (
            ("_state_pub", self.node.destroy_publisher),
            ("_navigate_sub", self.node.destroy_subscription),
            ("_navigate_srv", self.node.destroy_service),
            ("_stop_sub", self.node.destroy_subscription),
            ("_stop_srv", self.node.destroy_service),
            ("_attach_sub", self.node.destroy_subscription),
            ("_attach_srv", self.node.destroy_service),
            ("_execute_action_sub", self.node.destroy_subscription),
            ("_execute_action_srv", self.node.destroy_service),
            ("_joint_sub", self.node.destroy_subscription),
            ("_joint_srv", self.node.destroy_service),
        ):
            entity = getattr(self, attr)
            if entity is not None:
                destroy(entity)
                setattr(self, attr, None)

    def _on_navigate(self, msg: FleetNavigate) -> None:
        self._log_rejections(self._dispatch_navigate(msg))

    def _on_navigate_service(self, request, response):
        response.ack = command_ack_to_msg(self._dispatch_navigate(request))
        return response

    def _dispatch_navigate(self, msg: FleetNavigate) -> PbfCommandAck:
        frame_rejections = _navigation_frame_rejections(msg)
        if frame_rejections:
            return PbfCommandAck(
                command_id=msg.command_id,
                source=_source_from_msg(msg),
                sim_time=float(getattr(self.command_dispatcher.sim_core, "sim_time", 0.0)),
                rejected=frame_rejections,
            )
        return self.command_dispatcher.navigate(
            _navigation_goals_from_msg(msg, xy_offset=self._rmf_frame_offset),
            source=_source_from_msg(msg),
            command_id=msg.command_id or None,
        )

    def _on_stop(self, msg: FleetStop) -> None:
        self._log_rejections(self._dispatch_stop(msg))

    def _on_stop_service(self, request, response):
        response.ack = command_ack_to_msg(self._dispatch_stop(request))
        return response

    def _dispatch_stop(self, msg: FleetStop) -> PbfCommandAck:
        return self.command_dispatcher.stop(
            tuple(msg.names),
            source=_source_from_msg(msg),
            command_id=msg.command_id or None,
        )

    def _on_attach(self, msg: FleetAttach) -> None:
        self._log_rejections(self._dispatch_attach(msg))

    def _on_attach_service(self, request, response):
        response.ack = command_ack_to_msg(self._dispatch_attach(request))
        return response

    def _dispatch_attach(self, msg: FleetAttach) -> PbfCommandAck:
        return self.command_dispatcher.attach(
            _attach_commands_from_msg(msg),
            source=_source_from_msg(msg),
            command_id=msg.command_id or None,
        )

    def _on_execute_action(self, msg: FleetExecuteAction) -> None:
        self._log_rejections(self._dispatch_execute_action(msg))

    def _on_execute_action_service(self, request, response):
        response.ack = command_ack_to_msg(self._dispatch_execute_action(request))
        return response

    def _dispatch_execute_action(self, msg: FleetExecuteAction) -> PbfCommandAck:
        return self.command_dispatcher.execute_action(
            _action_commands_from_msg(msg),
            source=_source_from_msg(msg),
            command_id=msg.command_id or None,
        )

    def _on_joint_command(self, msg: FleetJointCommand) -> None:
        self._log_rejections(self._dispatch_joint_command(msg))

    def _on_joint_command_service(self, request, response):
        response.ack = command_ack_to_msg(self._dispatch_joint_command(request))
        return response

    def _dispatch_joint_command(self, msg: FleetJointCommand) -> PbfCommandAck:
        source = _source_from_msg(msg)
        command_id = msg.command_id or None
        commands, validation_rejected = _joint_commands_from_msg(msg)
        if not commands:
            return PbfCommandAck(
                command_id=msg.command_id,
                source=source,
                sim_time=float(getattr(self.command_dispatcher.sim_core, "sim_time", 0.0)),
                rejected=validation_rejected,
            )
        ack = self.command_dispatcher.joint_command(commands, source=source, command_id=command_id)
        if not validation_rejected:
            return ack
        return PbfCommandAck(
            command_id=ack.command_id,
            source=ack.source,
            sim_time=ack.sim_time,
            accepted_names=ack.accepted_names,
            rejected={**dict(ack.rejected), **validation_rejected},
        )

    def _log_rejections(self, ack: PbfCommandAck) -> None:
        if not ack.rejected:
            return
        details = ", ".join(f"{name}: {reason}" for name, reason in ack.rejected.items())
        self.node.get_logger().warning(f"Fleet command '{ack.command_id}' rejected targets: {details}")


def fleet_state_to_msg(
    states: Iterable[RobotState3D],
    stamp=None,
    frame_id: str = "odom",
    xy_offset: tuple[float, float] = (0.0, 0.0),
) -> FleetState:
    """Convert Python fleet state values into a ROS fleet state message."""
    _require_fleet_msgs()
    msg = FleetState()
    msg.header = Header(frame_id=frame_id)
    if stamp is not None:
        msg.header.stamp = stamp
    msg.robots = [robot_state3d_to_msg(state, xy_offset=xy_offset) for state in states]
    return msg


def robot_state3d_to_msg(state: RobotState3D, xy_offset: tuple[float, float] = (0.0, 0.0)) -> RobotState3DMsg:
    """Convert one 3D robot state into its ROS message encoding."""
    _require_fleet_msgs()
    msg = RobotState3DMsg()
    msg.name = state.name
    msg.object_id = int(state.object_id)
    msg.pose = Pose(
        position=Point(
            x=state.position[0] + xy_offset[0],
            y=state.position[1] + xy_offset[1],
            z=state.position[2],
        ),
        orientation=Quaternion(
            x=state.orientation[0],
            y=state.orientation[1],
            z=state.orientation[2],
            w=state.orientation[3],
        ),
    )
    msg.twist = Twist(
        linear=Vector3(
            x=state.linear_velocity[0],
            y=state.linear_velocity[1],
            z=state.linear_velocity[2],
        ),
        angular=Vector3(
            x=state.angular_velocity[0],
            y=state.angular_velocity[1],
            z=state.angular_velocity[2],
        ),
    )
    msg.is_moving = bool(state.is_moving)
    msg.has_battery_soc = state.battery_soc is not None
    msg.battery_soc = float(state.battery_soc or 0.0)
    msg.has_is_charging = state.is_charging is not None
    msg.is_charging = bool(state.is_charging)
    return msg


def command_ack_to_msg(ack: PbfCommandAck) -> CommandAck:
    """Convert a Python command acknowledgement into a ROS message."""
    _require_fleet_msgs()
    msg = CommandAck()
    msg.command_id = ack.command_id
    msg.source = ack.source
    msg.sim_time = ack.sim_time
    msg.accepted_names = list(ack.accepted_names)
    msg.rejected_names = list(ack.rejected.keys())
    msg.reject_reasons = list(ack.rejected.values())
    return msg


def _navigation_goals_from_msg(
    msg: FleetNavigate,
    xy_offset: tuple[float, float] = (0.0, 0.0),
) -> list[RobotGoalCommand2D | RobotGoalCommand3D]:
    return [_goal_2d_from_msg(goal, xy_offset=xy_offset) for goal in msg.goals_2d] + [
        _goal_3d_from_msg(goal, xy_offset=xy_offset) for goal in msg.goals_3d
    ]


def _navigation_frame_rejections(msg: FleetNavigate) -> dict[str, str]:
    frame_id = getattr(getattr(msg, "header", None), "frame_id", "")
    if frame_id in ("", "odom"):
        return {}
    names = [goal.name for goal in msg.goals_2d] + [goal.name for goal in msg.goals_3d]
    return {name: f"unsupported frame_id {frame_id!r}; expected 'odom'" for name in names}


def _attach_commands_from_msg(msg: FleetAttach) -> list[RobotAttachCommand]:
    commands = []
    for command in msg.commands:
        offset = command.offset
        offset_orientation = [
            float(offset.orientation.x),
            float(offset.orientation.y),
            float(offset.orientation.z),
            float(offset.orientation.w),
        ]
        if all(v == 0.0 for v in offset_orientation):
            offset_orientation = [0.0, 0.0, 0.0, 1.0]
        search_radius = float(command.search_radius)
        if search_radius <= 0.0:
            search_radius = DEFAULT_ATTACH_SEARCH_RADIUS
        commands.append(
            RobotAttachCommand(
                name=command.name,
                attach=bool(command.attach),
                object_name=command.object_name,
                parent_link=command.parent_link or "base_link",
                offset=PbfPose(
                    position=[
                        float(offset.position.x),
                        float(offset.position.y),
                        float(offset.position.z),
                    ],
                    orientation=offset_orientation,
                ),
                search_radius=search_radius,
            )
        )
    return commands


def _action_commands_from_msg(msg: FleetExecuteAction) -> list[RobotActionCommand]:
    return [
        RobotActionCommand(
            name=command.name,
            action_type=command.action_type,
            action_params_json=command.action_params_json,
            command_id=msg.command_id or None,
        )
        for command in msg.commands
    ]


def _goal_2d_from_msg(msg: RobotGoal2D, xy_offset: tuple[float, float] = (0.0, 0.0)) -> RobotGoalCommand2D:
    return RobotGoalCommand2D(
        name=msg.name,
        position=(float(msg.position[0]) - xy_offset[0], float(msg.position[1]) - xy_offset[1]),
        yaw=float(msg.yaw),
        z=float(msg.z),
    )


def _goal_3d_from_msg(msg: RobotGoal3D, xy_offset: tuple[float, float] = (0.0, 0.0)) -> RobotGoalCommand3D:
    return RobotGoalCommand3D(
        name=msg.name,
        position=(
            float(msg.pose.position.x) - xy_offset[0],
            float(msg.pose.position.y) - xy_offset[1],
            float(msg.pose.position.z),
        ),
        orientation=(
            float(msg.pose.orientation.x),
            float(msg.pose.orientation.y),
            float(msg.pose.orientation.z),
            float(msg.pose.orientation.w),
        ),
    )


def _joint_commands_from_msg(
    msg: FleetJointCommand,
) -> tuple[list[RobotJointPositionsCommand | RobotNamedJointPositionsCommand], dict[str, str]]:
    commands: list[RobotJointPositionsCommand | RobotNamedJointPositionsCommand] = [
        _joint_positions_from_msg(command) for command in msg.joint_position_commands
    ]
    rejected: dict[str, str] = {}
    for command in msg.named_joint_position_commands:
        try:
            commands.append(_named_joint_positions_from_msg(command))
        except ValueError as exc:
            rejected[command.name] = str(exc)
    return commands, rejected


def _joint_positions_from_msg(msg: RobotJointPositionsCommandMsg) -> RobotJointPositionsCommand:
    return RobotJointPositionsCommand(name=msg.name, positions=tuple(float(value) for value in msg.positions))


def _named_joint_positions_from_msg(msg: RobotNamedJointPositionsCommandMsg) -> RobotNamedJointPositionsCommand:
    if len(msg.joint_names) != len(msg.positions):
        raise ValueError(
            f"Named joint command for '{msg.name}' has {len(msg.joint_names)} joint names "
            f"but {len(msg.positions)} positions"
        )
    return RobotNamedJointPositionsCommand(
        name=msg.name,
        positions={name: float(position) for name, position in zip(msg.joint_names, msg.positions)},
    )


def _source_from_msg(msg) -> str:
    return msg.source or "ros2"


def _node_frame_offset(node) -> tuple[float, float]:
    value = getattr(node, "rmf_frame_offset", (0.0, 0.0))
    if isinstance(value, (list, tuple)) and len(value) >= 2:
        return (float(value[0]), float(value[1]))
    return (0.0, 0.0)
