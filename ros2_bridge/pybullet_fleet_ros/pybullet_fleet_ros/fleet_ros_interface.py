"""ROS wrappers for the transport-neutral fleet API."""

from __future__ import annotations

from typing import Iterable

from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import Header

from pybullet_fleet.fleet_api import (
    CommandAck as PbfCommandAck,
    FleetCommandDispatcher,
    FleetStateProvider,
    RobotGoalCommand2D,
    RobotGoalCommand3D,
    RobotJointPositionsCommand,
    RobotNamedJointPositionsCommand,
    RobotState3D,
)
try:
    from pybullet_fleet_msgs.msg import (
        CommandAck,
        FleetJointCommand,
        FleetNavigate,
        FleetState,
        RobotGoal2D,
        RobotGoal3D,
        RobotJointPositionsCommand as RobotJointPositionsCommandMsg,
        RobotNamedJointPositionsCommand as RobotNamedJointPositionsCommandMsg,
        RobotState3D as RobotState3DMsg,
    )
    from pybullet_fleet_msgs.srv import FleetJointCommand as FleetJointCommandSrv
    from pybullet_fleet_msgs.srv import FleetNavigate as FleetNavigateSrv

    _FLEET_MSGS_IMPORT_ERROR: ImportError | None = None
except ImportError as exc:
    CommandAck = None
    FleetJointCommand = None
    FleetNavigate = None
    FleetState = None
    RobotGoal2D = None
    RobotGoal3D = None
    RobotJointPositionsCommandMsg = None
    RobotNamedJointPositionsCommandMsg = None
    RobotState3DMsg = None
    FleetJointCommandSrv = None
    FleetNavigateSrv = None
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
        self.state_provider = FleetStateProvider(sim_core)
        self.command_dispatcher = FleetCommandDispatcher(sim_core)
        self._rmf_frame_offset = _node_frame_offset(node)

        self._state_pub = None
        self._navigate_sub = None
        self._navigate_srv = None
        self._joint_sub = None
        self._joint_srv = None

        if not config.enabled:
            return
        _require_fleet_msgs()

        if config.states:
            self._state_pub = node.create_publisher(FleetState, "/fleet/states", 10)
        if config.navigate:
            self._navigate_sub = node.create_subscription(FleetNavigate, "/fleet/navigate", self._on_navigate, 10)
            self._navigate_srv = node.create_service(FleetNavigateSrv, "/fleet/navigate", self._on_navigate_service)
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
            ("_joint_sub", self.node.destroy_subscription),
            ("_joint_srv", self.node.destroy_service),
        ):
            entity = getattr(self, attr)
            if entity is not None:
                destroy(entity)
                setattr(self, attr, None)

    def _on_navigate(self, msg: FleetNavigate) -> None:
        self._dispatch_navigate(msg)

    def _on_navigate_service(self, request, response):
        response.ack = command_ack_to_msg(self._dispatch_navigate(request))
        return response

    def _dispatch_navigate(self, msg: FleetNavigate) -> PbfCommandAck:
        return self.command_dispatcher.navigate(
            _navigation_goals_from_msg(msg, xy_offset=self._rmf_frame_offset),
            source=_source_from_msg(msg),
            command_id=msg.command_id or None,
        )

    def _on_joint_command(self, msg: FleetJointCommand) -> None:
        try:
            self._dispatch_joint_command(msg)
        except ValueError as exc:
            self.node.get_logger().warning(str(exc))

    def _on_joint_command_service(self, request, response):
        response.ack = command_ack_to_msg(self._dispatch_joint_command(request))
        return response

    def _dispatch_joint_command(self, msg: FleetJointCommand) -> PbfCommandAck:
        source = _source_from_msg(msg)
        command_id = msg.command_id or None
        try:
            commands = _joint_commands_from_msg(msg)
        except ValueError as exc:
            return PbfCommandAck(
                command_id=command_id or "invalid",
                source=source,
                sim_time=float(getattr(self.command_dispatcher.sim_core, "sim_time", 0.0)),
                rejected={"request": str(exc)},
            )
        return self.command_dispatcher.joint_command(commands, source=source, command_id=command_id)


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
) -> list[RobotJointPositionsCommand | RobotNamedJointPositionsCommand]:
    commands: list[RobotJointPositionsCommand | RobotNamedJointPositionsCommand] = [
        _joint_positions_from_msg(command) for command in msg.joint_position_commands
    ]
    commands.extend(_named_joint_positions_from_msg(command) for command in msg.named_joint_position_commands)
    return commands


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
