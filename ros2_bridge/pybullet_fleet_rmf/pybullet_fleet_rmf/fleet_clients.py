"""RMF client implementations for PyBulletFleet transports."""

from __future__ import annotations

import logging
import math
import threading
from typing import Any, Optional

from geometry_msgs.msg import Pose as RosPose, Point, Quaternion
from rclpy.node import Node
from std_msgs.msg import Header
from std_srvs.srv import SetBool

from pybullet_fleet_msgs.msg import FleetState, RobotAttachCommand, RobotGoal2D
from pybullet_fleet_msgs.srv import FleetAttach as FleetAttachSrv
from pybullet_fleet_msgs.srv import FleetNavigate as FleetNavigateSrv
from pybullet_fleet_msgs.srv import FleetStop as FleetStopSrv

from pybullet_fleet.fleet_api import (
    FleetCommandDispatcher,
    FleetStateProvider,
    RobotAttachCommand as PbfRobotAttachCommand,
    RobotGoalCommand2D,
)
from pybullet_fleet.geometry import Pose as PbfPose
from pybullet_fleet_rmf.client_interface import RobotUpdateData
from pybullet_fleet_rmf.per_robot_ros_client import PerRobotRosClient

logger = logging.getLogger(__name__)


class PerRobotRosClientFactory:
    """Factory for the existing per-robot ROS action/service client."""

    def __init__(self, node: Node, map_name: str = "L1") -> None:
        self._node = node
        self._map_name = map_name

    def robot(self, robot_name: str) -> PerRobotRosClient:
        """Create one per-robot ROS client."""
        return PerRobotRosClient(robot_name=robot_name, node=self._node, map_name=self._map_name)


class _RmfRobotClientBase:
    """Shared RMF-facing per-robot facade logic for fleet clients.

    Subclasses share RMF command bookkeeping and delegate transport-specific
    calls to their fleet backend. Charging remains transport-specific.
    """

    def __init__(self, robot_name: str, fleet: Any) -> None:
        self._name = robot_name
        self._fleet = fleet
        self._map_name = fleet._default_map_name
        self._active_cmd_id = 0
        self._active_target: Optional[list] = None

    def get_data(self) -> RobotUpdateData | None:
        data = self._fleet.get_state(self._name)
        if data is None:
            return None
        reached_target = (
            self._active_target is not None
            and _distance_xy(data.position, self._active_target) <= self._fleet._completion_radius
        )
        if reached_target:
            self._fleet.mark_completed(self._name, self._active_cmd_id)
            data.last_completed_cmd_id = max(data.last_completed_cmd_id, self._active_cmd_id)
            self._active_target = None
        return data

    def set_map_name(self, map_name: str) -> None:
        self._map_name = map_name
        self._fleet.set_robot_map_name(self._name, map_name)

    def navigate(self, cmd_id: int, position: list, map_name: str, speed_limit: float = 0.0) -> bool:
        del speed_limit
        self._active_cmd_id = cmd_id
        self._active_target = list(position)
        return self._fleet._command_navigate(self._name, cmd_id, position, map_name)

    def stop(self) -> bool:
        self._active_cmd_id += 1
        self._active_target = None
        return self._fleet._command_stop(self._name, self._active_cmd_id)

    def toggle_attach(self, attach: bool, cmd_id: int) -> bool:
        self._active_cmd_id = cmd_id
        return self._fleet._command_attach(self._name, cmd_id, attach=attach, search_radius=1.0)

    def attach_object(
        self,
        attach: bool,
        cmd_id: int,
        object_name: str = "",
        parent_link: str = "",
        offset_position: tuple = (0.0, 0.0, 0.0),
        offset_orientation: tuple = (0.0, 0.0, 0.0, 1.0),
        search_radius: float = 0.0,
    ) -> bool:
        self._active_cmd_id = cmd_id
        return self._fleet._command_attach(
            self._name,
            cmd_id,
            attach=attach,
            object_name=object_name,
            parent_link=parent_link,
            offset_position=offset_position,
            offset_orientation=offset_orientation,
            search_radius=search_radius,
        )


class RosRmfFleetClient:
    """Shared ROS fleet-endpoint client for RMF.

    The client consumes ``/fleet/states`` once and sends navigation commands
    through the ``/fleet/navigate`` service. Per-robot service clients are still
    used for delivery/charging compatibility until fleet-level equivalents are
    added.
    """

    def __init__(
        self,
        node: Node,
        map_name: str = "L1",
        *,
        completion_radius: float = 0.25,
    ) -> None:
        self._node = node
        self._default_map_name = map_name
        self._completion_radius = float(completion_radius)
        self._lock = threading.Lock()
        self._states: dict[str, RobotUpdateData] = {}
        self._map_names: dict[str, str] = {}
        self._state_sub = node.create_subscription(FleetState, "/fleet/states", self._on_fleet_state, 10)
        self._navigate_client = node.create_client(FleetNavigateSrv, "/fleet/navigate")
        self._stop_client = node.create_client(FleetStopSrv, "/fleet/stop")
        self._attach_client = node.create_client(FleetAttachSrv, "/fleet/attach")

    def robot(self, robot_name: str) -> "RosRmfFleetRobotClient":
        """Return a per-robot facade over the shared fleet endpoints."""
        return RosRmfFleetRobotClient(robot_name, self)

    def get_state(self, robot_name: str) -> RobotUpdateData | None:
        """Return cached state for ``robot_name``."""
        with self._lock:
            data = self._states.get(robot_name)
            if data is None:
                return None
            return RobotUpdateData(
                map=data.map,
                position=list(data.position),
                battery_soc=data.battery_soc,
                last_completed_cmd_id=data.last_completed_cmd_id,
            )

    def set_robot_map_name(self, robot_name: str, map_name: str) -> None:
        """Set the map used for one robot's RMF state updates."""
        with self._lock:
            self._map_names[robot_name] = map_name
            data = self._states.get(robot_name)
            if data is not None:
                data.map = map_name

    def _on_fleet_state(self, msg: FleetState) -> None:
        with self._lock:
            for robot in msg.robots:
                battery_soc = robot.battery_soc if robot.has_battery_soc else 1.0
                prior = self._states.get(robot.name)
                last_completed = prior.last_completed_cmd_id if prior is not None else 0
                map_name = self._map_names.get(robot.name, self._default_map_name)
                self._states[robot.name] = RobotUpdateData(
                    map=map_name,
                    position=[
                        float(robot.pose.position.x),
                        float(robot.pose.position.y),
                        _quat_to_yaw(robot.pose.orientation),
                    ],
                    battery_soc=float(battery_soc),
                    last_completed_cmd_id=last_completed,
                )

    def mark_completed(self, robot_name: str, cmd_id: int) -> None:
        """Mark an RMF command id as completed for one robot.

        This updates only the RMF-facing completion watermark returned by
        ``RobotUpdateData.last_completed_cmd_id``. It does not mean every
        PyBulletFleet action has finished; navigation completion is still
        detected separately from pose proximity in the per-robot facade.
        """
        with self._lock:
            data = self._states.get(robot_name)
            if data is None:
                return
            data.last_completed_cmd_id = max(data.last_completed_cmd_id, cmd_id)

    def call_navigate(self, robot_name: str, cmd_id: int, position: list, map_name: str) -> bool:
        """Send one fleet navigation request."""
        if not self._navigate_client.service_is_ready():
            if not self._navigate_client.wait_for_service(timeout_sec=5.0):
                logger.error("[%s] /fleet/navigate service not available", robot_name)
                return False

        req = FleetNavigateSrv.Request()
        req.header = _command_header(self._node)
        req.command_id = str(cmd_id)
        req.source = "rmf"
        goal = RobotGoal2D()
        goal.name = robot_name
        goal.position = [float(position[0]), float(position[1])]
        goal.yaw = float(position[2]) if len(position) > 2 else 0.0
        goal.z = 0.0
        req.goals_2d = [goal]
        future = self._navigate_client.call_async(req)
        future.add_done_callback(lambda done: self._on_navigate_ack(done, robot_name, cmd_id))
        return True

    def _command_navigate(self, robot_name: str, cmd_id: int, position: list, map_name: str) -> bool:
        """Transport hook used by the shared per-robot facade."""
        return self.call_navigate(robot_name, cmd_id, position, map_name)

    def _on_navigate_ack(self, future, robot_name: str, cmd_id: int) -> None:
        try:
            response = future.result()
        except Exception as exc:  # noqa: B902
            logger.error("[%s] /fleet/navigate call failed: %s", robot_name, exc)
            return
        ack = getattr(response, "ack", None)
        if ack is None:
            logger.warning("[%s] /fleet/navigate returned no ack", robot_name)
            return
        if robot_name in getattr(ack, "rejected_names", []):
            logger.warning("[%s] /fleet/navigate rejected cmd %s", robot_name, cmd_id)

    def call_stop(self, robot_name: str, cmd_id: int) -> bool:
        """Send one fleet stop request."""
        if not self._stop_client.service_is_ready():
            if not self._stop_client.wait_for_service(timeout_sec=2.0):
                logger.error("[%s] /fleet/stop service not available", robot_name)
                return False

        req = FleetStopSrv.Request()
        req.header = _command_header(self._node)
        req.command_id = str(cmd_id)
        req.source = "rmf"
        req.names = [robot_name]
        future = self._stop_client.call_async(req)
        future.add_done_callback(lambda done: self._on_stop_ack(done, robot_name, cmd_id))
        return True

    def _command_stop(self, robot_name: str, cmd_id: int) -> bool:
        """Transport hook used by the shared per-robot facade."""
        return self.call_stop(robot_name, cmd_id)

    def _on_stop_ack(self, future, robot_name: str, cmd_id: int) -> None:
        try:
            response = future.result()
        except Exception as exc:  # noqa: B902
            logger.error("[%s] /fleet/stop call failed: %s", robot_name, exc)
            return
        ack = getattr(response, "ack", None)
        if ack is None:
            logger.warning("[%s] /fleet/stop returned no ack", robot_name)
            return
        if robot_name in getattr(ack, "rejected_names", []):
            logger.warning("[%s] /fleet/stop rejected cmd %s", robot_name, cmd_id)
            return
        self.mark_completed(robot_name, cmd_id)

    def call_attach(
        self,
        robot_name: str,
        cmd_id: int,
        *,
        attach: bool,
        object_name: str = "",
        parent_link: str = "",
        offset_position: tuple = (0.0, 0.0, 0.0),
        offset_orientation: tuple = (0.0, 0.0, 0.0, 1.0),
        search_radius: float = 0.0,
    ) -> bool:
        """Send one fleet attach/detach request."""
        if not self._attach_client.service_is_ready():
            if not self._attach_client.wait_for_service(timeout_sec=5.0):
                logger.error("[%s] /fleet/attach service not available", robot_name)
                return False

        req = FleetAttachSrv.Request()
        req.header = _command_header(self._node)
        req.command_id = str(cmd_id)
        req.source = "rmf"
        command = RobotAttachCommand()
        command.name = robot_name
        command.attach = bool(attach)
        command.object_name = object_name
        command.parent_link = parent_link
        command.offset = RosPose(
            position=Point(
                x=float(offset_position[0]),
                y=float(offset_position[1]),
                z=float(offset_position[2]),
            ),
            orientation=Quaternion(
                x=float(offset_orientation[0]),
                y=float(offset_orientation[1]),
                z=float(offset_orientation[2]),
                w=float(offset_orientation[3]),
            ),
        )
        command.search_radius = float(search_radius)
        req.commands = [command]
        future = self._attach_client.call_async(req)
        future.add_done_callback(lambda done: self._on_attach_ack(done, robot_name, cmd_id))
        return True

    def _command_attach(
        self,
        robot_name: str,
        cmd_id: int,
        *,
        attach: bool,
        object_name: str = "",
        parent_link: str = "",
        offset_position: tuple = (0.0, 0.0, 0.0),
        offset_orientation: tuple = (0.0, 0.0, 0.0, 1.0),
        search_radius: float = 0.0,
    ) -> bool:
        """Transport hook used by the shared per-robot facade."""
        return self.call_attach(
            robot_name,
            cmd_id,
            attach=attach,
            object_name=object_name,
            parent_link=parent_link,
            offset_position=offset_position,
            offset_orientation=offset_orientation,
            search_radius=search_radius,
        )

    def _on_attach_ack(self, future, robot_name: str, cmd_id: int) -> None:
        try:
            response = future.result()
        except Exception as exc:  # noqa: B902
            logger.error("[%s] /fleet/attach call failed: %s", robot_name, exc)
            return
        ack = getattr(response, "ack", None)
        if ack is None:
            logger.warning("[%s] /fleet/attach returned no ack", robot_name)
            return
        if robot_name in getattr(ack, "rejected_names", []):
            logger.warning("[%s] /fleet/attach rejected cmd %s", robot_name, cmd_id)
            return
        self.mark_completed(robot_name, cmd_id)


class RosRmfFleetRobotClient(_RmfRobotClientBase):
    """Per-robot facade over :class:`RosRmfFleetClient`."""

    def __init__(self, robot_name: str, fleet: RosRmfFleetClient) -> None:
        super().__init__(robot_name, fleet)
        self._node = fleet._node
        self._charging_client = self._node.create_client(SetBool, f"/{robot_name}/set_charging")

    def start_charge(self, cmd_id: int) -> bool:
        self._active_cmd_id = cmd_id
        if not self._charging_client.service_is_ready():
            if not self._charging_client.wait_for_service(timeout_sec=2.0):
                logger.error("[%s] set_charging service not available", self._name)
                return False
        future = self._charging_client.call_async(_set_bool_request(True))
        future.add_done_callback(lambda done: self._on_set_bool_done(done, cmd_id, "set_charging"))
        return True

    def stop_charge(self) -> bool:
        if not self._charging_client.service_is_ready():
            return False
        future = self._charging_client.call_async(_set_bool_request(False))
        future.add_done_callback(lambda done: self._log_set_bool_result(done, "stop_charge"))
        return True

    def _on_set_bool_done(self, future, cmd_id: int, label: str) -> None:
        self._log_set_bool_result(future, label)
        self._fleet.mark_completed(self._name, cmd_id)

    def _log_set_bool_result(self, future, label: str) -> None:
        try:
            result = future.result()
        except Exception as exc:  # noqa: B902
            logger.error("[%s] %s service call failed: %s", self._name, label, exc)
            return
        if result is None or not getattr(result, "success", False):
            msg = getattr(result, "message", "no response")
            logger.warning("[%s] %s failed: %s", self._name, label, msg)


class PythonRmfFleetClient:
    """Direct in-process fleet client for Plugin Only deployments.

    This client consumes :class:`FleetStateProvider` and
    :class:`FleetCommandDispatcher` directly. It does not create ROS bridge
    clients, so the RMF adapter can command the simulation without DDS
    serialization or per-robot bridge endpoints in the control path.
    """

    def __init__(
        self,
        provider: FleetStateProvider,
        dispatcher: FleetCommandDispatcher,
        map_name: str = "L1",
        *,
        completion_radius: float = 0.25,
    ) -> None:
        self._provider = provider
        self._dispatcher = dispatcher
        self._default_map_name = map_name
        self._completion_radius = float(completion_radius)
        self._lock = threading.Lock()
        self._map_names: dict[str, str] = {}
        self._completed: dict[str, int] = {}

    @classmethod
    def from_sim_core(
        cls,
        sim_core: Any,
        map_name: str = "L1",
        *,
        completion_radius: float = 0.25,
    ) -> "PythonRmfFleetClient":
        """Build provider and dispatcher from one simulation core."""
        return cls(
            FleetStateProvider(sim_core),
            FleetCommandDispatcher(sim_core),
            map_name=map_name,
            completion_radius=completion_radius,
        )

    def robot(self, robot_name: str) -> "PythonRmfFleetRobotClient":
        """Return a per-robot facade over the direct fleet API."""
        return PythonRmfFleetRobotClient(robot_name, self)

    def get_state(self, robot_name: str) -> RobotUpdateData | None:
        """Return current state for ``robot_name`` from the provider."""
        for state in self._provider.get_states_3d(names=[robot_name]):
            yaw = _quat_tuple_to_yaw(state.orientation)
            with self._lock:
                map_name = self._map_names.get(robot_name, self._default_map_name)
                completed = self._completed.get(robot_name, 0)
            return RobotUpdateData(
                map=map_name,
                position=[float(state.position[0]), float(state.position[1]), yaw],
                battery_soc=float(state.battery_soc if state.battery_soc is not None else 1.0),
                last_completed_cmd_id=completed,
            )
        return None

    def set_robot_map_name(self, robot_name: str, map_name: str) -> None:
        """Set the map used for one robot's RMF state updates."""
        with self._lock:
            self._map_names[robot_name] = map_name

    def mark_completed(self, robot_name: str, cmd_id: int) -> None:
        """Mark an RMF command id as completed for one robot.

        This updates only the RMF-facing completion watermark returned by
        ``RobotUpdateData.last_completed_cmd_id``. It does not mean every
        PyBulletFleet action has finished; navigation completion is still
        detected separately from pose proximity in the per-robot facade.
        """
        with self._lock:
            self._completed[robot_name] = max(self._completed.get(robot_name, 0), int(cmd_id))

    def _command_navigate(self, robot_name: str, cmd_id: int, position: list, map_name: str) -> bool:
        """Dispatch one direct fleet navigation command."""
        del map_name
        ack = self._dispatcher.navigate(
            [
                RobotGoalCommand2D(
                    name=robot_name,
                    position=(float(position[0]), float(position[1])),
                    yaw=float(position[2]) if len(position) > 2 else 0.0,
                    z=0.0,
                )
            ],
            source="rmf-python",
            command_id=str(cmd_id),
        )
        return _ack_accepts(ack, robot_name, "navigate")

    def _command_stop(self, robot_name: str, cmd_id: int) -> bool:
        """Dispatch one direct fleet stop command."""
        ack = self._dispatcher.stop([robot_name], source="rmf-python", command_id=str(cmd_id))
        if not _ack_accepts(ack, robot_name, "stop"):
            return False
        self.mark_completed(robot_name, cmd_id)
        return True

    def _command_attach(
        self,
        robot_name: str,
        cmd_id: int,
        *,
        attach: bool,
        object_name: str = "",
        parent_link: str = "",
        offset_position: tuple = (0.0, 0.0, 0.0),
        offset_orientation: tuple = (0.0, 0.0, 0.0, 1.0),
        search_radius: float = 0.0,
    ) -> bool:
        """Dispatch one direct fleet attach/detach command."""
        ack = self._dispatcher.attach(
            [
                PbfRobotAttachCommand(
                    name=robot_name,
                    attach=bool(attach),
                    object_name=object_name,
                    parent_link=parent_link or "base_link",
                    offset=PbfPose(
                        position=[
                            float(offset_position[0]),
                            float(offset_position[1]),
                            float(offset_position[2]),
                        ],
                        orientation=[
                            float(offset_orientation[0]),
                            float(offset_orientation[1]),
                            float(offset_orientation[2]),
                            float(offset_orientation[3]),
                        ],
                    ),
                    search_radius=float(search_radius or 0.5),
                )
            ],
            source="rmf-python",
            command_id=str(cmd_id),
        )
        if not _ack_accepts(ack, robot_name, "attach"):
            return False
        self.mark_completed(robot_name, cmd_id)
        return True

    def navigate(self, robot_name: str, cmd_id: int, position: list, map_name: str) -> bool:
        """Backward-compatible direct navigation helper."""
        return self._command_navigate(robot_name, cmd_id, position, map_name)

    def stop(self, robot_name: str, cmd_id: int) -> bool:
        """Backward-compatible direct stop helper."""
        return self._command_stop(robot_name, cmd_id)

    def attach(
        self,
        robot_name: str,
        cmd_id: int,
        *,
        attach: bool,
        object_name: str = "",
        parent_link: str = "",
        offset_position: tuple = (0.0, 0.0, 0.0),
        offset_orientation: tuple = (0.0, 0.0, 0.0, 1.0),
        search_radius: float = 0.0,
    ) -> bool:
        """Backward-compatible direct attach/detach helper."""
        return self._command_attach(
            robot_name,
            cmd_id,
            attach=attach,
            object_name=object_name,
            parent_link=parent_link,
            offset_position=offset_position,
            offset_orientation=offset_orientation,
            search_radius=search_radius,
        )

    def set_charging(self, robot_name: str, cmd_id: int, charging: bool) -> bool:
        """Set charging directly on an agent when the core agent supports it."""
        agent = _direct_agent_by_name(self._dispatcher.sim_core, robot_name)
        if agent is None:
            logger.warning("[%s] direct set_charging rejected: unknown or ambiguous robot", robot_name)
            return False
        agent.set_charging(bool(charging))
        if charging:
            self.mark_completed(robot_name, cmd_id)
        return True


class PythonRmfFleetRobotClient(_RmfRobotClientBase):
    """Per-robot RMF facade over :class:`PythonRmfFleetClient`."""

    def __init__(self, robot_name: str, fleet: PythonRmfFleetClient) -> None:
        super().__init__(robot_name, fleet)

    def start_charge(self, cmd_id: int) -> bool:
        self._active_cmd_id = cmd_id
        return self._fleet.set_charging(self._name, cmd_id, True)

    def stop_charge(self) -> bool:
        return self._fleet.set_charging(self._name, self._active_cmd_id, False)


def create_rmf_client_factory(
    mode: str,
    node: Node,
    map_name: str = "L1",
    *,
    sim_core: Any | None = None,
    provider: FleetStateProvider | None = None,
    dispatcher: FleetCommandDispatcher | None = None,
):
    """Create an RMF client factory for ``mode``."""
    normalized = (mode or "per_robot_ros").strip().lower()
    if normalized == "per_robot_ros":
        return PerRobotRosClientFactory(node, map_name=map_name)
    if normalized == "fleet_ros":
        return RosRmfFleetClient(node, map_name=map_name)
    if normalized == "python_fleet":
        if provider is None or dispatcher is None:
            if sim_core is None:
                raise ValueError("python_fleet mode requires sim_core or provider+dispatcher")
            return PythonRmfFleetClient.from_sim_core(sim_core, map_name=map_name)
        return PythonRmfFleetClient(provider, dispatcher, map_name=map_name)
    raise ValueError(f"Unknown RMF client mode: {mode!r}")


def _set_bool_request(value: bool) -> SetBool.Request:
    req = SetBool.Request()
    req.data = bool(value)
    return req


def _command_header(node: Node, frame_id: str = "odom") -> Header:
    header = Header(frame_id=frame_id)
    try:
        stamp = node.get_clock().now().to_msg()
        header.stamp = stamp
    except Exception:  # noqa: B902
        pass
    return header


def _distance_xy(a: list, b: list) -> float:
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


def _ack_accepts(ack, robot_name: str, command_type: str) -> bool:
    if robot_name in ack.accepted_names:
        return True
    reason = ack.rejected.get(robot_name, "not accepted")
    logger.warning("[%s] direct %s rejected: %s", robot_name, command_type, reason)
    return False


def _direct_agent_by_name(sim_core: Any, robot_name: str) -> Any | None:
    matches = [agent for agent in getattr(sim_core, "agents", ()) if getattr(agent, "name", None) == robot_name]
    return matches[0] if len(matches) == 1 else None


def _quat_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _quat_tuple_to_yaw(q) -> float:
    x, y, z, w = q
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)
