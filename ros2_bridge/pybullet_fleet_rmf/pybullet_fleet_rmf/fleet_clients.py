"""RMF client implementations for PyBulletFleet transports."""

from __future__ import annotations

import logging
import math
import threading
from typing import Optional

from geometry_msgs.msg import Pose as RosPose, Point, Quaternion
from rclpy.node import Node
from std_srvs.srv import SetBool

from pybullet_fleet_msgs.msg import FleetState, RobotGoal2D
from pybullet_fleet_msgs.srv import AttachObject
from pybullet_fleet_msgs.srv import FleetNavigate as FleetNavigateSrv

from pybullet_fleet_rmf.client_interface import RobotUpdateData
from pybullet_fleet_rmf.robot_client_api import RobotClientAPI

logger = logging.getLogger(__name__)


class PerRobotRosClientFactory:
    """Factory for the existing per-robot ROS action/service client."""

    def __init__(self, node: Node, map_name: str = "L1") -> None:
        self._node = node
        self._map_name = map_name

    def robot(self, robot_name: str) -> RobotClientAPI:
        """Create one legacy per-robot ROS client."""
        return RobotClientAPI(robot_name=robot_name, node=self._node, map_name=self._map_name)


class RosFleetClient:
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

    def robot(self, robot_name: str) -> "RosFleetRobotClient":
        """Return a per-robot facade over the shared fleet endpoints."""
        return RosFleetRobotClient(robot_name, self)

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
        """Update completion state for a robot."""
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


class RosFleetRobotClient:
    """Per-robot facade over :class:`RosFleetClient`."""

    def __init__(self, robot_name: str, fleet: RosFleetClient) -> None:
        self._name = robot_name
        self._fleet = fleet
        self._node = fleet._node
        self._map_name = fleet._default_map_name
        self._active_cmd_id = 0
        self._active_target: Optional[list] = None
        self._attach_client = self._node.create_client(SetBool, f"/{robot_name}/toggle_attach")
        self._attach_object_client = self._node.create_client(AttachObject, f"/{robot_name}/attach_object")
        self._charging_client = self._node.create_client(SetBool, f"/{robot_name}/set_charging")

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
        return self._fleet.call_navigate(self._name, cmd_id, position, map_name)

    def stop(self) -> bool:
        data = self.get_data()
        if data is None:
            self._active_target = None
            return True
        self._active_cmd_id += 1
        self._active_target = None
        return self._fleet.call_navigate(self._name, self._active_cmd_id, data.position, data.map)

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

    def toggle_attach(self, attach: bool, cmd_id: int) -> bool:
        self._active_cmd_id = cmd_id
        if not self._attach_client.wait_for_service(timeout_sec=5.0):
            logger.error("[%s] toggle_attach service not available", self._name)
            return False
        future = self._attach_client.call_async(_set_bool_request(attach))
        future.add_done_callback(lambda done: self._on_set_bool_done(done, cmd_id, "toggle_attach"))
        return True

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
        if not self._attach_object_client.wait_for_service(timeout_sec=5.0):
            logger.error("[%s] attach_object service not available", self._name)
            return False
        req = AttachObject.Request()
        req.attach = attach
        req.object_name = object_name
        req.parent_link = parent_link
        req.offset = RosPose(
            position=Point(x=offset_position[0], y=offset_position[1], z=offset_position[2]),
            orientation=Quaternion(
                x=offset_orientation[0],
                y=offset_orientation[1],
                z=offset_orientation[2],
                w=offset_orientation[3],
            ),
        )
        req.search_radius = float(search_radius)
        future = self._attach_object_client.call_async(req)
        future.add_done_callback(lambda done: self._on_set_bool_done(done, cmd_id, "attach_object"))
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


def create_rmf_client_factory(mode: str, node: Node, map_name: str = "L1"):
    """Create an RMF client factory for ``mode``."""
    normalized = (mode or "per_robot_ros").strip().lower()
    if normalized == "per_robot_ros":
        return PerRobotRosClientFactory(node, map_name=map_name)
    if normalized == "fleet_ros":
        return RosFleetClient(node, map_name=map_name)
    raise ValueError(f"Unknown RMF client mode: {mode!r}")


def _set_bool_request(value: bool) -> SetBool.Request:
    req = SetBool.Request()
    req.data = bool(value)
    return req


def _distance_xy(a: list, b: list) -> float:
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


def _quat_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)
