"""Robot client API for PyBulletFleet bridge (EasyFullControl pattern).

Provides a ``RobotClientAPI`` that communicates with the PyBulletFleet
bridge node via ROS 2 topics and actions:

- **State**: Subscribes to ``/<robot>/odom`` for position/orientation.
- **Navigate**: Sends ``NavigateToPose`` action goals.
- **Stop**: Cancels active NavigateToPose goals.

This replaces the old ``PybulletCommandHandle`` and follows the
pattern established by ``rmf_demos_fleet_adapter/RobotClientAPI.py``.
"""

import logging
import math
import threading
import time
from dataclasses import dataclass
from typing import Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import SetBool

from pybullet_fleet_msgs.srv import AttachObject

logger = logging.getLogger(__name__)


@dataclass
class RobotUpdateData:
    """Snapshot of robot state for EasyFullControl updates."""

    map: str
    position: list  # [x, y, yaw]
    battery_soc: float
    last_completed_cmd_id: int

    def is_command_completed(self, cmd_id: int) -> bool:
        """Check if the given command has been completed."""
        return self.last_completed_cmd_id >= cmd_id


class RobotClientAPI:
    """Per-robot API wrapper for communicating with PyBulletFleet bridge.

    Subscribes to ``/<robot_name>/odom`` for state and sends
    ``NavigateToPose`` action goals for navigation commands.

    Args:
        robot_name: Name of the robot (must match bridge spawn name).
        node: ROS 2 node for creating subs/action clients.
        map_name: Default map name for RMF state updates.
    """

    def __init__(
        self,
        robot_name: str,
        node: Node,
        map_name: str = "L1",
    ):
        self._name = robot_name
        self._node = node
        self._map_name = map_name
        self._lock = threading.Lock()

        # Current state (from odom)
        self._position = [0.0, 0.0, 0.0]  # [x, y, yaw]
        self._has_odom = False  # True after first odom callback
        self._last_completed_cmd_id = 0
        self._active_cmd_id = 0

        # Battery simulation: start at 80% and charge to 100% over 30s.
        # This prevents the charging deadlock where RMF waits for a
        # positive charging rate but battery is already 100%.
        self._battery_soc = 0.80
        self._battery_start_time = time.monotonic()
        self._battery_charge_duration = 30.0  # seconds to reach 100%

        # Subscribe to odom
        self._odom_sub = node.create_subscription(
            Odometry,
            f"/{robot_name}/odom",
            self._odom_callback,
            10,
        )

        # NavigateToPose action client
        self._nav_client = ActionClient(
            node,
            NavigateToPose,
            f"/{robot_name}/navigate_to_pose",
        )
        self._active_goal_handle = None

        # toggle_attach service client (for delivery pick/drop)
        self._attach_client = node.create_client(
            SetBool,
            f"/{robot_name}/toggle_attach",
        )

        # attach_object service client (detailed attach/detach)
        self._attach_object_client = node.create_client(
            AttachObject,
            f"/{robot_name}/attach_object",
        )

    def get_data(self) -> Optional[RobotUpdateData]:
        """Return current robot state snapshot.

        Returns:
            RobotUpdateData with current position, map, battery,
            and last completed command ID. Returns ``None`` until
            the first odom message is received (prevents registering
            the robot at (0,0,0) which is off the navigation graph).
        """
        with self._lock:
            if not self._has_odom:
                return None
            # Simulate battery charging: 80% → 100% over _battery_charge_duration
            if self._battery_soc < 1.0:
                elapsed = time.monotonic() - self._battery_start_time
                self._battery_soc = min(
                    1.0,
                    0.80 + 0.20 * min(1.0, elapsed / self._battery_charge_duration),
                )
            return RobotUpdateData(
                map=self._map_name,
                position=list(self._position),
                battery_soc=self._battery_soc,
                last_completed_cmd_id=self._last_completed_cmd_id,
            )

    def set_map_name(self, map_name: str) -> None:
        """Update the current map/level name (e.g. after elevator transition)."""
        if self._map_name != map_name:
            logger.info("[%s] Map changed: %s -> %s", self._name, self._map_name, map_name)
            self._map_name = map_name

    def navigate(
        self,
        cmd_id: int,
        position: list,
        map_name: str,
        speed_limit: float = 0.0,
    ) -> bool:
        """Send a NavigateToPose goal to the bridge.

        Args:
            cmd_id: Command sequence ID for completion tracking.
            position: Target [x, y, yaw].
            map_name: Target map name (unused in single-map demo).
            speed_limit: Speed limit (unused in current bridge).

        Returns:
            True if the goal was sent successfully.
        """
        self._active_cmd_id = cmd_id
        goal = PoseStamped()
        goal.header.frame_id = "odom"
        goal.pose.position.x = float(position[0])
        goal.pose.position.y = float(position[1])
        goal.pose.orientation = self._yaw_to_quat(float(position[2]) if len(position) > 2 else 0.0)

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            logger.error("[%s] NavigateToPose server not available", self._name)
            return False

        future = self._nav_client.send_goal_async(nav_goal)
        future.add_done_callback(self._on_goal_accepted)
        return True

    def stop(self) -> bool:
        """Cancel the active navigation goal.

        Returns:
            True always — cancelling when idle is a no-op, not an error.
        """
        logger.info("[%s] Stop requested", self._name)
        if self._active_goal_handle is not None:
            self._active_goal_handle.cancel_goal_async()
            self._active_goal_handle = None
        return True

    def toggle_attach(self, attach: bool, cmd_id: int) -> bool:
        """Request attach/detach via the bridge's toggle_attach service.

        Mirrors rmf_demos' ``RobotAPI.toggle_attach``.  The service call
        is async — when the bridge completes the PickAction/DropAction it
        returns success, and we update ``_last_completed_cmd_id``.

        Args:
            attach: True to pick (attach), False to drop (detach).
            cmd_id: Command ID for completion tracking.

        Returns:
            True if the service call was sent successfully.
        """
        self._active_cmd_id = cmd_id

        if not self._attach_client.wait_for_service(timeout_sec=5.0):
            logger.error("[%s] toggle_attach service not available", self._name)
            return False

        req = SetBool.Request(data=attach)
        future = self._attach_client.call_async(req)
        future.add_done_callback(self._on_attach_done)
        return True

    def _on_attach_done(self, future) -> None:
        """Callback when toggle_attach service response arrives."""
        try:
            result = future.result()
        except Exception as e:
            logger.error("[%s] toggle_attach service call failed: %s", self._name, e)
            return

        if result is not None and result.success:
            logger.info("[%s] toggle_attach completed (cmd %d): %s", self._name, self._active_cmd_id, result.message)
            with self._lock:
                self._last_completed_cmd_id = self._active_cmd_id
        else:
            msg = result.message if result else "no response"
            logger.warning("[%s] toggle_attach failed (cmd %d): %s", self._name, self._active_cmd_id, msg)
            # Still mark as completed so RMF can proceed
            with self._lock:
                self._last_completed_cmd_id = self._active_cmd_id

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
        """Request detailed attach/detach via the attach_object service.

        Args:
            attach: True to pick, False to drop.
            cmd_id: Command ID for completion tracking.
            object_name: Target object name (empty = nearest/first).
            parent_link: Link name to attach to (empty = base_link).
            offset_position: Attachment offset position [x, y, z].
            offset_orientation: Attachment offset quaternion [x, y, z, w].
            search_radius: Search radius for nearest mode (0 = default 0.5m).

        Returns:
            True if the service call was sent successfully.
        """
        self._active_cmd_id = cmd_id

        if not self._attach_object_client.wait_for_service(timeout_sec=5.0):
            logger.error("[%s] attach_object service not available", self._name)
            return False

        from geometry_msgs.msg import Pose as RosPose, Point, Quaternion

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
        future.add_done_callback(self._on_attach_object_done)
        return True

    def _on_attach_object_done(self, future) -> None:
        """Callback when attach_object service response arrives."""
        try:
            result = future.result()
        except Exception as e:
            logger.error("[%s] attach_object service call failed: %s", self._name, e)
            return

        if result is not None and result.success:
            logger.info(
                "[%s] attach_object completed (cmd %d): %s (obj='%s')",
                self._name,
                self._active_cmd_id,
                result.message,
                result.attached_object_name,
            )
            with self._lock:
                self._last_completed_cmd_id = self._active_cmd_id
        else:
            msg = result.message if result else "no response"
            logger.warning("[%s] attach_object failed (cmd %d): %s", self._name, self._active_cmd_id, msg)
            with self._lock:
                self._last_completed_cmd_id = self._active_cmd_id

    # -- Callbacks ----------------------------------------------------------

    def _odom_callback(self, msg: Odometry):
        """Update current position from odom."""
        yaw = self._quat_to_yaw(msg.pose.pose.orientation)
        with self._lock:
            self._position = [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                yaw,
            ]
            self._has_odom = True

    def _on_goal_accepted(self, future):
        """Handle goal acceptance response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            logger.warning("[%s] NavigateToPose goal rejected", self._name)
            return
        self._active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav_complete)

    def _on_nav_complete(self, future):
        """Handle navigation result -- mark command as completed."""
        result = future.result()
        self._active_goal_handle = None
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            logger.info("[%s] Navigation completed (cmd %d)", self._name, self._active_cmd_id)
            with self._lock:
                self._last_completed_cmd_id = self._active_cmd_id
        else:
            logger.warning(
                "[%s] Navigation failed (status=%s)",
                self._name,
                result.status,
            )

    # -- Helpers ------------------------------------------------------------

    @staticmethod
    def _yaw_to_quat(yaw: float) -> Quaternion:
        """Convert yaw angle to geometry_msgs/Quaternion."""
        q = Quaternion()
        q.w = math.cos(yaw / 2.0)
        q.z = math.sin(yaw / 2.0)
        return q

    @staticmethod
    def _quat_to_yaw(q) -> float:
        """Convert geometry_msgs/Quaternion to yaw angle."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
