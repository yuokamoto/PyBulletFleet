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
from dataclasses import dataclass
from typing import Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node

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
        self._battery_soc = 1.0  # Simulated: always full
        self._last_completed_cmd_id = 0
        self._active_cmd_id = 0

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

    def get_data(self) -> Optional[RobotUpdateData]:
        """Return current robot state snapshot.

        Returns:
            RobotUpdateData with current position, map, battery,
            and last completed command ID. None if no odom received yet.
        """
        with self._lock:
            return RobotUpdateData(
                map=self._map_name,
                position=list(self._position),
                battery_soc=self._battery_soc,
                last_completed_cmd_id=self._last_completed_cmd_id,
            )

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
