"""Composable per-robot ROS interface groups.

These groups are intentionally thin in Phase 1: ``RobotHandler`` remains the
compatibility facade and owns callback implementations, while each group owns
creation and cleanup of one resource family. Later phases can move callback
logic into these classes without changing the bridge-facing lifecycle.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as PathMsg
from sensor_msgs.msg import BatteryState, JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory

if TYPE_CHECKING:
    from .robot_handler import RobotHandler


class RobotInterfaceGroup:
    """Base class for a per-robot ROS interface group."""

    def __init__(self, owner: "RobotHandler") -> None:
        self.owner = owner

    def destroy(self) -> None:
        """Release ROS resources owned by this group."""


class StatePublisherHandler(RobotInterfaceGroup):
    """Own per-robot state publishers."""

    def __init__(self, owner: "RobotHandler") -> None:
        super().__init__(owner)
        node = owner._node
        ns = owner._ns
        owner._odom_pub = node.create_publisher(Odometry, f"/{ns}/odom", 10)
        owner._joint_pub = node.create_publisher(JointState, f"/{ns}/joint_states", 10)
        owner._plan_pub = node.create_publisher(PathMsg, f"/{ns}/plan", 10)
        owner._goal_pub = node.create_publisher(PoseStamped, f"/{ns}/current_goal", 10)
        owner._diag_pub = node.create_publisher(DiagnosticArray, f"/{ns}/diagnostics", 10)
        owner._battery_pub = None
        if owner.agent.battery_plugin is not None:
            owner._battery_pub = node.create_publisher(BatteryState, f"/{ns}/battery_state", 10)

    def destroy(self) -> None:
        owner = self.owner
        node = owner._node
        node.destroy_publisher(owner._odom_pub)
        node.destroy_publisher(owner._joint_pub)
        node.destroy_publisher(owner._plan_pub)
        node.destroy_publisher(owner._goal_pub)
        node.destroy_publisher(owner._diag_pub)
        if owner._battery_pub is not None:
            node.destroy_publisher(owner._battery_pub)


class TfPublisherHandler(RobotInterfaceGroup):
    """Marker group for per-robot TF publishing.

    The actual broadcaster is shared by ``BridgeNode`` and the send logic still
    lives on ``RobotHandler.post_step`` in this phase.
    """


class CommandTopicHandler(RobotInterfaceGroup):
    """Own per-robot command topic subscriptions."""

    def __init__(self, owner: "RobotHandler") -> None:
        super().__init__(owner)
        node = owner._node
        ns = owner._ns
        owner._cmd_vel_sub = node.create_subscription(Twist, f"/{ns}/cmd_vel", owner._cmd_vel_cb, 10)
        owner._goal_pose_sub = node.create_subscription(PoseStamped, f"/{ns}/goal_pose", owner._goal_pose_cb, 10)
        owner._path_sub = node.create_subscription(PathMsg, f"/{ns}/path", owner._path_cb, 10)
        owner._joint_traj_sub = node.create_subscription(
            JointTrajectory,
            f"/{ns}/joint_trajectory",
            owner._joint_traj_cb,
            10,
        )
        owner._joint_cmd_sub = node.create_subscription(
            Float64MultiArray,
            f"/{ns}/joint_commands",
            owner._joint_cmd_cb,
            10,
        )

    def destroy(self) -> None:
        owner = self.owner
        node = owner._node
        node.destroy_subscription(owner._cmd_vel_sub)
        node.destroy_subscription(owner._goal_pose_sub)
        node.destroy_subscription(owner._path_sub)
        node.destroy_subscription(owner._joint_traj_sub)
        node.destroy_subscription(owner._joint_cmd_sub)


class NavigationActionHandler(RobotInterfaceGroup):
    """Own per-robot navigation and joint trajectory action servers."""

    def __init__(self, owner: "RobotHandler") -> None:
        super().__init__(owner)
        owner._setup_action_servers(owner._node, owner._ns)

    def destroy(self) -> None:
        owner = self.owner
        owner._nav_action.destroy()
        owner._follow_path_action.destroy()
        owner._follow_jt_action.destroy()


class ExecuteActionHandler(RobotInterfaceGroup):
    """Own per-robot ExecuteAction topic and action server."""

    def __init__(self, owner: "RobotHandler") -> None:
        super().__init__(owner)
        owner._setup_execute_action(owner._node, owner._ns)

    def destroy(self) -> None:
        owner = self.owner
        owner._node.destroy_subscription(owner._exec_action_sub)
        owner._exec_action_server.destroy()


class ServiceHandler(RobotInterfaceGroup):
    """Own per-robot services."""

    def __init__(self, owner: "RobotHandler") -> None:
        super().__init__(owner)
        owner._charging_srv = None
        owner._setup_attach_service(owner._node, owner._ns)
        owner._setup_attach_object_service(owner._node, owner._ns)
        if owner.agent.battery_plugin is not None:
            owner._setup_charging_service(owner._node, owner._ns)

    def destroy(self) -> None:
        owner = self.owner
        node = owner._node
        node.destroy_service(owner._attach_srv)
        node.destroy_service(owner._attach_object_srv)
        if owner._charging_srv is not None:
            node.destroy_service(owner._charging_srv)
