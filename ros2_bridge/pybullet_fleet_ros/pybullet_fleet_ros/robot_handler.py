"""Per-robot ROS interface handler.

Creates publishers, subscribers, and action servers for a single Agent.
Passes body-frame cmd_vel directly to the KinematicController's set_velocity().
The controller handles kinematics (body→world rotation) internally.

Layers:
- **Topic layer:**  goal_pose / path / joint_trajectory / joint_commands / execute_action → Agent API
- **Action layer:** NavigateToPose / FollowPath / FollowJointTrajectory / ExecuteAction
- **Status layer:** plan / current_goal / diagnostics publishers
"""

import logging
from typing import TYPE_CHECKING, Optional

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path as PathMsg
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory

from .interface_config import PerRobotApiConfig
from .robot_handler_base import RobotHandlerBase
from .robot_handler_groups import (
    CommandTopicHandler,
    ExecuteActionHandler,
    NavigationActionHandler,
    ServiceHandler,
    StatePublisherHandler,
    TfPublisherHandler,
)

if TYPE_CHECKING:
    from pybullet_fleet.agent import Agent
    from rclpy.node import Node
    from tf2_ros import TransformBroadcaster

logger = logging.getLogger(__name__)


class RobotHandler(RobotHandlerBase):
    """Manages all ROS 2 interfaces for a single simulated Agent.

    Creates:
    - ``/{name}/cmd_vel`` subscriber (Twist) → KinematicController.set_velocity()
    - ``/{name}/goal_pose`` subscriber (PoseStamped) → Agent.set_goal_pose()
    - ``/{name}/path`` subscriber (Path) → Agent.set_path()
    - ``/{name}/joint_trajectory`` subscriber (JointTrajectory) → Agent.set_joints_targets_by_name()
    - ``/{name}/joint_commands`` subscriber (Float64MultiArray) → Agent.set_all_joints_targets()
    - ``/{name}/odom`` publisher (Odometry)
    - ``/{name}/joint_states`` publisher (JointState)
    - ``/{name}/plan`` publisher (Path) — current path being followed
    - ``/{name}/current_goal`` publisher (PoseStamped) — current goal pose
    - ``/{name}/diagnostics`` publisher (DiagnosticArray) — robot status
    - TF broadcast: ``odom`` → ``{name}/base_link``
    - ``/{name}/navigate_to_pose`` action server (NavigateToPose)
    - ``/{name}/follow_path`` action server (FollowPath)
    - ``/{name}/follow_joint_trajectory`` action server (FollowJointTrajectory)
    - ``/{name}/execute_action`` subscriber (ExecuteActionGoal) → fire-and-forget action queue
    - ``/{name}/execute_action`` action server (ExecuteAction) → blocking with feedback
    - ``/{name}/toggle_attach`` service (SetBool) — rmf_demos cart delivery compat
    - ``/{name}/attach_object`` service (AttachObject) — detailed attach/detach
    - ``/{name}/set_charging`` service (SetBool) — start/stop battery charging
    - ``/{name}/battery_state`` publisher (BatteryState) — battery status and SOC
    """

    def __init__(
        self,
        node: "Node",
        agent: "Agent",
        tf_broadcaster: Optional["TransformBroadcaster"] = None,
        interface_config: Optional[PerRobotApiConfig] = None,
    ):
        super().__init__(node, agent, tf_broadcaster)
        # Backward-compat aliases for internal use
        self._node = node
        self._tf_broadcaster = tf_broadcaster
        self._latest_twist: Optional[Twist] = None

        ns = self.ns
        self._ns = ns

        # sim↔RMF frame offset, set on the bridge node from config.
        # (0, 0) = sim and RMF share a frame (the usual case).
        off = getattr(node, "rmf_frame_offset", (0.0, 0.0))
        self._rmf_offset = (float(off[0]), float(off[1]))

        api = interface_config or PerRobotApiConfig()
        self._interface_groups = []
        self._state_publishers = None
        self._tf_publisher = None
        self._command_topics = None
        self._navigation_actions = None
        self._execute_actions = None
        self._services = None

        if api.state_publishers:
            self._state_publishers = StatePublisherHandler(self)
            self._interface_groups.append(self._state_publishers)
        if api.tf:
            self._tf_publisher = TfPublisherHandler(self)
            self._interface_groups.append(self._tf_publisher)
        if api.command_topics:
            self._command_topics = CommandTopicHandler(self)
            self._interface_groups.append(self._command_topics)
        if api.actions:
            self._navigation_actions = NavigationActionHandler(self)
            self._execute_actions = ExecuteActionHandler(self)
            self._interface_groups.extend([self._navigation_actions, self._execute_actions])
        if api.services:
            self._services = ServiceHandler(self)
            self._interface_groups.append(self._services)

        logger.info("RobotHandler created for '%s' (object_id=%d)", ns, agent.object_id)

    def _cmd_vel_cb(self, msg: Twist) -> None:
        """Compatibility forwarding for older tests/custom users."""
        if self._command_topics is None:
            return
        self._command_topics.cmd_vel_cb(msg)

    def _goal_pose_cb(self, msg: PoseStamped) -> None:
        """Compatibility forwarding for older tests/custom users."""
        if self._command_topics is None:
            return
        self._command_topics.goal_pose_cb(msg)

    def _path_cb(self, msg: PathMsg) -> None:
        """Compatibility forwarding for older tests/custom users."""
        if self._command_topics is None:
            return
        self._command_topics.path_cb(msg)

    def _joint_traj_cb(self, msg: JointTrajectory) -> None:
        """Compatibility forwarding for older tests/custom users."""
        if self._command_topics is None:
            return
        self._command_topics.joint_traj_cb(msg)

    def _joint_cmd_cb(self, msg: Float64MultiArray) -> None:
        """Compatibility forwarding for older tests/custom users."""
        if self._command_topics is None:
            return
        self._command_topics.joint_cmd_cb(msg)

    def pre_step(self, dt: float = 0.0, stamp: Optional[TimeMsg] = None) -> None:
        """Apply stored command topics before the simulation step."""
        if self._command_topics is None:
            return
        self._command_topics.pre_step(dt=dt, stamp=stamp)

    def _shift_xy(self, x: float, y: float, sign: int) -> tuple:
        """Apply the sim↔RMF frame offset to a planar position.

        ``sign=+1`` converts sim→RMF (for poses published to RMF); ``sign=-1``
        converts RMF→sim (for goals received from RMF). A (0, 0) offset is a
        no-op.
        """
        return x + sign * self._rmf_offset[0], y + sign * self._rmf_offset[1]

    def post_step(self, dt: float = 0.0, stamp: Optional[TimeMsg] = None) -> None:
        """Publish state and TF after each step."""
        if self._state_publishers is not None:
            self._state_publishers.post_step(dt=dt, stamp=stamp)
        if self._tf_publisher is not None:
            self._tf_publisher.post_step(dt=dt, stamp=stamp)

    def destroy(self) -> None:
        """Clean up ROS interfaces."""
        for group in reversed(self._interface_groups):
            group.destroy()
        logger.info("RobotHandler destroyed for '%s'", self._ns)
