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
import math
import time
from typing import TYPE_CHECKING, Optional

import numpy as np
from builtin_interfaces.msg import Time as TimeMsg
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as PathMsg
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory

from .conversions import (
    joint_trajectory_to_targets,
    make_diagnostic_msg,
    make_joint_state_msg,
    make_odom_msg,
    make_transform_stamped,
    pbf_path_to_ros,
    pbf_pose_to_pose_stamped,
    ros_path_to_pbf,
    ros_pose_stamped_to_pbf,
    ros_pose_to_pbf,
)

from .robot_handler_base import RobotHandlerBase

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
    """

    def __init__(
        self,
        node: "Node",
        agent: "Agent",
        tf_broadcaster: Optional["TransformBroadcaster"] = None,
    ):
        super().__init__(node, agent, tf_broadcaster)
        # Backward-compat aliases for internal use
        self._node = node
        self._tf_broadcaster = tf_broadcaster
        self._latest_twist: Optional[Twist] = None

        ns = self.ns
        self._ns = ns

        # --- Publishers (existing) ---
        self._odom_pub = node.create_publisher(Odometry, f"/{ns}/odom", 10)
        self._joint_pub = node.create_publisher(JointState, f"/{ns}/joint_states", 10)

        # --- Status publishers (new) ---
        self._plan_pub = node.create_publisher(PathMsg, f"/{ns}/plan", 10)
        self._goal_pub = node.create_publisher(PoseStamped, f"/{ns}/current_goal", 10)
        self._diag_pub = node.create_publisher(DiagnosticArray, f"/{ns}/diagnostics", 10)

        # --- Subscribers (existing) ---
        self._cmd_vel_sub = node.create_subscription(Twist, f"/{ns}/cmd_vel", self._cmd_vel_cb, 10)

        # --- Navigation topic subscribers (new) ---
        self._goal_pose_sub = node.create_subscription(PoseStamped, f"/{ns}/goal_pose", self._goal_pose_cb, 10)
        self._path_sub = node.create_subscription(PathMsg, f"/{ns}/path", self._path_cb, 10)

        # --- Arm topic subscribers (new) ---
        self._joint_traj_sub = node.create_subscription(JointTrajectory, f"/{ns}/joint_trajectory", self._joint_traj_cb, 10)
        self._joint_cmd_sub = node.create_subscription(Float64MultiArray, f"/{ns}/joint_commands", self._joint_cmd_cb, 10)

        # --- Action servers ---
        self._setup_action_servers(node, ns)

        # --- ExecuteAction topic + action ---
        self._setup_execute_action(node, ns)

        # --- Attach/detach services ---
        self._setup_attach_service(node, ns)
        self._setup_attach_object_service(node, ns)

        logger.info("RobotHandler created for '%s' (object_id=%d)", ns, agent.object_id)

    # ------------------------------------------------------------------
    # Action server setup
    # ------------------------------------------------------------------

    def _setup_action_servers(self, node: "Node", ns: str) -> None:
        """Create NavigateToPose, FollowPath, and FollowJointTrajectory action servers."""
        from control_msgs.action import FollowJointTrajectory
        from nav2_msgs.action import FollowPath, NavigateToPose
        from rclpy.action import ActionServer, CancelResponse, GoalResponse
        from rclpy.callback_groups import ReentrantCallbackGroup

        self._action_group = ReentrantCallbackGroup()

        def _accept_goal(goal_request):  # type: ignore[no-untyped-def]
            return GoalResponse.ACCEPT

        def _accept_cancel(goal_handle):  # type: ignore[no-untyped-def]
            self.agent.stop()
            return CancelResponse.ACCEPT

        self._nav_action = ActionServer(
            node,
            NavigateToPose,
            f"/{ns}/navigate_to_pose",
            execute_callback=self._navigate_execute,
            goal_callback=_accept_goal,
            cancel_callback=_accept_cancel,
            callback_group=self._action_group,
        )
        self._follow_path_action = ActionServer(
            node,
            FollowPath,
            f"/{ns}/follow_path",
            execute_callback=self._follow_path_execute,
            goal_callback=_accept_goal,
            cancel_callback=_accept_cancel,
            callback_group=self._action_group,
        )
        self._follow_jt_action = ActionServer(
            node,
            FollowJointTrajectory,
            f"/{ns}/follow_joint_trajectory",
            execute_callback=self._follow_jt_execute,
            goal_callback=_accept_goal,
            cancel_callback=_accept_cancel,
            callback_group=self._action_group,
        )

    def _setup_execute_action(self, node: "Node", ns: str) -> None:
        """Create /{robot}/execute_action topic subscriber and action server.

        Topic subscriber (fire-and-forget):
            Receives ``ExecuteActionGoal`` messages and queues actions
            via ``agent.add_action()``.  Status is observable through
            the ``/{robot}/diagnostics`` topic.

        Action server (blocking):
            Receives ``ExecuteAction`` goals and polls until the queued
            action completes, publishing progress feedback.
        """
        from pybullet_fleet_msgs.action import ExecuteAction
        from pybullet_fleet_msgs.msg import ExecuteActionGoal
        from rclpy.action import ActionServer, CancelResponse, GoalResponse
        from rclpy.callback_groups import ReentrantCallbackGroup

        # Topic subscriber — fire and forget
        self._exec_action_sub = node.create_subscription(
            ExecuteActionGoal,
            f"/{ns}/execute_action",
            self._execute_action_topic_cb,
            10,
        )

        # Action server — blocking with feedback
        self._exec_action_group = ReentrantCallbackGroup()
        self._exec_action_server = ActionServer(
            node,
            ExecuteAction,
            f"/{ns}/execute_action_blocking",
            execute_callback=self._execute_action_execute,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=self._execute_action_cancel,
            callback_group=self._exec_action_group,
        )

    def _execute_action_topic_cb(self, msg) -> None:
        """Handle execute_action topic — parse and queue action (fire-and-forget)."""
        from .action_parser import parse_action_goal

        action = parse_action_goal(msg.action_type, msg.action_params_json)
        if action is None:
            logger.error(
                "'%s': failed to parse execute_action topic: type='%s'",
                self._ns,
                msg.action_type,
            )
            return
        self.agent.add_action(action)
        logger.info(
            "'%s': execute_action topic → queued %s (queue size: %d)",
            self._ns,
            type(action).__name__,
            self.agent.get_action_queue_size(),
        )

    def _execute_action_cancel(self, goal_handle):
        """Cancel callback for ExecuteAction — cancel current agent action."""
        from rclpy.action import CancelResponse

        self.agent.stop()
        return CancelResponse.ACCEPT

    def _execute_action_execute(self, goal_handle):
        """Execute callback for ExecuteAction action server.

        Parses the goal, queues the action, and polls until completion.
        """
        from pybullet_fleet.types import ActionStatus
        from pybullet_fleet_msgs.action import ExecuteAction

        from .action_parser import parse_action_goal

        goal_msg = goal_handle.request.goal
        action = parse_action_goal(goal_msg.action_type, goal_msg.action_params_json)
        if action is None:
            goal_handle.abort()
            result = ExecuteAction.Result()
            result.success = False
            result.message = f"Failed to parse action: type='{goal_msg.action_type}'"
            return result

        self.agent.add_action(action)
        logger.info(
            "'%s': ExecuteAction started — %s",
            self._ns,
            type(action).__name__,
        )

        feedback = ExecuteAction.Feedback()
        poll_period = 0.1

        while True:
            if goal_handle.is_cancel_requested:
                action.cancel()
                goal_handle.canceled()
                result = ExecuteAction.Result()
                result.success = False
                result.message = "Cancelled"
                return result

            status = action.status
            if status == ActionStatus.COMPLETED:
                break
            if status == ActionStatus.FAILED:
                goal_handle.abort()
                result = ExecuteAction.Result()
                result.success = False
                result.message = f"{type(action).__name__} failed"
                return result
            if status == ActionStatus.CANCELLED:
                goal_handle.abort()
                result = ExecuteAction.Result()
                result.success = False
                result.message = f"{type(action).__name__} cancelled"
                return result

            # Publish feedback
            feedback.status = (
                ExecuteAction.Feedback.STATUS_IN_PROGRESS
                if status == ActionStatus.IN_PROGRESS
                else ExecuteAction.Feedback.STATUS_NOT_STARTED
            )
            feedback.progress = getattr(action, "progress", 0.0)
            goal_handle.publish_feedback(feedback)

            time.sleep(poll_period)

        goal_handle.succeed()
        logger.info("'%s': ExecuteAction succeeded — %s", self._ns, type(action).__name__)
        result = ExecuteAction.Result()
        result.success = True
        result.message = f"{type(action).__name__} completed"
        return result

    def _setup_attach_service(self, node: "Node", ns: str) -> None:
        """Create /{robot}/toggle_attach SetBool service for delivery pick/drop.

        Mirrors rmf_demos' ``toggle_attach`` endpoint.  When called with
        ``data=True``, queues a PickAction on the agent; ``data=False``
        queues a DropAction.  The service blocks until the action
        completes (same pattern as the NavigateToPose action server).
        """
        from rclpy.callback_groups import ReentrantCallbackGroup
        from std_srvs.srv import SetBool

        self._attach_group = ReentrantCallbackGroup()
        self._attach_srv = node.create_service(
            SetBool,
            f"/{ns}/toggle_attach",
            self._toggle_attach_cb,
            callback_group=self._attach_group,
        )

    def _toggle_attach_cb(self, request, response):  # type: ignore[no-untyped-def]
        """Handle toggle_attach service — immediate attach/detach.

        Calls ``agent.attach_object()`` / ``agent.detach_object()``
        directly (no action queue).  Finds the nearest pickable object
        within 1.0m search radius for attach, or detaches the first
        attached object for detach.
        """
        if request.data:
            obj = self._find_nearest_pickable(search_radius=1.0)
            if obj is None:
                response.success = False
                response.message = "No pickable object within 1.0m"
                return response
            ok = self.agent.attach_object(obj, keep_world_pose=True)
            response.success = ok
            response.message = f"attached '{obj.name}'" if ok else f"attach failed for '{obj.name}'"
        else:
            attached = self.agent.get_attached_objects()
            if not attached:
                response.success = False
                response.message = "No attached object to detach"
                return response
            obj = attached[0]
            ok = self.agent.detach_object(obj)
            response.success = ok
            response.message = f"detached '{obj.name}'" if ok else f"detach failed for '{obj.name}'"

        logger.info("'%s': toggle_attach done — %s", self._ns, response.message)
        return response

    def _setup_attach_object_service(self, node: "Node", ns: str) -> None:
        """Create /{robot}/attach_object service for detailed attach/detach.

        Unlike ``toggle_attach`` (SetBool, rmf_demos compat), this service
        allows specifying the target object by name, the parent link, an
        attachment offset, and a search radius.  Useful for external systems
        and scripted scenarios that need fine-grained control.
        """
        from pybullet_fleet_msgs.srv import AttachObject
        from rclpy.callback_groups import ReentrantCallbackGroup

        self._attach_object_group = ReentrantCallbackGroup()
        self._attach_object_srv = node.create_service(
            AttachObject,
            f"/{ns}/attach_object",
            self._attach_object_cb,
            callback_group=self._attach_object_group,
        )

    def _attach_object_cb(self, request, response):  # type: ignore[no-untyped-def]
        """Handle attach_object service — immediate attach/detach with parameters.

        Calls ``agent.attach_object()`` / ``agent.detach_object()``
        directly (no action queue, no approach phase).  For action-based
        pick/drop with approach, use ``ExecuteAction.action`` instead.
        """
        from pybullet_fleet.geometry import Pose as PbfPose

        object_name = request.object_name if request.object_name else ""
        parent_link = request.parent_link if request.parent_link else "base_link"
        search_radius = request.search_radius if request.search_radius > 0 else 0.5

        # Build offset Pose from geometry_msgs/Pose
        offset_pos = [
            request.offset.position.x,
            request.offset.position.y,
            request.offset.position.z,
        ]
        offset_ori = [
            request.offset.orientation.x,
            request.offset.orientation.y,
            request.offset.orientation.z,
            request.offset.orientation.w,
        ]
        # Treat identity quaternion (0,0,0,0) as (0,0,0,1)
        if all(v == 0.0 for v in offset_ori):
            offset_ori = [0.0, 0.0, 0.0, 1.0]
        attach_offset = PbfPose(position=offset_pos, orientation=offset_ori)

        if request.attach:
            # --- Attach ---
            target_obj = None
            if object_name:
                sim_core = getattr(self.agent, "sim_core", None)
                if sim_core is not None:
                    for obj in sim_core.sim_objects:
                        if obj.name == object_name:
                            target_obj = obj
                            break
                if target_obj is None:
                    response.success = False
                    response.message = f"Object '{object_name}' not found"
                    response.attached_object_name = ""
                    return response
            else:
                target_obj = self._find_nearest_pickable(search_radius=search_radius)
                if target_obj is None:
                    response.success = False
                    response.message = f"No pickable object within {search_radius}m"
                    response.attached_object_name = ""
                    return response

            ok = self.agent.attach_object(
                target_obj,
                parent_link_index=parent_link,
                relative_pose=attach_offset,
            )
            response.success = ok
            response.attached_object_name = target_obj.name or ""
            response.message = f"attached '{target_obj.name}'" if ok else f"attach failed for '{target_obj.name}'"
        else:
            # --- Detach ---
            target_obj = None
            if object_name:
                for obj in self.agent.get_attached_objects():
                    if obj.name == object_name:
                        target_obj = obj
                        break
                if target_obj is None:
                    response.success = False
                    response.message = f"Object '{object_name}' not attached"
                    response.attached_object_name = ""
                    return response
            else:
                attached = self.agent.get_attached_objects()
                if not attached:
                    response.success = False
                    response.message = "No attached object to detach"
                    response.attached_object_name = ""
                    return response
                target_obj = attached[0]

            ok = self.agent.detach_object(target_obj)
            response.success = ok
            response.attached_object_name = target_obj.name or ""
            response.message = f"detached '{target_obj.name}'" if ok else f"detach failed for '{target_obj.name}'"

        logger.info("'%s': attach_object done — %s", self._ns, response.message)
        return response

    # ------------------------------------------------------------------
    # Attach helpers
    # ------------------------------------------------------------------

    def _find_nearest_pickable(self, search_radius: float = 1.0):
        """Find the nearest pickable, unattached SimObject within radius.

        Delegates to ``SimObject.find_nearest_pickable()`` on the agent.

        Returns:
            SimObject or None
        """
        return self.agent.find_nearest_pickable(search_radius=search_radius)

    def _cmd_vel_cb(self, msg: Twist) -> None:
        """Store latest cmd_vel for application in next step.

        The controller switches to VELOCITY mode automatically when
        :meth:`set_velocity` is called — no reattach needed.
        """
        self._latest_twist = msg

    # ------------------------------------------------------------------
    # Navigation / Arm topic callbacks
    # ------------------------------------------------------------------

    def _goal_pose_cb(self, msg: PoseStamped) -> None:
        """Receive goal pose → agent navigates to it.

        The controller switches to POSE mode automatically when
        ``set_goal_pose()`` is called.
        """
        pose = ros_pose_stamped_to_pbf(msg)
        self.agent.set_goal_pose(pose)
        logger.info("'%s': goal_pose received → navigating to (%.2f, %.2f)", self._ns, pose.x, pose.y)

    def _path_cb(self, msg: PathMsg) -> None:
        """Receive nav path → agent follows it.

        The controller switches to POSE mode automatically when
        ``set_path()`` is called.
        """
        waypoints = ros_path_to_pbf(msg)
        if not waypoints:
            logger.warning("'%s': empty path received, ignoring", self._ns)
            return
        self.agent.set_path(waypoints)
        logger.info("'%s': path received (%d waypoints)", self._ns, len(waypoints))

    def _joint_traj_cb(self, msg: JointTrajectory) -> None:
        """Receive joint trajectory → apply final waypoint targets."""
        targets = joint_trajectory_to_targets(msg)
        if not targets:
            logger.warning("'%s': empty joint trajectory, ignoring", self._ns)
            return
        self.agent.set_joints_targets_by_name(targets)
        logger.info("'%s': joint_trajectory received (%d joints)", self._ns, len(targets))

    def _joint_cmd_cb(self, msg: Float64MultiArray) -> None:
        """Receive raw joint positions → set all joints."""
        positions = list(msg.data)
        if not positions:
            return
        self.agent.set_all_joints_targets(positions)

    def pre_step(self, dt: float = 0.0, stamp: Optional[TimeMsg] = None) -> None:
        """Apply stored cmd_vel before the simulation step.

        Called once per sim step by BridgeNode._on_pre_step().
        Passes body-frame Twist directly to the controller's set_velocity().
        The controller handles kinematics (body→world rotation) internally.
        """
        self._apply_cmd_vel()

    def _apply_cmd_vel(self) -> None:
        """Internal: apply stored cmd_vel as a velocity command."""
        if self._latest_twist is None:
            return
        ctrl = self.agent._controller
        if ctrl is None:
            logger.warning("RobotHandler '%s': no controller set, ignoring cmd_vel", self._ns)
            self._latest_twist = None
            return

        twist = self._latest_twist
        # Pass body-frame Twist directly — controller handles kinematics
        ctrl.set_velocity(
            vx=twist.linear.x,
            vy=twist.linear.y,
            vz=twist.linear.z,
            wx=twist.angular.x,
            wy=twist.angular.y,
            wz=twist.angular.z,
        )

    def post_step(self, dt: float = 0.0, stamp: Optional[TimeMsg] = None) -> None:
        """Publish odom, joint_states, TF, and status after each step."""
        if stamp is None:
            stamp = TimeMsg()
        pose = self.agent.get_pose()
        velocity = list(self.agent.velocity)
        angular_vel = self.agent.angular_velocity

        # Odometry
        odom_msg = make_odom_msg(
            pose,
            velocity,
            angular_vel,
            frame_id="odom",
            child_frame_id=f"{self._ns}/base_link",
            stamp=stamp,
        )
        self._odom_pub.publish(odom_msg)

        # TF: odom → {ns}/base_link
        if self._tf_broadcaster is not None:
            tf_msg = make_transform_stamped(
                pose,
                parent_frame="odom",
                child_frame=f"{self._ns}/base_link",
                stamp=stamp,
            )
            self._tf_broadcaster.sendTransform(tf_msg)

        # Joint states (only if robot has joints)
        num_joints = self.agent.get_num_joints()
        if num_joints > 0:
            self._publish_joint_states(stamp)

        # Status & diagnostics
        self._publish_status(stamp)

    def _publish_joint_states(self, stamp: TimeMsg) -> None:
        """Publish sensor_msgs/JointState for all joints."""
        joints_by_name = self.agent.get_all_joints_state_by_name()
        if not joints_by_name:
            return

        names = list(joints_by_name.keys())
        positions = [joints_by_name[n][0] for n in names]
        velocities = [joints_by_name[n][1] for n in names]

        msg = make_joint_state_msg(names, positions, velocities, stamp=stamp)
        self._joint_pub.publish(msg)

    # ------------------------------------------------------------------
    # Status publishing
    # ------------------------------------------------------------------

    def _publish_status(self, stamp: TimeMsg) -> None:
        """Publish plan, current_goal, and diagnostics."""
        pose = self.agent.get_pose()
        velocity = self.agent.velocity
        speed = float(np.linalg.norm(velocity))

        # Current goal — publish with bearing-based orientation so the RViz
        # arrow (Pose display) points from robot toward the goal position.
        goal = self.agent.goal_pose
        if goal is not None:
            # dx = goal.position[0] - pose.position[0]
            # dy = goal.position[1] - pose.position[1]
            # dist = math.sqrt(dx * dx + dy * dy)
            # if dist > 0.01:
            #     # Compute yaw from current position toward goal
            #     yaw = math.atan2(dy, dx)
            #     half = yaw * 0.5
            #     bearing_orientation = [0.0, 0.0, math.sin(half), math.cos(half)]
            #     from pybullet_fleet.geometry import Pose as PbfPose

            #     goal_for_viz = PbfPose(position=goal.position, orientation=bearing_orientation)
            #     goal_msg = pbf_pose_to_pose_stamped(goal_for_viz, stamp=stamp)
            # else:
            # Very close — use the goal's own orientation
            goal_msg = pbf_pose_to_pose_stamped(goal, stamp=stamp)
            self._goal_pub.publish(goal_msg)
            dist = float(np.linalg.norm(np.array(pose.position) - np.array(goal.position)))
        else:
            dist = 0.0

        # Current path — publish remaining waypoints
        waypoints = getattr(self.agent, "path", [])
        current_idx = getattr(self.agent, "current_waypoint_index", 0)
        if waypoints and current_idx < len(waypoints):
            remaining = waypoints[current_idx:]
            plan_msg = pbf_path_to_ros(remaining, stamp=stamp)
            self._plan_pub.publish(plan_msg)

        # Diagnostics
        current_action = self.agent.get_current_action()
        action_type = type(current_action).__name__ if current_action else ""
        action_status = str(current_action.status.name) if current_action else ""

        # Action queue summary
        queue = self.agent._action_queue
        queue_items = []
        if current_action:
            queue_items.append(f"{type(current_action).__name__}({current_action.status.name})")
        for a in queue:
            queue_items.append(f"{type(a).__name__}({a.status.name})")
        action_queue_str = ", ".join(queue_items) if queue_items else ""
        queue_size = len(queue_items)

        diag_msg = make_diagnostic_msg(
            robot_name=self._ns,
            is_moving=self.agent.is_moving,
            action_type=action_type,
            action_status=action_status,
            distance_to_goal=dist,
            linear_speed=speed,
            action_queue=action_queue_str,
            action_queue_size=queue_size,
            stamp=stamp,
        )
        self._diag_pub.publish(diag_msg)

    # ------------------------------------------------------------------
    # Action server execute callbacks
    # ------------------------------------------------------------------

    def _navigate_execute(self, goal_handle):  # type: ignore[no-untyped-def]
        """Execute NavigateToPose action — set goal and poll until arrival."""
        from nav2_msgs.action import NavigateToPose

        goal_pose = ros_pose_to_pbf(goal_handle.request.pose.pose)
        self.agent.set_goal_pose(goal_pose)
        logger.info("'%s': NavigateToPose started → (%.2f, %.2f)", self._ns, goal_pose.x, goal_pose.y)

        feedback = NavigateToPose.Feedback()
        poll_period = 0.1  # seconds

        while self.agent.is_moving:
            if goal_handle.is_cancel_requested:
                self.agent.stop()
                goal_handle.canceled()
                logger.info("'%s': NavigateToPose canceled", self._ns)
                return NavigateToPose.Result()

            # Publish feedback
            current = self.agent.get_pose()
            feedback.current_pose = pbf_pose_to_pose_stamped(current)
            dx = current.x - goal_pose.x
            dy = current.y - goal_pose.y
            feedback.distance_remaining = float(math.sqrt(dx * dx + dy * dy))
            goal_handle.publish_feedback(feedback)

            time.sleep(poll_period)

        goal_handle.succeed()
        logger.info("'%s': NavigateToPose succeeded", self._ns)
        return NavigateToPose.Result()

    def _follow_path_execute(self, goal_handle):  # type: ignore[no-untyped-def]
        """Execute FollowPath action — set path and poll until completion."""
        from nav2_msgs.action import FollowPath

        waypoints = ros_path_to_pbf(goal_handle.request.path)
        if not waypoints:
            goal_handle.abort()
            return FollowPath.Result()

        self.agent.set_path(waypoints)
        logger.info("'%s': FollowPath started (%d waypoints)", self._ns, len(waypoints))

        feedback = FollowPath.Feedback()
        poll_period = 0.1

        while self.agent.is_moving:
            if goal_handle.is_cancel_requested:
                self.agent.stop()
                goal_handle.canceled()
                return FollowPath.Result()

            current = self.agent.get_pose()
            feedback.distance_to_goal = float(
                math.sqrt((current.x - waypoints[-1].x) ** 2 + (current.y - waypoints[-1].y) ** 2)
            )
            goal_handle.publish_feedback(feedback)
            time.sleep(poll_period)

        goal_handle.succeed()
        logger.info("'%s': FollowPath succeeded", self._ns)
        return FollowPath.Result()

    def _follow_jt_execute(self, goal_handle):  # type: ignore[no-untyped-def]
        """Execute FollowJointTrajectory — apply each trajectory point sequentially."""
        from control_msgs.action import FollowJointTrajectory

        traj = goal_handle.request.trajectory
        joint_names = list(traj.joint_names)

        if not traj.points:
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        logger.info(
            "'%s': FollowJointTrajectory started (%d points, %d joints)",
            self._ns,
            len(traj.points),
            len(joint_names),
        )

        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = joint_names
        poll_period = 0.05  # 50ms — check at sim rate
        timeout_per_point = 10.0  # seconds max per waypoint

        for point in traj.points:
            # Apply this waypoint's targets
            targets: dict = {}
            for i, name in enumerate(joint_names):
                if i < len(point.positions):
                    targets[name] = point.positions[i]
            self.agent.set_joints_targets_by_name(targets)

            # Wait for joints to settle or timeout
            elapsed = 0.0
            while elapsed < timeout_per_point:
                if goal_handle.is_cancel_requested:
                    self.agent.stop()
                    goal_handle.canceled()
                    return FollowJointTrajectory.Result()

                if self.agent.are_joints_at_targets_by_name(targets):
                    break

                # Build feedback
                joints_state = self.agent.get_all_joints_state_by_name()
                actual_pos = [joints_state.get(n, (0.0, 0.0))[0] for n in joint_names]
                desired_pos = [targets.get(n, 0.0) for n in joint_names]
                error_pos = [a - d for a, d in zip(actual_pos, desired_pos)]

                feedback.actual.positions = actual_pos
                feedback.desired.positions = desired_pos
                feedback.error.positions = error_pos
                goal_handle.publish_feedback(feedback)

                time.sleep(poll_period)
                elapsed += poll_period

        goal_handle.succeed()
        logger.info("'%s': FollowJointTrajectory succeeded", self._ns)
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    def destroy(self) -> None:
        """Clean up ROS interfaces."""
        # Publishers
        self._node.destroy_publisher(self._odom_pub)
        self._node.destroy_publisher(self._joint_pub)
        self._node.destroy_publisher(self._plan_pub)
        self._node.destroy_publisher(self._goal_pub)
        self._node.destroy_publisher(self._diag_pub)
        # Subscribers
        self._node.destroy_subscription(self._cmd_vel_sub)
        self._node.destroy_subscription(self._goal_pose_sub)
        self._node.destroy_subscription(self._path_sub)
        self._node.destroy_subscription(self._joint_traj_sub)
        self._node.destroy_subscription(self._joint_cmd_sub)
        # Action servers
        self._nav_action.destroy()
        self._follow_path_action.destroy()
        self._follow_jt_action.destroy()
        # ExecuteAction
        self._node.destroy_subscription(self._exec_action_sub)
        self._exec_action_server.destroy()
        # Attach services
        self._node.destroy_service(self._attach_srv)
        self._node.destroy_service(self._attach_object_srv)
        logger.info("RobotHandler destroyed for '%s'", self._ns)
