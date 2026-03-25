"""Per-robot ROS interface handler.

Creates publishers, subscribers, and action servers for a single Agent.
Passes body-frame cmd_vel directly to the VelocityController's set_velocity().
The controller handles kinematics (body→world rotation) internally.

Layers:
- **Topic layer:**  goal_pose / path / joint_trajectory / joint_commands → Agent API
- **Action layer:** NavigateToPose / FollowPath / FollowJointTrajectory
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

if TYPE_CHECKING:
    from pybullet_fleet.agent import Agent
    from pybullet_fleet.controller import VelocityController
    from rclpy.node import Node
    from tf2_ros import TransformBroadcaster

logger = logging.getLogger(__name__)


class RobotHandler:
    """Manages all ROS 2 interfaces for a single simulated Agent.

    Creates:
    - ``/{name}/cmd_vel`` subscriber (Twist) → VelocityController.set_velocity()
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
    """

    def __init__(
        self,
        node: "Node",
        agent: "Agent",
        vel_controller: Optional["VelocityController"] = None,
        tf_broadcaster: Optional["TransformBroadcaster"] = None,
    ):
        self._node = node
        self.agent = agent
        self._vel_controller = vel_controller
        self._tf_broadcaster = tf_broadcaster
        self._latest_twist: Optional[Twist] = None

        ns = agent.name or f"robot{agent.object_id}"
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

        # --- Action servers (new) ---
        self._setup_action_servers(node, ns)

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

    def _cmd_vel_cb(self, msg: Twist) -> None:
        """Store latest cmd_vel for application in next step.

        If a goal-based navigation was active, reattach the velocity controller
        so that controller-mode velocity commands take effect.
        """
        self._latest_twist = msg
        # Reattach velocity controller (goal-based nav detaches it)
        if self._vel_controller is not None and self.agent._controller is None:
            self.agent.stop()  # cancel any TPI navigation in progress
            self.agent.set_controller(self._vel_controller)

    # ------------------------------------------------------------------
    # Navigation / Arm topic callbacks
    # ------------------------------------------------------------------

    def _goal_pose_cb(self, msg: PoseStamped) -> None:
        """Receive goal pose → agent navigates to it.

        Detaches the velocity controller so the legacy TPI trajectory code
        drives the robot toward the goal.  The controller is reattached when
        a ``cmd_vel`` message arrives.
        """
        self.agent.set_controller(None)  # switch to TPI mode
        pose = ros_pose_stamped_to_pbf(msg)
        self.agent.set_goal_pose(pose)
        logger.info("'%s': goal_pose received → navigating to (%.2f, %.2f)", self._ns, pose.x, pose.y)

    def _path_cb(self, msg: PathMsg) -> None:
        """Receive nav path → agent follows it.

        Detaches the velocity controller so TPI trajectory code runs.
        """
        waypoints = ros_path_to_pbf(msg)
        if not waypoints:
            logger.warning("'%s': empty path received, ignoring", self._ns)
            return
        self.agent.set_controller(None)  # switch to TPI mode
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

    def apply_cmd_vel(self) -> None:
        """Apply stored cmd_vel as a velocity command via VelocityController.

        Called once per sim step by BridgeNode._step_callback().
        Passes body-frame Twist directly to the controller's set_velocity().
        The controller handles kinematics (body→world rotation) internally.
        """
        if self._latest_twist is None:
            return
        if self._vel_controller is None:
            logger.warning("RobotHandler '%s': no controller set, ignoring cmd_vel", self._ns)
            self._latest_twist = None
            return

        twist = self._latest_twist
        # Pass body-frame Twist directly — controller handles kinematics
        self._vel_controller.set_velocity(
            vx=twist.linear.x,
            vy=twist.linear.y,
            vz=twist.linear.z,
            wx=twist.angular.x,
            wy=twist.angular.y,
            wz=twist.angular.z,
        )

    def publish_state(self, stamp: TimeMsg) -> None:
        """Publish odom, joint_states, TF, and status for this agent."""
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
            dx = goal.position[0] - pose.position[0]
            dy = goal.position[1] - pose.position[1]
            dist = math.sqrt(dx * dx + dy * dy)
            if dist > 0.01:
                # Compute yaw from current position toward goal
                yaw = math.atan2(dy, dx)
                half = yaw * 0.5
                bearing_orientation = [0.0, 0.0, math.sin(half), math.cos(half)]
                from pybullet_fleet.geometry import Pose as PbfPose

                goal_for_viz = PbfPose(position=goal.position, orientation=bearing_orientation)
                goal_msg = pbf_pose_to_pose_stamped(goal_for_viz, stamp=stamp)
            else:
                # Very close — use the goal's own orientation
                goal_msg = pbf_pose_to_pose_stamped(goal, stamp=stamp)
            self._goal_pub.publish(goal_msg)
            dist = float(np.linalg.norm(np.array(pose.position) - np.array(goal.position)))
        else:
            dist = 0.0

        # Current path — publish remaining waypoints
        waypoints = getattr(self.agent, "_path", [])
        current_idx = getattr(self.agent, "_current_waypoint_index", 0)
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
        self.agent.set_controller(None)  # switch to TPI mode for goal-based nav
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

        self.agent.set_controller(None)  # switch to TPI mode for path following
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
        logger.info("RobotHandler destroyed for '%s'", self._ns)
