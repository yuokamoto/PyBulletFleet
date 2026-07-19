"""Composable per-robot ROS interface groups.

``RobotHandler`` remains the compatibility facade, while each group owns one
resource family and its callbacks. The facade can compose only the groups enabled
by bridge configuration.
"""

from __future__ import annotations

import logging
import math
import time
from typing import TYPE_CHECKING

import numpy as np
from builtin_interfaces.msg import Time as TimeMsg
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as PathMsg
from sensor_msgs.msg import BatteryState, JointState
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
    from .robot_handler import RobotHandler

logger = logging.getLogger(__name__)


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

    def post_step(self, dt: float = 0.0, stamp: TimeMsg | None = None) -> None:
        """Publish odom, joint states, status, and battery state."""
        owner = self.owner
        if stamp is None:
            stamp = TimeMsg()
        pose = owner.agent.get_pose()
        velocity = list(owner.agent.velocity)
        angular_vel = owner.agent.angular_velocity

        odom_msg = make_odom_msg(
            pose,
            velocity,
            angular_vel,
            frame_id="odom",
            child_frame_id=f"{owner._ns}/base_link",
            stamp=stamp,
        )
        op = odom_msg.pose.pose.position
        op.x, op.y = owner._shift_xy(op.x, op.y, +1)
        owner._odom_pub.publish(odom_msg)

        if owner.agent.get_num_joints() > 0:
            self._publish_joint_states(stamp)

        self._publish_status(stamp)

        if owner._battery_pub is not None:
            self._publish_battery_state(stamp)

    def _publish_battery_state(self, stamp: TimeMsg) -> None:
        """Publish sensor_msgs/BatteryState with current SOC and status."""
        owner = self.owner
        bp = owner.agent.battery_plugin
        msg = BatteryState()
        msg.header.stamp = stamp
        msg.header.frame_id = owner._ns
        msg.percentage = float(owner.agent.battery_soc)
        msg.present = True

        if owner.agent.is_charging:
            if owner.agent.battery_soc >= 1.0:
                msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
            else:
                msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        elif owner.agent.is_moving:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        elif bp is not None and bp.idle_rate > 0:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        else:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING

        msg.voltage = float("nan")
        msg.current = float("nan")
        msg.charge = float("nan")
        msg.capacity = float("nan")
        msg.design_capacity = float("nan")
        msg.temperature = float("nan")

        owner._battery_pub.publish(msg)

    def _publish_joint_states(self, stamp: TimeMsg) -> None:
        """Publish sensor_msgs/JointState for all joints."""
        owner = self.owner
        joints_by_name = owner.agent.get_all_joints_state_by_name()
        if not joints_by_name:
            return

        names = list(joints_by_name.keys())
        positions = [joints_by_name[n][0] for n in names]
        velocities = [joints_by_name[n][1] for n in names]

        msg = make_joint_state_msg(names, positions, velocities, stamp=stamp)
        owner._joint_pub.publish(msg)

    def _publish_status(self, stamp: TimeMsg) -> None:
        """Publish plan, current_goal, and diagnostics."""
        owner = self.owner
        pose = owner.agent.get_pose()
        velocity = owner.agent.velocity
        speed = float(np.linalg.norm(velocity))

        goal = owner.agent.goal_pose
        if goal is not None:
            goal_msg = pbf_pose_to_pose_stamped(goal, stamp=stamp)
            ggp = goal_msg.pose.position
            ggp.x, ggp.y = owner._shift_xy(ggp.x, ggp.y, +1)
            owner._goal_pub.publish(goal_msg)
            dist = float(np.linalg.norm(np.array(pose.position) - np.array(goal.position)))
        else:
            dist = 0.0

        waypoints = getattr(owner.agent, "path", [])
        current_idx = getattr(owner.agent, "current_waypoint_index", 0)
        if waypoints and current_idx < len(waypoints):
            remaining = waypoints[current_idx:]
            plan_msg = pbf_path_to_ros(remaining, stamp=stamp)
            for ps in plan_msg.poses:
                ps.pose.position.x, ps.pose.position.y = owner._shift_xy(ps.pose.position.x, ps.pose.position.y, +1)
            owner._plan_pub.publish(plan_msg)

        current_action = owner.agent.get_current_action()
        action_type = type(current_action).__name__ if current_action else ""
        action_status = str(current_action.status.name) if current_action else ""

        queue = owner.agent._action_queue
        queue_items = []
        if current_action:
            queue_items.append(f"{type(current_action).__name__}({current_action.status.name})")
        for action in queue:
            queue_items.append(f"{type(action).__name__}({action.status.name})")
        action_queue_str = ", ".join(queue_items) if queue_items else ""
        queue_size = len(queue_items)

        diag_msg = make_diagnostic_msg(
            robot_name=owner._ns,
            is_moving=owner.agent.is_moving,
            action_type=action_type,
            action_status=action_status,
            distance_to_goal=dist,
            linear_speed=speed,
            action_queue=action_queue_str,
            action_queue_size=queue_size,
            stamp=stamp,
        )
        owner._diag_pub.publish(diag_msg)


class TfPublisherHandler(RobotInterfaceGroup):
    """Own per-robot TF publishing."""

    def post_step(self, dt: float = 0.0, stamp: TimeMsg | None = None) -> None:
        """Publish odom -> base_link TF for this robot."""
        owner = self.owner
        if owner._tf_broadcaster is None:
            return
        if stamp is None:
            stamp = TimeMsg()
        pose = owner.agent.get_pose()
        tf_msg = make_transform_stamped(
            pose,
            parent_frame="odom",
            child_frame=f"{owner._ns}/base_link",
            stamp=stamp,
        )
        tt = tf_msg.transform.translation
        tt.x, tt.y = owner._shift_xy(tt.x, tt.y, +1)
        owner._tf_broadcaster.sendTransform(tf_msg)


class CommandTopicHandler(RobotInterfaceGroup):
    """Own per-robot command topic subscriptions."""

    def __init__(self, owner: "RobotHandler") -> None:
        super().__init__(owner)
        node = owner._node
        ns = owner._ns
        owner._cmd_vel_sub = node.create_subscription(Twist, f"/{ns}/cmd_vel", self.cmd_vel_cb, 10)
        owner._goal_pose_sub = node.create_subscription(PoseStamped, f"/{ns}/goal_pose", self.goal_pose_cb, 10)
        owner._path_sub = node.create_subscription(PathMsg, f"/{ns}/path", self.path_cb, 10)
        owner._joint_traj_sub = node.create_subscription(
            JointTrajectory,
            f"/{ns}/joint_trajectory",
            self.joint_traj_cb,
            10,
        )
        owner._joint_cmd_sub = node.create_subscription(
            Float64MultiArray,
            f"/{ns}/joint_commands",
            self.joint_cmd_cb,
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

    def pre_step(self, dt: float = 0.0, stamp: TimeMsg | None = None) -> None:
        """Apply stored cmd_vel before the simulation step."""
        self._apply_cmd_vel()

    def cmd_vel_cb(self, msg: Twist) -> None:
        """Store latest cmd_vel for application in next step."""
        self.owner._latest_twist = msg

    def goal_pose_cb(self, msg: PoseStamped) -> None:
        """Receive goal pose -> agent navigates to it."""
        owner = self.owner
        gp = msg.pose.position
        gp.x, gp.y = owner._shift_xy(gp.x, gp.y, -1)
        pose = ros_pose_stamped_to_pbf(msg)
        owner.agent.set_goal_pose(pose)
        logger.info("'%s': goal_pose received -> navigating to (%.2f, %.2f)", owner._ns, pose.x, pose.y)

    def path_cb(self, msg: PathMsg) -> None:
        """Receive nav path -> agent follows it."""
        owner = self.owner
        waypoints = ros_path_to_pbf(msg)
        if not waypoints:
            logger.warning("'%s': empty path received, ignoring", owner._ns)
            return
        owner.agent.set_path(waypoints)
        logger.info("'%s': path received (%d waypoints)", owner._ns, len(waypoints))

    def joint_traj_cb(self, msg: JointTrajectory) -> None:
        """Receive joint trajectory -> apply final waypoint targets."""
        owner = self.owner
        targets = joint_trajectory_to_targets(msg)
        if not targets:
            logger.warning("'%s': empty joint trajectory, ignoring", owner._ns)
            return

        valid = set(owner.agent.get_all_joints_state_by_name().keys())
        unknown = [name for name in targets if name not in valid]
        if msg.points and len(msg.points[-1].positions) != len(msg.joint_names):
            owner._node.get_logger().warning(
                f"'{owner._ns}': joint_trajectory length mismatch — {len(msg.joint_names)} joint_names "
                f"but {len(msg.points[-1].positions)} positions; extra entries ignored"
            )
        if unknown:
            owner._node.get_logger().warning(
                f"'{owner._ns}': joint_trajectory has unknown joint(s) {unknown} — skipped. "
                f"This robot's joints: {sorted(valid)}"
            )

        owner.agent.set_joints_targets_by_name(targets)
        logger.info(
            "'%s': joint_trajectory received (%d joints, %d applied)",
            owner._ns,
            len(targets),
            len(targets) - len(unknown),
        )

    def joint_cmd_cb(self, msg: Float64MultiArray) -> None:
        """Receive raw joint positions -> set all joints."""
        positions = list(msg.data)
        if positions:
            self.owner.agent.set_all_joints_targets(positions)

    def _apply_cmd_vel(self) -> None:
        """Apply stored cmd_vel as a velocity command."""
        owner = self.owner
        if owner._latest_twist is None:
            return
        ctrl = owner.agent._controller
        if ctrl is None:
            logger.warning("RobotHandler '%s': no controller set, ignoring cmd_vel", owner._ns)
            owner._latest_twist = None
            return

        twist = owner._latest_twist
        ctrl.set_velocity(
            vx=twist.linear.x,
            vy=twist.linear.y,
            vz=twist.linear.z,
            wx=twist.angular.x,
            wy=twist.angular.y,
            wz=twist.angular.z,
        )


class NavigationActionHandler(RobotInterfaceGroup):
    """Own per-robot navigation and joint trajectory action servers."""

    def __init__(self, owner: "RobotHandler") -> None:
        super().__init__(owner)
        self._setup_action_servers()

    def destroy(self) -> None:
        owner = self.owner
        owner._nav_action.destroy()
        owner._follow_path_action.destroy()
        owner._follow_jt_action.destroy()

    def _setup_action_servers(self) -> None:
        """Create NavigateToPose, FollowPath, and FollowJointTrajectory action servers."""
        from control_msgs.action import FollowJointTrajectory
        from nav2_msgs.action import FollowPath, NavigateToPose
        from rclpy.action import ActionServer, CancelResponse, GoalResponse
        from rclpy.callback_groups import ReentrantCallbackGroup

        owner = self.owner
        owner._action_group = ReentrantCallbackGroup()

        def _accept_goal(goal_request):  # type: ignore[no-untyped-def]
            return GoalResponse.ACCEPT

        def _accept_cancel(goal_handle):  # type: ignore[no-untyped-def]
            owner.agent.stop()
            return CancelResponse.ACCEPT

        owner._nav_action = ActionServer(
            owner._node,
            NavigateToPose,
            f"/{owner._ns}/navigate_to_pose",
            execute_callback=self.navigate_execute,
            goal_callback=_accept_goal,
            cancel_callback=_accept_cancel,
            callback_group=owner._action_group,
        )
        owner._follow_path_action = ActionServer(
            owner._node,
            FollowPath,
            f"/{owner._ns}/follow_path",
            execute_callback=self.follow_path_execute,
            goal_callback=_accept_goal,
            cancel_callback=_accept_cancel,
            callback_group=owner._action_group,
        )
        owner._follow_jt_action = ActionServer(
            owner._node,
            FollowJointTrajectory,
            f"/{owner._ns}/follow_joint_trajectory",
            execute_callback=self.follow_jt_execute,
            goal_callback=_accept_goal,
            cancel_callback=_accept_cancel,
            callback_group=owner._action_group,
        )

    def navigate_execute(self, goal_handle):  # type: ignore[no-untyped-def]
        """Execute NavigateToPose action — set goal and poll until arrival."""
        from nav2_msgs.action import NavigateToPose

        owner = self.owner
        gp = goal_handle.request.pose.pose.position
        gp.x, gp.y = owner._shift_xy(gp.x, gp.y, -1)
        goal_pose = ros_pose_to_pbf(goal_handle.request.pose.pose)
        owner.agent.set_goal_pose(goal_pose)
        owner._node.get_logger().info(f"'{owner._ns}': NavigateToPose started -> ({goal_pose.x:.2f}, {goal_pose.y:.2f})")

        feedback = NavigateToPose.Feedback()
        poll_period = 0.1

        while owner.agent.is_moving:
            if goal_handle.is_cancel_requested:
                owner.agent.stop()
                goal_handle.canceled()
                owner._node.get_logger().info(f"'{owner._ns}': NavigateToPose canceled")
                return NavigateToPose.Result()

            current = owner.agent.get_pose()
            feedback.current_pose = pbf_pose_to_pose_stamped(current)
            dx = current.x - goal_pose.x
            dy = current.y - goal_pose.y
            feedback.distance_remaining = float(math.sqrt(dx * dx + dy * dy))
            goal_handle.publish_feedback(feedback)

            time.sleep(poll_period)

        goal_handle.succeed()
        owner._node.get_logger().info(f"'{owner._ns}': NavigateToPose succeeded")
        return NavigateToPose.Result()

    def follow_path_execute(self, goal_handle):  # type: ignore[no-untyped-def]
        """Execute FollowPath action — set path and poll until completion."""
        from nav2_msgs.action import FollowPath

        owner = self.owner
        waypoints = ros_path_to_pbf(goal_handle.request.path)
        if not waypoints:
            goal_handle.abort()
            return FollowPath.Result()

        owner.agent.set_path(waypoints)
        owner._node.get_logger().info(f"'{owner._ns}': FollowPath started ({len(waypoints)} waypoints)")

        feedback = FollowPath.Feedback()
        poll_period = 0.1

        while owner.agent.is_moving:
            if goal_handle.is_cancel_requested:
                owner.agent.stop()
                goal_handle.canceled()
                return FollowPath.Result()

            current = owner.agent.get_pose()
            feedback.distance_to_goal = float(
                math.sqrt((current.x - waypoints[-1].x) ** 2 + (current.y - waypoints[-1].y) ** 2)
            )
            goal_handle.publish_feedback(feedback)
            time.sleep(poll_period)

        goal_handle.succeed()
        owner._node.get_logger().info(f"'{owner._ns}': FollowPath succeeded")
        return FollowPath.Result()

    def follow_jt_execute(self, goal_handle):  # type: ignore[no-untyped-def]
        """Execute FollowJointTrajectory — apply each trajectory point sequentially."""
        from control_msgs.action import FollowJointTrajectory

        owner = self.owner
        traj = goal_handle.request.trajectory
        joint_names = list(traj.joint_names)

        if not traj.points:
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        logger.info(
            "'%s': FollowJointTrajectory started (%d points, %d joints)",
            owner._ns,
            len(traj.points),
            len(joint_names),
        )

        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = joint_names
        poll_period = 0.05
        timeout_per_point = 10.0

        for point in traj.points:
            targets: dict = {}
            for i, name in enumerate(joint_names):
                if i < len(point.positions):
                    targets[name] = point.positions[i]
            owner.agent.set_joints_targets_by_name(targets)

            elapsed = 0.0
            while elapsed < timeout_per_point:
                if goal_handle.is_cancel_requested:
                    owner.agent.stop()
                    goal_handle.canceled()
                    return FollowJointTrajectory.Result()

                if owner.agent.are_joints_at_targets_by_name(targets):
                    break

                joints_state = owner.agent.get_all_joints_state_by_name()
                actual_pos = [joints_state.get(n, (0.0, 0.0))[0] for n in joint_names]
                desired_pos = [targets.get(n, 0.0) for n in joint_names]
                error_pos = [actual - desired for actual, desired in zip(actual_pos, desired_pos)]

                feedback.actual.positions = actual_pos
                feedback.desired.positions = desired_pos
                feedback.error.positions = error_pos
                goal_handle.publish_feedback(feedback)

                time.sleep(poll_period)
                elapsed += poll_period

        goal_handle.succeed()
        logger.info("'%s': FollowJointTrajectory succeeded", owner._ns)
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result


class ExecuteActionHandler(RobotInterfaceGroup):
    """Own per-robot ExecuteAction topic and action server."""

    def __init__(self, owner: "RobotHandler") -> None:
        super().__init__(owner)
        self._setup_execute_action()

    def destroy(self) -> None:
        owner = self.owner
        owner._node.destroy_subscription(owner._exec_action_sub)
        owner._exec_action_server.destroy()

    def _setup_execute_action(self) -> None:
        """Create /{robot}/execute_action topic subscriber and action server."""
        from pybullet_fleet_msgs.action import ExecuteAction
        from pybullet_fleet_msgs.msg import ExecuteActionGoal
        from rclpy.action import ActionServer, GoalResponse
        from rclpy.callback_groups import ReentrantCallbackGroup

        owner = self.owner
        owner._exec_action_sub = owner._node.create_subscription(
            ExecuteActionGoal,
            f"/{owner._ns}/execute_action",
            self.execute_action_topic_cb,
            10,
        )

        owner._exec_action_group = ReentrantCallbackGroup()
        owner._exec_action_server = ActionServer(
            owner._node,
            ExecuteAction,
            f"/{owner._ns}/execute_action_blocking",
            execute_callback=self.execute_action_execute,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=self.execute_action_cancel,
            callback_group=owner._exec_action_group,
        )

    def execute_action_topic_cb(self, msg) -> None:
        """Handle execute_action topic — parse and queue action."""
        from .action_parser import parse_action_goal

        owner = self.owner
        action = parse_action_goal(msg.action_type, msg.action_params_json)
        if action is None:
            logger.error(
                "'%s': failed to parse execute_action topic: type='%s'",
                owner._ns,
                msg.action_type,
            )
            return
        owner.agent.add_action(action)
        logger.info(
            "'%s': execute_action topic -> queued %s (queue size: %d)",
            owner._ns,
            type(action).__name__,
            owner.agent.get_action_queue_size(),
        )

    def execute_action_cancel(self, goal_handle):
        """Cancel callback for ExecuteAction."""
        from rclpy.action import CancelResponse

        self.owner.agent.stop()
        return CancelResponse.ACCEPT

    def execute_action_execute(self, goal_handle):
        """Execute callback for ExecuteAction action server."""
        from pybullet_fleet.types import ActionStatus
        from pybullet_fleet_msgs.action import ExecuteAction

        from .action_parser import parse_action_goal

        owner = self.owner
        goal_msg = goal_handle.request.goal
        action = parse_action_goal(goal_msg.action_type, goal_msg.action_params_json)
        if action is None:
            goal_handle.abort()
            result = ExecuteAction.Result()
            result.success = False
            result.message = f"Failed to parse action: type='{goal_msg.action_type}'"
            return result

        owner.agent.add_action(action)
        logger.info("'%s': ExecuteAction started — %s", owner._ns, type(action).__name__)

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

            feedback.status = (
                ExecuteAction.Feedback.STATUS_IN_PROGRESS
                if status == ActionStatus.IN_PROGRESS
                else ExecuteAction.Feedback.STATUS_NOT_STARTED
            )
            feedback.progress = getattr(action, "progress", 0.0)
            goal_handle.publish_feedback(feedback)

            time.sleep(poll_period)

        goal_handle.succeed()
        logger.info("'%s': ExecuteAction succeeded — %s", owner._ns, type(action).__name__)
        result = ExecuteAction.Result()
        result.success = True
        result.message = f"{type(action).__name__} completed"
        return result


class ServiceHandler(RobotInterfaceGroup):
    """Own per-robot services."""

    def __init__(self, owner: "RobotHandler") -> None:
        super().__init__(owner)
        owner._charging_srv = None
        self._setup_attach_service()
        self._setup_attach_object_service()
        if owner.agent.battery_plugin is not None:
            self._setup_charging_service()

    def destroy(self) -> None:
        owner = self.owner
        node = owner._node
        node.destroy_service(owner._attach_srv)
        node.destroy_service(owner._attach_object_srv)
        if owner._charging_srv is not None:
            node.destroy_service(owner._charging_srv)

    def _setup_attach_service(self) -> None:
        """Create /{robot}/toggle_attach SetBool service."""
        from rclpy.callback_groups import ReentrantCallbackGroup
        from std_srvs.srv import SetBool

        owner = self.owner
        owner._attach_group = ReentrantCallbackGroup()
        owner._attach_srv = owner._node.create_service(
            SetBool,
            f"/{owner._ns}/toggle_attach",
            self.toggle_attach_cb,
            callback_group=owner._attach_group,
        )

    def toggle_attach_cb(self, request, response):  # type: ignore[no-untyped-def]
        """Handle toggle_attach service — immediate attach/detach."""
        owner = self.owner
        if request.data:
            obj = self._find_nearest_pickable(search_radius=1.0)
            if obj is None:
                response.success = False
                response.message = "No pickable object within 1.0m"
                return response
            ok = owner.agent.attach_object(obj, keep_world_pose=True)
            response.success = ok
            response.message = f"attached '{obj.name}'" if ok else f"attach failed for '{obj.name}'"
        else:
            attached = owner.agent.get_attached_objects()
            if not attached:
                response.success = False
                response.message = "No attached object to detach"
                return response
            obj = attached[0]
            ok = owner.agent.detach_object(obj)
            response.success = ok
            response.message = f"detached '{obj.name}'" if ok else f"detach failed for '{obj.name}'"

        logger.info("'%s': toggle_attach done — %s", owner._ns, response.message)
        return response

    def _setup_attach_object_service(self) -> None:
        """Create /{robot}/attach_object service for detailed attach/detach."""
        from pybullet_fleet_msgs.srv import AttachObject
        from rclpy.callback_groups import ReentrantCallbackGroup

        owner = self.owner
        owner._attach_object_group = ReentrantCallbackGroup()
        owner._attach_object_srv = owner._node.create_service(
            AttachObject,
            f"/{owner._ns}/attach_object",
            self.attach_object_cb,
            callback_group=owner._attach_object_group,
        )

    def attach_object_cb(self, request, response):  # type: ignore[no-untyped-def]
        """Handle attach_object service — immediate attach/detach with parameters."""
        from pybullet_fleet.geometry import Pose as PbfPose

        owner = self.owner
        command = request.command
        if command.name and command.name != owner._ns:
            response.success = False
            response.message = f"Command target '{command.name}' does not match service robot '{owner._ns}'"
            response.attached_object_name = ""
            return response

        object_name = command.object_name if command.object_name else ""
        parent_link = command.parent_link if command.parent_link else "base_link"
        search_radius = command.search_radius if command.search_radius > 0 else 0.5

        offset_pos = [
            command.offset.position.x,
            command.offset.position.y,
            command.offset.position.z,
        ]
        offset_ori = [
            command.offset.orientation.x,
            command.offset.orientation.y,
            command.offset.orientation.z,
            command.offset.orientation.w,
        ]
        if all(v == 0.0 for v in offset_ori):
            offset_ori = [0.0, 0.0, 0.0, 1.0]
        attach_offset = PbfPose(position=offset_pos, orientation=offset_ori)

        if command.attach:
            target_obj = None
            if object_name:
                sim_core = getattr(owner.agent, "sim_core", None)
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

            ok = owner.agent.attach_object(
                target_obj,
                parent_link_index=parent_link,
                relative_pose=attach_offset,
            )
            response.success = ok
            response.attached_object_name = target_obj.name or ""
            response.message = f"attached '{target_obj.name}'" if ok else f"attach failed for '{target_obj.name}'"
        else:
            target_obj = None
            if object_name:
                for obj in owner.agent.get_attached_objects():
                    if obj.name == object_name:
                        target_obj = obj
                        break
                if target_obj is None:
                    response.success = False
                    response.message = f"Object '{object_name}' not attached"
                    response.attached_object_name = ""
                    return response
            else:
                attached = owner.agent.get_attached_objects()
                if not attached:
                    response.success = False
                    response.message = "No attached object to detach"
                    response.attached_object_name = ""
                    return response
                target_obj = attached[0]

            ok = owner.agent.detach_object(target_obj)
            response.success = ok
            response.attached_object_name = target_obj.name or ""
            response.message = f"detached '{target_obj.name}'" if ok else f"detach failed for '{target_obj.name}'"

        logger.info("'%s': attach_object done — %s", owner._ns, response.message)
        return response

    def _find_nearest_pickable(self, search_radius: float = 1.0):
        """Find the nearest pickable, unattached SimObject within radius."""
        return self.owner.agent.find_nearest_pickable(search_radius=search_radius)

    def _setup_charging_service(self) -> None:
        """Create /{robot}/set_charging SetBool service."""
        from std_srvs.srv import SetBool

        owner = self.owner
        owner._charging_srv = owner._node.create_service(
            SetBool,
            f"/{owner._ns}/set_charging",
            self.set_charging_cb,
        )

    def set_charging_cb(self, request, response):  # type: ignore[no-untyped-def]
        """Handle set_charging service — start/stop battery charging."""
        owner = self.owner
        if owner.agent.battery_plugin is None:
            response.success = False
            response.message = "Battery simulation not configured for this agent"
            return response
        owner.agent.set_charging(request.data)
        state = "started" if request.data else "stopped"
        response.success = True
        response.message = f"Charging {state}"
        logger.info("'%s': set_charging -> %s", owner._ns, state)
        return response
