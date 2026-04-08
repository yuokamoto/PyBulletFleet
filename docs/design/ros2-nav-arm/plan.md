# ROS 2 Nav2 + Arm Interface Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use executing-plans to implement this plan task-by-task.

**Goal:** Add Nav2-compatible navigation topics/actions, arm control topics/actions, status/diagnostics publishers, and RViz demo launch files to the ROS 2 bridge.

**Architecture:** Extend `RobotHandler` with 3 layers: (1) topic subscribers for simple publish→move, (2) action servers for Nav2/MoveIt compatibility, (3) status publishers for diagnostics/RViz. New conversion helpers in `conversions.py`. Demo launch files with RViz configs.

**Tech Stack:** Python 3.10+, rclpy, nav2_msgs, control_msgs, trajectory_msgs, diagnostic_msgs, Docker (Jazzy)

---

## Task Dependency Graph

```
Task 1 (conversions)  ──┐
                        ├──► Task 3 (topic subs) ──┐
Task 2 (package deps) ──┘                          ├──► Task 5 (action servers) ──► Task 7 (launch + rviz)
                        ┌──► Task 4 (status pubs) ──┘                              Task 8 (client scripts)
                        │
Task 1 ─────────────────┘
                                                                                   Task 6 (Docker) [PARALLEL]
```

- **PARALLEL:** Tasks 1+2 can run together. Task 6 is independent.
- **SERIAL:** Task 3 depends on 1+2. Task 4 depends on 1. Task 5 depends on 3+4. Tasks 7+8 depend on 5.

---

## Task 1: Conversion Helpers (PARALLEL with Task 2)

**Files:**
- Modify: `ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/conversions.py`

**Step 1: Add imports and new conversion functions**

Add these imports at the top of `conversions.py`:

```python
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMsg
from trajectory_msgs.msg import JointTrajectory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
```

Add these functions after existing functions:

```python
def pbf_pose_to_pose_stamped(
    pbf_pose: Pose,
    frame_id: str = "odom",
    stamp: Optional[TimeMsg] = None,
) -> PoseStamped:
    """Convert PyBulletFleet Pose → geometry_msgs/PoseStamped."""
    msg = PoseStamped()
    msg.header = Header(frame_id=frame_id)
    if stamp:
        msg.header.stamp = stamp
    msg.pose = pbf_pose_to_ros(pbf_pose)
    return msg


def ros_pose_stamped_to_pbf(msg: PoseStamped) -> Pose:
    """Convert geometry_msgs/PoseStamped → PyBulletFleet Pose."""
    return ros_pose_to_pbf(msg.pose)


def pbf_path_to_ros(
    waypoints: list,
    frame_id: str = "odom",
    stamp: Optional[TimeMsg] = None,
) -> PathMsg:
    """Convert list of PyBulletFleet Pose → nav_msgs/Path."""
    msg = PathMsg()
    msg.header = Header(frame_id=frame_id)
    if stamp:
        msg.header.stamp = stamp
    for wp in waypoints:
        msg.poses.append(pbf_pose_to_pose_stamped(wp, frame_id=frame_id, stamp=stamp))
    return msg


def ros_path_to_pbf(msg: PathMsg) -> list:
    """Convert nav_msgs/Path → list of PyBulletFleet Pose."""
    return [ros_pose_stamped_to_pbf(ps) for ps in msg.poses]


def joint_trajectory_to_targets(msg: JointTrajectory) -> dict:
    """Extract final waypoint from JointTrajectory as {name: position} dict.

    Takes the last trajectory point's positions and maps them to joint names.
    """
    if not msg.points:
        return {}
    last_point = msg.points[-1]
    targets = {}
    for i, name in enumerate(msg.joint_names):
        if i < len(last_point.positions):
            targets[name] = last_point.positions[i]
    return targets


def make_diagnostic_msg(
    robot_name: str,
    is_moving: bool,
    action_type: str = "",
    action_status: str = "",
    distance_to_goal: float = 0.0,
    linear_speed: float = 0.0,
    action_queue: str = "",
    action_queue_size: int = 0,
    stamp: Optional[TimeMsg] = None,
) -> DiagnosticArray:
    """Create diagnostic_msgs/DiagnosticArray with robot status."""
    status = DiagnosticStatus()
    status.name = robot_name
    status.level = DiagnosticStatus.OK
    status.message = f"{action_type} {action_status}" if action_type else ("moving" if is_moving else "idle")
    status.values = [
        KeyValue(key="is_moving", value=str(is_moving).lower()),
        KeyValue(key="action_type", value=action_type),
        KeyValue(key="action_status", value=action_status),
        KeyValue(key="distance_to_goal", value=f"{distance_to_goal:.3f}"),
        KeyValue(key="linear_speed", value=f"{linear_speed:.3f}"),
        KeyValue(key="action_queue", value=action_queue),
        KeyValue(key="action_queue_size", value=str(action_queue_size)),
    ]
    msg = DiagnosticArray()
    if stamp:
        msg.header.stamp = stamp
    msg.status = [status]
    return msg
```

**Step 2: Verify imports work**

Run: `cd docker && docker compose build bridge` (will be tested after Task 2 adds deps)

**Step 3: Commit**

```bash
git add ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/conversions.py
git commit -m "feat(ros2): add nav/arm/diagnostic conversion helpers"
```

---

## Task 2: Package Dependencies (PARALLEL with Task 1)

**Files:**
- Modify: `ros2_bridge/pybullet_fleet_ros/package.xml`
- Modify: `docker/Dockerfile.jazzy`

**Step 1: Add dependencies to package.xml**

Add after existing `<depend>` entries:

```xml
  <depend>trajectory_msgs</depend>
  <depend>diagnostic_msgs</depend>
```

**Step 2: Add apt packages to Dockerfile**

Add to the `apt-get install` block in `docker/Dockerfile.jazzy`:

```dockerfile
    ros-jazzy-trajectory-msgs \
    ros-jazzy-diagnostic-msgs \
    ros-jazzy-rviz2 \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
```

Note: `trajectory_msgs` and `diagnostic_msgs` may already be pulled in transitively, but explicit is better. `rviz2` and `robot_state_publisher` are needed for demo launches.

**Step 3: Commit**

```bash
git add ros2_bridge/pybullet_fleet_ros/package.xml docker/Dockerfile.jazzy
git commit -m "feat(ros2): add trajectory/diagnostic/visualization deps"
```

---

## Task 3: Topic Subscribers — Nav + Arm (SERIAL, depends on Tasks 1+2)

**Files:**
- Modify: `ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/robot_handler.py`

**Step 1: Add imports**

At top of robot_handler.py, add:

```python
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMsg
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory

from .conversions import (
    # existing imports...
    ros_pose_stamped_to_pbf,
    ros_path_to_pbf,
    joint_trajectory_to_targets,
)
```

**Step 2: Add subscribers in `__init__`**

After the existing `self._cmd_vel_sub` line, add:

```python
        # Navigation topic subscribers
        self._goal_pose_sub = node.create_subscription(
            PoseStamped, f"/{ns}/goal_pose", self._goal_pose_cb, 10
        )
        self._path_sub = node.create_subscription(
            PathMsg, f"/{ns}/path", self._path_cb, 10
        )

        # Arm topic subscribers
        self._joint_traj_sub = node.create_subscription(
            JointTrajectory, f"/{ns}/joint_trajectory", self._joint_traj_cb, 10
        )
        self._joint_cmd_sub = node.create_subscription(
            Float64MultiArray, f"/{ns}/joint_commands", self._joint_cmd_cb, 10
        )
```

**Step 3: Add callback methods**

```python
    def _goal_pose_cb(self, msg: PoseStamped) -> None:
        """Receive goal pose → agent navigates to it."""
        pose = ros_pose_stamped_to_pbf(msg)
        self.agent.set_goal_pose(pose)
        logger.info("'%s': goal_pose received → navigating to (%.2f, %.2f)", self._ns, pose.x, pose.y)

    def _path_cb(self, msg: PathMsg) -> None:
        """Receive nav path → agent follows it."""
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
```

**Step 4: Update `destroy()` to clean up new subscriptions**

```python
    def destroy(self) -> None:
        """Clean up ROS interfaces."""
        self._node.destroy_publisher(self._odom_pub)
        self._node.destroy_publisher(self._joint_pub)
        self._node.destroy_subscription(self._cmd_vel_sub)
        self._node.destroy_subscription(self._goal_pose_sub)
        self._node.destroy_subscription(self._path_sub)
        self._node.destroy_subscription(self._joint_traj_sub)
        self._node.destroy_subscription(self._joint_cmd_sub)
        logger.info("RobotHandler destroyed for '%s'", self._ns)
```

**Step 5: Build and verify in Docker**

Run: `cd docker && docker compose build bridge`

**Step 6: Commit**

```bash
git add ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/robot_handler.py
git commit -m "feat(ros2): add goal_pose/path/joint_trajectory/joint_commands topic subscribers"
```

---

## Task 4: Status & Visualization Publishers (SERIAL, depends on Task 1)

**Files:**
- Modify: `ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/robot_handler.py`

**Step 1: Add publisher imports and creation**

Add to imports:

```python
from diagnostic_msgs.msg import DiagnosticArray

from .conversions import (
    # ...existing + new...
    pbf_pose_to_pose_stamped,
    pbf_path_to_ros,
    make_diagnostic_msg,
)
```

Add publishers in `__init__` after subscribers:

```python
        # Status & visualization publishers
        self._plan_pub = node.create_publisher(PathMsg, f"/{ns}/plan", 10)
        self._goal_pub = node.create_publisher(PoseStamped, f"/{ns}/current_goal", 10)
        self._diag_pub = node.create_publisher(DiagnosticArray, f"/{ns}/diagnostics", 10)
```

**Step 2: Add status publishing to `publish_state()`**

After existing odom/tf/joint_states publishing, add:

```python
        # Status & visualization
        self._publish_status(stamp)

    def _publish_status(self, stamp) -> None:
        """Publish plan, current_goal, and diagnostics."""
        import numpy as np

        pose = self.agent.get_pose()
        velocity = self.agent.velocity
        speed = float(np.linalg.norm(velocity))

        # Current goal
        goal = self.agent.goal_pose
        if goal is not None:
            goal_msg = pbf_pose_to_pose_stamped(goal, stamp=stamp)
            self._goal_pub.publish(goal_msg)

            # Distance to goal
            dist = float(np.linalg.norm(
                np.array(pose.position) - np.array(goal.position)
            ))
        else:
            dist = 0.0

        # Current path (if following a path, Agent stores it in _path_waypoints)
        waypoints = getattr(self.agent, '_path_waypoints', [])
        current_idx = getattr(self.agent, '_current_waypoint_index', 0)
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
```

**Step 3: Update `destroy()` with new publishers**

Add to destroy():

```python
        self._node.destroy_publisher(self._plan_pub)
        self._node.destroy_publisher(self._goal_pub)
        self._node.destroy_publisher(self._diag_pub)
```

**Step 4: Commit**

```bash
git add ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/robot_handler.py
git commit -m "feat(ros2): add plan/current_goal/diagnostics status publishers"
```

---

## Task 5: Action Servers (SERIAL, depends on Tasks 3+4)

**Files:**
- Modify: `ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/robot_handler.py`

**Step 1: Add action imports**

```python
import asyncio
import math

from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from nav2_msgs.action import NavigateToPose, FollowPath
from control_msgs.action import FollowJointTrajectory
```

**Step 2: Create action servers in `__init__`**

After publishers, add:

```python
        # Action servers (ReentrantCallbackGroup allows concurrent goals)
        self._action_group = ReentrantCallbackGroup()

        self._nav_action = ActionServer(
            node,
            NavigateToPose,
            f"/{ns}/navigate_to_pose",
            execute_callback=self._navigate_execute,
            goal_callback=self._default_goal_cb,
            cancel_callback=self._default_cancel_cb,
            callback_group=self._action_group,
        )

        self._follow_path_action = ActionServer(
            node,
            FollowPath,
            f"/{ns}/follow_path",
            execute_callback=self._follow_path_execute,
            goal_callback=self._default_goal_cb,
            cancel_callback=self._default_cancel_cb,
            callback_group=self._action_group,
        )

        self._follow_jt_action = ActionServer(
            node,
            FollowJointTrajectory,
            f"/{ns}/follow_joint_trajectory",
            execute_callback=self._follow_jt_execute,
            goal_callback=self._default_goal_cb,
            cancel_callback=self._default_cancel_cb,
            callback_group=self._action_group,
        )
```

**Step 3: Add goal/cancel callbacks**

```python
    def _default_goal_cb(self, goal_request):
        """Accept all goals."""
        return GoalResponse.ACCEPT

    def _default_cancel_cb(self, goal_handle):
        """Accept all cancel requests."""
        self.agent.stop()
        return CancelResponse.ACCEPT
```

**Step 4: Implement NavigateToPose execute**

```python
    async def _navigate_execute(self, goal_handle):
        """Execute NavigateToPose action — set goal and poll until arrival."""
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

            await asyncio.sleep(poll_period)

        goal_handle.succeed()
        logger.info("'%s': NavigateToPose succeeded", self._ns)
        return NavigateToPose.Result()
```

**Step 5: Implement FollowPath execute**

```python
    async def _follow_path_execute(self, goal_handle):
        """Execute FollowPath action — set path and poll until completion."""
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
            feedback.distance_to_goal = float(math.sqrt(
                (current.x - waypoints[-1].x) ** 2 +
                (current.y - waypoints[-1].y) ** 2
            ))
            goal_handle.publish_feedback(feedback)
            await asyncio.sleep(poll_period)

        goal_handle.succeed()
        logger.info("'%s': FollowPath succeeded", self._ns)
        return FollowPath.Result()
```

**Step 6: Implement FollowJointTrajectory execute**

```python
    async def _follow_jt_execute(self, goal_handle):
        """Execute FollowJointTrajectory — apply each trajectory point sequentially."""
        traj = goal_handle.request.trajectory
        joint_names = list(traj.joint_names)

        if not traj.points:
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        logger.info("'%s': FollowJointTrajectory started (%d points, %d joints)",
                     self._ns, len(traj.points), len(joint_names))

        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = joint_names
        poll_period = 0.05  # 50ms — check at sim rate
        timeout_per_point = 10.0  # seconds max per waypoint

        for point_idx, point in enumerate(traj.points):
            # Apply this waypoint's targets
            targets = {}
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

                await asyncio.sleep(poll_period)
                elapsed += poll_period

        goal_handle.succeed()
        logger.info("'%s': FollowJointTrajectory succeeded", self._ns)
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result
```

**Step 7: Update `destroy()` with action server cleanup**

Add to destroy():

```python
        self._nav_action.destroy()
        self._follow_path_action.destroy()
        self._follow_jt_action.destroy()
```

**Step 8: Build and test in Docker**

Run:
```bash
cd docker && docker compose build bridge
docker compose run --rm bridge ros2 run pybullet_fleet_ros bridge_node --ros-args -p num_robots:=1 -p gui:=false
```

In another terminal:
```bash
docker compose exec bridge bash -c "source /opt/bridge_ws/install/setup.bash && ros2 action list"
```

Expected output includes: `/{name}/navigate_to_pose`, `/{name}/follow_path`, `/{name}/follow_joint_trajectory`

**Step 9: Commit**

```bash
git add ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/robot_handler.py
git commit -m "feat(ros2): add NavigateToPose/FollowPath/FollowJointTrajectory action servers"
```

---

## Task 6: Docker Image Update (PARALLEL — independent)

**Files:**
- Modify: `docker/Dockerfile.jazzy`

Already covered in Task 2. Verify Docker builds correctly:

```bash
cd docker && docker compose build bridge
```

If additional runtime issues, fix in this task.

---

## Task 7: Demo Launch Files + RViz Configs (SERIAL, depends on Task 5)

**Files:**
- Create: `ros2_bridge/pybullet_fleet_ros/launch/nav_demo.launch.py`
- Create: `ros2_bridge/pybullet_fleet_ros/launch/arm_demo.launch.py`
- Create: `ros2_bridge/pybullet_fleet_ros/config/nav_demo.rviz`
- Create: `ros2_bridge/pybullet_fleet_ros/config/arm_demo.rviz`
- Modify: `ros2_bridge/pybullet_fleet_ros/setup.py` (include data files)

**Step 1: Create nav_demo.launch.py**

```python
"""Launch bridge with a mobile robot and RViz for navigation demo."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory("pybullet_fleet_ros")
    rviz_config = os.path.join(pkg_dir, "config", "nav_demo.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("num_robots", default_value="1"),
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("target_rtf", default_value="1.0"),
            Node(
                package="pybullet_fleet_ros",
                executable="bridge_node",
                name="pybullet_fleet_bridge",
                parameters=[
                    {
                        "num_robots": LaunchConfiguration("num_robots"),
                        "gui": LaunchConfiguration("gui"),
                        "target_rtf": LaunchConfiguration("target_rtf"),
                        "publish_rate": 50.0,
                    }
                ],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            ),
        ]
    )
```

**Step 2: Create arm_demo.launch.py**

```python
"""Launch bridge with an arm robot and RViz for arm control demo."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory("pybullet_fleet_ros")
    rviz_config = os.path.join(pkg_dir, "config", "arm_demo.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("target_rtf", default_value="1.0"),
            Node(
                package="pybullet_fleet_ros",
                executable="bridge_node",
                name="pybullet_fleet_bridge",
                parameters=[
                    {
                        "num_robots": 0,  # No mobile robots
                        "robot_urdf": "robots/arm_robot.urdf",
                        "gui": LaunchConfiguration("gui"),
                        "target_rtf": LaunchConfiguration("target_rtf"),
                        "publish_rate": 50.0,
                    }
                ],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            ),
        ]
    )
```

**Step 3: Create nav_demo.rviz**

Minimal RViz config with: Fixed Frame=odom, Grid, TF, Odometry, Path (/{name}/plan), Pose (/{name}/current_goal).

(Full YAML RViz config — generate one with sensible defaults for TF + Path + Odometry displays.)

**Step 4: Create arm_demo.rviz**

Minimal RViz config with: Fixed Frame=odom, Grid, TF, RobotModel (if available), JointState displays.

**Step 5: Update setup.py to include config and launch data files**

In `ros2_bridge/pybullet_fleet_ros/setup.py`, ensure `data_files` includes:

```python
data_files=[
    ("share/ament_index/resource_index/packages", ["resource/pybullet_fleet_ros"]),
    ("share/pybullet_fleet_ros", ["package.xml"]),
    ("share/pybullet_fleet_ros/launch", glob("launch/*.py")),
    ("share/pybullet_fleet_ros/config", glob("config/*")),
],
```

**Step 6: Commit**

```bash
git add ros2_bridge/pybullet_fleet_ros/launch/ ros2_bridge/pybullet_fleet_ros/config/ ros2_bridge/pybullet_fleet_ros/setup.py
git commit -m "feat(ros2): add nav_demo + arm_demo launch files with RViz configs"
```

---

## Task 8: Client Scripts (SERIAL, depends on Task 5)

**Files:**
- Create: `ros2_bridge/scripts/send_path.py`
- Create: `ros2_bridge/scripts/send_joint_trajectory.py`
- Modify: `ros2_bridge/scripts/send_nav_goal.py` (update from stub to working)

**Step 1: Create send_path.py**

```python
#!/usr/bin/env python3
"""Publish a square path for a robot to follow.

Usage:
    ros2 run pybullet_fleet_ros send_path.py          # default robot0
    python3 send_path.py --robot robot1 --size 3.0
"""

import argparse
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


def make_square_path(cx: float, cy: float, size: float, num_per_side: int = 5) -> Path:
    """Create a square path centered at (cx, cy)."""
    path = Path()
    path.header.frame_id = "odom"
    half = size / 2.0
    corners = [
        (cx - half, cy - half),
        (cx + half, cy - half),
        (cx + half, cy + half),
        (cx - half, cy + half),
        (cx - half, cy - half),  # close the loop
    ]
    for i in range(len(corners) - 1):
        x0, y0 = corners[i]
        x1, y1 = corners[i + 1]
        for j in range(num_per_side):
            t = j / num_per_side
            ps = PoseStamped()
            ps.header.frame_id = "odom"
            ps.pose.position.x = x0 + (x1 - x0) * t
            ps.pose.position.y = y0 + (y1 - y0) * t
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
    return path


def main():
    parser = argparse.ArgumentParser(description="Send a square path to a robot")
    parser.add_argument("--robot", default="robot0", help="Robot name (default: robot0)")
    parser.add_argument("--size", type=float, default=2.0, help="Square side length (default: 2.0)")
    parser.add_argument("--cx", type=float, default=0.0, help="Center X")
    parser.add_argument("--cy", type=float, default=0.0, help="Center Y")
    args = parser.parse_args()

    rclpy.init()
    node = Node("send_path_client")
    pub = node.create_publisher(Path, f"/{args.robot}/path", 10)

    # Wait for subscriber
    import time
    time.sleep(1.0)

    path = make_square_path(args.cx, args.cy, args.size)
    pub.publish(path)
    node.get_logger().info(f"Published square path ({len(path.poses)} waypoints) to /{args.robot}/path")

    # Keep alive briefly for message delivery
    time.sleep(0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

**Step 2: Create send_joint_trajectory.py**

```python
#!/usr/bin/env python3
"""Send a joint trajectory to an arm robot.

Usage:
    python3 send_joint_trajectory.py --robot arm0
    python3 send_joint_trajectory.py --robot arm0 --positions 0.5 1.0 -0.5 0.0
"""

import argparse

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


def main():
    parser = argparse.ArgumentParser(description="Send a joint trajectory")
    parser.add_argument("--robot", default="robot0", help="Robot name")
    parser.add_argument("--joints", nargs="+", default=["joint_1", "joint_2", "joint_3", "joint_4"],
                        help="Joint names")
    parser.add_argument("--positions", nargs="+", type=float, default=[0.5, 1.0, -0.5, 0.0],
                        help="Target joint positions")
    args = parser.parse_args()

    rclpy.init()
    node = Node("send_joint_trajectory_client")
    pub = node.create_publisher(JointTrajectory, f"/{args.robot}/joint_trajectory", 10)

    import time
    time.sleep(1.0)

    msg = JointTrajectory()
    msg.joint_names = args.joints
    point = JointTrajectoryPoint()
    point.positions = args.positions
    point.time_from_start = Duration(sec=2, nanosec=0)
    msg.points = [point]

    pub.publish(msg)
    node.get_logger().info(
        f"Published JointTrajectory to /{args.robot}/joint_trajectory: "
        f"joints={args.joints}, positions={args.positions}"
    )

    time.sleep(0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

**Step 3: Update send_nav_goal.py (replace stub)**

Replace the Phase 2 stub with a working `NavigateToPose` action client.

```python
#!/usr/bin/env python3
"""Send a NavigateToPose goal to a robot.

Usage:
    python3 send_nav_goal.py --robot robot0 --x 5.0 --y 3.0
"""

import argparse

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


def main():
    parser = argparse.ArgumentParser(description="Send NavigateToPose goal")
    parser.add_argument("--robot", default="robot0")
    parser.add_argument("--x", type=float, default=5.0)
    parser.add_argument("--y", type=float, default=3.0)
    parser.add_argument("--z", type=float, default=0.05)
    parser.add_argument("--yaw", type=float, default=0.0)
    args = parser.parse_args()

    rclpy.init()
    node = Node("send_nav_goal_client")
    client = ActionClient(node, NavigateToPose, f"/{args.robot}/navigate_to_pose")

    node.get_logger().info(f"Waiting for /{args.robot}/navigate_to_pose action server...")
    client.wait_for_server()

    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = "odom"
    goal.pose.pose.position.x = args.x
    goal.pose.pose.position.y = args.y
    goal.pose.pose.position.z = args.z
    goal.pose.pose.orientation.w = 1.0

    node.get_logger().info(f"Sending NavigateToPose goal: ({args.x}, {args.y}, {args.z})")

    future = client.send_goal_async(goal, feedback_callback=lambda fb: node.get_logger().info(
        f"  distance_remaining: {fb.feedback.distance_remaining:.2f}"
    ))
    rclpy.spin_until_future_complete(node, future)

    goal_handle = future.result()
    if not goal_handle.accepted:
        node.get_logger().error("Goal rejected!")
        return

    node.get_logger().info("Goal accepted, waiting for result...")
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)

    node.get_logger().info("NavigateToPose completed!")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

**Step 4: Commit**

```bash
git add ros2_bridge/scripts/
git commit -m "feat(ros2): add send_path/send_joint_trajectory/send_nav_goal client scripts"
```

---

## Task 9: Docker Build + Integration Smoke Test (SERIAL, final)

**Step 1: Build Docker image**

```bash
cd docker && docker compose build bridge
```

**Step 2: Integration smoke test**

```bash
# Terminal 1: Start bridge
docker compose run --rm bridge ros2 run pybullet_fleet_ros bridge_node --ros-args -p num_robots:=1 -p gui:=false

# Terminal 2: Check topics and actions exist
docker compose exec bridge bash -c "source /opt/bridge_ws/install/setup.bash && ros2 topic list"
docker compose exec bridge bash -c "source /opt/bridge_ws/install/setup.bash && ros2 action list"
```

Expected topics include: `/robot0/goal_pose`, `/robot0/path`, `/robot0/plan`, `/robot0/current_goal`, `/robot0/diagnostics`

Expected actions include: `/robot0/navigate_to_pose`, `/robot0/follow_path`, `/robot0/follow_joint_trajectory`

**Step 3: Test goal_pose topic**

```bash
docker compose exec bridge bash -c 'source /opt/bridge_ws/install/setup.bash && ros2 topic pub --once /robot0/goal_pose geometry_msgs/PoseStamped "{header: {frame_id: odom}, pose: {position: {x: 3.0, y: 2.0, z: 0.05}, orientation: {w: 1.0}}}"'
```

Verify robot moves by checking diagnostics:
```bash
docker compose exec bridge bash -c 'source /opt/bridge_ws/install/setup.bash && ros2 topic echo /robot0/diagnostics --once'
```

**Step 4: Final commit**

```bash
git add -A
git commit -m "feat(ros2): Nav2 + arm interfaces, action servers, diagnostics, RViz demos"
```

---

## Checklist Before Claiming Complete

- [ ] `docker compose build bridge` succeeds
- [ ] `ros2 topic list` shows all new topics
- [ ] `ros2 action list` shows NavigateToPose, FollowPath, FollowJointTrajectory
- [ ] goal_pose topic → robot moves
- [ ] path topic → robot follows path
- [ ] joint_trajectory topic → arm moves
- [ ] diagnostics topic shows is_moving status
- [ ] NavigateToPose action completes with succeeded
- [ ] send_path.py client works
- [ ] send_nav_goal.py client works
- [ ] nav_demo.launch.py starts bridge + RViz
