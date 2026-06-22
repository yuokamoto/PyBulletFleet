# ROS 2 Nav2 + Arm Interface — Agent Specification

## Requirements

### Functional

- Topic subscribers: `goal_pose` (PoseStamped), `path` (Path), `joint_trajectory` (JointTrajectory), `joint_commands` (Float64MultiArray)
- Action servers: `navigate_to_pose` (NavigateToPose), `follow_path` (FollowPath), `follow_joint_trajectory` (FollowJointTrajectory)
- Status publishers: `plan` (Path), `current_goal` (PoseStamped), `diagnostics` (DiagnosticArray)
- Demo launch files: nav_demo + arm_demo with RViz configs
- Client scripts: send_path.py, send_joint_trajectory.py

### Non-Functional

- Action server callbacks must be non-blocking (use `rclpy.callback_groups.ReentrantCallbackGroup`)
- No changes to `pybullet_fleet/` core
- All new publishers use QoS depth 10
- Action feedback published at sim step rate (via publish callback)

## Constraints

- Must work inside Docker (Jazzy)
- nav2_msgs, control_msgs, trajectory_msgs, diagnostic_msgs already available in Docker image
- Action servers share the single BridgeNode (no separate executors)
- `diagnostic_msgs` must be added to package.xml and Dockerfile

## Approach

Extend `RobotHandler` with nav/arm capabilities. Keep the existing class structure — add methods and publishers/subscribers in `__init__`. Action servers use rclpy's `ActionServer` with `ReentrantCallbackGroup` for concurrent goals.

For action completion monitoring, use a polling pattern in the action callback: check `agent.is_moving` / `agent.are_joints_at_targets()` in a loop with `asyncio.sleep()` between checks (rclpy action servers support `async` execute callbacks).

## Design

### Architecture

```
RobotHandler (extended)
  ├── Existing: cmd_vel (sub), odom (pub), joint_states (pub), TF
  │
  ├── NEW Topic subs:
  │   ├── /{name}/goal_pose    (PoseStamped)  → agent.set_goal_pose()
  │   ├── /{name}/path         (Path)         → agent.set_path()
  │   ├── /{name}/joint_trajectory (JointTrajectory) → agent.set_joints_targets_by_name()
  │   └── /{name}/joint_commands   (Float64MultiArray) → agent.set_all_joints_targets()
  │
  ├── NEW Action servers:
  │   ├── /{name}/navigate_to_pose       (NavigateToPose)
  │   ├── /{name}/follow_path            (FollowPath)
  │   └── /{name}/follow_joint_trajectory (FollowJointTrajectory)
  │
  └── NEW Status pubs:
      ├── /{name}/plan          (Path)
      ├── /{name}/current_goal  (PoseStamped)
      └── /{name}/diagnostics   (DiagnosticArray)
```

### Key Components

| Component | Responsibility | Location |
|-----------|---------------|----------|
| Topic subscribers | goal_pose/path/joint_trajectory/joint_commands → Agent API | `ros2_bridge/.../robot_handler.py` |
| Action servers | NavigateToPose/FollowPath/FollowJointTrajectory with feedback | `ros2_bridge/.../robot_handler.py` |
| Status publishers | plan/current_goal/diagnostics | `ros2_bridge/.../robot_handler.py` |
| Conversion helpers | Path↔nav_msgs, PoseStamped, DiagnosticArray builders | `ros2_bridge/.../conversions.py` |
| Package deps | diagnostic_msgs, trajectory_msgs | `ros2_bridge/.../package.xml` |
| Docker deps | ros-jazzy-diagnostic-msgs, ros-jazzy-trajectory-msgs | `docker/Dockerfile.jazzy` |
| Nav demo launch | Bridge + RViz with nav config | `ros2_bridge/.../launch/nav_demo.launch.py` |
| Arm demo launch | Bridge + RViz with arm config | `ros2_bridge/.../launch/arm_demo.launch.py` |
| RViz configs | Display configs for nav and arm demos | `ros2_bridge/.../config/nav_demo.rviz`, `arm_demo.rviz` |
| Client scripts | send_path.py, send_joint_trajectory.py | `ros2_bridge/scripts/` |

### Data Flow

**Topic (goal_pose):**
```
User publishes PoseStamped → _goal_pose_cb() → ros_pose_to_pbf() → agent.set_goal_pose()
                                                                  → publish current_goal
```

**Topic (path):**
```
User publishes nav_msgs/Path → _path_cb() → [ros_pose_to_pbf(wp) for wp in path.poses]
                                           → agent.set_path(waypoints)
                                           → publish plan
```

**Topic (joint_trajectory):**
```
User publishes JointTrajectory → _joint_traj_cb()
  → extract last point's positions + joint_names
  → agent.set_joints_targets_by_name({name: pos})
```

**Topic (joint_commands):**
```
User publishes Float64MultiArray → _joint_cmd_cb()
  → agent.set_all_joints_targets(data)
```

**Action (NavigateToPose):**
```
Client sends goal → _navigate_execute(goal_handle)
  → agent.set_goal_pose(goal.pose)
  → loop:
      sleep(dt) → publish feedback(current_pose, distance_remaining)
      if not agent.is_moving → break
  → goal_handle.succeed() with result
```

**Action (FollowJointTrajectory):**
```
Client sends goal → _follow_jt_execute(goal_handle)
  → for each trajectory point:
      agent.set_joints_targets_by_name({name: pos})
      wait until are_joints_at_targets() or timeout
      publish feedback(actual, desired, error)
  → goal_handle.succeed()
```

**Diagnostics (published every publish cycle):**
```
publish_state() → _publish_diagnostics()
  → DiagnosticArray with:
      is_moving, action_type, action_status, distance_to_goal, linear_speed,
      action_queue, action_queue_size
```

### Code Patterns

**Existing pattern — RobotHandler subscriber (from robot_handler.py):**
```python
# Subscriber
self._cmd_vel_sub = node.create_subscription(Twist, f"/{ns}/cmd_vel", self._cmd_vel_cb, 10)

def _cmd_vel_cb(self, msg: Twist) -> None:
    self._latest_twist = msg
```

**Existing pattern — conversion function (from conversions.py):**
```python
def ros_pose_to_pbf(ros_pose: PoseMsg) -> Pose:
    return Pose(
        position=[ros_pose.position.x, ros_pose.position.y, ros_pose.position.z],
        orientation=[ros_pose.orientation.x, ros_pose.orientation.y,
                     ros_pose.orientation.z, ros_pose.orientation.w],
    )
```

**rclpy Action Server pattern:**
```python
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup

self._nav_action = ActionServer(
    node,
    NavigateToPose,
    f"/{ns}/navigate_to_pose",
    execute_callback=self._navigate_execute,
    callback_group=ReentrantCallbackGroup(),
)

async def _navigate_execute(self, goal_handle):
    feedback = NavigateToPose.Feedback()
    while agent.is_moving:
        # publish feedback
        goal_handle.publish_feedback(feedback)
        await asyncio.sleep(0.1)
    goal_handle.succeed()
    return NavigateToPose.Result()
```

**Agent API mapping:**
```python
# Navigation
agent.set_goal_pose(Pose)           # Single goal
agent.set_path([Pose, ...])         # Path following
agent.is_moving                     # bool — movement in progress
agent.goal_pose                     # Optional[Pose] — current goal
agent.velocity                      # [vx, vy, vz] — current velocity

# Joint control
agent.set_joints_targets_by_name({"joint1": 1.57, "joint2": 0.5})
agent.set_all_joints_targets([0.0, 1.57, -1.57, 0.0])
agent.are_joints_at_targets()       # bool — joints settled
agent.get_all_joints_state_by_name()  # {name: (pos, vel)}

# Action system
agent.get_current_action()          # Optional[Action]
agent.is_action_queue_empty()       # bool
```

## File References

Files the plan agent MUST read before planning:

- `ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/robot_handler.py` — current handler (extend this)
- `ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/conversions.py` — existing conversions (add new ones here)
- `ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/bridge_node.py` — main node (minor changes)
- `ros2_bridge/pybullet_fleet_ros/package.xml` — add dependencies
- `docker/Dockerfile.jazzy` — add apt packages
- `pybullet_fleet/agent.py:847-1010` — set_goal_pose, set_path API
- `pybullet_fleet/agent.py:1940-2030` — set_joint_target, set_joints_targets_by_name API
- `pybullet_fleet/agent.py:463-520` — is_moving, velocity, angular_velocity, goal_pose properties
- `pybullet_fleet/agent.py:1605-1640` — get_current_action, is_action_queue_empty
- `pybullet_fleet/agent.py:2080-2160` — are_joints_at_targets, are_joints_at_targets_by_name
- `ros2_bridge/pybullet_fleet_ros/launch/bridge.launch.py` — existing launch (pattern for new ones)
- `ros2_bridge/scripts/teleop_cmd_vel.py` — existing client script (pattern)

## Success Criteria

- [ ] `ros2 topic pub /{name}/goal_pose geometry_msgs/PoseStamped` → robot moves to goal
- [ ] `ros2 topic pub /{name}/path nav_msgs/Path` → robot follows path
- [ ] `ros2 topic pub /{name}/joint_trajectory trajectory_msgs/JointTrajectory` → arm moves
- [ ] `ros2 topic pub /{name}/joint_commands std_msgs/Float64MultiArray` → joints move
- [ ] `ros2 topic echo /{name}/plan` shows nav_msgs/Path when path is active
- [ ] `ros2 topic echo /{name}/current_goal` shows goal during navigation
- [ ] `ros2 topic echo /{name}/diagnostics` shows is_moving, action_status, action_queue
- [ ] `ros2 action send_goal /{name}/navigate_to_pose nav2_msgs/NavigateToPose` → feedback + SUCCEEDED
- [ ] `ros2 action send_goal /{name}/follow_path nav2_msgs/FollowPath` → SUCCEEDED
- [ ] `ros2 action send_goal /{name}/follow_joint_trajectory control_msgs/FollowJointTrajectory` → SUCCEEDED
- [ ] `ros2 launch pybullet_fleet_ros nav_demo.launch.py` → RViz shows robot + path + goal
- [ ] `ros2 launch pybullet_fleet_ros arm_demo.launch.py` → RViz shows arm + joints
- [ ] Docker build succeeds with new dependencies
