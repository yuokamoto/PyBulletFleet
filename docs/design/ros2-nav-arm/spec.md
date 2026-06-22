# ROS 2 Nav2 + Arm Interface Specification

**Date:** 2026-03-24
**Status:** Draft
**Depends on:** ROS 2 Bridge Phase 1 (committed: `aaed8dd`)

## Context

The ROS 2 bridge has core functionality (cmd_vel, odom, joint_states, TF, simulation_interfaces services, /clock). Users now need Nav2-compatible navigation and arm control interfaces to integrate with the ROS 2 ecosystem (Nav2 stack, MoveIt, RViz). Both **simple topic-based** and **standard action server** interfaces are needed — topics for quick prototyping, actions for production Nav2/MoveIt compatibility.

## Decision

Add a 3-layer interface to RobotHandler:

1. **Topic layer (simplest):** Publish a message → robot moves immediately. No action server needed.
2. **Action layer (standard):** Nav2/MoveIt-compatible action servers with feedback and result.
3. **Status & visualization layer:** Diagnostic topics + RViz-friendly publishers for monitoring.

Plus RViz demo launch files so users can visualize robots out of the box.

## Requirements

### Topic Subscribers (publish → robot moves)

| Topic | Message Type | Behavior |
|-------|-------------|----------|
| `/{name}/goal_pose` | `geometry_msgs/PoseStamped` | `agent.set_goal_pose()` |
| `/{name}/path` | `nav_msgs/Path` | `agent.set_path()` |
| `/{name}/joint_trajectory` | `trajectory_msgs/JointTrajectory` | `agent.set_joints_targets_by_name()` — apply last waypoint's positions |
| `/{name}/joint_commands` | `std_msgs/Float64MultiArray` | `agent.set_all_joints_targets()` — raw joint positions |

### Action Servers (Nav2/MoveIt compatible)

| Action | Message Type | Behavior |
|--------|-------------|----------|
| `/{name}/navigate_to_pose` | `nav2_msgs/NavigateToPose` | `set_goal_pose()` + poll `is_moving` for feedback/result |
| `/{name}/follow_path` | `nav2_msgs/FollowPath` | `set_path()` + poll waypoint progress |
| `/{name}/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` | Apply trajectory points sequentially + poll `are_joints_at_targets()` |

### Status & Visualization Publishers

| Topic | Message Type | Content |
|-------|-------------|---------|
| `/{name}/plan` | `nav_msgs/Path` | Current path being followed |
| `/{name}/current_goal` | `geometry_msgs/PoseStamped` | Current goal pose |
| `/{name}/diagnostics` | `diagnostic_msgs/DiagnosticArray` | is_moving, action_type, action_status, distance_to_goal, linear_speed, action_queue, action_queue_size |

### Demo & RViz

| File | Content |
|------|---------|
| `launch/nav_demo.launch.py` | Bridge + RViz for mobile robot navigation |
| `launch/arm_demo.launch.py` | Bridge + RViz for arm control |
| `config/nav_demo.rviz` | TF, Path, Pose, Odometry displays |
| `config/arm_demo.rviz` | TF, RobotModel, JointState displays |
| `scripts/send_path.py` | Publish nav_msgs/Path (square path demo) |
| `scripts/send_joint_trajectory.py` | Publish JointTrajectory |

## Constraints

- No custom messages — standard ROS 2 only
- No MoveGroup action (MoveIt integration is future scope)
- All action servers must be non-blocking (rclpy executor-compatible)
- No changes to PyBulletFleet core (`pybullet_fleet/` directory)
- multi_robot_fleet launch deferred to Open-RMF integration

## Out of Scope

- MoveIt `MoveGroup` action server
- Open-RMF fleet adapter
- Costmap / LaserScan simulation
- Custom message types
- Humble Dockerfile

## Open Questions

- [x] How to report status without custom messages? → `diagnostic_msgs/DiagnosticArray`
- [ ] `FollowJointTrajectory` timing: respect `time_from_start` or apply waypoints at sim rate?

## Success Criteria

- [ ] `ros2 topic pub /{name}/goal_pose geometry_msgs/PoseStamped ...` → robot navigates
- [ ] `ros2 topic pub /{name}/path nav_msgs/Path ...` → robot follows path
- [ ] `ros2 topic pub /{name}/joint_trajectory trajectory_msgs/JointTrajectory ...` → arm moves
- [ ] `ros2 topic echo /{name}/plan` shows current path
- [ ] `ros2 topic echo /{name}/diagnostics` shows is_moving, action_status, action_queue
- [ ] NavigateToPose action completes with SUCCEEDED
- [ ] FollowJointTrajectory action completes with SUCCEEDED
- [ ] RViz displays robot TF + path + goal marker via nav_demo.launch.py
- [ ] RViz displays robot model + joint states via arm_demo.launch.py
