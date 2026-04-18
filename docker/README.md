# Docker — ROS 2 Bridge

Docker environment for the [`ros2_bridge/`](../ros2_bridge/) package (ROS 2 Jazzy).
The Dockerfile builds both `pybullet_fleet` and `pybullet_fleet_ros` inside
a single colcon workspace.

## Build

```bash
cd docker
docker compose build bridge
```

## Quick Start

All demos use **two terminals**: Terminal 1 runs the bridge in foreground,
Terminal 2 sends commands via `docker exec`.

### Navigation Demo — TurtleBot3 (Differential Drive + RViz)

Uses `ros-jazzy-turtlebot3-description` (installed in the Docker image).

**Terminal 1** — launch the bridge with PyBullet GUI + RViz:

```bash
cd docker
xhost +local:docker   # first time only

GUI=true docker compose run --rm --name pbf_bridge bridge \
  ros2 launch pybullet_fleet_ros tb3_demo.launch.py gui:=true
```

**Terminal 2** — send a navigation goal:

```bash
docker exec pbf_bridge bash -c 'source /rmf_demos_ws/install/setup.bash && \
  ros2 topic pub --once /tb3_0/goal_pose geometry_msgs/PoseStamped \
    "{header: {frame_id: odom}, pose: {position: {x: 2.0, y: 1.0, z: 0.01}, orientation: {w: 1.0}}}"'
```

The TurtleBot3 rotates toward the goal and drives forward. Both PyBullet GUI and
RViz (Odometry arrow, Path, Current Goal, RobotModel) update in real time.

You can also use the Waffle model: `model:=waffle`.

Stop with **Ctrl+C** in Terminal 1.

### Arm Demo — UR5e (Fixed-Base 6-DOF Arm + RViz)

Uses `ros-jazzy-ur-description` (installed in the Docker image).

**Terminal 1**:

```bash
cd docker
xhost +local:docker

GUI=true docker compose run --rm --name pbf_bridge bridge \
  ros2 launch pybullet_fleet_ros ur5e_demo.launch.py gui:=true
```

**Terminal 2** — send a joint trajectory via action (with feedback):

```bash
docker exec pbf_bridge bash -c 'source /rmf_demos_ws/install/setup.bash && \
  python3 /opt/pybullet_fleet/scripts/send_joint_goal_ur5e.py --robot ur5e_0 \
    --positions 0.0 -1.57 1.57 -1.57 -1.57 0.0'
```

RViz shows the UR5e arm model updating as all 6 joints move to target positions.

### Headless (No GUI, No RViz)

```bash
cd docker
docker compose up bridge                                        # 3 robots, headless
NUM_ROBOTS=5 PUBLISH_RATE=50.0 docker compose up bridge         # custom
```

### Launch Arguments

The `tb3_demo.launch.py` and `ur5e_demo.launch.py` accept:

| Argument | Default | Description |
|----------|---------|-------------|
| `gui` | `false` | Enable PyBullet GUI |
| `rviz` | `true` | Launch RViz + robot_state_publisher |
| `target_rtf` | `1.0` | Real-time factor |
| `publish_rate` | `50.0` | odom / joint_states publish rate (Hz) |

```bash
# GUI only, no RViz
GUI=true docker compose run --rm --name pbf_bridge bridge \
  ros2 launch pybullet_fleet_ros tb3_demo.launch.py gui:=true rviz:=false

# Waffle variant
GUI=true docker compose run --rm --name pbf_bridge bridge \
  ros2 launch pybullet_fleet_ros tb3_demo.launch.py model:=waffle gui:=true
```

### Environment Variables (docker compose)

| Variable | Default | Description |
|----------|---------|-------------|
| `NUM_ROBOTS` | 3 | Number of robots (headless mode) |
| `GUI` | false | Enable PyBullet GUI window |
| `PUBLISH_RATE` | 10.0 | odom / joint_states publish rate (Hz) |
| `PHYSICS` | false | Enable physics simulation |
| `TARGET_RTF` | 1.0 | Real-time factor |
| `ROS_DOMAIN_ID` | 42 | ROS 2 domain ID |

> **Note:** Source code is volume-mounted, so local Python edits take effect on container restart.

## ROS Interface Reference

### Topics (per robot, e.g. `/robot0/...`)

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `cmd_vel` | Twist | Sub | Velocity command (body frame) |
| `goal_pose` | PoseStamped | Sub | Goal pose → robot navigates |
| `path` | Path | Sub | Waypoint path → robot follows |
| `joint_trajectory` | JointTrajectory | Sub | Joint targets (topic) |
| `joint_commands` | Float64MultiArray | Sub | Raw joint positions |
| `odom` | Odometry | Pub | Odometry |
| `joint_states` | JointState | Pub | Joint positions/velocities |
| `plan` | Path | Pub | Remaining path being followed |
| `current_goal` | PoseStamped | Pub | Current goal (arrow points toward target) |
| `diagnostics` | DiagnosticArray | Pub | Robot status (is_moving, action_queue, etc.) |

### Action Servers (per robot)

| Action | Type | Description |
|--------|------|-------------|
| `navigate_to_pose` | NavigateToPose | Nav2-compatible navigation |
| `follow_path` | FollowPath | Nav2-compatible path following |
| `follow_joint_trajectory` | FollowJointTrajectory | MoveIt-compatible joint control |

### Simulation Services

| Service | Description |
|---------|-------------|
| `/sim/get_entities` | List all entities |
| `/sim/get_entity_state` | Get entity pose |
| `/sim/spawn_entity` | Spawn a robot |
| `/sim/delete_entity` | Delete an entity |
| `/sim/step_simulation` | Step N simulation ticks |
| `/sim/set_simulation_state` | Pause / resume |
| `/sim/get_simulator_features` | Query supported features |

## Operations Guide

While the bridge is running in Terminal 1, use Terminal 2 for interaction.

> Helper scripts in `ros2_bridge/scripts/` are volume-mounted at `/opt/pybullet_fleet/scripts/`.

### Introspection

```bash
# List topics / services / actions
docker exec pbf_bridge bash -c "source /rmf_demos_ws/install/setup.bash && ros2 topic list"
docker exec pbf_bridge bash -c "source /rmf_demos_ws/install/setup.bash && ros2 action list"

# Check odometry
docker exec pbf_bridge bash -c 'source /rmf_demos_ws/install/setup.bash && \
  ros2 topic echo /tb3_0/odom --once'

# Check diagnostics
docker exec pbf_bridge bash -c 'source /rmf_demos_ws/install/setup.bash && \
  ros2 topic echo /tb3_0/diagnostics --once'

# List entities
docker exec pbf_bridge bash -c 'source /rmf_demos_ws/install/setup.bash && \
  python3 /opt/pybullet_fleet/scripts/query_entities.py'
```

### Navigation (goal_pose / path / action)

```bash
# Send goal pose (topic)
docker exec pbf_bridge bash -c 'source /rmf_demos_ws/install/setup.bash && \
  ros2 topic pub --once /tb3_0/goal_pose geometry_msgs/PoseStamped \
    "{header: {frame_id: odom}, pose: {position: {x: 2.0, y: 1.0, z: 0.01}, orientation: {w: 1.0}}}"'

# Send square path (script)
docker exec pbf_bridge bash -c 'source /rmf_demos_ws/install/setup.bash && \
  python3 /opt/pybullet_fleet/scripts/send_path.py --robot tb3_0 --size 2.0'

# NavigateToPose action (with distance_remaining feedback)
docker exec pbf_bridge bash -c 'source /rmf_demos_ws/install/setup.bash && \
  python3 /opt/pybullet_fleet/scripts/send_nav_goal.py --robot tb3_0 --x 2.0 --y 1.0'

# Check remaining path
docker exec pbf_bridge bash -c 'source /rmf_demos_ws/install/setup.bash && \
  ros2 topic echo /tb3_0/plan --once'
```

> **RViz tip:** Use the "2D Goal Pose" toolbar button to click-publish goals directly.

### Arm Control (joint_trajectory / action)

```bash
# FollowJointTrajectory action (with actual/desired/error feedback — recommended)
docker exec pbf_bridge bash -c 'source /rmf_demos_ws/install/setup.bash && \
  python3 /opt/pybullet_fleet/scripts/send_joint_goal_ur5e.py --robot ur5e_0 \
    --positions 0.0 -1.57 1.57 -1.57 -1.57 0.0'

# Joint trajectory topic (fire-and-forget)
docker exec pbf_bridge bash -c 'source /rmf_demos_ws/install/setup.bash && \
  python3 /opt/pybullet_fleet/scripts/send_joint_trajectory.py --robot ur5e_0 \
    --positions 0.0 -1.57 1.57 -1.57 -1.57 0.0'

# Check joint states
docker exec pbf_bridge bash -c 'source /rmf_demos_ws/install/setup.bash && \
  ros2 topic echo /ur5e_0/joint_states --once'
```

### Velocity Control

```bash
# Script — move for 5 seconds
docker exec pbf_bridge bash -c 'source /rmf_demos_ws/install/setup.bash && \
  python3 /opt/pybullet_fleet/scripts/teleop_cmd_vel.py --robot tb3_0 --vx 0.2 --wz 0.5 --duration 5.0'

# Raw topic pub (single)
docker exec pbf_bridge bash -c 'source /rmf_demos_ws/install/setup.bash && \
  ros2 topic pub --once /tb3_0/cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.22}, angular: {z: 0.5}}"'

# Keyboard teleop
docker exec -it pbf_bridge bash -c 'source /rmf_demos_ws/install/setup.bash && \
  ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r /cmd_vel:=/tb3_0/cmd_vel'
```

### Spawn / Delete Entities

```bash
# Spawn a UR5e arm at position (5, 3)
docker exec pbf_bridge bash -c 'source /rmf_demos_ws/install/setup.bash && \
  python3 /opt/pybullet_fleet/scripts/spawn_robots.py --name ur5e_1 --urdf ur5e --x 5.0 --y 3.0'

# Delete
docker exec pbf_bridge bash -c 'source /rmf_demos_ws/install/setup.bash && \
  ros2 service call /sim/delete_entity simulation_interfaces/srv/DeleteEntity "{entity: ur5e_1}"'
```

### Simulation Control

```bash
# Step 100 ticks
docker exec pbf_bridge bash -c 'source /rmf_demos_ws/install/setup.bash && \
  python3 /opt/pybullet_fleet/scripts/step_simulation.py --steps 100'

# Pause / Resume / Status
docker exec pbf_bridge bash -c 'source /rmf_demos_ws/install/setup.bash && \
  python3 /opt/pybullet_fleet/scripts/step_simulation.py --pause'
docker exec pbf_bridge bash -c 'source /rmf_demos_ws/install/setup.bash && \
  python3 /opt/pybullet_fleet/scripts/step_simulation.py --resume'
```

## Config-Driven Launch

Use `config_yaml` to spawn heterogeneous robots:

```bash
# Omni demo: 2 mobile robots + 1 floating cube (6-DoF)
GUI=true docker compose run --rm --name pbf_bridge bridge \
  ros2 run pybullet_fleet_ros bridge_node \
    --ros-args \
    -p config_yaml:=/rmf_demos_ws/src/pybullet_fleet_ros/config/bridge_omni_demo.yaml \
    -p gui:=true
```

The cube responds to full 6-DoF `cmd_vel` (including `angular.x` / `angular.y` for roll/pitch):

```bash
docker exec pbf_bridge bash -c 'source /rmf_demos_ws/install/setup.bash && \
  ros2 topic pub -r 10 /cube0/cmd_vel geometry_msgs/msg/Twist \
    "{linear: {z: 0.5}, angular: {x: 0.3, y: 0.2, z: 0.1}}"'
```

### Provided Config Files

| File | Description |
|------|-------------|
| `config/bridge_tb3.yaml` | TurtleBot3 Burger navigation (used by `tb3_demo.launch.py`) |
| `config/bridge_ur5e.yaml` | UR5e arm joint control (used by `ur5e_demo.launch.py`) |
| `config/bridge_nav.yaml` | Generic differential drive navigation (used by `nav_demo.launch.py`) |
| `config/bridge_arm.yaml` | Generic arm robot joint control (used by `arm_demo.launch.py`) |
| `config/bridge_omni_demo.yaml` | Omni + 6-DoF cube demo |

When `config_yaml` is not set, the bridge falls back to `num_robots` + `robot_urdf` parameters.

## All ROS Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `num_robots` | int | 1 | Number of robots to spawn |
| `robot_urdf` | str | `robots/mobile_robot.urdf` | Default URDF path |
| `gui` | bool | false | Enable PyBullet GUI |
| `physics` | bool | false | Enable physics simulation |
| `publish_rate` | float | 50.0 | odom / joint_states publish rate (Hz) |
| `target_rtf` | float | 1.0 | Real-time factor |
| `enable_sim_services` | bool | true | Enable simulation_interfaces services |
| `config_yaml` | str | `""` | YAML config (overrides `num_robots` / `robot_urdf`) |

## Automated Tests

```bash
# Integration smoke test (headless)
docker compose run --rm test

# colcon launch_testing
docker compose run --rm bridge bash -c "\
  source /ros_entrypoint.sh && \
  cd /rmf_demos_ws && \
  colcon test --packages-select pybullet_fleet_ros --event-handlers console_direct+"
```

## Smoke Test (Manual)

**Terminal 1**:

```bash
cd docker
xhost +local:docker
docker compose build bridge

GUI=true docker compose run --rm --name pbf_bridge bridge \
  ros2 launch pybullet_fleet_ros tb3_demo.launch.py gui:=true
```

**Terminal 2**:

```bash
# Verify topics and actions
docker exec pbf_bridge bash -c "source /rmf_demos_ws/install/setup.bash && ros2 topic list"
docker exec pbf_bridge bash -c "source /rmf_demos_ws/install/setup.bash && ros2 action list"

# Send goal and check diagnostics
docker exec pbf_bridge bash -c 'source /rmf_demos_ws/install/setup.bash && \
  ros2 topic pub --once /tb3_0/goal_pose geometry_msgs/PoseStamped \
    "{header: {frame_id: odom}, pose: {position: {x: 2.0, y: 1.0, z: 0.01}, orientation: {w: 1.0}}}"'
sleep 2
docker exec pbf_bridge bash -c 'source /rmf_demos_ws/install/setup.bash && \
  ros2 topic echo /tb3_0/diagnostics --once'
# Expected: is_moving=true

# Stop with Ctrl+C in Terminal 1
```

## Troubleshooting

### `Error in gladLoadGLX`

If PyBullet GUI fails to open inside Docker:

1. Run `xhost +local:docker` on the host
2. Ensure `/dev/dri` is mounted (included in compose by default)
3. Ensure Mesa/libglvnd packages are installed in the Dockerfile

### Service Calls Timeout

DDS discovery issues can occur when calling from a separate container.
Use `docker exec pbf_bridge` to run commands inside the same container.

### URDF Meshes in RViz

The default URDFs use primitive shapes (box, cylinder). To use custom meshes
(`.stl` / `.dae`), reference them via `<mesh filename="..."/>` in the URDF and
place mesh files in the `robots/` directory. `robot_state_publisher` will
publish them to RViz automatically.

## Open-RMF Integration Demo

Demonstrates PyBulletFleet integrated with Open-RMF task dispatching
using the **EasyFullControl** API (the recommended Python interface).

### Prerequisites

The Docker image builds `rmf_fleet_adapter_python` from source (the Jazzy
binary is C++ only). This provides the `rmf_adapter` Python module with
`easy_full_control` API. Build dependencies (`ros-jazzy-rmf-fleet-adapter`,
`pybind11-dev`, `pybind11_json_vendor`) are installed automatically.

### Launch

```bash
# Start bridge + fleet adapter (office demo, 2 robots)
docker compose run --rm bridge \
  ros2 launch pybullet_fleet_ros office_pybullet.launch.py

# In another terminal: dispatch a patrol task
docker compose run --rm bridge \
  ros2 run rmf_demos_tasks dispatch_patrol -p pantry lounge -n 3
```

### Architecture

```
Open-RMF Traffic Schedule
    │
    ▼
FleetAdapterNode (fleet_adapter.py)
    │  EasyFullControl API → RobotCallbacks.navigate()
    ▼
RobotClientAPI → NavigateToPose action goal
    │
    ▼
BridgeNode → RobotHandler → Agent.set_goal_pose()
    │
    ▼
PyBulletFleet (kinematic navigation)
```

Fleet adapter subscribes to each robot's `/odom` topic to report position
to Open-RMF continuously via `RobotState` updates.
