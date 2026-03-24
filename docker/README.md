# Docker — ROS 2 Bridge

Docker environment for the [`ros2_bridge/`](../ros2_bridge/) package (ROS 2 Jazzy).
The Dockerfile builds both `pybullet_fleet` and `pybullet_fleet_ros` inside
a single colcon workspace.

## Build

```bash
cd docker

# Build all services
docker compose build

# Build bridge only
docker compose build bridge
```

## Quick Start

```bash
cd docker

# Headless (3 robots, no GUI)
docker compose up bridge

# With PyBullet GUI
xhost +local:docker   # first time only
GUI=true docker compose up bridge

# Custom parameters
NUM_ROBOTS=5 GUI=true PUBLISH_RATE=50.0 docker compose up bridge
```

### Config-Driven Launch (Mixed Robot Types)

Use `config_yaml` to spawn heterogeneous robots (e.g. mobile robots + 6-DoF cubes):

```bash
cd docker

# Omni demo: 2 mobile robots + 1 floating cube (6-DoF)
docker compose run --rm bridge ros2 run pybullet_fleet_ros bridge_node \
  --ros-args \
  -p config_yaml:=/opt/bridge_ws/src/pybullet_fleet_ros/config/bridge_omni_demo.yaml \
  -p gui:=true
```

The cube responds to full 6-DoF `cmd_vel` (including `angular.x` / `angular.y` for roll/pitch):

```bash
# Move cube up + rotate in all axes
docker compose exec bridge bash -c "source /ros_entrypoint.sh && \
  ros2 topic pub -r 10 /cube0/cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0, y: 0, z: 0.5}, angular: {x: 0.3, y: 0.2, z: 0.1}}'"
```

See [`ros2_bridge/pybullet_fleet_ros/config/bridge_omni_demo.yaml`](../ros2_bridge/pybullet_fleet_ros/config/bridge_omni_demo.yaml) for the config format.
When `config_yaml` is not set, the bridge falls back to `num_robots` + `robot_urdf`.

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `NUM_ROBOTS` | 3 | Number of robots to spawn |
| `GUI` | false | Enable PyBullet GUI window |
| `PUBLISH_RATE` | 10.0 | odom / joint_states publish rate (Hz) |
| `PHYSICS` | false | Enable physics simulation |
| `TARGET_RTF` | 1.0 | Real-time factor |
| `ROS_DOMAIN_ID` | 42 | ROS 2 domain ID |

> **Note:** Source code is volume-mounted, so local Python edits take effect on container restart.

## Automated Test (Headless)

```bash
cd docker
docker compose run --rm test
```

Runs topic checks, service existence checks, `/clock` / odom publishing verification,
and GetEntities / GetSimulatorFeatures service calls automatically.

### colcon test (launch_testing)

For proper ROS 2 integration tests using `launch_testing`:

```bash
docker compose run --rm bridge bash -c "\
  source /ros_entrypoint.sh && \
  cd /opt/bridge_ws && \
  colcon test --packages-select pybullet_fleet_ros --event-handlers console_direct+"
```

This launches the bridge node as a managed process and verifies topics, services,
and message publishing via the rclpy API.

## Operations Guide

While `docker compose up bridge` is running, use another terminal to interact.
Each operation shows a **script** (simpler) and a **raw command** (no extra deps).

> Helper scripts in `ros2_bridge/scripts/` are volume-mounted at `/opt/pybullet_fleet/scripts/`.

Shorthand used below:

```bash
# All commands are prefixed with:
docker compose exec bridge bash -c "source /ros_entrypoint.sh && ..."
```

### Introspection

#### List all topics / services

```bash
docker compose exec bridge bash -c "source /ros_entrypoint.sh && ros2 topic list"
docker compose exec bridge bash -c "source /ros_entrypoint.sh && ros2 service list"
```

#### List entities

```bash
# Script
docker compose exec bridge bash -c "source /ros_entrypoint.sh && \
  python3 scripts/query_entities.py"

# Raw
docker compose exec bridge bash -c "source /ros_entrypoint.sh && \
  ros2 service call /sim/get_entities simulation_interfaces/srv/GetEntities '{}'"
```

#### Get entity state (pose)

```bash
# Script
docker compose exec bridge bash -c "source /ros_entrypoint.sh && \
  python3 scripts/query_entities.py --name robot0"

# Raw
docker compose exec bridge bash -c "source /ros_entrypoint.sh && \
  ros2 service call /sim/get_entity_state simulation_interfaces/srv/GetEntityState '{entity: robot0}'"
```

#### Get simulator features

```bash
docker compose exec bridge bash -c "source /ros_entrypoint.sh && \
  ros2 service call /sim/get_simulator_features simulation_interfaces/srv/GetSimulatorFeatures '{}'"
```

#### Check odometry

```bash
docker compose exec bridge bash -c "source /ros_entrypoint.sh && \
  timeout 2 ros2 topic echo /robot0/odom --once"
```

### Spawning & Deleting Entities

#### Spawn a robot

```bash
# Script
docker compose exec bridge bash -c "source /ros_entrypoint.sh && \
  python3 scripts/spawn_robots.py --name new_robot --urdf robots/arm_robot.urdf --x 5.0 --y 3.0"

# Raw
docker compose exec bridge bash -c "source /ros_entrypoint.sh && \
  ros2 service call /sim/spawn_entity simulation_interfaces/srv/SpawnEntity \
    '{name: new_robot, entity_resource: {uri: robots/arm_robot.urdf, resource_string: \"\"}, \
      allow_renaming: false, entity_namespace: \"\", \
      initial_pose: {header: {frame_id: \"\"}, \
        pose: {position: {x: 5.0, y: 3.0, z: 0.05}, \
               orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}'"
```

#### Delete a robot

```bash
docker compose exec bridge bash -c "source /ros_entrypoint.sh && \
  ros2 service call /sim/delete_entity simulation_interfaces/srv/DeleteEntity '{entity: new_robot}'"
```

### Velocity Control

#### Send cmd_vel (script — recommended)

```bash
# Move for 5 seconds
docker compose exec bridge bash -c "source /ros_entrypoint.sh && \
  python3 scripts/teleop_cmd_vel.py --robot robot0 --vx 1.0 --wz 0.2 --duration 5.0"

# Continuous until Ctrl+C
docker compose exec bridge bash -c "source /ros_entrypoint.sh && \
  python3 scripts/teleop_cmd_vel.py --robot robot0 --vx 0.5"
```

#### Send cmd_vel (raw topic pub)

```bash
# Single publish
docker compose exec bridge bash -c "source /ros_entrypoint.sh && \
  ros2 topic pub --once /robot0/cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}'"

# Continuous (10 Hz)
docker compose exec bridge bash -c "source /ros_entrypoint.sh && \
  ros2 topic pub -r 10 /robot0/cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 1.0, y: 0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}'"
```

#### Keyboard teleop

```bash
docker compose exec -it bridge bash -c "source /ros_entrypoint.sh && \
  ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r /cmd_vel:=/robot0/cmd_vel"
```

### Simulation Control

#### Step simulation

```bash
# Script
docker compose exec bridge bash -c "source /ros_entrypoint.sh && \
  python3 scripts/step_simulation.py --steps 100"

# Raw
docker compose exec bridge bash -c "source /ros_entrypoint.sh && \
  ros2 service call /sim/step_simulation simulation_interfaces/srv/StepSimulation '{steps: 100}'"
```

#### Pause / Resume

```bash
# Script
docker compose exec bridge bash -c "source /ros_entrypoint.sh && \
  python3 scripts/step_simulation.py --pause"
docker compose exec bridge bash -c "source /ros_entrypoint.sh && \
  python3 scripts/step_simulation.py --resume"
docker compose exec bridge bash -c "source /ros_entrypoint.sh && \
  python3 scripts/step_simulation.py --status"

# Raw
docker compose exec bridge bash -c "source /ros_entrypoint.sh && \
  ros2 service call /sim/set_simulation_state simulation_interfaces/srv/SetSimulationState \
    '{state: {state: 2}}'"   # Pause
docker compose exec bridge bash -c "source /ros_entrypoint.sh && \
  ros2 service call /sim/set_simulation_state simulation_interfaces/srv/SetSimulationState \
    '{state: {state: 1}}'"   # Resume
```

### Stop

```bash
docker compose down
```

## All ROS Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `num_robots` | int | 1 | Number of robots to spawn at startup |
| `robot_urdf` | str | `robots/mobile_robot.urdf` | Default URDF path |
| `gui` | bool | false | Enable PyBullet GUI |
| `physics` | bool | false | Enable physics simulation |
| `publish_rate` | float | 50.0 | odom/joint_states publish rate (Hz) |
| `target_rtf` | float | 1.0 | Real-time factor |
| `enable_sim_services` | bool | true | Enable simulation_interfaces services |
| `config_yaml` | str | `""` | Bridge YAML config file (overrides `num_robots`/`robot_urdf` with a `robots` list) |

When `config_yaml` is set, its `robots` list takes precedence over `num_robots` / `robot_urdf`.

To pass parameters not exposed as environment variables, override the command:

```bash
docker compose run --rm bridge \
  bash -c "ros2 run pybullet_fleet_ros bridge_node --ros-args \
    -p num_robots:=2 -p gui:=false"

# Or with config_yaml:
docker compose run --rm bridge ros2 run pybullet_fleet_ros bridge_node \
  --ros-args -p config_yaml:=/opt/bridge_ws/src/pybullet_fleet_ros/config/bridge_omni_demo.yaml -p gui:=true
```

## Troubleshooting

### `Error in gladLoadGLX`

If PyBullet GUI fails to open inside Docker:

1. Ensure `xhost +local:docker` has been run
2. Ensure `/dev/dri` is mounted (included in compose by default)
3. Ensure Mesa/libglvnd packages are installed in the Dockerfile

### Service Calls Timeout

DDS discovery issues can occur when calling from a separate container.
Use `docker compose exec bridge` to run commands inside the same container.
