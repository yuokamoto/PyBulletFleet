# ROS 2 Bridge — Agent Specification

**Depends on:** [Plugin Architecture](../plugin-architecture/agent.spec.md) Phase 2 (Controller ABC + OmniOmniVelocityController)

## Requirements

### Functional

- Single ROS 2 node (`pybullet_fleet_bridge`) that wraps `MultiRobotSimulationCore`
- Dynamically creates per-robot topic publishers/subscribers based on spawned agents
- Per-robot topics: `cmd_vel` (Twist sub), `odom` (Odometry pub), `joint_states` (JointState pub)
- TF broadcasting for all robots (world → base_link transforms)
- Nav2-compatible action servers: `NavigateToPose`, `FollowPath` per mobile robot
- `FollowJointTrajectory` action server per arm robot
- `/clock` publisher (`rosgraph_msgs/Clock`) driven by sim time for `use_sim_time` support
- `simulation_interfaces` service servers for sim-level control (broad coverage — see full list below)
- `simulation_interfaces` `SimulateSteps` action server for N-step execution with feedback
- Launch files for 5 example scenarios (teleop, nav2, fleet, arm, mobile_manipulator)
- Dockerfile.jazzy and Dockerfile.humble with docker-compose.yaml

### Non-Functional

- Bridge overhead < 5% of total sim step time for 10 robots
- Must not import `rclpy` from `pybullet_fleet/` core (ROS-independent core)
- Compatible with `p.DIRECT` mode (headless Docker CI)
- All examples must work via `docker compose up` without native ROS install

## Constraints

- Ubuntu 20.04 host — ROS 2 runs exclusively in Docker containers
- Python ≥ 3.10 (matches `pybullet_fleet` requirement)
- `simulation_interfaces` package is not yet available via apt for all distros — may need source build in Dockerfile
- No custom message types in Phase 1
- `pybullet_fleet` is installed via pip (editable) inside Docker container

## Approach

### Bridge Architecture

The bridge node runs `MultiRobotSimulationCore` in-process. A ROS timer drives
`sim.step()` at the target RTF. Per-robot handlers are created dynamically
when agents are spawned (either at startup or via `SpawnEntity` service).

Velocity commands (`cmd_vel`) flow through the **OmniOmniVelocityController** (Controller ABC)
rather than a direct `set_velocity()` on Agent. Each ROS-controlled agent is assigned
a `OmniOmniVelocityController` at spawn time via `agent.set_controller()`. The `RobotHandler`
stores a reference to the controller and calls `controller.set_velocity()` on each
`cmd_vel` message.

The `AgentManagerROSWrapper` wraps `AgentManager` to provide fleet-level
batch interfaces in Phase 2.

### Simulation Loop

```python
# Simplified bridge node lifecycle
class BridgeNode(Node):
    def __init__(self):
        self.sim = MultiRobotSimulationCore(SimulationParams(gui=False, physics=False))
        self.robot_handlers: Dict[int, RobotHandler] = {}
        self.sim_services = SimServices(self, self.sim)

        # /clock publisher for use_sim_time
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.sim_time = 0.0

        # Timer drives simulation at target_rtf
        dt = self.sim.params.timestep
        self.create_timer(dt, self._step_callback)

    def _spawn_robot(self, spawn_params) -> Agent:
        agent = Agent.from_params(spawn_params, sim_core=self.sim)
        # Attach OmniOmniVelocityController for cmd_vel control
        from pybullet_fleet.controller import OmniOmniVelocityController
        vel_ctrl = OmniOmniVelocityController()
        agent.set_controller(vel_ctrl)
        handler = RobotHandler(self, agent, vel_ctrl)
        self.robot_handlers[agent.object_id] = handler
        return agent

    def _step_callback(self):
        # Apply pending cmd_vel (calls OmniOmniVelocityController.set_velocity)
        for handler in self.robot_handlers.values():
            handler.apply_cmd_vel()

        self.sim.step()
        self.sim_time += self.sim.params.timestep

        # Publish /clock
        clock_msg = Clock()
        clock_msg.clock = Time(sec=int(self.sim_time), nanosec=int((self.sim_time % 1) * 1e9))
        self.clock_pub.publish(clock_msg)

        for handler in self.robot_handlers.values():
            handler.publish_state()  # odom, joint_states, tf
```

### Per-Robot Handler

```python
class RobotHandler:
    """Manages ROS interfaces for a single Agent."""

    def __init__(self, node: Node, agent: Agent, vel_controller: "OmniOmniVelocityController"):
        ns = agent.name  # e.g., "robot_0"
        self._vel_controller = vel_controller  # Reference to agent's OmniOmniVelocityController
        self._latest_twist: Optional[Twist] = None

        # Subscribers
        self.cmd_vel_sub = node.create_subscription(
            Twist, f'/{ns}/cmd_vel', self._cmd_vel_cb, 10)

        # Publishers
        self.odom_pub = node.create_publisher(Odometry, f'/{ns}/odom', 10)
        self.joint_pub = node.create_publisher(JointState, f'/{ns}/joint_states', 10)

        # Action servers (conditional)
        if agent.spawn_params.motion_mode in (MotionMode.OMNIDIRECTIONAL, MotionMode.DIFFERENTIAL):
            self.nav_action = ActionServer(node, NavigateToPose, f'/{ns}/navigate_to_pose', ...)
            self.path_action = ActionServer(node, FollowPath, f'/{ns}/follow_path', ...)

        if agent.ik_params is not None:
            self.joint_traj_action = ActionServer(node, FollowJointTrajectory,
                f'/{ns}/follow_joint_trajectory', ...)

    def _cmd_vel_cb(self, msg: Twist):
        # Store latest twist; applied in next step
        self._latest_twist = msg

    def apply_cmd_vel(self):
        """Apply stored cmd_vel as OmniOmniVelocityController command.

        Called once per sim step, before sim.step_once().
        Converts body-frame Twist to world-frame velocity.
        """
        if self._latest_twist is None:
            return
        twist = self._latest_twist
        yaw = self.agent.get_pose().yaw
        # Body-frame → world-frame rotation
        import math
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        vx_world = twist.linear.x * cos_yaw - twist.linear.y * sin_yaw
        vy_world = twist.linear.x * sin_yaw + twist.linear.y * cos_yaw
        # Set velocity on the OmniOmniVelocityController (NOT on Agent directly)
        self._vel_controller.set_velocity(
            vx=vx_world, vy=vy_world,
            vz=twist.linear.z, wz=twist.angular.z
        )
        self._latest_twist = None  # Require continuous publishing

    def publish_state(self):
        # Publish odom, joint_states, and tf
        pose = self.agent.get_pose()
        # ... convert to Odometry msg and publish
```

### AgentManager ROS Wrapper (Phase 2 preparation)

```python
class AgentManagerROSWrapper:
    """Wraps AgentManager for fleet-level ROS operations."""

    def __init__(self, node: Node, agent_manager: AgentManager):
        self._mgr = agent_manager
        # Phase 2: batch topics
        # self.batch_cmd_sub = node.create_subscription(
        #     FleetCmdVel, '/fleet/cmd_vel_batch', self._batch_cmd_cb, 10)

    def create_handlers(self, node: Node) -> Dict[int, RobotHandler]:
        """Create a RobotHandler for each agent in the manager."""
        handlers = {}
        for agent in self._mgr.objects:
            handlers[agent.object_id] = RobotHandler(node, agent)
        return handlers
```

### Simulation Interfaces Implementation

```python
class SimServices:
    """simulation_interfaces service + action implementations."""

    def __init__(self, node: Node, sim: MultiRobotSimulationCore):
        self.sim = sim

        # --- Services (Phase 1: broad coverage) ---
        node.create_service(GetSimulatorFeatures, '/sim/get_simulator_features', self._get_features)
        node.create_service(SpawnEntity, '/sim/spawn_entity', self._spawn_entity)
        node.create_service(DeleteEntity, '/sim/delete_entity', self._delete_entity)
        node.create_service(GetEntityState, '/sim/get_entity_state', self._get_entity_state)
        node.create_service(SetEntityState, '/sim/set_entity_state', self._set_entity_state)
        node.create_service(GetEntities, '/sim/get_entities', self._get_entities)
        node.create_service(GetEntitiesStates, '/sim/get_entities_states', self._get_entities_states)
        node.create_service(GetEntityInfo, '/sim/get_entity_info', self._get_entity_info)
        node.create_service(GetEntityBounds, '/sim/get_entity_bounds', self._get_entity_bounds)
        node.create_service(StepSimulation, '/sim/step_simulation', self._step_sim)
        node.create_service(GetSimulationState, '/sim/get_simulation_state', self._get_sim_state)
        node.create_service(SetSimulationState, '/sim/set_simulation_state', self._set_sim_state)
        node.create_service(ResetSimulation, '/sim/reset_simulation', self._reset_sim)
        node.create_service(GetSpawnables, '/sim/get_spawnables', self._get_spawnables)

        # --- Actions ---
        self._simulate_steps_server = ActionServer(
            node, SimulateSteps, '/sim/simulate_steps', self._simulate_steps_cb)

    def _spawn_entity(self, request, response):
        # Parse URDF path / model name from request
        # Create AgentSpawnParams, call Agent.from_params()
        # Create new RobotHandler for the spawned agent
        ...

    def _get_entity_state(self, request, response):
        # Find agent by name → agent.get_pose() → fill response
        ...

    def _get_entities(self, request, response):
        # Return list of all agent/object names and IDs
        ...

    def _get_entities_states(self, request, response):
        # Batch: return states for all (or filtered) entities
        ...

    def _get_entity_bounds(self, request, response):
        # p.getAABB(body_id) → Bounds msg
        ...

    def _get_sim_state(self, request, response):
        # Return PAUSED / PLAYING / STEPPING
        ...

    def _set_sim_state(self, request, response):
        # Pause / resume / step
        ...

    def _reset_sim(self, request, response):
        # Reset simulation to initial state
        ...

    def _get_spawnables(self, request, response):
        # Scan robots/ directory for available URDFs
        ...

    def _simulate_steps_cb(self, goal_handle):
        # Execute N steps with feedback (current step / sim time)
        for i in range(goal_handle.request.num_steps):
            self.sim.step()
            feedback = SimulateSteps.Feedback()
            feedback.steps_done = i + 1
            goal_handle.publish_feedback(feedback)
        ...
```

## Design

### Key Components

| Component | Responsibility | Location |
|-----------|---------------|----------|
| `BridgeNode` | Main node, sim loop timer, handler orchestration, OmniOmniVelocityController attachment | `ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/bridge_node.py` |
| `RobotHandler` | Per-robot topics, cmd_vel → OmniOmniVelocityController, state publishing | `ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/robot_handler.py` |
| `SimServices` | simulation_interfaces services + SimulateSteps action, /clock | `ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/sim_services.py` |
| `OmniOmniVelocityController` | cmd_vel velocity → pose update (from plugin architecture) | `pybullet_fleet/controller.py` |
| `AgentManagerROSWrapper` | Fleet-level batch operations (Phase 2) | `ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/agent_manager_wrapper.py` |
| `Dockerfile.jazzy` | Docker image for Jazzy | `docker/Dockerfile.jazzy` |
| `Dockerfile.humble` | Docker image for Humble | `docker/Dockerfile.humble` |
| `docker-compose.yaml` | One-command launch | `docker/docker-compose.yaml` |

### Data Flow

```
                      ROS 2 DDS
                         │
    ┌────────────────────┼────────────────────┬────────────┐
    │                    │                    │            │
    ▼                    ▼                    ▼            ▼
cmd_vel (Twist)    NavigateToPose       SpawnEntity    Other
    │              (Action Goal)        (Service)      srvs
    │                    │                    │
    ▼                    ▼                    ▼
RobotHandler       RobotHandler         SimServices
 .apply_cmd_vel()        │                    │
    │                    │                    │
    ▼                    ▼                    ▼
OmniOmniVelocityController  agent.add_action(   Agent.from_params()
 .set_velocity(      MoveAction(path))   + agent.set_controller(
   vx, vy, vz, wz)                         OmniOmniVelocityController())
    │                    │                    │
    ▼                    ▼                    ▼
Controller.compute() ───────┬─────────────────┘
 (called in agent.update())  │
               ▼             │
    MultiRobotSimulationCore.step()  ◄────────┘
               │
               ▼
    agent.update(dt)  [for each agent]
      │
      ├── controller.compute(agent, dt)  ← OmniOmniVelocityController
      │   └── Euler integration → agent.set_pose()
      │
      └── (or) legacy TPI mode if no controller
               │
               ▼
    RobotHandler.publish_state()
               │
    ┌──────────┼──────────┬───────┐
    ▼          ▼          ▼       ▼
  odom     joint_states   tf    /clock
  (pub)      (pub)       (pub)  (pub)
```

### Action Server ↔ PyBulletFleet Action Mapping

| ROS 2 Action | PyBulletFleet Action | Conversion |
|-------------|---------------------|------------|
| `NavigateToPose` | `MoveAction(path=Path.from_positions([[x,y,z]]))` | Goal pose → single-waypoint path |
| `FollowPath` | `MoveAction(path=Path.from_positions(poses))` | Nav path → Path waypoints |
| `FollowJointTrajectory` | `JointAction(targets=...)` | Trajectory points → joint targets |

### Pose Conversion

```python
# ROS geometry_msgs/Pose → PyBulletFleet Pose
def ros_pose_to_pbf(ros_pose) -> Pose:
    return Pose(
        position=[ros_pose.position.x, ros_pose.position.y, ros_pose.position.z],
        orientation=[ros_pose.orientation.x, ros_pose.orientation.y,
                     ros_pose.orientation.z, ros_pose.orientation.w]
    )

# PyBulletFleet Pose → ROS geometry_msgs/Pose
def pbf_pose_to_ros(pbf_pose) -> PoseMsg:
    msg = PoseMsg()
    msg.position.x, msg.position.y, msg.position.z = pbf_pose.position
    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = pbf_pose.orientation
    return msg
```

### Docker Strategy

```dockerfile
# docker/Dockerfile.jazzy
FROM ros:jazzy-ros-base

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-jazzy-nav2-msgs \
    ros-jazzy-control-msgs \
    ros-jazzy-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

# Install simulation_interfaces from source (not yet in apt for all distros)
WORKDIR /opt/sim_interfaces_ws
RUN git clone https://github.com/ros-simulation/simulation_interfaces.git src/simulation_interfaces \
    && . /opt/ros/jazzy/setup.sh \
    && colcon build --packages-select simulation_interfaces

# Install pybullet_fleet (editable mode)
COPY . /opt/pybullet_fleet
WORKDIR /opt/pybullet_fleet
RUN pip install -e .

# Build ROS 2 bridge
WORKDIR /opt/bridge_ws
RUN ln -s /opt/pybullet_fleet/ros2_bridge/pybullet_fleet_ros src/pybullet_fleet_ros \
    && ln -s /opt/pybullet_fleet/ros2_bridge/pybullet_fleet_bringup src/pybullet_fleet_bringup \
    && . /opt/ros/jazzy/setup.sh \
    && . /opt/sim_interfaces_ws/install/setup.sh \
    && colcon build
```

```yaml
# docker/docker-compose.yaml
services:
  pybullet_fleet_bridge:
    build:
      context: ..
      dockerfile: docker/Dockerfile.jazzy
    command: >
      bash -c "source /opt/bridge_ws/install/setup.bash &&
               ros2 launch pybullet_fleet_ros teleop_mobile.launch.py"
    network_mode: host  # For DDS discovery + X11
    environment:
      - DISPLAY=${DISPLAY}
      - ROS_DOMAIN_ID=42
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix
      - ..:/opt/pybullet_fleet  # Live code mount for dev
```

### Launch File Pattern

```python
# ros2_bridge/pybullet_fleet_ros/launch/teleop_mobile.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pybullet_fleet_ros',
            executable='bridge_node',
            name='pybullet_fleet_bridge',
            parameters=[{
                'config_yaml': 'config/config.yaml',
                'num_robots': 1,
                'robot_urdf': 'robots/mobile_robot.urdf',
                'publish_rate': 50.0,  # Hz
                'gui': True,
            }],
            output='screen',
        ),
    ])
```

### Configuration

Bridge node accepts parameters:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `config_yaml` | string | `""` | Path to PyBulletFleet YAML config |
| `num_robots` | int | 1 | Number of robots to spawn |
| `robot_urdf` | string | `""` | URDF path for default robot |
| `publish_rate` | float | 50.0 | State publish rate (Hz) |
| `gui` | bool | false | Enable PyBullet GUI |
| `physics` | bool | false | Enable physics simulation |
| `enable_nav_actions` | bool | true | Create NavigateToPose/FollowPath servers |
| `enable_joint_actions` | bool | true | Create FollowJointTrajectory servers |
| `enable_sim_services` | bool | true | Create simulation_interfaces services |

## File References

Files the plan agent MUST read before planning:

- `pybullet_fleet/core_simulation.py` — `MultiRobotSimulationCore`, `SimulationParams`, sim loop
- `pybullet_fleet/agent.py` — `Agent`, `AgentSpawnParams`, `set_controller()`, velocity/pose control API
- `pybullet_fleet/controller.py` — `Controller` ABC, `OmniOmniVelocityController`, `TPIController` **(new, from plugin architecture)**
- `pybullet_fleet/agent_manager.py` — `AgentManager`, batch operations, object_ids mapping
- `pybullet_fleet/action.py` — `MoveAction`, `JointAction` action lifecycle
- `pybullet_fleet/geometry.py` — `Pose`, `Path` (ROS2-compatible conventions)
- `pybullet_fleet/types.py` — `MotionMode`, `ActionStatus` enums
- `pybullet_fleet/sim_object.py` — `SimObject` base class
- `docs/design/plugin-architecture/agent.spec.md` — Controller ABC and OmniOmniVelocityController interface
- `examples/action_system_demo.py` — Example of action dispatch pattern
- `examples/pick_drop_arm_action_demo.py` — Arm action example
- `config/config.yaml` — Default config structure
- `robots/mobile_robot.urdf` — Mobile robot URDF
- `robots/arm_robot.urdf` — Arm robot URDF
- `robots/mobile_manipulator.urdf` — Mobile manipulator URDF

## Success Criteria

- [ ] `docker compose up` starts bridge with zero native ROS install on Ubuntu 20.04 host
- [ ] `ros2 topic list` shows expected per-robot topics
- [ ] `/clock` publishes advancing sim time; `ros2 topic echo /clock` works
- [ ] `ros2 topic echo /robot_0/odom` shows live odometry data
- [ ] `teleop_twist_keyboard` → `cmd_vel` → robot moves in PyBullet
- [ ] `NavigateToPose` action goal → robot navigates to pose → action succeeds
- [ ] `FollowPath` action goal → robot follows path → action succeeds
- [ ] `FollowJointTrajectory` action goal → arm moves → action succeeds
- [ ] `SpawnEntity` service → new robot appears with its own topics
- [ ] `GetSimulatorFeatures` service returns correct feature list
- [ ] `GetEntities` service returns all spawned agents
- [ ] `SimulateSteps` action executes N steps with feedback
- [ ] `ResetSimulation` resets and re-creates all entities
- [ ] RViz shows all robot TF frames and odometry
- [ ] 10 robots running with bridge overhead < 5% of step time
- [ ] CI passes on both Jazzy and Humble Docker images
- [ ] All 5 launch files work via `docker compose`
