# Open-RMF Fleet Adapter Example — Agent Specification

## Requirements

### Functional

- `fleet_adapter.py` — ROS 2 node wrapping `rmf_fleet_adapter` Python API
- `rmf_command_handle.py` — `RobotCommandHandle` per robot, translates RMF navigate → `NavigateToPose` action call
- `rmf_demo.launch.py` — one-command launch: bridge + fleet adapter
- `rmf_fleet.yaml` — fleet name, robot footprint, max velocity, map name
- Docker image with `ros-jazzy-rmf-fleet-adapter` installed
- EventBus `agent_spawned` / `agent_removed` integration for auto robot registration (PR 1 dependency)
- rmf_demos office world loaded via `load_rmf_world()` (PR 2 dependency)

### Non-Functional

- No changes to `pybullet_fleet/` core
- Example-level quality (documented, runnable, not production-grade fleet adapter)
- Docker build time increase < 2 minutes from rmf package install

## Constraints

- `rmf_fleet_adapter` Python API must be available as `import rmf_adapter` in Docker
- If `rmf_adapter` not available at import, fleet adapter node logs warning and exits gracefully
- All RMF communication goes through standard ROS 2 topics/actions (no direct `pybullet_fleet` import in fleet adapter node)
- Fleet adapter node runs in separate process from bridge node (two executables)

## Approach

Implement a minimal fleet adapter following the `rmf_demos_fleet_adapter` pattern:
1. Instantiate `rmf_adapter.Adapter` with fleet configuration
2. For each robot: create `RobotUpdateHandle` to report position/battery/mode
3. Implement `RobotCommandHandle` that sends `NavigateToPose` action goals to the bridge
4. The bridge's existing `RobotHandler` handles the actual movement via `Agent`

separation: Fleet adapter speaks only ROS 2 topics/actions. It does NOT import `pybullet_fleet`.

## Design

### Component Table

| Component | File | Purpose |
|-----------|------|---------|
| `FleetAdapterNode` | `ros2_bridge/.../fleet_adapter.py` | Main adapter node — creates `rmf_adapter.Adapter`, registers robots |
| `PybulletCommandHandle` | `ros2_bridge/.../rmf_command_handle.py` | `RobotCommandHandle` impl — navigate, stop, dock via ROS actions |
| Launch | `ros2_bridge/.../launch/rmf_demo.launch.py` | Bridge + fleet adapter launch |
| Config | `ros2_bridge/.../config/rmf_fleet.yaml` | Fleet configuration |
| Docker | `docker/Dockerfile.jazzy` | Add `rmf-fleet-adapter` dependency |

### Data Flow

```
┌────────────────────────────────────────────────────────────────────┐
│ Open-RMF Task Dispatch                                             │
│  ros2 run rmf_demos_tasks dispatch_patrol -p A B -n 3              │
│                                                                    │
│  Open-RMF Traffic Schedule → selects robot → issues navigate()     │
└──────────────────────────┬─────────────────────────────────────────┘
                           │
                           ▼
┌──────────────────────────────────────────────┐
│ PybulletCommandHandle.navigate(waypoints)    │
│                                              │
│  1. Convert RMF waypoints [(x,y,yaw,t),...]  │
│     → geometry_msgs/PoseStamped              │
│  2. Send NavigateToPose action goal           │
│     → /{robot_name}/navigate_to_pose         │
│  3. Monitor action feedback/result            │
│  4. On complete: execution.finished()         │
└──────────────────┬───────────────────────────┘
                   │  ROS 2 action call
                   ▼
┌──────────────────────────────────────────────┐
│ RobotHandler (bridge_node.py)                │
│                                              │
│  NavigateToPose goal → agent.set_goal_pose() │
│  Polls agent.is_moving → action feedback     │
│  Publishes odom, tf, joint_states            │
└──────────────────┬───────────────────────────┘
                   │  internal
                   ▼
┌──────────────────────────────────────────────┐
│ Agent (pybullet_fleet)                       │
│  TPI navigation → PyBullet pose update       │
└──────────────────────────────────────────────┘

Meanwhile, continuously:
  /{robot_name}/odom → PybulletCommandHandle._odom_callback()
    → robot_update_handle.update_position(map, pos, yaw)
```

### Code Patterns

#### fleet_adapter.py

```python
#!/usr/bin/env python3
"""Open-RMF fleet adapter for PyBulletFleet simulation.

Bridges Open-RMF task dispatching with PyBulletFleet's ROS 2 bridge.
Each simulated robot is registered with Open-RMF via rmf_fleet_adapter
Python API. Navigation commands are forwarded as NavigateToPose actions.

Usage:
    ros2 run pybullet_fleet_ros fleet_adapter --ros-args -p config_file:=rmf_fleet.yaml
"""
import rclpy
from rclpy.node import Node

try:
    import rmf_adapter
    HAS_RMF = True
except ImportError:
    HAS_RMF = False


class FleetAdapterNode(Node):
    def __init__(self):
        super().__init__("pybullet_fleet_adapter")

        if not HAS_RMF:
            self.get_logger().error(
                "rmf_adapter not available. Install ros-jazzy-rmf-fleet-adapter."
            )
            return

        # Load fleet config
        config_file = self.declare_parameter("config_file", "").value
        self._fleet_config = self._load_config(config_file)

        # Create rmf_adapter.Adapter
        self._adapter = rmf_adapter.Adapter.make("pybullet_fleet_adapter")
        self._fleet_handle = self._adapter.add_fleet(
            self._fleet_config["fleet_name"],
            self._make_fleet_configuration(),
        )

        # Discover robots from /sim/get_entities or config
        self._command_handles = {}
        self._discover_and_register_robots()

        self._adapter.start()
        self.get_logger().info("Open-RMF fleet adapter started")

    def _discover_and_register_robots(self):
        """Register each robot with Open-RMF."""
        for robot_name in self._fleet_config["robots"]:
            from pybullet_fleet_ros.rmf_command_handle import PybulletCommandHandle
            cmd_handle = PybulletCommandHandle(
                robot_name=robot_name,
                node=self,
                fleet_config=self._fleet_config,
            )
            self._fleet_handle.add_robot(
                cmd_handle,
                robot_name,
                rmf_adapter.RobotMode.MODE_IDLE,
                cmd_handle.get_current_position(),
                cmd_handle.get_current_map(),
            )
            self._command_handles[robot_name] = cmd_handle
```

#### rmf_command_handle.py

```python
"""RobotCommandHandle implementation for PyBulletFleet.

Translates Open-RMF navigate/stop/dock commands into
NavigateToPose ROS 2 action calls to the bridge node.
"""
import math
from typing import Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node


class PybulletCommandHandle:
    """Per-robot command handle bridging Open-RMF → PyBulletFleet bridge."""

    def __init__(self, robot_name: str, node: Node, fleet_config: dict):
        self._name = robot_name
        self._node = node
        self._map_name = fleet_config.get("map_name", "L1")

        # Current state (from odom)
        self._position = [0.0, 0.0, 0.0]
        self._yaw = 0.0

        # Subscribe to odom for position tracking
        self._odom_sub = node.create_subscription(
            Odometry,
            f"/{robot_name}/odom",
            self._odom_callback,
            10,
        )

        # NavigateToPose action client
        self._nav_client = ActionClient(
            node,
            NavigateToPose,
            f"/{robot_name}/navigate_to_pose",
        )

        self._update_handle = None  # Set by fleet_handle.add_robot()
        self._execution = None  # Current RMF execution handle

    def navigate(self, waypoints, execution):
        """Called by Open-RMF when a navigation task is assigned."""
        self._execution = execution

        # Use last waypoint as goal (simplification for example)
        wp = waypoints[-1]
        goal = PoseStamped()
        goal.header.frame_id = "odom"
        goal.pose.position.x = wp.position[0]
        goal.pose.position.y = wp.position[1]
        goal.pose.orientation = self._yaw_to_quat(wp.position[2])  # yaw

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal

        self._nav_client.wait_for_server(timeout_sec=5.0)
        future = self._nav_client.send_goal_async(nav_goal)
        future.add_done_callback(self._on_goal_accepted)

    def _on_goal_accepted(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            if self._execution:
                self._execution.error("Goal rejected")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav_complete)

    def _on_nav_complete(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            if self._execution:
                self._execution.finished()
        else:
            if self._execution:
                self._execution.error("Navigation failed")

    def stop(self):
        """Called by Open-RMF to cancel current task."""
        # Cancel active NavigateToPose goal
        ...

    def dock(self, dock_name, execution):
        """Docking not supported in example — report error."""
        execution.error("Docking not supported")

    def _odom_callback(self, msg: Odometry):
        self._position = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ]
        self._yaw = self._quat_to_yaw(msg.pose.pose.orientation)

        # Report to Open-RMF
        if self._update_handle:
            self._update_handle.update_position(
                self._map_name, self._position[:2], self._yaw
            )

    def get_current_position(self):
        return self._position[:2]

    def get_current_map(self):
        return self._map_name
```

#### rmf_fleet.yaml

```yaml
# Open-RMF fleet configuration for PyBulletFleet demo
fleet_name: "pybullet_fleet"
map_name: "L1"

robots:
  - pb_0
  - pb_1
  - pb_2
  - pb_3
  - pb_4

robot_config:
  max_linear_velocity: 2.0
  max_angular_velocity: 1.57
  footprint_radius: 0.3
  finishing_radius: 0.5
  urdf_path: "robots/mobile_robot.urdf"

# Open-RMF capabilities
task_capabilities:
  loop: true
  delivery: false
  clean: false
```

#### rmf_demo.launch.py

```python
"""Launch PyBulletFleet bridge + Open-RMF fleet adapter.

Usage:
    ros2 launch pybullet_fleet_ros rmf_demo.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bridge_node = Node(
        package="pybullet_fleet_ros",
        executable="bridge_node",
        name="pybullet_bridge",
        parameters=[{"config_file": "config/rmf_fleet_bridge.yaml"}],
    )

    fleet_adapter = Node(
        package="pybullet_fleet_ros",
        executable="fleet_adapter",
        name="pybullet_fleet_adapter",
        parameters=[
            {"config_file": "config/rmf_fleet.yaml"},
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([bridge_node, fleet_adapter])
```

### EventBus Integration (in bridge_node.py, future)

```python
# After EventBus is available (PR 1):
# In BridgeNode.__init__():
self.sim.events.on("agent_spawned", self._on_agent_spawned)
self.sim.events.on("agent_removed", self._on_agent_removed)

def _on_agent_spawned(self, agent):
    """Auto-register RobotHandler when agent spawns via EventBus."""
    self._register_handler(agent)

def _on_agent_removed(self, agent):
    """Auto-cleanup when agent is removed."""
    self.remove_robot(agent.name)
```

## File References

Files the plan agent MUST read before planning:

- `ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/bridge_node.py` — BridgeNode architecture
- `ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/robot_handler.py` — RobotHandler per-robot topics/actions
- `ros2_bridge/pybullet_fleet_ros/launch/tb3_demo.launch.py` — existing launch pattern
- `ros2_bridge/pybullet_fleet_ros/config/bridge_tb3.yaml` — existing config pattern
- `docker/Dockerfile.jazzy` — Docker setup (add rmf packages)
- `docker/README.md` — documentation (add Open-RMF section)
- `docs/design/eventbus/spec.md` — EventBus spec (PR 1 dependency)
- `docs/design/sdf-loader/spec.md` — SDF loader spec (PR 2 dependency)

## Success Criteria

- [ ] `docker compose up` で bridge + fleet adapter が起動
- [ ] `ros2 topic echo /fleet_states` で5台のロボット状態が表示
- [ ] `ros2 run rmf_demos_tasks dispatch_patrol` でパトロールタスクが dispatch
- [ ] PyBulletFleet のロボットがタスクに応じて移動
- [ ] タスク完了が Open-RMF に報告される
- [ ] EventBus `agent_spawned` でロボット追加時に自動登録（PR 1 依存）
- [ ] rmf_demos office world が `load_rmf_world()` で読み込まれて PyBullet に表示（PR 2 依存）
- [ ] Docker build が成功
- [ ] `docker/README.md` に Open-RMF demo 手順が記載
