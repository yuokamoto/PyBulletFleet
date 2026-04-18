# Open-RMF Fleet Adapter Example — Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use executing-plans to implement this plan task-by-task.

**Goal:** Add a minimal Open-RMF fleet adapter example that bridges Open-RMF task dispatching with PyBulletFleet via the ROS 2 bridge. Includes fleet adapter node, command handle, launch file, config, Docker integration, and documentation.

**Architecture:** Fleet adapter node (`fleet_adapter.py`) uses `rmf_adapter` Python API to register simulated robots with Open-RMF. Per-robot `PybulletCommandHandle` translates RMF navigate commands into `NavigateToPose` action goals sent to the bridge's `RobotHandler`. Position updates flow back via odom subscription. No changes to `pybullet_fleet/` core.

**Tech Stack:** ROS 2 Jazzy, `rmf_fleet_adapter` Python bindings (`rmf_adapter`), Docker, `NavigateToPose` action.

**Depends on:** EventBus (PR 1), SDF Loader (PR 2).

---

### Task 1: Docker — Add rmf packages (SERIAL)

**Files:**
- Modify: `docker/Dockerfile.jazzy`

**Step 1: Add rmf package dependencies**

In `docker/Dockerfile.jazzy`, in the `apt-get install` block, add after the `ros-jazzy-turtlebot3-description` line:

```dockerfile
    # Open-RMF fleet adapter (for rmf_demos integration)
    ros-jazzy-rmf-fleet-adapter \
    ros-jazzy-rmf-demos-tasks \
```

**Step 2: Build Docker to verify**

```bash
cd docker && docker compose build bridge
```
Expected: Build succeeds

**Step 3: Commit**

```bash
git add docker/Dockerfile.jazzy
git commit -m "feat(rmf): add rmf-fleet-adapter to Docker image"
```

---

### Task 2: Fleet config YAML (SERIAL)

**Files:**
- Create: `ros2_bridge/pybullet_fleet_ros/config/rmf_fleet.yaml`

**Step 1: Create config**

```yaml
# Open-RMF fleet configuration for PyBulletFleet demo
#
# Used by the fleet adapter to register robots with Open-RMF.
# Robot names must match the bridge spawn config.
#
# Usage:
#   ros2 run pybullet_fleet_ros fleet_adapter \
#     --ros-args -p config_file:=/path/to/rmf_fleet.yaml

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

# Open-RMF capabilities
task_capabilities:
  loop: true
  delivery: false
  clean: false
```

**Step 2: Create bridge config for rmf demo**

Create `ros2_bridge/pybullet_fleet_ros/config/bridge_rmf.yaml`:

```yaml
# Bridge config for Open-RMF demo — 5 omnidirectional robots
#
# Usage (Docker):
#   ros2 launch pybullet_fleet_ros rmf_demo.launch.py

simulation:
  gui: false
  physics: false

robots:
  - name: pb_0
    urdf_path: "robots/mobile_robot.urdf"
    pose: [0.0, 0.0, 0.1]
    yaw: 0.0
    motion_mode: "omnidirectional"
    max_linear_vel: 2.0
    max_linear_accel: 5.0

  - name: pb_1
    urdf_path: "robots/mobile_robot.urdf"
    pose: [2.0, 0.0, 0.1]
    yaw: 0.0
    motion_mode: "omnidirectional"
    max_linear_vel: 2.0
    max_linear_accel: 5.0

  - name: pb_2
    urdf_path: "robots/mobile_robot.urdf"
    pose: [4.0, 0.0, 0.1]
    yaw: 0.0
    motion_mode: "omnidirectional"
    max_linear_vel: 2.0
    max_linear_accel: 5.0

  - name: pb_3
    urdf_path: "robots/mobile_robot.urdf"
    pose: [6.0, 0.0, 0.1]
    yaw: 0.0
    motion_mode: "omnidirectional"
    max_linear_vel: 2.0
    max_linear_accel: 5.0

  - name: pb_4
    urdf_path: "robots/mobile_robot.urdf"
    pose: [8.0, 0.0, 0.1]
    yaw: 0.0
    motion_mode: "omnidirectional"
    max_linear_vel: 2.0
    max_linear_accel: 5.0
```

**Step 3: Commit**

```bash
git add ros2_bridge/pybullet_fleet_ros/config/rmf_fleet.yaml \
        ros2_bridge/pybullet_fleet_ros/config/bridge_rmf.yaml
git commit -m "feat(rmf): add fleet adapter and bridge config files"
```

---

### Task 3: RobotCommandHandle implementation (SERIAL)

**Files:**
- Create: `ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/rmf_command_handle.py`

**Step 1: Implement PybulletCommandHandle**

```python
"""RobotCommandHandle implementation for PyBulletFleet.

Translates Open-RMF navigate/stop/dock commands into
NavigateToPose ROS 2 action calls to the bridge node.

Each instance manages one robot: subscribes to its odom for position
reporting and sends NavigateToPose goals for navigation tasks.
"""

import math
from typing import Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node

import logging

logger = logging.getLogger(__name__)


class PybulletCommandHandle:
    """Per-robot command handle bridging Open-RMF → PyBulletFleet bridge.

    Implements the interface expected by ``rmf_fleet_adapter``'s
    ``FleetUpdateHandle.add_robot()``: navigate, stop, dock callbacks.

    Args:
        robot_name: Name of the robot (must match bridge spawn name)
        node: ROS 2 node for creating publishers/subscribers/action clients
        fleet_config: Fleet configuration dict (from rmf_fleet.yaml)
    """

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
        self._active_goal_handle = None  # For cancel

    def navigate(self, waypoints, execution):
        """Called by Open-RMF when a navigation task is assigned.

        Converts the last RMF waypoint to a NavigateToPose goal and sends
        it to the bridge's action server.
        """
        self._execution = execution
        logger.info(f"[{self._name}] Navigate request: {len(waypoints)} waypoints")

        # Use last waypoint as goal (simplification for example)
        wp = waypoints[-1]
        goal = PoseStamped()
        goal.header.frame_id = "odom"
        goal.pose.position.x = wp.position[0]
        goal.pose.position.y = wp.position[1]
        goal.pose.orientation = self._yaw_to_quat(wp.position[2])

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            logger.error(f"[{self._name}] NavigateToPose server not available")
            if self._execution:
                self._execution.error("NavigateToPose server not available")
            return

        future = self._nav_client.send_goal_async(nav_goal)
        future.add_done_callback(self._on_goal_accepted)

    def _on_goal_accepted(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            logger.warning(f"[{self._name}] NavigateToPose goal rejected")
            if self._execution:
                self._execution.error("Goal rejected")
            return
        self._active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav_complete)

    def _on_nav_complete(self, future):
        result = future.result()
        self._active_goal_handle = None
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            logger.info(f"[{self._name}] Navigation completed")
            if self._execution:
                self._execution.finished()
        else:
            logger.warning(f"[{self._name}] Navigation failed (status={result.status})")
            if self._execution:
                self._execution.error(f"Navigation failed (status={result.status})")

    def stop(self):
        """Called by Open-RMF to cancel current task."""
        logger.info(f"[{self._name}] Stop requested")
        if self._active_goal_handle is not None:
            self._active_goal_handle.cancel_goal_async()
            self._active_goal_handle = None

    def dock(self, dock_name, execution):
        """Docking not supported in example — report error."""
        logger.warning(f"[{self._name}] Docking not supported")
        execution.error("Docking not supported in PyBulletFleet example")

    def _odom_callback(self, msg: Odometry):
        """Update current position from odom and report to Open-RMF."""
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
        """Return current [x, y] position."""
        return self._position[:2]

    def get_current_map(self):
        """Return current map name."""
        return self._map_name

    @staticmethod
    def _yaw_to_quat(yaw: float) -> Quaternion:
        """Convert yaw angle to geometry_msgs/Quaternion."""
        q = Quaternion()
        q.w = math.cos(yaw / 2.0)
        q.z = math.sin(yaw / 2.0)
        return q

    @staticmethod
    def _quat_to_yaw(q) -> float:
        """Convert geometry_msgs/Quaternion to yaw angle."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
```

**Step 2: Commit**

```bash
git add ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/rmf_command_handle.py
git commit -m "feat(rmf): add PybulletCommandHandle for Open-RMF navigation"
```

---

### Task 4: Fleet adapter node (SERIAL, depends on Task 3)

**Files:**
- Create: `ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/fleet_adapter.py`

**Step 1: Implement FleetAdapterNode**

```python
#!/usr/bin/env python3
"""Open-RMF fleet adapter for PyBulletFleet simulation.

Bridges Open-RMF task dispatching with PyBulletFleet's ROS 2 bridge.
Each simulated robot is registered with Open-RMF via rmf_fleet_adapter
Python API. Navigation commands are forwarded as NavigateToPose actions.

Usage:
    ros2 run pybullet_fleet_ros fleet_adapter \
        --ros-args -p config_file:=/path/to/rmf_fleet.yaml
"""

import logging
from typing import Dict

import rclpy
import yaml
from rclpy.node import Node

try:
    import rmf_adapter

    HAS_RMF = True
except ImportError:
    HAS_RMF = False

from .rmf_command_handle import PybulletCommandHandle

logger = logging.getLogger(__name__)


class FleetAdapterNode(Node):
    """ROS 2 node wrapping rmf_fleet_adapter for PyBulletFleet.

    Reads fleet configuration from YAML, creates an ``rmf_adapter.Adapter``,
    and registers each robot with a ``PybulletCommandHandle`` that forwards
    navigate commands as ``NavigateToPose`` action goals.

    Parameters (ROS):
        config_file (str): Path to rmf_fleet.yaml
    """

    def __init__(self):
        super().__init__("pybullet_fleet_adapter")

        if not HAS_RMF:
            self.get_logger().error(
                "rmf_adapter not available. "
                "Install: apt install ros-jazzy-rmf-fleet-adapter"
            )
            return

        # Load fleet config
        self.declare_parameter("config_file", "")
        config_file = (
            self.get_parameter("config_file").get_parameter_value().string_value
        )
        if not config_file:
            self.get_logger().error("config_file parameter is required")
            return

        self._fleet_config = self._load_config(config_file)
        if self._fleet_config is None:
            return

        # Create rmf_adapter.Adapter
        self._adapter = rmf_adapter.Adapter.make("pybullet_fleet_adapter")
        fleet_config = self._make_fleet_configuration()
        self._fleet_handle = self._adapter.add_fleet(
            self._fleet_config["fleet_name"],
            fleet_config,
        )

        # Register robots
        self._command_handles: Dict[str, PybulletCommandHandle] = {}
        self._register_robots()

        self._adapter.start()
        self.get_logger().info(
            f"Open-RMF fleet adapter started: "
            f"{len(self._command_handles)} robots registered"
        )

    def _load_config(self, config_file: str) -> dict:
        """Load fleet YAML config."""
        try:
            with open(config_file, "r") as f:
                return yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Failed to load config: {e}")
            return None

    def _make_fleet_configuration(self):
        """Create rmf_adapter fleet configuration."""
        robot_config = self._fleet_config.get("robot_config", {})
        config = rmf_adapter.FleetConfiguration(
            self._fleet_config["fleet_name"],
        )
        # Set robot traits
        config.linear_velocity = robot_config.get("max_linear_velocity", 2.0)
        config.angular_velocity = robot_config.get("max_angular_velocity", 1.57)
        config.footprint_radius = robot_config.get("footprint_radius", 0.3)
        config.finishing_radius = robot_config.get("finishing_radius", 0.5)
        return config

    def _register_robots(self):
        """Register each robot with Open-RMF."""
        for robot_name in self._fleet_config.get("robots", []):
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
            self.get_logger().info(f"Registered robot: {robot_name}")


def main(args=None):
    rclpy.init(args=args)
    node = FleetAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

**Step 2: Register entry point in setup.cfg/pyproject.toml**

Check `ros2_bridge/pybullet_fleet_ros/setup.cfg` or `setup.py` and add:

```python
'fleet_adapter = pybullet_fleet_ros.fleet_adapter:main',
```

to the `console_scripts` entry points.

**Step 3: Commit**

```bash
git add ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/fleet_adapter.py
git commit -m "feat(rmf): add FleetAdapterNode with robot registration"
```

---

### Task 5: Launch file (SERIAL, depends on Task 4)

**Files:**
- Create: `ros2_bridge/pybullet_fleet_ros/launch/rmf_demo.launch.py`

**Step 1: Create launch file**

```python
"""Launch PyBulletFleet bridge + Open-RMF fleet adapter.

Starts the simulation bridge with 5 robots and the Open-RMF fleet adapter
that registers all robots with Open-RMF for task dispatching.

Usage::

    ros2 launch pybullet_fleet_ros rmf_demo.launch.py
    ros2 launch pybullet_fleet_ros rmf_demo.launch.py gui:=true

Then dispatch a patrol task::

    ros2 run rmf_demos_tasks dispatch_patrol -p A B -n 3
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("pybullet_fleet_ros")
    src_config = "/opt/bridge_ws/src/pybullet_fleet_ros/config"

    # Resolve config files (installed or source)
    bridge_config = os.path.join(pkg_dir, "config", "bridge_rmf.yaml")
    if not os.path.exists(bridge_config):
        bridge_config = os.path.join(src_config, "bridge_rmf.yaml")

    fleet_config = os.path.join(pkg_dir, "config", "rmf_fleet.yaml")
    if not os.path.exists(fleet_config):
        fleet_config = os.path.join(src_config, "rmf_fleet.yaml")

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
                        "config_yaml": bridge_config,
                        "gui": LaunchConfiguration("gui"),
                        "target_rtf": LaunchConfiguration("target_rtf"),
                    }
                ],
                output="screen",
            ),
            Node(
                package="pybullet_fleet_ros",
                executable="fleet_adapter",
                name="pybullet_fleet_adapter",
                parameters=[
                    {
                        "config_file": fleet_config,
                        "use_sim_time": True,
                    }
                ],
                output="screen",
            ),
        ]
    )
```

**Step 2: Commit**

```bash
git add ros2_bridge/pybullet_fleet_ros/launch/rmf_demo.launch.py
git commit -m "feat(rmf): add rmf_demo launch file (bridge + fleet adapter)"
```

---

### Task 6: EventBus integration in bridge_node.py (SERIAL, depends on PR 1)

**Files:**
- Modify: `ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/bridge_node.py`

**Step 1: Add EventBus auto-registration**

In `bridge_node.py`, after the bridge node creates the sim core and spawns robots (in `__init__`), add EventBus hooks:

```python
        # EventBus: auto-register/unregister handlers when agents spawn/despawn
        self.sim.events.on("agent_spawned", self._on_agent_spawned)
        self.sim.events.on("agent_removed", self._on_agent_removed)
```

Add the callback methods to `BridgeNode`:

```python
    def _on_agent_spawned(self, agent):
        """Auto-register RobotHandler when agent spawns via EventBus."""
        if agent.name and agent.name not in self._handlers:
            self._register_handler(agent)
            self.get_logger().info(f"EventBus: auto-registered handler for {agent.name}")

    def _on_agent_removed(self, agent):
        """Auto-cleanup when agent is removed via EventBus."""
        if agent.name and agent.name in self._handlers:
            # Clean up handler (remove publishers, subscribers, etc.)
            del self._handlers[agent.name]
            self.get_logger().info(f"EventBus: removed handler for {agent.name}")
```

**Step 2: Commit**

```bash
git add ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/bridge_node.py
git commit -m "feat(rmf): add EventBus auto-registration for RobotHandlers"
```

---

### Task 7: Docker README documentation (SERIAL)

**Files:**
- Modify: `docker/README.md`

**Step 1: Add Open-RMF section**

Append to `docker/README.md`:

```markdown
## Open-RMF Integration Demo

Demonstrates PyBulletFleet integrated with Open-RMF task dispatching.

### Prerequisites

The Docker image includes `ros-jazzy-rmf-fleet-adapter` and `ros-jazzy-rmf-demos-tasks`.

### Launch

```bash
# Start bridge + fleet adapter (5 robots)
docker compose run --rm bridge \
  ros2 launch pybullet_fleet_ros rmf_demo.launch.py

# In another terminal: dispatch a patrol task
docker compose run --rm bridge \
  ros2 run rmf_demos_tasks dispatch_patrol -p 0.0,0.0 5.0,5.0 -n 3
```

### Architecture

```
Open-RMF Traffic Schedule
    │
    ▼
FleetAdapterNode (fleet_adapter.py)
    │  rmf_adapter.Adapter → selects robot → navigate()
    ▼
PybulletCommandHandle → NavigateToPose action goal
    │
    ▼
BridgeNode → RobotHandler → Agent.set_goal_pose()
    │
    ▼
PyBulletFleet (kinematic navigation)
```

Fleet adapter subscribes to each robot's `/odom` topic to report position
to Open-RMF continuously.
```

**Step 2: Commit**

```bash
git add docker/README.md
git commit -m "docs: add Open-RMF demo instructions to Docker README"
```

---

### Task 8: Setup.py entry point + colcon build verification (SERIAL, depends on Task 4)

**Files:**
- Modify: `ros2_bridge/pybullet_fleet_ros/setup.py` or `setup.cfg`

**Step 1: Add fleet_adapter entry point**

Read current setup file:
```bash
cat ros2_bridge/pybullet_fleet_ros/setup.py
```

Add `'fleet_adapter = pybullet_fleet_ros.fleet_adapter:main'` to `console_scripts`.

Also add config and launch data files if not already present:

```python
data_files=[
    ...
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
],
```

**Step 2: Build in Docker to verify**

```bash
cd docker && docker compose build bridge
docker compose run --rm bridge bash -c "
  source /opt/ros/jazzy/setup.bash
  source /opt/bridge_ws/install/setup.bash
  ros2 run pybullet_fleet_ros fleet_adapter --help
"
```

**Step 3: Commit**

```bash
git add ros2_bridge/pybullet_fleet_ros/setup.py
git commit -m "feat(rmf): register fleet_adapter entry point"
```

---

### Task 9: Docker build + integration test (SERIAL, depends on all)

**Step 1: Full Docker build**

```bash
cd docker && docker compose build bridge
```
Expected: Build succeeds

**Step 2: Verify bridge + fleet adapter start**

```bash
docker compose run --rm bridge bash -c "
  source /opt/bridge_ws/install/setup.bash
  timeout 10 ros2 launch pybullet_fleet_ros rmf_demo.launch.py 2>&1 || true
"
```
Expected: Both nodes start, fleet adapter registers 5 robots (or logs `rmf_adapter not available` if package not available — which is acceptable for example-level code).

**Step 3: Commit any fixes**

```bash
git add -A
git commit -m "fix(rmf): integration fixes from Docker build test"
```

---

### Task 10: Final verification (SERIAL)

**Step 1: Run pybullet_fleet core tests**

```bash
make verify
```
Expected: ALL PASS (no changes to core `pybullet_fleet/` code in this PR)

**Step 2: Docker build clean**

```bash
cd docker && docker compose build bridge
```
Expected: Success

**Step 3: Final commit**

```bash
make format
git add -A
git commit -m "style: format Open-RMF fleet adapter code"
```
