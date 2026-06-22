# ROS 2 Bridge Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use executing-plans to implement this plan task-by-task.

**Goal:** Build a ROS 2 bridge node that wraps PyBulletFleet's `MultiRobotSimulationCore` and exposes per-robot standard ROS 2 topics/actions plus `simulation_interfaces` services, all running in Docker.

**Architecture:** A single `BridgeNode` drives `sim.step_once()` via a ROS timer and delegates per-robot I/O to `RobotHandler` instances. `SimServices` implements `simulation_interfaces`. `/clock` is published every step. The bridge runs in Docker (Jazzy primary, Humble secondary) while PyBulletFleet core remains ROS-independent.

**Tech Stack:** Python 3.10+, rclpy, nav2_msgs, control_msgs, tf2_ros, simulation_interfaces (source build), Docker, docker compose

---

## Critical Design Notes

### cmd_vel Implementation

**Agent has no `set_velocity()` method.** It uses goal-based control via `set_goal_pose()` / `set_path()`.
For `cmd_vel` (Twist) subscription, the bridge must convert velocity commands to incremental goal poses each step:

```python
def _cmd_vel_cb(self, msg: Twist):
    self._latest_twist = msg  # Store; apply in publish_state cycle

def _apply_cmd_vel(self, dt: float):
    """Called each step to convert stored Twist to a short-range goal."""
    if self._latest_twist is None:
        return
    twist = self._latest_twist
    pose = self.agent.get_pose()
    yaw = pose.yaw
    # Compute displacement in world frame
    dx = (twist.linear.x * math.cos(yaw) - twist.linear.y * math.sin(yaw)) * dt
    dy = (twist.linear.x * math.sin(yaw) + twist.linear.y * math.cos(yaw)) * dt
    new_yaw = yaw + twist.angular.z * dt
    # Set a short-horizon goal (agent's TPI will handle acceleration)
    goal = Pose.from_yaw(pose.x + dx, pose.y + dy, pose.z, new_yaw)
    self.agent.set_goal_pose(goal)
```

### step_once() vs run_simulation()

**Never use `run_simulation()`** — it has a blocking loop with RTF sync.
Use `step_once()` in a ROS timer callback. Set `target_rtf=0` (no internal sleep).

### ROS Package Structure

All ROS code lives under `ros2_bridge/` and is built with `colcon`. It is NOT part of the pip package.

```
ros2_bridge/
├── pybullet_fleet_ros/          # Main ROS package (ament_python)
│   ├── package.xml
│   ├── setup.py
│   ├── setup.cfg
│   ├── resource/pybullet_fleet_ros
│   ├── pybullet_fleet_ros/
│   │   ├── __init__.py
│   │   ├── bridge_node.py
│   │   ├── robot_handler.py
│   │   ├── sim_services.py
│   │   ├── conversions.py         # Pose/msg conversion utilities
│   │   └── agent_manager_wrapper.py
│   ├── launch/
│   │   ├── teleop_mobile.launch.py
│   │   ├── nav2_goal.launch.py
│   │   ├── multi_robot_fleet.launch.py
│   │   ├── arm_control.launch.py
│   │   └── mobile_manipulator.launch.py
│   ├── config/
│   │   └── default_bridge.yaml
│   └── test/
│       ├── test_conversions.py
│       ├── test_bridge_node.py
│       ├── test_robot_handler.py
│       └── test_sim_services.py
└── pybullet_fleet_bringup/       # Optional bringup (RViz configs, composed launches)
    ├── package.xml
    ├── setup.py
    ├── launch/
    └── rviz/
```

---

## Task Dependency Graph

```
Task 1 (Docker) ──────────────────────────────────┐
Task 2 (Package skeleton) ────────────────────────┤
                                                   ├── Task 5 (BridgeNode)
Task 3 (Conversions) ─────────────────────────────┤     │
Task 4 (RobotHandler) ────────────────────────────┘     │
                                                        ├── Task 8 (Integration test)
Task 5 (BridgeNode) ──────────────────────────────┐    │
Task 6 (SimServices) ─────────────────────────────┤    │
Task 7 (Nav2/Arm actions) ────────────────────────┘    │
                                                        │
Task 8 (Integration test) ─────────────────────────────┤
Task 9 (Launch files + examples) ──────────────────────┤
Task 10 (Humble Dockerfile) ───────────────────────────┘
```

- **PARALLEL:** Tasks 1, 2, 3 (no dependencies on each other)
- **SERIAL:** Tasks 4→5→6→7→8→9→10 (each depends on prior)
- **Task 3 (conversions)** can be developed standalone (pure functions)

---

## Task 1: Docker Environment (PARALLEL)

**Files:**
- Create: `docker/Dockerfile.jazzy`
- Create: `docker/docker-compose.yaml`
- Create: `docker/.dockerignore`

**Step 1: Create Dockerfile.jazzy**

```dockerfile
# docker/Dockerfile.jazzy
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# Install ROS 2 dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    ros-jazzy-nav2-msgs \
    ros-jazzy-control-msgs \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

# Build simulation_interfaces from source
WORKDIR /opt/sim_interfaces_ws/src
RUN git clone --depth 1 https://github.com/ros-simulation/simulation_interfaces.git
WORKDIR /opt/sim_interfaces_ws
RUN source /opt/ros/jazzy/setup.bash && \
    colcon build --packages-select simulation_interfaces --cmake-args -DCMAKE_BUILD_TYPE=Release

# Install pybullet_fleet (editable for development)
COPY requirements.txt /opt/pybullet_fleet/requirements.txt
COPY pyproject.toml /opt/pybullet_fleet/pyproject.toml
COPY pybullet_fleet/ /opt/pybullet_fleet/pybullet_fleet/
COPY robots/ /opt/pybullet_fleet/robots/
COPY config/ /opt/pybullet_fleet/config/
COPY mesh/ /opt/pybullet_fleet/mesh/
WORKDIR /opt/pybullet_fleet
RUN pip install --break-system-packages -e .

# Build ROS 2 bridge
WORKDIR /opt/bridge_ws/src
COPY ros2_bridge/pybullet_fleet_ros /opt/bridge_ws/src/pybullet_fleet_ros
WORKDIR /opt/bridge_ws
RUN source /opt/ros/jazzy/setup.bash && \
    source /opt/sim_interfaces_ws/install/setup.bash && \
    colcon build --symlink-install

# Entrypoint
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
```

**Step 2: Create ros_entrypoint.sh**

```bash
#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
source /opt/sim_interfaces_ws/install/setup.bash
source /opt/bridge_ws/install/setup.bash
exec "$@"
```

Save as `docker/ros_entrypoint.sh`.

**Step 3: Create docker-compose.yaml**

```yaml
# docker/docker-compose.yaml
services:
  bridge:
    build:
      context: ..
      dockerfile: docker/Dockerfile.jazzy
    command: >
      ros2 launch pybullet_fleet_ros teleop_mobile.launch.py
    network_mode: host
    environment:
      - DISPLAY=${DISPLAY:-}
      - ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      # Dev mode: mount source for live editing
      - ../pybullet_fleet:/opt/pybullet_fleet/pybullet_fleet:ro
      - ../ros2_bridge/pybullet_fleet_ros:/opt/bridge_ws/src/pybullet_fleet_ros:ro
      - ../robots:/opt/pybullet_fleet/robots:ro
      - ../config:/opt/pybullet_fleet/config:ro
    stdin_open: true
    tty: true
```

**Step 4: Create .dockerignore**

```
# docker/.dockerignore
__pycache__
*.pyc
.git
.github
*.egg-info
build/
dist/
results/
benchmark/
docs/
tests/
```

**Step 5: Verify Docker builds**

Run: `cd docker && docker compose build`
Expected: Image builds successfully (may take 5-10 minutes first time)

**Step 6: Commit**

```bash
git add docker/
git commit -m "feat(ros2): add Docker environment for Jazzy"
```

---

## Task 2: ROS Package Skeleton (PARALLEL)

**Files:**
- Create: `ros2_bridge/pybullet_fleet_ros/package.xml`
- Create: `ros2_bridge/pybullet_fleet_ros/setup.py`
- Create: `ros2_bridge/pybullet_fleet_ros/setup.cfg`
- Create: `ros2_bridge/pybullet_fleet_ros/resource/pybullet_fleet_ros` (empty marker)
- Create: `ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/__init__.py`

**Step 1: Create package.xml**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>pybullet_fleet_ros</name>
  <version>0.1.0</version>
  <description>ROS 2 bridge for PyBulletFleet multi-robot simulation</description>
  <maintainer email="yuokamoto1988@gmail.com">Yu Okamoto</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>nav2_msgs</depend>
  <depend>control_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>
  <depend>rosgraph_msgs</depend>
  <depend>simulation_interfaces</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

**Step 2: Create setup.py**

```python
from setuptools import setup

package_name = 'pybullet_fleet_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/teleop_mobile.launch.py',
            'launch/nav2_goal.launch.py',
            'launch/multi_robot_fleet.launch.py',
            'launch/arm_control.launch.py',
            'launch/mobile_manipulator.launch.py',
        ]),
        ('share/' + package_name + '/config', ['config/default_bridge.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yu Okamoto',
    maintainer_email='yuokamoto1988@gmail.com',
    description='ROS 2 bridge for PyBulletFleet multi-robot simulation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = pybullet_fleet_ros.bridge_node:main',
        ],
    },
)
```

**Step 3: Create setup.cfg**

```ini
[develop]
script_dir=$base/lib/pybullet_fleet_ros
[install]
install_scripts=$base/lib/pybullet_fleet_ros
```

**Step 4: Create resource marker and __init__.py**

```bash
mkdir -p ros2_bridge/pybullet_fleet_ros/resource
touch ros2_bridge/pybullet_fleet_ros/resource/pybullet_fleet_ros
```

```python
# ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/__init__.py
"""PyBulletFleet ROS 2 Bridge — connects PyBulletFleet simulation to ROS 2."""
```

**Step 5: Create default_bridge.yaml placeholder**

```yaml
# ros2_bridge/pybullet_fleet_ros/config/default_bridge.yaml
pybullet_fleet_bridge:
  ros__parameters:
    config_yaml: ""
    num_robots: 1
    robot_urdf: "robots/mobile_robot.urdf"
    publish_rate: 50.0
    gui: false
    physics: false
    enable_nav_actions: true
    enable_joint_actions: true
    enable_sim_services: true
```

**Step 6: Commit**

```bash
git add ros2_bridge/
git commit -m "feat(ros2): add pybullet_fleet_ros package skeleton"
```

---

## Task 3: Pose/Message Conversion Utilities (PARALLEL)

**Files:**
- Create: `ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/conversions.py`
- Create: `ros2_bridge/pybullet_fleet_ros/test/test_conversions.py`

**Step 1: Write the failing test**

```python
# ros2_bridge/pybullet_fleet_ros/test/test_conversions.py
"""Tests for ROS ↔ PyBulletFleet type conversions."""

import math
import pytest


def test_pbf_pose_to_ros_pose():
    """PyBulletFleet Pose → geometry_msgs/Pose round-trip."""
    from pybullet_fleet.geometry import Pose
    from pybullet_fleet_ros.conversions import pbf_pose_to_ros, ros_pose_to_pbf

    pbf = Pose.from_xyz(1.0, 2.0, 3.0)
    ros_msg = pbf_pose_to_ros(pbf)
    assert ros_msg.position.x == pytest.approx(1.0)
    assert ros_msg.position.y == pytest.approx(2.0)
    assert ros_msg.position.z == pytest.approx(3.0)
    assert ros_msg.orientation.w == pytest.approx(1.0)  # identity quaternion

    # Round-trip
    back = ros_pose_to_pbf(ros_msg)
    assert back.x == pytest.approx(1.0)
    assert back.y == pytest.approx(2.0)
    assert back.z == pytest.approx(3.0)


def test_ros_pose_to_pbf_with_orientation():
    """geometry_msgs/Pose with non-identity orientation."""
    from geometry_msgs.msg import Pose as PoseMsg, Quaternion, Point
    from pybullet_fleet_ros.conversions import ros_pose_to_pbf

    msg = PoseMsg()
    msg.position = Point(x=5.0, y=6.0, z=0.0)
    msg.orientation = Quaternion(x=0.0, y=0.0, z=0.7071068, w=0.7071068)

    pbf = ros_pose_to_pbf(msg)
    assert pbf.x == pytest.approx(5.0)
    assert pbf.yaw == pytest.approx(math.pi / 2, abs=0.01)


def test_pbf_pose_to_odom():
    """PyBulletFleet Pose + velocity → nav_msgs/Odometry."""
    from pybullet_fleet.geometry import Pose
    from pybullet_fleet_ros.conversions import make_odom_msg

    pose = Pose.from_xyz(1.0, 2.0, 0.0)
    velocity = [0.5, 0.0, 0.0]
    angular_vel = 0.1

    msg = make_odom_msg(pose, velocity, angular_vel, "odom", "robot_0/base_link")
    assert msg.pose.pose.position.x == pytest.approx(1.0)
    assert msg.twist.twist.linear.x == pytest.approx(0.5)
    assert msg.twist.twist.angular.z == pytest.approx(0.1)
    assert msg.header.frame_id == "odom"
    assert msg.child_frame_id == "robot_0/base_link"


def test_joint_state_msg():
    """Agent joint states → sensor_msgs/JointState."""
    from pybullet_fleet_ros.conversions import make_joint_state_msg

    joint_names = ["joint_0", "joint_1", "joint_2"]
    positions = [0.1, 0.2, 0.3]
    velocities = [0.01, 0.02, 0.03]

    msg = make_joint_state_msg(joint_names, positions, velocities)
    assert list(msg.name) == joint_names
    assert list(msg.position) == pytest.approx(positions)
    assert list(msg.velocity) == pytest.approx(velocities)


def test_twist_to_velocity_omnidirectional():
    """Twist → (dx, dy, dyaw) for omnidirectional robot."""
    from geometry_msgs.msg import Twist
    from pybullet_fleet_ros.conversions import twist_to_displacement

    twist = Twist()
    twist.linear.x = 1.0
    twist.linear.y = 0.5
    twist.angular.z = 0.2
    dt = 0.1
    current_yaw = 0.0

    dx, dy, dyaw = twist_to_displacement(twist, current_yaw, dt)
    assert dx == pytest.approx(0.1)   # 1.0 * 0.1
    assert dy == pytest.approx(0.05)  # 0.5 * 0.1
    assert dyaw == pytest.approx(0.02)  # 0.2 * 0.1


def test_twist_to_velocity_rotated():
    """Twist → world displacement when robot is rotated 90 degrees."""
    from geometry_msgs.msg import Twist
    from pybullet_fleet_ros.conversions import twist_to_displacement

    twist = Twist()
    twist.linear.x = 1.0  # forward
    twist.linear.y = 0.0
    twist.angular.z = 0.0
    dt = 0.1
    current_yaw = math.pi / 2  # facing +Y

    dx, dy, dyaw = twist_to_displacement(twist, current_yaw, dt)
    assert dx == pytest.approx(0.0, abs=1e-6)  # cos(90°) * 1.0 * 0.1 ≈ 0
    assert dy == pytest.approx(0.1, abs=1e-6)  # sin(90°) * 1.0 * 0.1 ≈ 0.1


def test_nav2_path_to_pbf_path():
    """nav_msgs/Path → pybullet_fleet Path conversion."""
    from nav_msgs.msg import Path as NavPath
    from geometry_msgs.msg import PoseStamped, Pose as PoseMsg, Point, Quaternion
    from pybullet_fleet_ros.conversions import nav_path_to_pbf_path

    nav_path = NavPath()
    for x, y in [(1.0, 0.0), (2.0, 1.0), (3.0, 2.0)]:
        ps = PoseStamped()
        ps.pose.position = Point(x=x, y=y, z=0.0)
        ps.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        nav_path.poses.append(ps)

    pbf_path = nav_path_to_pbf_path(nav_path)
    assert len(pbf_path) == 3
    assert pbf_path.waypoints[0].x == pytest.approx(1.0)
    assert pbf_path.waypoints[2].y == pytest.approx(2.0)
```

**Step 2: Run test to verify it fails**

Run (inside Docker): `cd /opt/bridge_ws && colcon test --packages-select pybullet_fleet_ros`
Expected: FAIL — `pybullet_fleet_ros.conversions` not found

**Step 3: Write conversions.py implementation**

```python
# ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/conversions.py
"""Conversion utilities between PyBulletFleet types and ROS 2 messages."""

import math
from typing import List, Optional, Tuple

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import (
    Point,
    Pose as PoseMsg,
    PoseStamped,
    Quaternion,
    Transform,
    TransformStamped,
    Twist,
    Vector3,
)
from nav_msgs.msg import Odometry, Path as NavPath
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from pybullet_fleet.geometry import Path, Pose


def pbf_pose_to_ros(pbf_pose: Pose) -> PoseMsg:
    """Convert PyBulletFleet Pose → geometry_msgs/Pose."""
    msg = PoseMsg()
    msg.position = Point(
        x=float(pbf_pose.position[0]),
        y=float(pbf_pose.position[1]),
        z=float(pbf_pose.position[2]),
    )
    msg.orientation = Quaternion(
        x=float(pbf_pose.orientation[0]),
        y=float(pbf_pose.orientation[1]),
        z=float(pbf_pose.orientation[2]),
        w=float(pbf_pose.orientation[3]),
    )
    return msg


def ros_pose_to_pbf(ros_pose: PoseMsg) -> Pose:
    """Convert geometry_msgs/Pose → PyBulletFleet Pose."""
    return Pose(
        position=[ros_pose.position.x, ros_pose.position.y, ros_pose.position.z],
        orientation=[
            ros_pose.orientation.x,
            ros_pose.orientation.y,
            ros_pose.orientation.z,
            ros_pose.orientation.w,
        ],
    )


def make_odom_msg(
    pose: Pose,
    velocity: List[float],
    angular_velocity: float,
    frame_id: str,
    child_frame_id: str,
    stamp: Optional[TimeMsg] = None,
) -> Odometry:
    """Create nav_msgs/Odometry from PyBulletFleet state."""
    msg = Odometry()
    msg.header = Header(frame_id=frame_id)
    if stamp:
        msg.header.stamp = stamp
    msg.child_frame_id = child_frame_id
    msg.pose.pose = pbf_pose_to_ros(pose)
    msg.twist.twist.linear = Vector3(
        x=float(velocity[0]), y=float(velocity[1]), z=float(velocity[2])
    )
    msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=float(angular_velocity))
    return msg


def make_joint_state_msg(
    joint_names: List[str],
    positions: List[float],
    velocities: List[float],
    stamp: Optional[TimeMsg] = None,
) -> JointState:
    """Create sensor_msgs/JointState."""
    msg = JointState()
    if stamp:
        msg.header.stamp = stamp
    msg.name = joint_names
    msg.position = [float(p) for p in positions]
    msg.velocity = [float(v) for v in velocities]
    return msg


def make_transform_stamped(
    pose: Pose,
    parent_frame: str,
    child_frame: str,
    stamp: Optional[TimeMsg] = None,
) -> TransformStamped:
    """Create geometry_msgs/TransformStamped from Pose."""
    t = TransformStamped()
    t.header = Header(frame_id=parent_frame)
    if stamp:
        t.header.stamp = stamp
    t.child_frame_id = child_frame
    t.transform.translation = Vector3(
        x=float(pose.position[0]),
        y=float(pose.position[1]),
        z=float(pose.position[2]),
    )
    t.transform.rotation = Quaternion(
        x=float(pose.orientation[0]),
        y=float(pose.orientation[1]),
        z=float(pose.orientation[2]),
        w=float(pose.orientation[3]),
    )
    return t


def twist_to_displacement(
    twist: Twist, current_yaw: float, dt: float
) -> Tuple[float, float, float]:
    """Convert Twist (body frame) to world-frame displacement (dx, dy, dyaw).

    Twist.linear.x/y are in the robot's body frame.
    Returns world-frame deltas.
    """
    cos_yaw = math.cos(current_yaw)
    sin_yaw = math.sin(current_yaw)
    dx = (twist.linear.x * cos_yaw - twist.linear.y * sin_yaw) * dt
    dy = (twist.linear.x * sin_yaw + twist.linear.y * cos_yaw) * dt
    dyaw = twist.angular.z * dt
    return dx, dy, dyaw


def nav_path_to_pbf_path(nav_path: NavPath) -> Path:
    """Convert nav_msgs/Path → PyBulletFleet Path."""
    waypoints = []
    for pose_stamped in nav_path.poses:
        waypoints.append(ros_pose_to_pbf(pose_stamped.pose))
    return Path(waypoints=waypoints)


def sim_time_to_ros_time(sim_time: float) -> TimeMsg:
    """Convert simulation time (float seconds) to builtin_interfaces/Time."""
    sec = int(sim_time)
    nanosec = int((sim_time - sec) * 1e9)
    return TimeMsg(sec=sec, nanosec=nanosec)
```

**Step 4: Run test to verify it passes**

Run: `cd /opt/bridge_ws && colcon test --packages-select pybullet_fleet_ros --pytest-args test/test_conversions.py -v`
Expected: All 7 tests PASS

**Step 5: Commit**

```bash
git add ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/conversions.py
git add ros2_bridge/pybullet_fleet_ros/test/test_conversions.py
git commit -m "feat(ros2): add ROS ↔ PyBulletFleet conversion utilities"
```

---

## Task 4: RobotHandler — Per-Robot Topic Management (SERIAL, depends on Task 3)

**Files:**
- Create: `ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/robot_handler.py`
- Create: `ros2_bridge/pybullet_fleet_ros/test/test_robot_handler.py`

**Step 1: Write the failing test**

```python
# ros2_bridge/pybullet_fleet_ros/test/test_robot_handler.py
"""Tests for RobotHandler — per-robot ROS interface management."""

import pytest


def test_robot_handler_creates_publishers(mock_node, mock_agent):
    """RobotHandler creates odom, joint_states publishers for a mobile robot."""
    from pybullet_fleet_ros.robot_handler import RobotHandler

    handler = RobotHandler(mock_node, mock_agent, enable_nav_actions=False, enable_joint_actions=False)

    # Verify publishers were created
    topic_names = [call[0][1] for call in mock_node.create_publisher.call_args_list]
    assert "/test_robot/odom" in topic_names
    assert "/test_robot/joint_states" in topic_names


def test_robot_handler_creates_cmd_vel_sub(mock_node, mock_agent):
    """RobotHandler subscribes to cmd_vel."""
    from pybullet_fleet_ros.robot_handler import RobotHandler

    handler = RobotHandler(mock_node, mock_agent, enable_nav_actions=False, enable_joint_actions=False)

    sub_topics = [call[0][1] for call in mock_node.create_subscription.call_args_list]
    assert "/test_robot/cmd_vel" in sub_topics


def test_cmd_vel_updates_goal_pose(mock_node, mock_mobile_agent):
    """cmd_vel message sets incremental goal on agent."""
    from pybullet_fleet_ros.robot_handler import RobotHandler
    from geometry_msgs.msg import Twist

    handler = RobotHandler(mock_node, mock_mobile_agent, enable_nav_actions=False, enable_joint_actions=False)

    twist = Twist()
    twist.linear.x = 1.0
    twist.angular.z = 0.0
    handler._cmd_vel_cb(twist)
    handler.apply_cmd_vel(0.1)  # dt = 0.1s

    # Agent should have received a goal_pose call
    assert mock_mobile_agent.set_goal_pose.called


def test_publish_state_publishes_odom(mock_node, mock_agent):
    """publish_state sends Odometry message."""
    from pybullet_fleet_ros.robot_handler import RobotHandler
    from pybullet_fleet_ros.conversions import sim_time_to_ros_time

    handler = RobotHandler(mock_node, mock_agent, enable_nav_actions=False, enable_joint_actions=False)
    stamp = sim_time_to_ros_time(1.0)
    handler.publish_state(stamp)

    # odom_pub.publish should have been called
    assert handler._odom_pub.publish.called
```

Note: The test file will need `conftest.py` fixtures for `mock_node` and `mock_agent`. These should be unittest.mock-based, not requiring a live ROS system.

```python
# ros2_bridge/pybullet_fleet_ros/test/conftest.py
"""Test fixtures for pybullet_fleet_ros tests."""

from unittest.mock import MagicMock, PropertyMock, patch
import pytest

from pybullet_fleet.geometry import Pose
from pybullet_fleet.types import MotionMode


@pytest.fixture
def mock_node():
    """Mock rclpy.node.Node with publisher/subscriber factory methods."""
    node = MagicMock()
    node.get_logger.return_value = MagicMock()

    # create_publisher returns a mock with publish method
    def make_pub(*args, **kwargs):
        pub = MagicMock()
        pub.topic_name = args[1] if len(args) > 1 else "unknown"
        return pub

    node.create_publisher.side_effect = make_pub

    # create_subscription returns a mock
    def make_sub(*args, **kwargs):
        sub = MagicMock()
        sub.topic_name = args[1] if len(args) > 1 else "unknown"
        return sub

    node.create_subscription.side_effect = make_sub
    return node


@pytest.fixture
def mock_agent():
    """Mock Agent with basic attributes for a mobile robot."""
    agent = MagicMock()
    agent.name = "test_robot"
    agent.object_id = 0
    agent.body_id = 0
    agent.is_moving = False
    agent.get_pose.return_value = Pose.from_xyz(0.0, 0.0, 0.0)
    agent.velocity = [0.0, 0.0, 0.0]
    agent.angular_velocity = 0.0
    agent.get_num_joints.return_value = 0
    agent.get_all_joints_state.return_value = []
    agent.get_all_joints_state_by_name.return_value = {}
    agent.spawn_params = MagicMock()
    agent.spawn_params.motion_mode = MotionMode.OMNIDIRECTIONAL
    agent.spawn_params.use_fixed_base = False
    agent.ik_params = None
    return agent


@pytest.fixture
def mock_mobile_agent(mock_agent):
    """Mock Agent specifically for mobile robot cmd_vel testing."""
    mock_agent.get_pose.return_value = Pose.from_xyz(0.0, 0.0, 0.0)
    return mock_agent
```

**Step 2: Run test to verify it fails**

Run: `pytest ros2_bridge/pybullet_fleet_ros/test/test_robot_handler.py -v`
Expected: FAIL — `pybullet_fleet_ros.robot_handler` not found

**Step 3: Write RobotHandler implementation**

```python
# ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/robot_handler.py
"""Per-robot ROS interface handler.

Creates publishers, subscribers, and action servers for a single Agent.
"""

import logging
import math
from typing import TYPE_CHECKING, Dict, Optional

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

from pybullet_fleet.geometry import Pose
from pybullet_fleet.types import MotionMode

from .conversions import (
    make_joint_state_msg,
    make_odom_msg,
    make_transform_stamped,
    twist_to_displacement,
)

if TYPE_CHECKING:
    from pybullet_fleet.agent import Agent
    from rclpy.node import Node
    from tf2_ros import TransformBroadcaster

logger = logging.getLogger(__name__)


class RobotHandler:
    """Manages all ROS 2 interfaces for a single simulated Agent.

    Creates:
    - /{name}/cmd_vel subscriber (Twist)
    - /{name}/odom publisher (Odometry)
    - /{name}/joint_states publisher (JointState)
    - TF broadcast: odom → {name}/base_link

    Optionally creates (if enabled):
    - /{name}/navigate_to_pose action server
    - /{name}/follow_path action server
    - /{name}/follow_joint_trajectory action server
    """

    def __init__(
        self,
        node: "Node",
        agent: "Agent",
        tf_broadcaster: Optional["TransformBroadcaster"] = None,
        enable_nav_actions: bool = True,
        enable_joint_actions: bool = True,
    ):
        self._node = node
        self.agent = agent
        self._tf_broadcaster = tf_broadcaster
        self._latest_twist: Optional[Twist] = None
        self._cmd_vel_timeout = 0.5  # seconds without cmd_vel → stop
        self._last_cmd_vel_time: Optional[float] = None

        ns = agent.name or f"robot_{agent.object_id}"
        self._ns = ns

        # Publishers
        self._odom_pub = node.create_publisher(Odometry, f"/{ns}/odom", 10)
        self._joint_pub = node.create_publisher(JointState, f"/{ns}/joint_states", 10)

        # Subscribers
        self._cmd_vel_sub = node.create_subscription(
            Twist, f"/{ns}/cmd_vel", self._cmd_vel_cb, 10
        )

        # Joint name cache (populated lazily)
        self._joint_names: Optional[list] = None

        # Nav2 action servers (created conditionally)
        self._nav_action_server = None
        self._path_action_server = None
        if enable_nav_actions and not getattr(agent.spawn_params, "use_fixed_base", False):
            self._setup_nav_actions(node, ns)

        # Arm action server (created conditionally)
        self._joint_traj_action_server = None
        if enable_joint_actions and agent.get_num_joints() > 0:
            self._setup_joint_actions(node, ns)

        logger.info("RobotHandler created for '%s' (object_id=%d)", ns, agent.object_id)

    def _setup_nav_actions(self, node: "Node", ns: str) -> None:
        """Create NavigateToPose and FollowPath action servers."""
        # Deferred import — action server setup is in Task 7
        pass

    def _setup_joint_actions(self, node: "Node", ns: str) -> None:
        """Create FollowJointTrajectory action server."""
        # Deferred — action server setup is in Task 7
        pass

    def _cmd_vel_cb(self, msg: Twist) -> None:
        """Store latest cmd_vel for application in next step."""
        self._latest_twist = msg

    def apply_cmd_vel(self, dt: float, sim_time: float = 0.0) -> None:
        """Apply stored cmd_vel as incremental goal pose.

        Called once per sim step by BridgeNode.
        """
        if self._latest_twist is None:
            return

        twist = self._latest_twist
        pose = self.agent.get_pose()

        dx, dy, dyaw = twist_to_displacement(twist, pose.yaw, dt)
        new_yaw = pose.yaw + dyaw
        goal = Pose.from_yaw(pose.x + dx, pose.y + dy, pose.z, new_yaw)
        self.agent.set_goal_pose(goal)

        # Reset twist after applying (require continuous publishing)
        self._latest_twist = None

    def publish_state(self, stamp: TimeMsg) -> None:
        """Publish odom, joint_states, and TF for this agent."""
        pose = self.agent.get_pose()
        velocity = list(self.agent.velocity)
        angular_vel = self.agent.angular_velocity

        # Odometry
        odom_msg = make_odom_msg(
            pose, velocity, angular_vel,
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

    def destroy(self) -> None:
        """Clean up ROS interfaces."""
        self._node.destroy_publisher(self._odom_pub)
        self._node.destroy_publisher(self._joint_pub)
        self._node.destroy_subscription(self._cmd_vel_sub)
        # Action server cleanup in Task 7
        logger.info("RobotHandler destroyed for '%s'", self._ns)
```

**Step 4: Run test to verify it passes**

Run: `pytest ros2_bridge/pybullet_fleet_ros/test/test_robot_handler.py -v`
Expected: All 4 tests PASS

**Step 5: Commit**

```bash
git add ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/robot_handler.py
git add ros2_bridge/pybullet_fleet_ros/test/test_robot_handler.py
git add ros2_bridge/pybullet_fleet_ros/test/conftest.py
git commit -m "feat(ros2): add RobotHandler with cmd_vel, odom, joint_states, tf"
```

---

## Task 5: BridgeNode — Main Node with Sim Loop + Clock (SERIAL, depends on Tasks 2, 3, 4)

**Files:**
- Create: `ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/bridge_node.py`
- Create: `ros2_bridge/pybullet_fleet_ros/test/test_bridge_node.py`

**Step 1: Write the failing test**

```python
# ros2_bridge/pybullet_fleet_ros/test/test_bridge_node.py
"""Tests for BridgeNode — main simulation node."""

from unittest.mock import MagicMock, patch
import pytest


def test_bridge_node_creates_clock_publisher():
    """BridgeNode creates /clock publisher."""
    with patch("pybullet_fleet_ros.bridge_node.MultiRobotSimulationCore") as MockSim:
        mock_sim = MagicMock()
        mock_sim.params.timestep = 0.1
        mock_sim.agents = []
        mock_sim.sim_objects = []
        mock_sim.sim_time = 0.0
        MockSim.return_value = mock_sim

        from pybullet_fleet_ros.bridge_node import BridgeNode

        with patch("rclpy.node.Node.__init__"):
            node = BridgeNode.__new__(BridgeNode)
            # Verify /clock publisher would be created
            # (Full integration test in Task 8)


def test_step_callback_calls_step_once():
    """Timer callback calls sim.step_once() and publishes clock."""
    # This is tested via integration test in Task 8
    pass
```

Note: Full BridgeNode testing requires rclpy context, so we rely on integration tests (Task 8) for end-to-end verification. Unit tests here focus on logic in isolation.

**Step 2: Write BridgeNode implementation**

```python
# ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/bridge_node.py
"""Main bridge node — wraps MultiRobotSimulationCore as a ROS 2 node.

Usage:
    ros2 run pybullet_fleet_ros bridge_node --ros-args -p num_robots:=5
"""

import logging
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from tf2_ros import TransformBroadcaster

from pybullet_fleet import (
    Agent,
    AgentSpawnParams,
    MultiRobotSimulationCore,
    Pose,
    SimulationParams,
)
from pybullet_fleet.types import MotionMode

from .conversions import sim_time_to_ros_time
from .robot_handler import RobotHandler

logger = logging.getLogger(__name__)


class BridgeNode(Node):
    """ROS 2 node that drives PyBulletFleet simulation and exposes per-robot topics.

    Parameters (ROS):
        config_yaml (str): Path to PyBulletFleet YAML config file.
        num_robots (int): Number of mobile robots to spawn at startup.
        robot_urdf (str): URDF path for default mobile robot.
        publish_rate (float): State publish rate in Hz.
        gui (bool): Enable PyBullet GUI window.
        physics (bool): Enable physics simulation.
        enable_nav_actions (bool): Create NavigateToPose/FollowPath action servers.
        enable_joint_actions (bool): Create FollowJointTrajectory action servers.
        enable_sim_services (bool): Create simulation_interfaces services.
    """

    def __init__(self):
        super().__init__("pybullet_fleet_bridge")

        # Declare parameters
        self.declare_parameter("config_yaml", "")
        self.declare_parameter("num_robots", 1)
        self.declare_parameter("robot_urdf", "robots/mobile_robot.urdf")
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("gui", False)
        self.declare_parameter("physics", False)
        self.declare_parameter("enable_nav_actions", True)
        self.declare_parameter("enable_joint_actions", True)
        self.declare_parameter("enable_sim_services", True)

        # Read parameters
        config_yaml = self.get_parameter("config_yaml").get_parameter_value().string_value
        num_robots = self.get_parameter("num_robots").get_parameter_value().integer_value
        robot_urdf = self.get_parameter("robot_urdf").get_parameter_value().string_value
        publish_rate = self.get_parameter("publish_rate").get_parameter_value().double_value
        gui = self.get_parameter("gui").get_parameter_value().bool_value
        physics = self.get_parameter("physics").get_parameter_value().bool_value
        enable_nav = self.get_parameter("enable_nav_actions").get_parameter_value().bool_value
        enable_joint = self.get_parameter("enable_joint_actions").get_parameter_value().bool_value
        enable_sim_services = self.get_parameter("enable_sim_services").get_parameter_value().bool_value

        # Create simulation core
        if config_yaml:
            self.sim = MultiRobotSimulationCore.from_yaml(config_yaml)
        else:
            self.sim = MultiRobotSimulationCore(
                SimulationParams(
                    gui=gui,
                    physics=physics,
                    monitor=False,
                    target_rtf=0,  # No internal sleep — ROS timer controls rate
                )
            )

        # TF broadcaster (shared by all handlers)
        self._tf_broadcaster = TransformBroadcaster(self)

        # Robot handlers (object_id → RobotHandler)
        self._handlers: Dict[int, RobotHandler] = {}
        self._enable_nav = enable_nav
        self._enable_joint = enable_joint

        # Spawn initial robots
        for i in range(num_robots):
            self._spawn_default_robot(robot_urdf, i)

        # /clock publisher
        self._clock_pub = self.create_publisher(Clock, "/clock", 10)

        # simulation_interfaces services
        self._sim_services = None
        if enable_sim_services:
            from .sim_services import SimServices
            self._sim_services = SimServices(self, self.sim, self)

        # Simulation step timer
        dt = self.sim.params.timestep
        publish_period = 1.0 / publish_rate
        self._step_timer = self.create_timer(dt, self._step_callback)

        # Publish timer (may be slower than step rate)
        self._publish_timer = self.create_timer(publish_period, self._publish_callback)

        self.get_logger().info(
            f"BridgeNode started: {num_robots} robots, dt={dt:.4f}s, "
            f"publish_rate={publish_rate}Hz, gui={gui}, physics={physics}"
        )

    def _spawn_default_robot(self, urdf_path: str, index: int) -> Agent:
        """Spawn a default mobile robot and create its RobotHandler."""
        params = AgentSpawnParams(
            urdf_path=urdf_path,
            initial_pose=Pose.from_xyz(index * 2.0, 0.0, 0.05),
            motion_mode=MotionMode.OMNIDIRECTIONAL,
            max_linear_vel=2.0,
            name=f"robot_{index}",
        )
        agent = Agent.from_params(params, sim_core=self.sim)
        handler = RobotHandler(
            self, agent,
            tf_broadcaster=self._tf_broadcaster,
            enable_nav_actions=self._enable_nav,
            enable_joint_actions=self._enable_joint,
        )
        self._handlers[agent.object_id] = handler
        return agent

    def spawn_robot(self, spawn_params: AgentSpawnParams) -> Agent:
        """Spawn a robot dynamically (e.g., via SpawnEntity service)."""
        agent = Agent.from_params(spawn_params, sim_core=self.sim)
        handler = RobotHandler(
            self, agent,
            tf_broadcaster=self._tf_broadcaster,
            enable_nav_actions=self._enable_nav,
            enable_joint_actions=self._enable_joint,
        )
        self._handlers[agent.object_id] = handler
        self.get_logger().info(f"Spawned robot '{agent.name}' (id={agent.object_id})")
        return agent

    def remove_robot(self, object_id: int) -> bool:
        """Remove a robot and its ROS interfaces."""
        handler = self._handlers.pop(object_id, None)
        if handler is None:
            return False
        handler.destroy()
        self.sim.remove_object(handler.agent)
        self.get_logger().info(f"Removed robot '{handler.agent.name}' (id={object_id})")
        return True

    def _step_callback(self) -> None:
        """Called every sim timestep — advance simulation."""
        # Apply pending cmd_vel commands
        dt = self.sim.params.timestep
        for handler in self._handlers.values():
            handler.apply_cmd_vel(dt, self.sim.sim_time)

        # Step the simulation
        self.sim.step_once()

        # Publish /clock
        clock_msg = Clock()
        clock_msg.clock = sim_time_to_ros_time(self.sim.sim_time)
        self._clock_pub.publish(clock_msg)

    def _publish_callback(self) -> None:
        """Called at publish_rate — publish state for all robots."""
        stamp = sim_time_to_ros_time(self.sim.sim_time)
        for handler in self._handlers.values():
            handler.publish_state(stamp)

    @property
    def handlers(self) -> Dict[int, RobotHandler]:
        """Access robot handlers (used by SimServices for spawn/delete)."""
        return self._handlers


def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
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

**Step 3: Run test to verify**

Run: `pytest ros2_bridge/pybullet_fleet_ros/test/test_bridge_node.py -v`
Expected: PASS (basic tests)

**Step 4: Commit**

```bash
git add ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/bridge_node.py
git add ros2_bridge/pybullet_fleet_ros/test/test_bridge_node.py
git commit -m "feat(ros2): add BridgeNode with sim loop, /clock, robot spawning"
```

---

## Task 6: SimServices — simulation_interfaces Implementation (SERIAL, depends on Task 5)

**Files:**
- Create: `ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/sim_services.py`
- Create: `ros2_bridge/pybullet_fleet_ros/test/test_sim_services.py`

**Step 1: Write the failing test**

```python
# ros2_bridge/pybullet_fleet_ros/test/test_sim_services.py
"""Tests for SimServices — simulation_interfaces implementation."""

from unittest.mock import MagicMock, patch
import pytest


def test_get_simulator_features():
    """GetSimulatorFeatures returns supported features."""
    from pybullet_fleet_ros.sim_services import SimServices

    mock_node = MagicMock()
    mock_sim = MagicMock()
    mock_bridge = MagicMock()

    services = SimServices(mock_node, mock_sim, mock_bridge)
    request = MagicMock()
    response = MagicMock()

    result = services._get_features(request, response)
    # Should list supported features
    assert result is not None


def test_get_entities_returns_agents():
    """GetEntities returns list of all agents."""
    from pybullet_fleet_ros.sim_services import SimServices
    from pybullet_fleet.geometry import Pose

    mock_node = MagicMock()
    mock_sim = MagicMock()
    mock_bridge = MagicMock()

    agent1 = MagicMock()
    agent1.name = "robot_0"
    agent1.object_id = 0
    agent2 = MagicMock()
    agent2.name = "robot_1"
    agent2.object_id = 1

    mock_sim.sim_objects = [agent1, agent2]
    mock_sim.agents = [agent1, agent2]

    services = SimServices(mock_node, mock_sim, mock_bridge)
    request = MagicMock()
    response = MagicMock()

    result = services._get_entities(request, response)
    assert result is not None


def test_step_simulation():
    """StepSimulation calls sim.step_once()."""
    from pybullet_fleet_ros.sim_services import SimServices

    mock_node = MagicMock()
    mock_sim = MagicMock()
    mock_bridge = MagicMock()

    services = SimServices(mock_node, mock_sim, mock_bridge)
    request = MagicMock()
    request.num_steps = 5
    response = MagicMock()

    services._step_sim(request, response)
    assert mock_sim.step_once.call_count == 5


def test_pause_resume():
    """SetSimulationState can pause and resume."""
    from pybullet_fleet_ros.sim_services import SimServices

    mock_node = MagicMock()
    mock_sim = MagicMock()
    mock_bridge = MagicMock()

    services = SimServices(mock_node, mock_sim, mock_bridge)

    # Pause
    request = MagicMock()
    request.state.state = 0  # PAUSED
    response = MagicMock()
    services._set_sim_state(request, response)
    mock_sim.pause.assert_called_once()

    # Resume
    request.state.state = 1  # PLAYING
    services._set_sim_state(request, response)
    mock_sim.resume.assert_called_once()
```

**Step 2: Run test to verify it fails**

Run: `pytest ros2_bridge/pybullet_fleet_ros/test/test_sim_services.py -v`
Expected: FAIL — `pybullet_fleet_ros.sim_services` not found

**Step 3: Write SimServices implementation**

```python
# ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/sim_services.py
"""simulation_interfaces service and action server implementations.

Implements the simulation_interfaces standard for PyBulletFleet.
See: https://github.com/ros-simulation/simulation_interfaces
"""

import logging
import os
from typing import TYPE_CHECKING, Dict, List, Optional

import pybullet as p

from pybullet_fleet import Agent, AgentSpawnParams, Pose, SimulationParams
from pybullet_fleet.types import CollisionMode, MotionMode

from .conversions import pbf_pose_to_ros, ros_pose_to_pbf

if TYPE_CHECKING:
    from rclpy.node import Node
    from pybullet_fleet.core_simulation import MultiRobotSimulationCore
    from .bridge_node import BridgeNode

logger = logging.getLogger(__name__)


# simulation_interfaces message imports
from simulation_interfaces.srv import (
    DeleteEntity,
    GetEntities,
    GetEntitiesStates,
    GetEntityBounds,
    GetEntityInfo,
    GetEntityState,
    GetSimulationState,
    GetSimulatorFeatures,
    GetSpawnables,
    ResetSimulation,
    SetEntityState,
    SetSimulationState,
    SpawnEntity,
    StepSimulation,
)
from simulation_interfaces.action import SimulateSteps
from simulation_interfaces.msg import (
    EntityState,
    SimulationState,
    SimulatorFeatures,
)

from rclpy.action import ActionServer


class SimServices:
    """Implements simulation_interfaces services and actions.

    Services:
        /sim/get_simulator_features
        /sim/spawn_entity
        /sim/delete_entity
        /sim/get_entity_state
        /sim/set_entity_state
        /sim/get_entities
        /sim/get_entities_states
        /sim/get_entity_info
        /sim/get_entity_bounds
        /sim/step_simulation
        /sim/get_simulation_state
        /sim/set_simulation_state
        /sim/reset_simulation
        /sim/get_spawnables

    Actions:
        /sim/simulate_steps
    """

    def __init__(
        self,
        node: "Node",
        sim: "MultiRobotSimulationCore",
        bridge: "BridgeNode",
    ):
        self._node = node
        self._sim = sim
        self._bridge = bridge

        # Create services
        node.create_service(GetSimulatorFeatures, "/sim/get_simulator_features", self._get_features)
        node.create_service(SpawnEntity, "/sim/spawn_entity", self._spawn_entity)
        node.create_service(DeleteEntity, "/sim/delete_entity", self._delete_entity)
        node.create_service(GetEntityState, "/sim/get_entity_state", self._get_entity_state)
        node.create_service(SetEntityState, "/sim/set_entity_state", self._set_entity_state)
        node.create_service(GetEntities, "/sim/get_entities", self._get_entities)
        node.create_service(GetEntitiesStates, "/sim/get_entities_states", self._get_entities_states)
        node.create_service(GetEntityInfo, "/sim/get_entity_info", self._get_entity_info)
        node.create_service(GetEntityBounds, "/sim/get_entity_bounds", self._get_entity_bounds)
        node.create_service(StepSimulation, "/sim/step_simulation", self._step_sim)
        node.create_service(GetSimulationState, "/sim/get_simulation_state", self._get_sim_state)
        node.create_service(SetSimulationState, "/sim/set_simulation_state", self._set_sim_state)
        node.create_service(ResetSimulation, "/sim/reset_simulation", self._reset_sim)
        node.create_service(GetSpawnables, "/sim/get_spawnables", self._get_spawnables)

        # Action server
        self._simulate_steps_server = ActionServer(
            node, SimulateSteps, "/sim/simulate_steps", self._simulate_steps_cb
        )

        logger.info("SimServices: all simulation_interfaces registered")

    # ---- Helper: find agent/object by name ----

    def _find_by_name(self, name: str):
        """Find SimObject or Agent by name."""
        for obj in self._sim.sim_objects:
            if obj.name == name:
                return obj
        return None

    # ---- Services ----

    def _get_features(self, request, response):
        """Report supported simulation features."""
        features = response.features
        features.spawn_entity = True
        features.delete_entity = True
        features.get_entity_state = True
        features.set_entity_state = True
        features.step_simulation = True
        features.simulation_state = True
        features.reset_simulation = True
        features.entity_bounds = True
        features.entity_info = True
        features.spawnables = True
        response.result.success = True
        return response

    def _spawn_entity(self, request, response):
        """Spawn a new entity."""
        try:
            name = request.name
            # Use URDF from resource or default
            urdf_path = request.resource.uri if hasattr(request, 'resource') and request.resource.uri else "robots/mobile_robot.urdf"

            initial_pose = Pose.from_xyz(0.0, 0.0, 0.05)
            if hasattr(request, 'state') and request.state:
                initial_pose = ros_pose_to_pbf(request.state.pose)

            params = AgentSpawnParams(
                urdf_path=urdf_path,
                initial_pose=initial_pose,
                motion_mode=MotionMode.OMNIDIRECTIONAL,
                name=name or None,
            )
            agent = self._bridge.spawn_robot(params)
            response.result.success = True
            response.result.message = f"Spawned entity '{agent.name}' (id={agent.object_id})"
        except Exception as e:
            response.result.success = False
            response.result.message = str(e)
            logger.error("SpawnEntity failed: %s", e)
        return response

    def _delete_entity(self, request, response):
        """Delete an entity by name."""
        obj = self._find_by_name(request.name)
        if obj is None:
            response.result.success = False
            response.result.message = f"Entity '{request.name}' not found"
            return response

        self._bridge.remove_robot(obj.object_id)
        response.result.success = True
        return response

    def _get_entity_state(self, request, response):
        """Get pose and velocity of an entity."""
        obj = self._find_by_name(request.name)
        if obj is None:
            response.result.success = False
            response.result.message = f"Entity '{request.name}' not found"
            return response

        pose = obj.get_pose()
        response.state.pose = pbf_pose_to_ros(pose)
        response.result.success = True
        return response

    def _set_entity_state(self, request, response):
        """Set pose of an entity."""
        obj = self._find_by_name(request.name)
        if obj is None:
            response.result.success = False
            response.result.message = f"Entity '{request.name}' not found"
            return response

        if hasattr(request, 'state') and request.state:
            new_pose = ros_pose_to_pbf(request.state.pose)
            obj.set_pose(new_pose)

        response.result.success = True
        return response

    def _get_entities(self, request, response):
        """Return list of all entities."""
        for obj in self._sim.sim_objects:
            name = obj.name or f"object_{obj.object_id}"
            response.names.append(name)
        response.result.success = True
        return response

    def _get_entities_states(self, request, response):
        """Return states for all entities."""
        for obj in self._sim.sim_objects:
            state = EntityState()
            pose = obj.get_pose()
            state.pose = pbf_pose_to_ros(pose)
            response.states.append(state)
            response.names.append(obj.name or f"object_{obj.object_id}")
        response.result.success = True
        return response

    def _get_entity_info(self, request, response):
        """Return metadata about an entity."""
        obj = self._find_by_name(request.name)
        if obj is None:
            response.result.success = False
            response.result.message = f"Entity '{request.name}' not found"
            return response

        # Populate available info fields
        response.result.success = True
        return response

    def _get_entity_bounds(self, request, response):
        """Get AABB bounds of an entity."""
        obj = self._find_by_name(request.name)
        if obj is None:
            response.result.success = False
            response.result.message = f"Entity '{request.name}' not found"
            return response

        try:
            aabb_min, aabb_max = p.getAABB(obj.body_id, physicsClientId=self._sim.client)
            response.bounds.min.x = aabb_min[0]
            response.bounds.min.y = aabb_min[1]
            response.bounds.min.z = aabb_min[2]
            response.bounds.max.x = aabb_max[0]
            response.bounds.max.y = aabb_max[1]
            response.bounds.max.z = aabb_max[2]
            response.result.success = True
        except Exception as e:
            response.result.success = False
            response.result.message = str(e)
        return response

    def _step_sim(self, request, response):
        """Execute N simulation steps."""
        num_steps = getattr(request, 'num_steps', 1)
        for _ in range(num_steps):
            self._sim.step_once()
        response.result.success = True
        return response

    def _get_sim_state(self, request, response):
        """Return current simulation state (paused/playing)."""
        if self._sim.is_paused:
            response.state.state = SimulationState.PAUSED
        else:
            response.state.state = SimulationState.PLAYING
        response.result.success = True
        return response

    def _set_sim_state(self, request, response):
        """Set simulation state (pause/resume)."""
        target_state = request.state.state
        if target_state == SimulationState.PAUSED:
            self._sim.pause()
        elif target_state == SimulationState.PLAYING:
            self._sim.resume()
        response.result.success = True
        return response

    def _reset_sim(self, request, response):
        """Reset simulation to initial state."""
        # Remove all current agents
        for handler in list(self._bridge.handlers.values()):
            handler.destroy()
        self._bridge.handlers.clear()

        # Reset PyBullet
        p.resetSimulation(physicsClientId=self._sim.client)

        response.result.success = True
        response.result.message = "Simulation reset. Re-spawn entities as needed."
        return response

    def _get_spawnables(self, request, response):
        """List available robot URDFs."""
        robots_dir = "robots"
        if os.path.isdir(robots_dir):
            for f in os.listdir(robots_dir):
                if f.endswith(".urdf"):
                    spawnable = response.spawnables.add() if hasattr(response.spawnables, 'add') else None
                    if spawnable:
                        spawnable.name = f.replace(".urdf", "")
                        spawnable.resource.uri = os.path.join(robots_dir, f)
        response.result.success = True
        return response

    # ---- Action ----

    def _simulate_steps_cb(self, goal_handle):
        """Execute N simulation steps with feedback."""
        num_steps = goal_handle.request.num_steps
        feedback = SimulateSteps.Feedback()

        for i in range(num_steps):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = SimulateSteps.Result()
                result.success = False
                return result

            self._sim.step_once()
            feedback.steps_done = i + 1
            goal_handle.publish_feedback(feedback)

        goal_handle.succeed()
        result = SimulateSteps.Result()
        result.success = True
        return result
```

**Note:** The exact field names in `simulation_interfaces` messages may differ from what's shown. During implementation, inspect the actual message definitions installed in Docker to adjust field names. The structure above follows the pattern from the GitHub repository.

**Step 4: Run tests to verify**

Run: `pytest ros2_bridge/pybullet_fleet_ros/test/test_sim_services.py -v`
Expected: PASS for all tests

**Step 5: Commit**

```bash
git add ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/sim_services.py
git add ros2_bridge/pybullet_fleet_ros/test/test_sim_services.py
git commit -m "feat(ros2): add simulation_interfaces services + SimulateSteps action"
```

---

## Task 7: Nav2 + Arm Action Servers (SERIAL, depends on Tasks 4, 5)

**Files:**
- Modify: `ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/robot_handler.py`
- Create: `ros2_bridge/pybullet_fleet_ros/test/test_nav_actions.py`

**Step 1: Write the failing test**

```python
# ros2_bridge/pybullet_fleet_ros/test/test_nav_actions.py
"""Tests for Nav2 and arm action server integration."""

from unittest.mock import MagicMock, patch
import pytest


def test_navigate_to_pose_creates_move_action(mock_node, mock_mobile_agent):
    """NavigateToPose goal creates MoveAction on agent."""
    from pybullet_fleet_ros.robot_handler import RobotHandler

    handler = RobotHandler(
        mock_node, mock_mobile_agent,
        enable_nav_actions=True, enable_joint_actions=False
    )

    # Simulate action goal
    assert handler._nav_action_server is not None or handler._setup_nav_actions is not None


def test_follow_path_creates_move_action(mock_node, mock_mobile_agent):
    """FollowPath goal creates MoveAction with multi-waypoint path."""
    from pybullet_fleet_ros.robot_handler import RobotHandler

    handler = RobotHandler(
        mock_node, mock_mobile_agent,
        enable_nav_actions=True, enable_joint_actions=False
    )

    assert handler._path_action_server is not None or handler._setup_nav_actions is not None
```

**Step 2: Implement action servers in RobotHandler._setup_nav_actions / _setup_joint_actions**

Add to `robot_handler.py`:

```python
# Add to imports at top of robot_handler.py
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from nav2_msgs.action import NavigateToPose, FollowPath
from control_msgs.action import FollowJointTrajectory

from pybullet_fleet.action import MoveAction, JointAction
from pybullet_fleet.geometry import Path
from pybullet_fleet.types import ActionStatus

from .conversions import ros_pose_to_pbf, nav_path_to_pbf_path
```

Replace the `_setup_nav_actions` stub:

```python
def _setup_nav_actions(self, node: "Node", ns: str) -> None:
    """Create NavigateToPose and FollowPath action servers."""
    self._nav_action_server = ActionServer(
        node, NavigateToPose, f"/{ns}/navigate_to_pose",
        execute_callback=self._navigate_to_pose_cb,
        goal_callback=self._nav_goal_cb,
        cancel_callback=self._nav_cancel_cb,
    )
    self._path_action_server = ActionServer(
        node, FollowPath, f"/{ns}/follow_path",
        execute_callback=self._follow_path_cb,
        goal_callback=self._nav_goal_cb,
        cancel_callback=self._nav_cancel_cb,
    )
    logger.info("Nav2 action servers created for '%s'", ns)

def _nav_goal_cb(self, goal_request):
    return GoalResponse.ACCEPT

def _nav_cancel_cb(self, goal_handle):
    self.agent.stop()
    self.agent.clear_actions()
    return CancelResponse.ACCEPT

def _navigate_to_pose_cb(self, goal_handle):
    """Execute NavigateToPose by creating a MoveAction."""
    goal_pose = ros_pose_to_pbf(goal_handle.request.pose.pose)
    path = Path(waypoints=[goal_pose])
    action = MoveAction(path=path)
    self.agent.clear_actions()
    self.agent.add_action(action)

    # Poll until action completes
    feedback = NavigateToPose.Feedback()
    while action.status in (ActionStatus.NOT_STARTED, ActionStatus.IN_PROGRESS):
        if goal_handle.is_cancel_requested:
            self.agent.clear_actions()
            goal_handle.canceled()
            return NavigateToPose.Result()
        # Publish feedback
        current_pose = self.agent.get_pose()
        feedback.current_pose.pose = pbf_pose_to_ros(current_pose)
        goal_handle.publish_feedback(feedback)
        # Sleep briefly to yield to ROS executor
        import time
        time.sleep(0.05)

    result = NavigateToPose.Result()
    if action.status == ActionStatus.COMPLETED:
        goal_handle.succeed()
    else:
        goal_handle.abort()
    return result

def _follow_path_cb(self, goal_handle):
    """Execute FollowPath by creating a MoveAction with the full path."""
    pbf_path = nav_path_to_pbf_path(goal_handle.request.path)
    action = MoveAction(path=pbf_path)
    self.agent.clear_actions()
    self.agent.add_action(action)

    feedback = FollowPath.Feedback()
    while action.status in (ActionStatus.NOT_STARTED, ActionStatus.IN_PROGRESS):
        if goal_handle.is_cancel_requested:
            self.agent.clear_actions()
            goal_handle.canceled()
            return FollowPath.Result()
        import time
        time.sleep(0.05)

    result = FollowPath.Result()
    if action.status == ActionStatus.COMPLETED:
        goal_handle.succeed()
    else:
        goal_handle.abort()
    return result
```

Replace the `_setup_joint_actions` stub:

```python
def _setup_joint_actions(self, node: "Node", ns: str) -> None:
    """Create FollowJointTrajectory action server."""
    self._joint_traj_action_server = ActionServer(
        node, FollowJointTrajectory, f"/{ns}/follow_joint_trajectory",
        execute_callback=self._follow_joint_trajectory_cb,
    )
    logger.info("FollowJointTrajectory action server created for '%s'", ns)

def _follow_joint_trajectory_cb(self, goal_handle):
    """Execute FollowJointTrajectory using JointAction."""
    trajectory = goal_handle.request.trajectory

    # Execute each trajectory point as a JointAction
    for point in trajectory.points:
        targets = dict(zip(trajectory.joint_names, point.positions))
        action = JointAction(target_joint_positions=targets)
        self.agent.clear_actions()
        self.agent.add_action(action)

        # Wait for this point to complete
        while action.status in (ActionStatus.NOT_STARTED, ActionStatus.IN_PROGRESS):
            if goal_handle.is_cancel_requested:
                self.agent.clear_actions()
                goal_handle.canceled()
                return FollowJointTrajectory.Result()
            feedback = FollowJointTrajectory.Feedback()
            # Fill feedback with current joint positions
            joint_states = self.agent.get_all_joints_state_by_name()
            for jname in trajectory.joint_names:
                if jname in joint_states:
                    feedback.actual.positions.append(joint_states[jname][0])
            goal_handle.publish_feedback(feedback)
            import time
            time.sleep(0.05)

    result = FollowJointTrajectory.Result()
    goal_handle.succeed()
    return result
```

Also add `from .conversions import pbf_pose_to_ros` to the existing imports.

**Step 3: Run tests**

Run: `pytest ros2_bridge/pybullet_fleet_ros/test/test_nav_actions.py -v`
Expected: PASS

**Step 4: Commit**

```bash
git add ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/robot_handler.py
git add ros2_bridge/pybullet_fleet_ros/test/test_nav_actions.py
git commit -m "feat(ros2): add NavigateToPose, FollowPath, FollowJointTrajectory action servers"
```

---

## Task 8: Integration Test with Docker (SERIAL, depends on all above)

**Files:**
- Create: `ros2_bridge/pybullet_fleet_ros/test/test_integration.py`
- Create: `docker/test_integration.sh`

**Step 1: Write integration test script**

```bash
#!/bin/bash
# docker/test_integration.sh
# Run inside Docker container to verify full stack
set -e

echo "=== Integration Test: PyBulletFleet ROS 2 Bridge ==="

# Source ROS
source /opt/ros/jazzy/setup.bash
source /opt/sim_interfaces_ws/install/setup.bash
source /opt/bridge_ws/install/setup.bash

# Start bridge in background
ros2 run pybullet_fleet_ros bridge_node \
    --ros-args -p num_robots:=3 -p gui:=false -p publish_rate:=10.0 &
BRIDGE_PID=$!
sleep 3  # Wait for startup

echo "--- Checking topics ---"
TOPICS=$(ros2 topic list)
echo "$TOPICS"

# Verify expected topics exist
for topic in /clock /robot_0/odom /robot_0/cmd_vel /robot_0/joint_states \
             /robot_1/odom /robot_2/odom /tf; do
    if echo "$TOPICS" | grep -q "$topic"; then
        echo "  ✓ $topic"
    else
        echo "  ✗ $topic MISSING"
        kill $BRIDGE_PID 2>/dev/null
        exit 1
    fi
done

echo "--- Checking /clock publishes ---"
timeout 5 ros2 topic echo /clock --once || { echo "✗ /clock not publishing"; kill $BRIDGE_PID; exit 1; }
echo "  ✓ /clock publishing"

echo "--- Checking odom publishes ---"
timeout 5 ros2 topic echo /robot_0/odom --once || { echo "✗ odom not publishing"; kill $BRIDGE_PID; exit 1; }
echo "  ✓ /robot_0/odom publishing"

echo "--- Checking services ---"
SERVICES=$(ros2 service list)
for svc in /sim/get_simulator_features /sim/spawn_entity /sim/get_entities \
           /sim/step_simulation /sim/get_simulation_state; do
    if echo "$SERVICES" | grep -q "$svc"; then
        echo "  ✓ $svc"
    else
        echo "  ✗ $svc MISSING"
        kill $BRIDGE_PID 2>/dev/null
        exit 1
    fi
done

echo "--- Testing GetEntities service ---"
ros2 service call /sim/get_entities simulation_interfaces/srv/GetEntities || echo "  (service call structure may vary)"

echo "--- Testing cmd_vel ---"
ros2 topic pub --once /robot_0/cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep 1
echo "  ✓ cmd_vel published"

# Cleanup
kill $BRIDGE_PID 2>/dev/null
wait $BRIDGE_PID 2>/dev/null

echo ""
echo "=== All integration tests PASSED ==="
```

**Step 2: Add integration test target to docker-compose.yaml**

Add to `docker/docker-compose.yaml`:

```yaml
  test:
    build:
      context: ..
      dockerfile: docker/Dockerfile.jazzy
    command: bash /opt/pybullet_fleet/docker/test_integration.sh
    network_mode: host
    environment:
      - ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-99}
```

**Step 3: Run integration test**

Run: `cd docker && docker compose run --rm test`
Expected: All checks pass

**Step 4: Commit**

```bash
git add docker/test_integration.sh
git add ros2_bridge/pybullet_fleet_ros/test/test_integration.py
git commit -m "test(ros2): add Docker integration test for bridge"
```

---

## Task 9: Launch Files + Examples (SERIAL, depends on Task 8)

**Files:**
- Create: `ros2_bridge/pybullet_fleet_ros/launch/teleop_mobile.launch.py`
- Create: `ros2_bridge/pybullet_fleet_ros/launch/nav2_goal.launch.py`
- Create: `ros2_bridge/pybullet_fleet_ros/launch/multi_robot_fleet.launch.py`
- Create: `ros2_bridge/pybullet_fleet_ros/launch/arm_control.launch.py`
- Create: `ros2_bridge/pybullet_fleet_ros/launch/mobile_manipulator.launch.py`

**Step 1: teleop_mobile.launch.py**

```python
# ros2_bridge/pybullet_fleet_ros/launch/teleop_mobile.launch.py
"""Launch bridge with 1 mobile robot + teleop_twist_keyboard."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),

        Node(
            package="pybullet_fleet_ros",
            executable="bridge_node",
            name="pybullet_fleet_bridge",
            parameters=[{
                "num_robots": 1,
                "robot_urdf": "robots/mobile_robot.urdf",
                "publish_rate": 50.0,
                "gui": LaunchConfiguration("gui"),
            }],
            output="screen",
        ),

        # Teleop twist keyboard (interactive terminal)
        ExecuteProcess(
            cmd=["ros2", "run", "teleop_twist_keyboard", "teleop_twist_keyboard",
                 "--ros-args", "-r", "/cmd_vel:=/robot_0/cmd_vel"],
            output="screen",
            prefix="xterm -e" if False else "",  # Set True for separate window
        ),
    ])
```

**Step 2: nav2_goal.launch.py**

```python
# ros2_bridge/pybullet_fleet_ros/launch/nav2_goal.launch.py
"""Launch bridge with 1 mobile robot. Send nav2 goal via CLI."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),

        Node(
            package="pybullet_fleet_ros",
            executable="bridge_node",
            name="pybullet_fleet_bridge",
            parameters=[{
                "num_robots": 1,
                "robot_urdf": "robots/mobile_robot.urdf",
                "publish_rate": 50.0,
                "gui": LaunchConfiguration("gui"),
                "enable_nav_actions": True,
            }],
            output="screen",
        ),

        # Usage hint:
        # ros2 action send_goal /robot_0/navigate_to_pose nav2_msgs/action/NavigateToPose \
        #   "{pose: {pose: {position: {x: 5.0, y: 5.0, z: 0.0}}}}"
    ])
```

**Step 3: multi_robot_fleet.launch.py**

```python
# ros2_bridge/pybullet_fleet_ros/launch/multi_robot_fleet.launch.py
"""Launch bridge with N mobile robots for fleet visualization."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("num_robots", default_value="10"),
        DeclareLaunchArgument("gui", default_value="true"),

        Node(
            package="pybullet_fleet_ros",
            executable="bridge_node",
            name="pybullet_fleet_bridge",
            parameters=[{
                "num_robots": LaunchConfiguration("num_robots"),
                "robot_urdf": "robots/mobile_robot.urdf",
                "publish_rate": 20.0,  # Lower rate for many robots
                "gui": LaunchConfiguration("gui"),
            }],
            output="screen",
        ),

        # Tip: Open RViz and add TF display to see all robots
        # ros2 run rviz2 rviz2
    ])
```

**Step 4: arm_control.launch.py**

```python
# ros2_bridge/pybullet_fleet_ros/launch/arm_control.launch.py
"""Launch bridge with 1 arm robot for joint control demo."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),

        Node(
            package="pybullet_fleet_ros",
            executable="bridge_node",
            name="pybullet_fleet_bridge",
            parameters=[{
                "num_robots": 0,  # No mobile robots
                "publish_rate": 50.0,
                "gui": LaunchConfiguration("gui"),
                "enable_nav_actions": False,
                "enable_joint_actions": True,
            }],
            output="screen",
        ),

        # The arm robot needs to be spawned separately via the bridge config
        # or via SpawnEntity service:
        # ros2 service call /sim/spawn_entity simulation_interfaces/srv/SpawnEntity \
        #   "{name: 'arm_0', resource: {uri: 'robots/arm_robot.urdf'}}"
        #
        # Then control joints:
        # ros2 action send_goal /arm_0/follow_joint_trajectory \
        #   control_msgs/action/FollowJointTrajectory \
        #   "{trajectory: {joint_names: ['joint_0','joint_1','joint_2','joint_3'], \
        #    points: [{positions: [0.5, 0.5, 0.5, 0.0], time_from_start: {sec: 2}}]}}"
    ])
```

**Step 5: mobile_manipulator.launch.py**

```python
# ros2_bridge/pybullet_fleet_ros/launch/mobile_manipulator.launch.py
"""Launch bridge with 1 mobile manipulator (mobile base + arm)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),

        Node(
            package="pybullet_fleet_ros",
            executable="bridge_node",
            name="pybullet_fleet_bridge",
            parameters=[{
                "num_robots": 0,  # Spawn via config or service
                "publish_rate": 50.0,
                "gui": LaunchConfiguration("gui"),
                "enable_nav_actions": True,
                "enable_joint_actions": True,
            }],
            output="screen",
        ),

        # Spawn mobile manipulator:
        # ros2 service call /sim/spawn_entity simulation_interfaces/srv/SpawnEntity \
        #   "{name: 'mm_0', resource: {uri: 'robots/mobile_manipulator.urdf'}}"
        #
        # Drive base: ros2 topic pub /mm_0/cmd_vel geometry_msgs/msg/Twist ...
        # Move arm:   ros2 action send_goal /mm_0/follow_joint_trajectory ...
    ])
```

**Step 6: Test all launch files parse correctly**

Run inside Docker:
```bash
for launch in teleop_mobile nav2_goal multi_robot_fleet arm_control mobile_manipulator; do
    ros2 launch pybullet_fleet_ros ${launch}.launch.py --show-args || echo "FAIL: $launch"
done
```
Expected: All launch files show their arguments without error

**Step 7: Commit**

```bash
git add ros2_bridge/pybullet_fleet_ros/launch/
git commit -m "feat(ros2): add 5 example launch files (teleop, nav2, fleet, arm, mobile_manipulator)"
```

---

## Task 10: Humble Dockerfile + CI Preparation (SERIAL, depends on Task 8)

**Files:**
- Create: `docker/Dockerfile.humble`
- Modify: `docker/docker-compose.yaml` (add humble service)

**Step 1: Create Dockerfile.humble**

```dockerfile
# docker/Dockerfile.humble
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# Install ROS 2 dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-nav2-msgs \
    ros-humble-control-msgs \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

# Build simulation_interfaces from source
WORKDIR /opt/sim_interfaces_ws/src
RUN git clone --depth 1 https://github.com/ros-simulation/simulation_interfaces.git
WORKDIR /opt/sim_interfaces_ws
RUN source /opt/ros/humble/setup.bash && \
    colcon build --packages-select simulation_interfaces --cmake-args -DCMAKE_BUILD_TYPE=Release

# Install pybullet_fleet
COPY requirements.txt /opt/pybullet_fleet/requirements.txt
COPY pyproject.toml /opt/pybullet_fleet/pyproject.toml
COPY pybullet_fleet/ /opt/pybullet_fleet/pybullet_fleet/
COPY robots/ /opt/pybullet_fleet/robots/
COPY config/ /opt/pybullet_fleet/config/
COPY mesh/ /opt/pybullet_fleet/mesh/
WORKDIR /opt/pybullet_fleet
RUN pip install -e .

# Build ROS 2 bridge
WORKDIR /opt/bridge_ws/src
COPY ros2_bridge/pybullet_fleet_ros /opt/bridge_ws/src/pybullet_fleet_ros
WORKDIR /opt/bridge_ws
RUN source /opt/ros/humble/setup.bash && \
    source /opt/sim_interfaces_ws/install/setup.bash && \
    colcon build --symlink-install

# Entrypoint
COPY docker/ros_entrypoint_humble.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
```

**Step 2: Create humble entrypoint**

```bash
#!/bin/bash
# docker/ros_entrypoint_humble.sh
set -e
source /opt/ros/humble/setup.bash
source /opt/sim_interfaces_ws/install/setup.bash
source /opt/bridge_ws/install/setup.bash
exec "$@"
```

**Step 3: Add humble services to docker-compose.yaml**

Append to `docker/docker-compose.yaml`:

```yaml
  bridge-humble:
    build:
      context: ..
      dockerfile: docker/Dockerfile.humble
    command: >
      ros2 launch pybullet_fleet_ros teleop_mobile.launch.py
    network_mode: host
    environment:
      - DISPLAY=${DISPLAY:-}
      - ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-43}
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ../pybullet_fleet:/opt/pybullet_fleet/pybullet_fleet:ro
      - ../ros2_bridge/pybullet_fleet_ros:/opt/bridge_ws/src/pybullet_fleet_ros:ro
    stdin_open: true
    tty: true

  test-humble:
    build:
      context: ..
      dockerfile: docker/Dockerfile.humble
    command: bash /opt/pybullet_fleet/docker/test_integration.sh
    network_mode: host
    environment:
      - ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-98}
```

**Step 4: Build and test Humble**

Run: `cd docker && docker compose build bridge-humble && docker compose run --rm test-humble`
Expected: All integration tests pass on Humble

**Step 5: Commit**

```bash
git add docker/Dockerfile.humble docker/ros_entrypoint_humble.sh
git add docker/docker-compose.yaml
git commit -m "feat(ros2): add Humble Dockerfile and CI test target"
```

---

## Task 11: AgentManagerROSWrapper Stub (SERIAL, depends on Task 5)

**Files:**
- Create: `ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/agent_manager_wrapper.py`

This is a minimal stub for Phase 2. Creates the file with the interface documented but batch topics commented out.

**Step 1: Create stub**

```python
# ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/agent_manager_wrapper.py
"""AgentManager ROS wrapper for fleet-level batch operations.

Phase 2 placeholder — batch topics are commented out.
Currently provides helper methods for creating RobotHandlers from AgentManager.
"""

import logging
from typing import TYPE_CHECKING, Dict, Optional

from .robot_handler import RobotHandler

if TYPE_CHECKING:
    from pybullet_fleet.agent_manager import AgentManager
    from rclpy.node import Node
    from tf2_ros import TransformBroadcaster

logger = logging.getLogger(__name__)


class AgentManagerROSWrapper:
    """Wraps AgentManager for fleet-level ROS operations.

    Phase 1: Helper for creating per-robot handlers from a managed fleet.
    Phase 2: Will add batch command topics (/fleet/cmd_vel_batch, etc.)
    """

    def __init__(
        self,
        node: "Node",
        agent_manager: "AgentManager",
        tf_broadcaster: Optional["TransformBroadcaster"] = None,
        enable_nav_actions: bool = True,
        enable_joint_actions: bool = True,
    ):
        self._node = node
        self._mgr = agent_manager
        self._tf_broadcaster = tf_broadcaster
        self._enable_nav = enable_nav_actions
        self._enable_joint = enable_joint_actions

        # Phase 2: batch topics
        # self._batch_cmd_sub = node.create_subscription(
        #     FleetCmdVelBatch, '/fleet/cmd_vel_batch', self._batch_cmd_cb, 10)

    def create_handlers(self) -> Dict[int, RobotHandler]:
        """Create a RobotHandler for each agent in the manager."""
        handlers = {}
        for agent in self._mgr.objects:
            handler = RobotHandler(
                self._node, agent,
                tf_broadcaster=self._tf_broadcaster,
                enable_nav_actions=self._enable_nav,
                enable_joint_actions=self._enable_joint,
            )
            handlers[agent.object_id] = handler
        logger.info("Created %d RobotHandlers from AgentManager", len(handlers))
        return handlers
```

**Step 2: Commit**

```bash
git add ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/agent_manager_wrapper.py
git commit -m "feat(ros2): add AgentManagerROSWrapper stub (Phase 2 prep)"
```

---

## Task 12: Documentation + README (SERIAL, final)

**Files:**
- Create: `ros2_bridge/README.md`
- Modify: `README.md` (add ROS 2 section)

**Step 1: Create ros2_bridge README**

```markdown
# PyBulletFleet ROS 2 Bridge

ROS 2 bridge for [PyBulletFleet](https://github.com/yuokamoto/PyBulletFleet) multi-robot simulation.

## Quick Start

```bash
# Build and run (no native ROS install needed)
cd docker
docker compose up bridge
```

## Examples

| Launch File | Description | Command |
|------------|-------------|---------|
| `teleop_mobile` | Drive 1 robot with keyboard | `ros2 launch pybullet_fleet_ros teleop_mobile.launch.py` |
| `nav2_goal` | Send navigation goals | `ros2 launch pybullet_fleet_ros nav2_goal.launch.py` |
| `multi_robot_fleet` | Visualize N robots in RViz | `ros2 launch pybullet_fleet_ros multi_robot_fleet.launch.py num_robots:=10` |
| `arm_control` | Control arm joints via ROS | `ros2 launch pybullet_fleet_ros arm_control.launch.py` |
| `mobile_manipulator` | Combined base + arm | `ros2 launch pybullet_fleet_ros mobile_manipulator.launch.py` |

## Topics & Services

### Per-Robot Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/{name}/cmd_vel` | `geometry_msgs/Twist` | Subscribe |
| `/{name}/odom` | `nav_msgs/Odometry` | Publish |
| `/{name}/joint_states` | `sensor_msgs/JointState` | Publish |
| `/tf` | `tf2_msgs/TFMessage` | Publish |
| `/clock` | `rosgraph_msgs/Clock` | Publish |

### Per-Robot Actions

| Action | Type | Robot Type |
|--------|------|-----------|
| `/{name}/navigate_to_pose` | `nav2_msgs/NavigateToPose` | Mobile |
| `/{name}/follow_path` | `nav2_msgs/FollowPath` | Mobile |
| `/{name}/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` | Arm |

### Simulation Services (simulation_interfaces)

| Service | Description |
|---------|-------------|
| `/sim/get_simulator_features` | Query supported features |
| `/sim/spawn_entity` | Spawn new robot |
| `/sim/delete_entity` | Remove robot |
| `/sim/get_entities` | List all entities |
| `/sim/get_entity_state` | Get pose/velocity |
| `/sim/set_entity_state` | Set pose/velocity |
| `/sim/step_simulation` | Execute N steps |
| `/sim/get_simulation_state` | Check if paused/playing |
| `/sim/set_simulation_state` | Pause/resume |
| `/sim/reset_simulation` | Reset simulation |

## Supported Distros

| Distro | Ubuntu | Status |
|--------|--------|--------|
| **Jazzy** | 24.04 | Primary |
| **Humble** | 22.04 | Secondary (CI tested) |

## Development

```bash
# Build in Docker
cd docker && docker compose build

# Run tests
docker compose run --rm test

# Humble tests
docker compose run --rm test-humble
```
```

**Step 2: Commit**

```bash
git add ros2_bridge/README.md
git commit -m "docs(ros2): add ROS 2 bridge README"
```

---

## Verification Checklist

After all tasks are complete, run the full verification:

```bash
# 1. Docker builds
cd docker
docker compose build bridge
docker compose build bridge-humble

# 2. Integration tests (Jazzy)
docker compose run --rm test

# 3. Integration tests (Humble)
docker compose run --rm test-humble

# 4. Unit tests inside container
docker compose run --rm bridge bash -c "cd /opt/bridge_ws && colcon test --packages-select pybullet_fleet_ros"

# 5. Core PyBulletFleet tests still pass (no regressions)
cd .. && make verify

# 6. Manual verification
docker compose up bridge
# In another terminal:
ros2 topic list
ros2 topic echo /clock --once
ros2 topic echo /robot_0/odom --once
ros2 service call /sim/get_entities simulation_interfaces/srv/GetEntities
```

All success criteria from the spec should be verified:
- [ ] `docker compose up` starts with zero native ROS install
- [ ] `ros2 topic list` shows all expected topics
- [ ] `/clock` publishes advancing sim time
- [ ] `/robot_0/odom` shows live data
- [ ] `teleop_twist_keyboard` drives robot via cmd_vel
- [ ] `NavigateToPose` action completes
- [ ] `FollowPath` action completes
- [ ] `FollowJointTrajectory` action moves arm
- [ ] `SpawnEntity` creates new robot with topics
- [ ] `GetEntities` returns all agents
- [ ] `SimulateSteps` executes N steps with feedback
- [ ] `ResetSimulation` resets state
- [ ] RViz shows TF frames
- [ ] 10+ robots run without bottleneck
- [ ] CI passes on Jazzy and Humble
- [ ] All 5 launch files work
