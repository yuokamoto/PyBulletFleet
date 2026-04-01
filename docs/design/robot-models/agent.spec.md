# Robot Models — Agent Specification

**Status:** Validated

## Requirements

### Functional
- `resolve_urdf(name_or_path)` resolves short name → absolute URDF path via 3-tier lookup
- `auto_detect_profile(body_id)` introspects loaded URDF → `RobotProfile` with EE link, joints, limits
- `RobotProfile` merges auto-detected values with optional manual overrides
- Existing demos accept `--robot <name>` CLI arg to switch model (same category only)
- Demo scripts use both `resolve_urdf()` (recommended) and direct `pybullet_data.getDataPath()` (educational)

### Non-Functional
- No new required dependencies
- `robot_descriptions` is optional (`pyproject.toml [models]` extra)
- Tier 1 models resolve in < 1ms (dict lookup + `os.path.join`)
- Auto-detection adds < 50ms per robot (temp load + inspect + remove)

## Constraints

- Only Apache-2.0 compatible licenses: BSD, MIT, Apache-2.0, Zlib
- No URDF/mesh files committed to repo
- No circular imports with `agent.py` (robot_models.py is standalone utility)
- ROS Tier 2 resolved via `ament_index_python` (ROS2) or `rospkg` (ROS1), both optional imports

## Approach

### Tier 1: pybullet_data (built-in)
```python
import pybullet_data
path = os.path.join(pybullet_data.getDataPath(), "franka_panda/panda.urdf")
```

### Tier 2: ROS packages
```python
from ament_index_python.packages import get_package_share_directory
path = os.path.join(get_package_share_directory("ur_description"), "urdf/ur5e.urdf")
```

### Tier 3: robot_descriptions (optional pip)
```python
from robot_descriptions import panda_description
path = panda_description.URDF_PATH
```

### Auto-detection via PyBullet API
```python
body_id = p.loadURDF(path)
for i in range(p.getNumJoints(body_id)):
    info = p.getJointInfo(body_id, i)
    # joint_name, joint_type, lower_limit, upper_limit, max_velocity, link_name
```

EE link heuristic: name match ("hand", "ee", "end_effector", "tool", "gripper") → leaf link fallback.

## Design

### Architecture

```
resolve_urdf("panda")
    → KNOWN_MODELS registry lookup
    → Tier 1: pybullet_data path join
    → Tier 2: ROS package path resolve (optional)
    → Tier 3: robot_descriptions import (optional)
    → fallback: treat as file path

auto_detect_profile(physics_client, urdf_path)
    → temp p.loadURDF()
    → introspect joints, links, limits
    → guess EE link
    → guess robot type (arm/mobile)
    → p.removeBody()
    → return RobotProfile
```

### Key Components

| Component | Responsibility | Location |
|-----------|---------------|----------|
| `resolve_urdf()` | Name → absolute URDF path | `pybullet_fleet/robot_models.py` |
| `KNOWN_MODELS` | Registry dict: name → (rel_path, tier, install_hint) | `pybullet_fleet/robot_models.py` |
| `RobotProfile` | Dataclass: urdf_path, ee_link, joints, max_vel, robot_type | `pybullet_fleet/robot_models.py` |
| `auto_detect_profile()` | PyBullet introspection → RobotProfile | `pybullet_fleet/robot_models.py` |
| `list_available_models()` | Print table of models + availability | `pybullet_fleet/robot_models.py` |

### Data Flow

1. Demo script: `--robot panda` → `resolve_urdf("panda")` → absolute path
2. `AgentSpawnParams(urdf_path=resolved_path, ...)`
3. Optionally: `auto_detect_profile(path)` → `RobotProfile` → override `AgentSpawnParams` fields

### Known Models Registry

```python
KNOWN_MODELS = {
    # Tier 1: pybullet_data (always available)
    "panda":      ModelEntry("franka_panda/panda.urdf",    "pybullet_data"),
    "kuka_iiwa":  ModelEntry("kuka_iiwa/model.urdf",       "pybullet_data"),
    "husky":      ModelEntry("husky/husky.urdf",            "pybullet_data"),
    "racecar":    ModelEntry("racecar/racecar.urdf",        "pybullet_data"),
    "table":      ModelEntry("table/table.urdf",            "pybullet_data"),
    "tray":       ModelEntry("tray/tray.urdf",              "pybullet_data"),
    "plane":      ModelEntry("plane.urdf",                  "pybullet_data"),
    # Tier 2: ROS packages
    "ur5e":       ModelEntry("ur_description", "ros", install="apt install ros-${ROS_DISTRO}-ur-description"),
    "turtlebot3": ModelEntry("turtlebot3_description", "ros", install="apt install ros-${ROS_DISTRO}-turtlebot3-description"),
    "fetch":      ModelEntry("fetch_description", "ros", install="apt install ros-${ROS_DISTRO}-fetch-description"),
    # Tier 3: robot_descriptions pip package
    "fetch_rd":   ModelEntry("fetch_description", "robot_descriptions", install="pip install robot_descriptions"),
    "tiago":      ModelEntry("tiago_description", "robot_descriptions", install="pip install robot_descriptions"),
}
```

### RobotProfile Auto-Detection

```python
@dataclass
class RobotProfile:
    urdf_path: str
    robot_type: str  # "arm", "mobile", "mobile_manipulator"
    ee_link_name: Optional[str] = None
    movable_joint_names: List[str] = field(default_factory=list)
    joint_lower_limits: List[float] = field(default_factory=list)
    joint_upper_limits: List[float] = field(default_factory=list)
    joint_max_velocities: List[float] = field(default_factory=list)
    max_linear_vel: Optional[float] = None
    max_angular_vel: Optional[float] = None
    motion_mode: Optional[str] = None  # "omnidirectional", "differential"
```

Robot type heuristic:
- Has revolute/prismatic joints and no wheels → "arm"
- Has continuous joints (wheels) and no end effector → "mobile"
- Has both → "mobile_manipulator"

### Demo CLI Pattern

```python
# examples/arm/pick_drop_arm_demo.py
import argparse
from pybullet_fleet.robot_models import resolve_urdf

parser = argparse.ArgumentParser()
parser.add_argument("--robot", default=None, help="Robot name (e.g. panda, kuka_iiwa) or URDF path")
args = parser.parse_args()

# Default to built-in URDF, override with --robot
if args.robot:
    urdf_path = resolve_urdf(args.robot)
else:
    urdf_path = os.path.join(os.path.dirname(__file__), "../robots/arm_robot.urdf")
```

### Code Patterns

Existing spawn pattern to follow (from `examples/arm/pick_drop_arm_action_demo.py`):
```python
arm_urdf = os.path.join(os.path.dirname(__file__), "../robots/arm_robot.urdf")
params = AgentSpawnParams(
    urdf_path=arm_urdf,
    initial_pose=Pose.from_xyz(0, 0, 0),
    use_fixed_base=True,
    ik_params=IKParams(),
    motion_mode=MotionMode.OMNIDIRECTIONAL,
)
agent = Agent.from_params(params, sim)
```

## File References

Files the plan agent MUST read before planning:

- `pybullet_fleet/agent.py:28-100` — AgentSpawnParams, IKParams dataclasses
- `pybullet_fleet/config_utils.py` — YAML loading
- `pybullet_fleet/__init__.py` — current exports
- `pyproject.toml` — dependencies and optional groups
- `examples/arm/pick_drop_arm_demo.py` — arm demo pattern
- `examples/scale/100robots_grid_demo.py` — fleet demo with config and argparse
- `examples/arm/pick_drop_arm_action_demo.py` — action-based arm demo
- `examples/mobile/path_following_demo.py` — mobile demo pattern
- `examples/arm/mobile_manipulator_demo.py` — mobile manipulator with IKParams
- `tests/conftest.py` — test fixtures and MockSimCore

## Success Criteria

- [ ] `resolve_urdf("panda")` returns valid path; file exists
- [ ] `resolve_urdf("robots/arm_robot.urdf")` returns the input unchanged (or resolved)
- [ ] `resolve_urdf("nonexistent")` raises `FileNotFoundError` with install hint
- [ ] `auto_detect_profile` on Panda finds `panda_hand` as EE link
- [ ] `auto_detect_profile` on Husky returns `robot_type="mobile"`, no EE
- [ ] `pick_drop_arm_demo.py --robot panda` runs successfully
- [ ] `100robots_grid_demo.py --robot husky` runs 100 Huskyss
- [ ] `list_available_models()` prints table with availability status
- [ ] Existing tests pass unchanged (866+)
- [ ] New tests for resolve_urdf, auto_detect_profile, RobotProfile
