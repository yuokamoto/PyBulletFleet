# Quick Start

This guide walks you through installing PyBulletFleet, running your first simulation,
and understanding the key configuration options.

## Prerequisites

Make sure you have the following installed:

- **Python 3.10+**
- **PyBullet** — physics engine backend
- **NumPy** — numerical operations
- **PyYAML** — configuration file parsing

All dependencies are installed automatically during the package install step below.

## Installation

### From PyPI (recommended)

```bash
pip install pybullet-fleet
```

### From source (for development)

Clone the repository and install in editable mode with development dependencies:

```bash
git clone https://github.com/yuokamoto/PyBulletFleet.git
cd PyBulletFleet
pip install -e ".[dev]"
```

This makes the `pybullet_fleet` package importable from anywhere while keeping
the source editable.

## Running Your First Simulation

Launch the 100-robot grid demo:

```bash
python examples/scale/100robots_grid_demo.py
```

You should see a PyBullet GUI window open with 100 robots spawned in a grid
formation. Each robot navigates toward a randomly assigned goal using
kinematic teleportation.

## YAML Configuration Basics

Simulation behaviour is controlled via YAML config files.
The default configuration lives at `config/config.yaml`.

Key settings you will want to tweak:

| Setting | Type | Default | Description |
|---------|------|---------|-------------|
| `gui` | bool | `true` | Open the PyBullet GUI window |
| `target_rtf` | float | `100` | Target Real-Time Factor (1.0 = real time, 0 = max speed) |
| `timestep` | float | `0.1` | Seconds per simulation step |
| `physics` | bool | `false` | Enable PyBullet physics engine (off for pure kinematic mode) |
| `collision_detection_method` | string | `"closest_points"` | `"closest_points"`, `"contact_points"`, or `"hybrid"` |
| `collision_safety_margin` | float | `0.02` | Safety buffer in metres for near-miss detection |
| `monitor` | bool | `true` | Enable the real-time data monitor |
| `enable_time_profiling` | bool | `true` | Print step-timing reports |
| `enable_floor` | bool | `true` | Load the default ground plane (`plane.urdf`). Set `false` for custom floor handling |

Example — run headless at 10× real-time with physics enabled:

```yaml
gui: false
target_rtf: 10
physics: true
collision_detection_method: "contact_points"
```

See [`config/config.yaml`](https://github.com/yuokamoto/PyBulletFleet/blob/main/config/config.yaml) for the full list of parameters — every key is documented with inline comments.

## Keyboard Controls

When running with `gui: true`, the following shortcuts are available:

| Key | Action | Description |
|-----|--------|-------------|
| `SPACE` | Pause / Resume | Toggle simulation pause |
| `v` | Visual shapes | Toggle visual shapes ON/OFF |
| `c` | Collision shapes | Toggle collision wireframes ON/OFF |
| `t` | Transparency | Toggle structure transparency ON/OFF |

```{note}
The `t` key toggles transparency one object at a time.
For scenes with hundreds of objects this can be slow — prefer setting
`enable_structure_transparency` in the config file instead.
```

## Examples

All example scripts live in the `examples/` directory, organised by category.
For step-by-step walkthroughs, see the **[Tutorials](../examples/index)** page.

### Basics (`examples/basics/`)

| Script | Description |
|--------|-------------|
| `robot_demo.py` | Basic robot creation with `Agent.from_mesh()` / `Agent.from_urdf()` |
| `action_system_demo.py` | High-level action system (MoveTo, Pick, Drop, Wait) |
| `collision_features_demo.py` | Spatial-hash collision detection features and visualisation |
| `memory_profiling_demo.py` | Memory usage tracking and profiling utilities |

### Arm (`examples/arm/`)

| Script | Description |
|--------|-------------|
| `pick_drop_arm_demo.py` | Single arm robot performing pick-and-drop operations |
| `pick_drop_arm_action_demo.py` | Arm robot using the action system for pick-and-drop |
| `pick_drop_arm_ee_demo.py` | Arm end-effector control via IK (low-level callback) |
| `pick_drop_arm_ee_action_demo.py` | Arm EE control using PoseAction (action queue) |
| `rail_arm_demo.py` | Rail arm (prismatic + revolute) pick-and-drop with EE control |
| `mobile_manipulator_demo.py` | Kinematic mobile manipulator — IK-based pick/drop with arm + base movement |

### Mobile (`examples/mobile/`)

| Script | Description |
|--------|-------------|
| `path_following_demo.py` | Compares omnidirectional and differential-drive motion modes |

### Scale (`examples/scale/`)

| Script | Description |
|--------|-------------|
| `100robots_grid_demo.py` | Grid-based multi-agent demo — best starting point |
| `100robots_cube_patrol_demo.py` | 100 agents patrolling between cubes |
| `pick_drop_arm_100robots_demo.py` | 100 arm robots with synchronised pick-and-drop |
| `pick_drop_mobile_100robots_demo.py` | 100 mobile robots picking and dropping objects |

### Models (`examples/models/`)

| Script | Description |
|--------|-------------|
| `resolve_urdf_demo.py` | URDF resolution patterns — by name, by path, and listing all models |
| `model_catalog_demo.py` | Visual grid catalog of all registered models from `KNOWN_MODELS` |
| `robot_descriptions_demo.py` | Using Tier 3 models from the `robot_descriptions` pip package |

Run any example with:

```bash
python examples/<category>/<script_name>.py
```

## Next Steps

- **Architecture overview** — [Architecture Overview](../architecture/overview) explains the core
  abstractions (simulation core, agents, actions, collision).
- **How-to guides** — see the `docs/` directory for topic-specific guides
  (collision tuning, memory profiling, performance optimisation).
- **Configuration reference** — [`config/config.yaml`](https://github.com/yuokamoto/PyBulletFleet/blob/main/config/config.yaml) documents every YAML key with inline comments. See also the [Configuration Files Guide](../configuration/reference).
- **API reference** — auto-generated from docstrings (coming soon).
