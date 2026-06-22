# Configuration Files Guide

## Overview

PyBulletFleet provides multiple configuration files for different simulation modes and benchmarking scenarios.

## Configuration Files

### Production Configurations

#### `config.yaml` (Default)
- General-purpose configuration
- Auto-selects collision detection method based on physics mode
- Recommended starting point for most use cases
- **All parameters are documented with inline comments** — open [`config/config.yaml`](https://github.com/yuokamoto/PyBulletFleet/blob/main/config/config.yaml) for the full reference with explanations, recommended values, and usage examples

#### `config_physics_off.yaml` (Kinematics Mode) ✅ Recommended
- **Use case**: Path planning, collision avoidance, high-speed simulation
- **Physics**: OFF (no `stepSimulation()`)
- **Collision method**: `closest_points` (distance-based, kinematics-safe)
- **Collision margin**: 2cm safety clearance
- **Target RTF**: 100x real-time
- **Key features**:
  - Fast and deterministic
  - Stable with `resetBasePositionAndOrientation()`
  - Safety margin detection (near-miss detection)

#### `config_physics_on.yaml` (Physics Mode) 🔬
- **Use case**: Physics simulation, debugging, contact analysis
- **Physics**: ON (`stepSimulation()` every step)
- **Collision method**: `contact_points` (actual contact manifold)
- **Target RTF**: 1x real-time
- **Key features**:
  - Realistic physics behavior (mass, friction, push-back)
  - Actual contact logging
  - Visual collision feedback (color change)

#### `config_hybrid.yaml` (Mixed Mode) 🚧 Advanced
- **Use case**: Mixed physics/kinematic scenarios
- **Physics**: ON (required)
- **Collision method**: `hybrid` (contact for physics, closest for kinematic)
- **Collision margin**: 5cm safety for kinematic pairs
- **Key features**:
  - Different detection for different object types
  - Slower due to branching overhead
  - Requires careful tuning

### Benchmark Configurations

#### `benchmark_physics_off_closest.yaml`
- Headless, profiling-enabled
- Physics OFF + CLOSEST_POINTS
- 10-second duration, maximum speed

#### `benchmark_physics_on_contact.yaml`
- Headless, profiling-enabled
- Physics ON + CONTACT_POINTS
- 10-second duration, maximum speed

#### `benchmark_hybrid.yaml`
- Headless, profiling-enabled
- Physics ON + HYBRID
- 10-second duration, maximum speed

## Agent / Robot Spawn Schema

Robot-spawn blocks (e.g. `mobile_robot:`, `arm_robot:` in
[`config/100robots_config.yaml`](https://github.com/yuokamoto/PyBulletFleet/blob/main/config/100robots_config.yaml))
are consumed by `AgentSpawnParams.from_dict` and then routed through
`Agent.from_params`.  Controller-related fields live **only** under a
nested `controller:` block — there is no top-level `max_linear_vel:` /
`max_angular_vel:` shortcut.  This keeps a single source of truth for
controller params across YAML, Python `dict`, and the Python kwarg API.

### Canonical block

```yaml
mobile_robot:
  urdf_path: "robots/mobile_robot.urdf"
  initial_z: 0.3
  use_fixed_base: false
  motion_mode: omnidirectional        # optional; deprecated as selector
  controller:                         # single source of truth
    type: omni                        # registry name (omni / differential / batch_omni / ...)
    max_linear_vel: 2.0               # scalar (magnitude) or [vx, vy, vz] per-axis cap
    max_angular_vel: 2.0              # scalar or [wx, wy, wz]
    max_linear_accel: 5.0
    max_angular_accel: 5.0
    navigation_2d: true               # preserve z during pose-mode trajectories
    cmd_vel_timeout: 0.5              # velocity-command watchdog (s); 0.0 disables
    default_direction: forward        # differential-drive only; forward / backward / auto
    # impl extras (routed to the controller subclass __init__):
    wheel_separation: 0.3             # DifferentialController
```

### Key routing rules

| YAML key | Where it goes | Notes |
|----------|---------------|-------|
| `controller.type` | Registry lookup → `Controller` subclass | Required for non-default controllers (everything except the `motion_mode` fallback). |
| `controller.max_*` | `ControllerParams` fields | Scalar or 3-element list.  Same value seen by per-agent and batched controllers. |
| `controller.navigation_2d`, `cmd_vel_timeout`, `default_direction` | `ControllerParams` fields | Behaviour flags shared by every controller. |
| `controller.<other>` | Subclass `__init__` kwarg | Routed via `inspect.signature` in `Controller.from_config`.  Unknown-to-`ControllerParams` keys are silently dropped from the params dict. |
| `motion_mode` | Fallback selector | Deprecated as a controller selector.  Only used when `controller=None` (no `type` given).  A warning is logged when `motion_mode` and `controller.type` disagree. |

### Equivalence with the Python API

The same dict is accepted verbatim by the Python kwarg `controller=`:

```python
agent = Agent.from_urdf(
    "robots/mobile_robot.urdf",
    controller={
        "type": "omni",
        "max_linear_vel": 2.0,
        "max_angular_vel": 2.0,
    },
)
```

See the [Controller Configuration how-to](../how-to/controller-config)
for the full guide to all five accepted forms of `controller=` (None /
string / dict / `Controller` instance / `ControllerParams` instance),
custom controllers, and the `motion_mode` vs `controller.type`
interaction rules.

## Centralized Default Management (`_defaults.py`)

All simulation parameter defaults live in a single module:
**`pybullet_fleet/_defaults.py`**.

```
_DEFAULTS dict
  └── simulation / agent / sim_object / shape / ik
       └── key: value  (the ONLY place defaults are defined)
```

### Override priority

| Layer | Mechanism | Example |
|-------|-----------|--------|
| 1. Code defaults | `_DEFAULTS` dict in `_defaults.py` | `"timestep": 0.1` |
| 2. `.env` file | Auto-loaded if `python-dotenv` is installed (`override=False` — fills missing keys only) | `PBF_SIMULATION_GUI=false` |
| 3. Environment vars | `PBF_{SECTION}_{KEY}` — shell env vars override `.env` | `PBF_SIMULATION_TIMESTEP=0.05` |
| 4. YAML config | `config.yaml` passed to `from_yaml()` | `timestep: 0.02` |
| 5. Constructor args | Explicit `SimulationParams(...)` keyword | `SimulationParams(timestep=0.01)` |

Later layers win.  The `_defaults.py` module exposes convenience dicts
(`SIMULATION`, `AGENT`, `SIM_OBJECT`, etc.) for direct import.

### Using defaults in your code

```python
from pybullet_fleet._defaults import get

timestep = get("simulation", "timestep")   # 0.1 (or env-overridden)
max_vel  = get("agent", "max_linear_vel")  # 2.0
```

## Usage Examples

### Load Configuration in Python

```python
from pybullet_fleet.core_simulation import MultiRobotSimulationCore

# Load default config
sim = MultiRobotSimulationCore.from_yaml("config/config.yaml")

# Load kinematics mode config
sim = MultiRobotSimulationCore.from_yaml("config/config_physics_off.yaml")

# Load physics mode config
sim = MultiRobotSimulationCore.from_yaml("config/config_physics_on.yaml")
```

### Run Config-Based Benchmark

```bash
# Run comparison benchmark using all config files
python benchmark/collision_methods_config_based.py
```

This will compare:
1. Physics OFF + CLOSEST_POINTS (kinematics)
2. Physics ON + CONTACT_POINTS (physics)
3. Physics ON + HYBRID (mixed)

## Key Parameters Explained

This section highlights the most important parameters. For a **complete list with detailed explanations**, see the inline comments in [`config/config.yaml`](https://github.com/yuokamoto/PyBulletFleet/blob/main/config/config.yaml).

### `physics` (bool)
- `false`: Kinematics mode (no `stepSimulation()`, fast, deterministic)
- `true`: Physics mode (`stepSimulation()` every step, realistic dynamics)

### `collision_detection_method` (string)
- `"closest_points"`: Distance-based detection (recommended for kinematics)
- `"contact_points"`: Physics contact manifold (recommended for physics)
- `"hybrid"`: Mixed approach — physics pairs use contact, kinematic pairs use closest (advanced)

If omitted, the method is auto-selected based on `physics`:
- `physics: false` → `"closest_points"`
- `physics: true` → `"contact_points"`

See [Collision Detection Narrow-Phase Details](narrow-phase-details-pybullet-apis) for design rationale, trade-offs, and method comparison.

### `collision_margin` (float, meters)
Safety clearance for `getClosestPoints()`. Default: `0.02` (2cm).
Only affects `CLOSEST_POINTS` and `HYBRID` modes.

See [Collision Configuration Guide](collision-margin) for tuning guidance.

### `multi_cell_threshold` (float, dimensionless multiplier)
Controls when objects are registered in multiple spatial-hash cells.
Objects larger than `cell_size × multi_cell_threshold` span multiple cells.
- `1.0`: Every object that exceeds one cell is multi-registered
- `1.5`: Default — objects >1.5× the cell size span multiple cells ✅
- `2.0+`: Only very large objects are multi-registered

**Unit**: Dimensionless (multiplier of `cell_size`). For example, with `cell_size=2.0` and `multi_cell_threshold=1.5`, the threshold becomes 3.0 m.

## Choosing the Right Configuration

### Decision Tree

```
Are you doing physics simulation (mass, friction, dynamics)?
├─ NO  → Use config_physics_off.yaml
│        ✅ Fast, deterministic, kinematics-safe
│        ✅ Safety margin detection
│        ✅ High-speed simulation (100x+)
│
└─ YES → Are all objects physics-based (mass > 0)?
         ├─ YES → Use config_physics_on.yaml
         │        ✅ Fastest physics mode
         │        ✅ Actual contact logging
         │
         └─ NO  → Use config_hybrid.yaml
                  ⚠️ Slower but handles mixed scenarios
                  ⚠️ Requires tuning
```

### Common Use Cases

| Use Case | Recommended Config | Why |
|----------|-------------------|-----|
| Path planning | `config_physics_off.yaml` | Fast, deterministic, safety margin |
| Multi-robot coordination | `config_physics_off.yaml` | High-speed, collision avoidance |
| Physics simulation | `config_physics_on.yaml` | Realistic dynamics |
| Mixed physics/kinematics | `config_hybrid.yaml` | Different detection per type |

## See Also

- [Collision Detection Overview](../architecture/collision-overview) - Design philosophy
- [Collision Configuration Guide](../how-to/collision-config) - Practical collision settings
- [Collision Detection Narrow-Phase Details](../architecture/collision-internals) - Implementation details
