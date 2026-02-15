# Configuration Files Guide

## Overview

PyBulletFleet provides multiple configuration files for different simulation modes and benchmarking scenarios.

## Configuration Files

### Production Configurations

#### `config.yaml` (Default)
- General-purpose configuration
- Auto-selects collision detection method based on physics mode
- Recommended starting point for most use cases

#### `config_physics_off.yaml` (Kinematics Mode) ✅ Recommended
- **Use case**: Path planning, collision avoidance, high-speed simulation
- **Physics**: OFF (no `stepSimulation()`)
- **Collision method**: `CLOSEST_POINTS` (distance-based, kinematics-safe)
- **Collision margin**: 2cm safety clearance
- **Speed**: 100x real-time
- **Key features**:
  - Fast and deterministic
  - Stable with `resetBasePositionAndOrientation()`
  - Safety margin detection (near-miss detection)

#### `config_physics_on.yaml` (Physics Mode) 🔬 Verification
- **Use case**: Physics verification, debugging, contact analysis
- **Physics**: ON (`stepSimulation()` every step)
- **Collision method**: `CONTACT_POINTS` (actual contact manifold)
- **Speed**: 1x real-time
- **Key features**:
  - Realistic physics behavior (mass, friction, push-back)
  - Actual contact logging
  - Visual collision feedback (color change)

#### `config_hybrid.yaml` (Mixed Mode) 🚧 Advanced
- **Use case**: Mixed physics/kinematic scenarios
- **Physics**: ON (required)
- **Collision method**: `HYBRID` (contact for physics, closest for kinematic)
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

## Usage Examples

### Load Configuration in Python

```python
from pybullet_fleet.core_simulation import SimulationParams

# Load default config
params = SimulationParams.from_config("config/config.yaml")

# Load kinematics mode config
params = SimulationParams.from_config("config/config_physics_off.yaml")

# Load physics mode config
params = SimulationParams.from_config("config/config_physics_on.yaml")
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

### `physics` (bool)
- `false`: Kinematics mode (no `stepSimulation()`, fast, deterministic)
- `true`: Physics mode (`stepSimulation()` every step, realistic dynamics)

### `collision_detection_method` (string)
- `"closest_points"`: Distance-based detection (recommended for kinematics)
  - Stable with manual pose updates
  - Supports safety margin
  - Works in Physics ON/OFF

- `"contact_points"`: Physics contact manifold (recommended for physics)
  - Fastest when `stepSimulation()` is called
  - Requires `physics=true` for reliability
  - Unstable for kinematic-kinematic pairs

- `"hybrid"`: Mixed approach (advanced)
  - Physics pairs: `getContactPoints()`
  - Kinematic pairs: `getClosestPoints()`
  - Slower due to branching

### `collision_margin` (float, meters)
Safety clearance for `getClosestPoints()`:
- `0.01-0.02`: Tight spaces (1-2cm)
- `0.02-0.05`: Normal operation (2-5cm) ✅ **Default**
- `0.05-0.10`: Conservative (5-10cm, high-speed)
- `0.0`: Contact detection only (touching)

**Note**: Only affects `CLOSEST_POINTS` and `HYBRID` modes.
`CONTACT_POINTS` always detects actual contact (margin=0).

## Auto-Selection Logic

If `collision_detection_method` is not specified in config:
```python
# Auto-selection
if physics == false:
    collision_detection_method = "closest_points"  # Kinematics-safe
else:
    collision_detection_method = "contact_points"  # Physics-accurate
```

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
| What-if analysis | `config_physics_off.yaml` | Deterministic, repeatable |
| Physics verification | `config_physics_on.yaml` | Realistic dynamics |
| Incident debugging | `config_physics_on.yaml` | Actual contact logs |
| Contact force analysis | `config_physics_on.yaml` | Physics-accurate |
| Mixed physics/kinematics | `config_hybrid.yaml` | Different detection per type |

## Performance Comparison

Based on `collision_methods_config_based.py` benchmark:

| Config | Method | Avg Collision Time | Speed |
|--------|--------|-------------------|-------|
| Physics OFF + CLOSEST | `closest_points` | ~0.7ms | ⚡ Baseline |
| Physics ON + CONTACT | `contact_points` | ~1.5ms | 2.1x slower |
| Physics ON + HYBRID | `hybrid` | ~1.6ms | 2.3x slower |

**Note**: Physics ON is slower due to `stepSimulation()` overhead, not just collision detection.

## Migration from Old Configs

### Old Config (No collision settings)
```yaml
physics: false
# No collision_detection_method
# No collision_margin
```

### New Config (Explicit and clear)
```yaml
physics: false
collision_detection_method: "closest_points"  # Explicit
collision_margin: 0.02                        # Safety clearance
```

**Auto-migration**: Old configs work without changes due to auto-selection logic.

## Troubleshooting

### "Missing collisions" in kinematics mode
**Solution**: Check that `collision_detection_method: "closest_points"` and `collision_margin` is appropriate (not too small).

### "Unstable collisions" in physics mode
**Solution**: Ensure `physics: true` and `collision_detection_method: "contact_points"`.

### "Slow performance" in physics mode
**Solution**: This is expected. Physics ON requires `stepSimulation()` every step. Use Physics OFF for speed.

### "Collisions detected too early"
**Solution**: Reduce `collision_margin` (e.g., from 0.05 to 0.02 or 0.01).

### "Collisions detected too late"
**Solution**: Increase `collision_margin` (e.g., from 0.02 to 0.05).

## See Also

- [docs/COLLISION_DETECTION_DESIGN.md](../docs/COLLISION_DETECTION_DESIGN.md) - Design philosophy
- [benchmark/COLLISION_METHODS_REALISTIC_ANALYSIS.md](../benchmark/COLLISION_METHODS_REALISTIC_ANALYSIS.md) - Performance analysis
- [Py Bullet Collision Detection Design Summary.docx](../Py%20Bullet%20Collision%20Detection%20Design%20Summary.docx) - Original design document
