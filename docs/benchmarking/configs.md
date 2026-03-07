# Benchmark Configuration Files

This directory contains configuration files dedicated to performance benchmarking.

## Config Files

### General Benchmark

#### `general.yaml` — Default Benchmark Config

**Purpose**: Configuration for general-purpose performance testing

**Used by**:
- `performance_benchmark.py` — Main benchmark
- `run_benchmark.py` — Orchestrator

**Key Parameters**:
```yaml
simulation:
  timestep: 0.1           # 10 Hz simulation
  speed: 0.0              # Maximum speed (no sleep)
  duration: 10.0          # 10 seconds
  physics: false          # Kinematics only (faster)
  collision_check_2d: false

agents:
  num_agents: 1000        # Default: 1000 agents
  mass: 0.0               # Kinematic control
```

**Usage**:
```bash
python performance_benchmark.py --config configs/general.yaml
python run_benchmark.py --config configs/general.yaml --runs 5
```

---

### Collision Detection Benchmarks

These configurations are for comparison benchmarks of collision detection methods.

#### `collision_physics_off.yaml` — Recommended: Fastest

**Purpose**: Benchmark for Physics OFF + CLOSEST_POINTS mode

**Used by**:
- `experiments/collision_methods_config_based.py`

**Key Features**:
- Physics engine OFF (fastest)
- `getClosestPoints()` with 2cm margin
- Kinematics-safe collision detection
- Deterministic results

**Key Parameters**:
```yaml
physics: false
collision_detection_method: "closest_points"
collision_margin: 0.02    # 2cm safety distance
speed: 0                  # Maximum speed
timestep: 0.00416666      # 1/240s (240 Hz)
enable_profiling: true
```

**Expected Performance**:
- Collision Time: ~1.24ms
- Total Step Time: ~1.51ms
- Collisions Detected: ~8.7 (includes near-misses)

**Usage**:
```bash
python experiments/collision_methods_config_based.py
# Uses this config by default for Physics OFF test
```

---

#### `collision_physics_on.yaml` — Physics Verification

**Purpose**: Benchmark for Physics ON + CONTACT_POINTS mode

**Used by**:
- `experiments/collision_methods_config_based.py`

**Key Features**:
- Physics engine ON (realistic)
- `getContactPoints()` for actual contact manifold
- Contact resolution (push-back, friction)
- Suitable for validation and debugging

**Key Parameters**:
```yaml
physics: true
collision_detection_method: "contact_points"
collision_margin: 0.0     # Actual contact only
speed: 0
timestep: 0.00416666
enable_profiling: true
```

**Expected Performance**:
- Collision Time: ~1.41ms (1.13x slower)
- Total Step Time: ~2.64ms (includes stepSimulation())
- Collisions Detected: ~4.2 (actual contacts only)

---

#### `collision_hybrid.yaml` — Advanced: Hybrid Mode

**Purpose**: Benchmark for Physics ON + HYBRID method

**Used by**:
- `experiments/collision_methods_config_based.py`

**Key Features**:
- Mixed detection method
  - `getContactPoints()` for physics pairs
  - `getClosestPoints()` for kinematic pairs
- Best of both worlds (at performance cost)
- 2cm margin for kinematics

**Key Parameters**:
```yaml
physics: true
collision_detection_method: "hybrid"
collision_margin: 0.02
speed: 0
timestep: 0.00416666
enable_profiling: true
```

**Expected Performance**:
- Collision Time: ~1.27ms (1.02x slower)
- Total Step Time: ~2.00ms
- Collisions Detected: ~10.9 (most sensitive)

---

## Comparison Summary

| Config | Physics | Method | Collision Time | Total Time | Use Case |
|--------|---------|--------|----------------|------------|----------|
| **collision_physics_off.yaml** (recommended) | OFF | CLOSEST | **1.24ms** | **1.51ms** | **Production** |
| collision_physics_on.yaml | ON | CONTACT | 1.41ms | 2.64ms | Verification |
| collision_hybrid.yaml | ON | HYBRID | 1.27ms | 2.00ms | Advanced |

**Recommendation**: Use **collision_physics_off.yaml** for production benchmarking (fastest and deterministic).

---

## Benchmark Results

For detailed benchmark results, see:

- **Performance Results**: [results](results)
- **Design Document**: [Collision Detection Architecture](../architecture/collision-detection)
- **Config Guide**: `config/` (production config directory)

---

## Quick Start

### Run General Performance Benchmark
```bash
cd benchmark
python performance_benchmark.py --agents 1000 --duration 10 --config configs/general.yaml
```

### Run Collision Detection Comparison
```bash
cd benchmark
python experiments/collision_methods_config_based.py
# Automatically uses all three collision configs
```

### Custom Benchmark
```bash
# Copy and modify a config
cp configs/collision_physics_off.yaml configs/my_custom.yaml
# Edit my_custom.yaml
# Run with custom config
python performance_benchmark.py --config configs/my_custom.yaml
```

---

## Config File Conventions

### Naming
- `general.yaml` — General-purpose benchmark
- `collision_*.yaml` — Collision detection benchmarks
- `<feature>_<variant>.yaml` — Feature-specific benchmarks

### Common Parameters
Recommended settings for all benchmark configs:

```yaml
speed: 0                    # Maximum speed (no sleep)
gui: false                  # Headless mode
monitor: false              # No GUI monitor
enable_profiling: true      # Enable profiling
log_level: error            # Suppress logs
```

### Performance-Critical Parameters
```yaml
physics: false              # Faster (kinematics only)
collision_check_2d: false   # 2D = 9 neighbors, 3D = 27 neighbors
timestep: 0.00416666        # 240 Hz (fine-grained) or 0.1 (10 Hz, coarse)
```

---

## Related Documentation

- **Main Benchmark Guide**: Benchmarking Overview
- **Collision Detection Design**: [Collision Detection Architecture](../architecture/collision-detection)
- **Production Configs**: `config/` (production configs are in a separate directory)
