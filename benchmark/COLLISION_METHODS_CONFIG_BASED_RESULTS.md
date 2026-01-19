# Config-Based Collision Detection Benchmark Results

**Date**: 2026-01-19  
**Benchmark**: `collision_methods_config_based.py`  
**Purpose**: Compare collision detection methods using production config files

## Test Configuration

### Test Parameters
- **Objects**: 100 (30% physics, 70% kinematic)
- **Steps**: 500 simulation steps
- **Duration**: ~10 seconds simulation time
- **Environment**: Headless (GUI disabled), profiling enabled

### Tested Configurations

1. **Physics OFF + CLOSEST_POINTS** (`benchmark_physics_off_closest.yaml`)
   - Kinematics mode (no `stepSimulation()`)
   - Distance-based detection (`getClosestPoints`)
   - 2cm safety margin

2. **Physics ON + CONTACT_POINTS** (`benchmark_physics_on_contact.yaml`)
   - Physics mode (`stepSimulation()` every step)
   - Contact manifold detection (`getContactPoints`)
   - No safety margin (actual contact only)

3. **Physics ON + HYBRID** (`benchmark_hybrid.yaml`)
   - Physics mode (required)
   - Mixed detection (contact for physics, closest for kinematic)
   - 2cm safety margin for kinematic pairs

## Results Summary

### Performance Comparison

| Configuration | Method | Collision Time | Step Time | Collisions |
|---------------|--------|---------------|-----------|------------|
| **Physics OFF + CLOSEST** ✅ | `closest_points` | **1.244ms** | 1.511ms | 8.7 |
| Physics ON + CONTACT | `contact_points` | 1.408ms (1.13x) | 2.640ms | 4.2 |
| Physics ON + HYBRID | `hybrid` | 1.270ms (1.02x) | 1.998ms | 10.9 |

### Key Findings

#### 1. **Physics OFF is Fastest** ⚡

**Winner**: Physics OFF + CLOSEST_POINTS

- **Collision detection**: 1.244ms (baseline)
- **Total step time**: 1.511ms (fastest overall)
- **Reason**: No `stepSimulation()` overhead

**Performance gain**:
- 13% faster collision detection than Physics ON + CONTACT
- 43% faster total step time than Physics ON + CONTACT
- 24% faster total step time than Physics ON + HYBRID

#### 2. **stepSimulation() Overhead is Significant** 📊

Physics ON modes pay a penalty for `stepSimulation()`:

| Mode | Collision Time | Step Time | stepSimulation Overhead |
|------|---------------|-----------|------------------------|
| Physics OFF | 1.244ms | 1.511ms | **0.267ms** |
| Physics ON (CONTACT) | 1.408ms | 2.640ms | **1.232ms** (4.6x) |
| Physics ON (HYBRID) | 1.270ms | 1.998ms | **0.728ms** (2.7x) |

**Conclusion**: `stepSimulation()` adds 0.7-1.2ms per step, **dominating the total time**.

#### 3. **Collision Detection Method Impact is Small** 🔍

When comparing **collision detection only** (excluding `stepSimulation`):

- `closest_points`: 1.244ms (Physics OFF)
- `contact_points`: 1.408ms (Physics ON, **1.13x slower**)
- `hybrid`: 1.270ms (Physics ON, **1.02x slower**)

**Surprising result**: `hybrid` is faster than `contact_points` in this scenario!

**Explanation**:
- Most pairs are kinematic-kinematic in this test (70% kinematic)
- `hybrid` uses `getClosestPoints` for these pairs
- `getClosestPoints` may be faster when contact cache is cold

#### 4. **Collision Detection Sensitivity** 🎯

Different methods detect different numbers of collisions:

- **Physics OFF (CLOSEST, 2cm margin)**: 8.7 collisions (proactive, safety margin)
- **Physics ON (CONTACT)**: 4.2 collisions (actual contacts only)
- **Physics ON (HYBRID, 2cm margin)**: 10.9 collisions (most sensitive)

**Interpretation**:
- `CLOSEST_POINTS` with margin detects **near-misses** (safety-focused)
- `CONTACT_POINTS` detects only **actual penetrations** (physics-accurate)
- `HYBRID` detects both (most comprehensive)

## Analysis

### When to Use Each Mode

#### ✅ Physics OFF + CLOSEST_POINTS (Recommended for Production)

**Best for**:
- ✅ Warehouse robot path planning
- ✅ Multi-robot coordination
- ✅ High-speed simulation (100x+ real-time)
- ✅ Collision avoidance (safety margin detection)

**Advantages**:
- **Fastest overall** (1.511ms per step)
- Deterministic (same input → same output)
- Stable with manual pose updates
- Safety margin detection (2cm clearance)

**Disadvantages**:
- No realistic physics behavior (no push-back, friction)

---

#### 🔬 Physics ON + CONTACT_POINTS (Verification Mode)

**Best for**:
- 🔬 Verify collision-free paths under physics
- 🐛 Debug actual collision incidents
- 📊 Analyze contact forces
- 🎓 Educational/visualization purposes

**Advantages**:
- Physics-accurate (actual contact manifold)
- Realistic dynamics (mass, inertia, friction)
- Actual contact logging

**Disadvantages**:
- **Slower** (2.640ms per step, 1.75x slower than Physics OFF)
- Requires fixed timestep (limits speed multiplier)
- Unstable for kinematic-kinematic pairs

---

#### 🚧 Physics ON + HYBRID (Advanced/Experimental)

**Best for**:
- 🔀 Mixed physics/kinematic scenarios
- 🎛️ Fine-tuned detection per object type
- 🧪 Experimental setups

**Advantages**:
- Comprehensive detection (10.9 collisions)
- Different detection for different object types
- Intermediate performance (1.998ms per step)

**Disadvantages**:
- More complex configuration
- Slower than Physics OFF
- Requires careful tuning

## Practical Recommendations

### Decision Tree

```
Q: Do you need realistic physics (mass, friction, dynamics)?
├─ NO  → Use Physics OFF + CLOSEST_POINTS ✅
│        - Fast (1.511ms/step)
│        - Deterministic
│        - Safety margin detection
│        - Config: config_physics_off.yaml
│
└─ YES → Q: Do you need contact force analysis?
         ├─ YES → Use Physics ON + CONTACT_POINTS 🔬
         │        - Physics-accurate
         │        - Actual contact logs
         │        - Config: config_physics_on.yaml
         │
         └─ NO  → Use Physics ON + HYBRID 🚧
                  - Comprehensive detection
                  - Mixed approach
                  - Config: config_hybrid.yaml
```

### Speed Multiplier Recommendations

| Mode | Max Recommended Speed | Reason |
|------|----------------------|--------|
| Physics OFF + CLOSEST | 100x+ | No `stepSimulation()` overhead |
| Physics ON + CONTACT | 1-10x | Fixed timestep required |
| Physics ON + HYBRID | 1-10x | Fixed timestep required |

## Benchmark Reproducibility

### Run the Benchmark

```bash
cd /home/rapyuta/rr_sim_evaluation/PyBulletFleet
python benchmark/collision_methods_config_based.py
```

### Expected Output

```
======================================================================
CONFIG-BASED COLLISION DETECTION COMPARISON
======================================================================

Physics OFF: 1.244ms (CLOSEST_POINTS)
Physics ON:  1.339ms (CONTACT_POINTS/HYBRID)
Difference:  1.08x

✅ Fastest: CLOSEST_POINTS (Physics OFF)
```

### Modify Test Parameters

Edit `collision_methods_config_based.py`:

```python
# Line ~260
num_objects = 100  # Change object count
num_steps = 500    # Change simulation steps
```

Or create custom config files in `config/` and add to the test list.

## Conclusion

**For production use**, **Physics OFF + CLOSEST_POINTS** is the clear winner:

✅ **1.75x faster** than Physics ON + CONTACT  
✅ **Deterministic** (reproducible results)  
✅ **Safety-focused** (detects near-misses)  
✅ **Scalable** (high-speed simulation)  

**For verification/debugging**, Physics ON modes provide:

🔬 Realistic physics behavior  
🐛 Actual contact logging  
📊 Contact force analysis  

**The config files make it easy to switch between modes** for different use cases:

```python
# Production: kinematics mode
params = SimulationParams.from_config("config/config_physics_off.yaml")

# Verification: physics mode
params = SimulationParams.from_config("config/config_physics_on.yaml")
```

---

**Related Documentation**:
- [config/README.md](../config/README.md) - Config files guide
- [docs/COLLISION_DETECTION_DESIGN.md](../docs/COLLISION_DETECTION_DESIGN.md) - Design philosophy
- [benchmark/COLLISION_METHODS_REALISTIC_ANALYSIS.md](COLLISION_METHODS_REALISTIC_ANALYSIS.md) - Detailed analysis
