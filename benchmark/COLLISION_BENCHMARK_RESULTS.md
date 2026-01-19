# Collision Detection Benchmark Results

**Purpose**: Physics ON vs OFF performance comparison
**Benchmark Script**: `collision_methods_config_based.py`
**Date**: 2026-01-19

---

## Quick Summary

| Mode | Method | Collision Time | Total Step Time | Collisions Detected |
|------|--------|---------------|-----------------|---------------------|
| **Physics OFF** ✅ | `getClosestPoints` | **1.24ms** | **1.51ms** | 8.7 |
| Physics ON | `getContactPoints` | 1.41ms | 2.64ms | 4.2 |
| Physics ON Hybrid | Mixed | 1.27ms | 2.00ms | 10.9 |

**Recommendation**: **Physics OFF mode** is **1.75x faster** and detects more near-miss collisions.

---

## Test Configuration

### Environment
- **Objects**: 100 (30% physics, 70% kinematic)
- **Steps**: 500 simulation steps (~10 sec)
- **Robot Movement**: Random walk with collision avoidance
- **Headless**: GUI disabled for clean profiling

### Tested Configurations

#### 1. Physics OFF + CLOSEST_POINTS ⭐ **Recommended**
```yaml
physics: false
collision_detection_method: "closest_points"
collision_margin: 0.02  # 2cm safety distance
```
- No `stepSimulation()` overhead
- Distance-based detection (predicts near-misses)
- Kinematics-safe and deterministic

#### 2. Physics ON + CONTACT_POINTS
```yaml
physics: true
collision_detection_method: "contact_points"
collision_margin: 0.0  # Actual contact only
```
- Physics engine contact manifold
- Actual penetration detection
- Requires `stepSimulation()` every step

#### 3. Physics ON + HYBRID
```yaml
physics: true
collision_detection_method: "hybrid"
collision_margin: 0.02  # For kinematic pairs
```
- Contact points for physics objects
- Closest points for kinematic pairs
- Best of both worlds (but slower)

---

## Detailed Results

### Performance Breakdown

```
Physics OFF (Kinematics Mode):
├─ Collision Detection: 1.24ms  ━━━━━━━━━━━━━━━━━━━━━ 82%
├─ State Updates:       0.15ms  ━━ 10%
└─ Other:               0.12ms  ━ 8%
Total:                  1.51ms

Physics ON (Contact Points):
├─ stepSimulation():    1.23ms  ━━━━━━━━━━━━━━━━━ 47%
├─ Collision Detection: 1.41ms  ━━━━━━━━━━━━━━━━━━━━━━ 53%
└─ Other:               0.00ms
Total:                  2.64ms  (1.75x slower)

Physics ON (Hybrid):
├─ stepSimulation():    0.73ms  ━━━━━━━━━━━ 37%
├─ Collision Detection: 1.27ms  ━━━━━━━━━━━━━━━━━━━━ 63%
└─ Other:               0.00ms
Total:                  2.00ms  (1.32x slower)
```

### Collision Detection Accuracy

| Mode | Avg Collisions | Safety Margin | Notes |
|------|---------------|---------------|-------|
| Physics OFF | **8.7** | 2cm | Detects near-misses ✅ |
| Physics ON (Contact) | 4.2 | 0cm | Only actual contacts |
| Physics ON (Hybrid) | 10.9 | 2cm | Most sensitive |

**Key Insight**: Physics OFF with `collision_margin=0.02` detects **2x more** potential collisions than contact-based detection, enabling **proactive avoidance**.

---

## Recommendations

### For Production Simulation ⭐
**Use Physics OFF mode**:
```python
params = SimulationParams(
    physics=False,
    collision_detection_method="closest_points",  # Auto-selected
    collision_margin=0.02,  # 2cm safety buffer
)
```

**Why**:
- ⚡ **1.75x faster** than Physics ON
- 🎯 **Deterministic** (same input → same output)
- 🛡️ **Safer** (detects near-misses before contact)
- 📈 **Scales better** (supports high speed multipliers)

### For Physics Verification 🔬
**Use Physics ON mode**:
```python
params = SimulationParams(
    physics=True,
    collision_detection_method="contact_points",
    timestep=1/240,
)
```

**Why**:
- 🔬 Realistic contact behavior (push-back, friction)
- 🐛 Contact logs for debugging
- ✅ Validates collision-free paths under physics

### For Advanced Use Cases
**Use Hybrid mode** if you need:
- Safety margins for kinematic objects
- Contact accuracy for physics objects
- Willing to accept ~1.3x performance cost

---

## Performance Scaling

### Object Count Impact

| Objects | Physics OFF | Physics ON | Speedup |
|---------|-------------|------------|---------|
| 50 | 0.8ms | 1.4ms | 1.75x |
| 100 | 1.5ms | 2.6ms | 1.73x |
| 200 | 3.1ms | 5.4ms | 1.74x |
| 500 | 8.2ms | 14.3ms | 1.74x |

**Conclusion**: Physics OFF maintains **~1.75x speedup** across all scales.

### Speed Multiplier Stability

| Speed | Physics OFF | Physics ON | Notes |
|-------|-------------|------------|-------|
| 1x | ✅ Stable | ✅ Stable | Both work |
| 10x | ✅ Stable | ⚠️ Marginal | Physics may drift |
| 100x | ✅ Stable | ❌ Unstable | Physics breaks down |

**Conclusion**: Only Physics OFF supports high-speed simulation.

---

## Related Documentation

- **Design Document**: `../docs/COLLISION_DETECTION_DESIGN.md` - Complete design philosophy and implementation details
- **Config Examples**: `../config/config_physics_off.yaml`, `../config/config_physics_on.yaml`
- **Benchmark Script**: `collision_methods_config_based.py`

---

## Running the Benchmark

```bash
cd PyBulletFleet/benchmark
python collision_methods_config_based.py
```

**Output**: Detailed timing breakdown and collision statistics.

---

*Last Updated: 2026-01-19*
*PyBulletFleet Collision Detection Benchmark*
